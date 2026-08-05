"""Wheel odometry for the LeKiwi 3-wheel omnidirectional base.

Reads the three Feetech STS3215 base servos over the serial bus, runs the
3-wheel omni forward kinematics, integrates the result into an SE(2) pose and
publishes ``nav_msgs/Odometry`` plus TF.

Dependencies are deliberately limited to ``rclpy``, ``numpy`` and ``pyserial``:
this runs on the D-Robotics X5 inside the Looper camera, which has ~1.3 GB of
writable rootfs and therefore cannot host ``torch`` (and thus not ``lerobot``).
For the same reason this module does *not* import ``tinynav.core.math_utils``,
which would drag in numba, cv2 and fufpy just to build a quaternion.

Two sampling paths
------------------
``--velocity-source position`` (default)
    Sync-read ``Present_Position`` and difference it across the wrap-around
    seam.  The encoder is 4096 ticks/rev, so one tick is ~0.088 deg of wheel
    rotation, i.e. ~77 um of travel with a 50 mm wheel.  Differencing gives
    exactly the displacement that happened, with no dependence on how the servo
    firmware filters anything.

``--velocity-source velocity``
    Sync-read ``Present_Velocity``, which is what upstream LeKiwi
    (``lekiwi.py:347``) uses.  Convenient, but it is quantised to whole ticks/s
    and the servo derives it internally over an unspecified window, so it is
    both coarse and lagged.  Integrating it accumulates that bias directly into
    the pose.  Kept as an option for cross-checking, not recommended for
    navigation.

Frames
------
The repo's existing convention is ``world`` as the root frame with ``camera`` as
the moving child (see ``perception_node.py`` and ``math_utils.np2tf``).  This
node measures *base* motion, not camera motion, so ``Odometry`` and TF are
``world -> base_link`` in REP-103 axes (``+x`` forward, ``+y`` left, ``+z`` up).

That message cannot drive navigation on its own.  ``planning_node``,
``cmd_vel_control`` and ``looper_bridge_node`` all consume a ``PoseStamped``
holding the *camera* pose in the *optical* convention (``+z`` forward, ``+x``
right, ``+y`` down) -- that is what the Looper VIO publishes, and feeding them
REP-103 base axes makes them read the forward axis as "up".  So this node also
publishes ``camera_pose_topic`` (default ``/wheel/camera_pose``), the same
integrated pose pushed through the fixed ``base_link -> camera`` extrinsic
``camera_offset_xyz`` and rotated into optical axes.  Point those three nodes at
it and the run navigates on wheel odometry with no code change.

No ``static_transform_publisher`` is involved, and none is needed: ``base_link``
appears nowhere else in the navigation stack.

Bus ownership
-------------
The Feetech bus is half duplex and a serial port has one owner.  This node
cannot share ``/dev/ttyACM0`` with ``tinynav/platforms/lekiwi_control.py`` (or a
LeRobot ``lekiwi_host``), because both would transmit onto the same wire.  Run
one or the other.  If you want this node to be the sole bus owner, start it with
``-p enable_wheel_command:=true`` and it will also consume ``Twist`` commands and
drive ``Goal_Velocity`` itself.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from tinynav.platforms.feetech_bus import (
    FakeFeetechBus,
    FeetechBus,
    FeetechBusError,
    configure_velocity_mode,
)
from tinynav.platforms.omni3_kinematics import (
    DEFAULT_BASE_RADIUS,
    DEFAULT_TICKS_PER_REV,
    DEFAULT_WHEEL_RADIUS,
    QUAT_BASE_CAMERA,
    Omni3Kinematics,
    base_pose_to_camera_pose,
    integrate_se2,
    wrap_tick_delta,
    yaw_to_quaternion,
)

# nav_msgs/Odometry carries a flat 36-element row-major 6x6 covariance over
# (x, y, z, roll, pitch, yaw).  These are the diagonal / cross-term offsets.
COV_XX, COV_XY, COV_XYAW = 0, 1, 5
COV_YX, COV_YY, COV_YYAW = 6, 7, 11
COV_ZZ = 14
COV_ROLL, COV_PITCH = 21, 28
COV_YAWX, COV_YAWY, COV_YAWYAW = 30, 31, 35


class WheelOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odometry")

        # -- hardware / geometry ------------------------------------------- #
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("wheel_motor_ids", [7, 8, 9])  # left, back, right
        self.declare_parameter("wheel_signs", [1.0, 1.0, 1.0])
        # NOTE: 0.05 / 0.125 are the *upstream defaults*, not measurements.  A 2%
        # wheel_radius error is a 2% distance scale error; a 2% base_radius error
        # is a permanent yaw scale error that never washes out.  Calibrate with
        # tool/wheel_odom_calibrate.py before trusting anything downstream.
        self.declare_parameter("wheel_radius", DEFAULT_WHEEL_RADIUS)
        self.declare_parameter("base_radius", DEFAULT_BASE_RADIUS)
        self.declare_parameter("wheel_mount_angles_deg", [240.0, 0.0, 120.0])
        self.declare_parameter("wheel_mount_offset_deg", -90.0)
        self.declare_parameter("ticks_per_rev", DEFAULT_TICKS_PER_REV)

        # -- loop ----------------------------------------------------------- #
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("velocity_source", "position")  # "position" | "velocity"
        self.declare_parameter("num_read_retries", 2)
        self.declare_parameter("read_timeout_s", 0.02)
        # Longer gaps are dropped rather than integrated: a stalled loop both
        # violates the constant-twist assumption and risks tick aliasing.
        self.declare_parameter("max_dt", 0.5)

        # -- ROS interface -------------------------------------------------- #
        # Default is a fresh topic so this node never silently fights the VIO.
        # Repo topology, verified against a live camera. On the RealSense path
        # perception_node publishes /slam/odometry and map_node subscribes to it.
        # On the Looper path perception_node does not run at all -- the camera
        # supplies depth and VIO and looper_bridge_node converts them -- and
        # *nothing* publishes /slam/odometry, so that subscription never fires and
        # map_node's continuous_odom_recorder stays empty. odom_topic:=/slam/odometry
        # therefore fills a genuinely vacant slot with no risk of a publisher fight.
        #
        # Filling it does not by itself make the robot navigate on wheel odometry:
        # the pose that drives behaviour bypasses /slam/* entirely. planning_node
        # and cmd_vel_control read the camera's PoseStamped topics directly
        # (/camera/camera/vio_image at 20 Hz, vio_100hz at 100 Hz), and map_node's
        # keyframe poses arrive on /slam/keyframe_odom, which the bridge derives
        # from that same VIO. Those are the three switches that matter, and each is
        # now a `pose_topic` parameter on its node.
        self.declare_parameter("odom_topic", "/wheel/odometry")
        self.declare_parameter("odom_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("qos_depth", 50)

        # -- camera-pose output --------------------------------------------- #
        # The three switches above are all PoseStamped in the camera's *optical*
        # convention (see base_pose_to_camera_pose), not Odometry in REP-103 base
        # axes, so nav_msgs/Odometry alone cannot drive them. This publishes the
        # same integrated pose in the form they consume, which is what makes an
        # odometry-navigated run a set of topic parameters instead of a rewrite.
        #
        # camera_offset_xyz is base_link -> camera as [forward, left, up] in
        # metres. The default is the LeKiwi + Looper mount measured on the robot:
        # 60 mm ahead of the base centre, 50 mm to its left, 180 mm above the
        # floor. There is deliberately no static_transform_publisher involved --
        # `base_link` appears nowhere else in the navigation stack, which works
        # entirely in camera poses, so a TF would be decoration.
        self.declare_parameter("camera_pose_topic", "/wheel/camera_pose")
        self.declare_parameter("camera_offset_xyz", [0.06, 0.05, 0.18])
        self.declare_parameter("camera_frame", "camera")

        # -- covariance model ---------------------------------------------- #
        # Two separate error mechanisms, because they grow at different rates and
        # conflating them is the usual way odometry covariance ends up wrong.
        #
        # (1) Random, per-sample measurement noise -> propagated through the SE(2)
        #     Jacobians, so it accumulates as a random walk (sigma ~ sqrt(N)).
        #     Derived geometrically from encoder quantisation and wheel slip; see
        #     _twist_covariance.
        self.declare_parameter("pos_quant_ticks", 1.0)
        self.declare_parameter("vel_noise_ticks", 8.0)
        self.declare_parameter("wheel_slip_frac", 0.05)
        # (2) Systematic scale error -> grows *linearly* with distance travelled,
        #     because it is the same miscalibration applied over and over. This is
        #     the term that actually dominates a real run, and a pure random-walk
        #     model underestimates it badly. Cannot be captured by white per-step
        #     noise, so it is added from path-length accumulators at publish time.
        #     Rationale for the defaults is in docs/x5/wheel_odometry.md; short
        #     version: a kiwi drive has 3 wheels and 3 DOF, so slip is completely
        #     unobservable, and the yaw lever arm is only base_radius (0.125 m)
        #     versus a differential drive's half-track, which amplifies every
        #     per-wheel error into yaw.
        self.declare_parameter("trans_scale_error", 0.03)  # fraction of distance
        self.declare_parameter("rot_scale_error", 0.05)  # fraction of rotation
        self.declare_parameter("rot_bias_per_m", 0.05)  # rad of yaw per metre driven
        # z / roll / pitch are not measured, but the base is rigid on a floor, so
        # a planar prior is more useful to a downstream filter than 1e6.
        self.declare_parameter("planar_variance", 1e-4)

        # -- optional: also drive the wheels (makes this node the bus owner) - #
        self.declare_parameter("enable_wheel_command", False)
        self.declare_parameter("cmd_vel_topic", "/lekiwi_control/cmd_vel")
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("max_wheel_raw", 3000)

        # -- offline / bring-up without hardware ---------------------------- #
        self.declare_parameter("fake_bus", False)
        self.declare_parameter("fake_wheel_ticks_per_s", [0.0, 0.0, 0.0])

        p = self.get_parameter
        self.motor_ids = [int(v) for v in p("wheel_motor_ids").value]
        if len(self.motor_ids) != 3:
            raise ValueError(f"wheel_motor_ids must list exactly 3 ids, got {self.motor_ids}")
        self.velocity_source = str(p("velocity_source").value)
        if self.velocity_source not in ("position", "velocity"):
            raise ValueError(f"velocity_source must be 'position' or 'velocity', got {self.velocity_source}")

        self.ticks_per_rev = int(p("ticks_per_rev").value)
        self.kin = Omni3Kinematics(
            wheel_radius=float(p("wheel_radius").value),
            base_radius=float(p("base_radius").value),
            mount_angles_deg=tuple(float(v) for v in p("wheel_mount_angles_deg").value),
            mount_offset_deg=float(p("wheel_mount_offset_deg").value),
            ticks_per_rev=self.ticks_per_rev,
            wheel_signs=tuple(float(v) for v in p("wheel_signs").value),
        )

        self.odom_frame = str(p("odom_frame").value)
        self.base_frame = str(p("base_frame").value)
        self.publish_tf = bool(p("publish_tf").value)
        self.num_read_retries = int(p("num_read_retries").value)
        self.max_dt = float(p("max_dt").value)
        self.planar_variance = float(p("planar_variance").value)
        self.enable_wheel_command = bool(p("enable_wheel_command").value)
        self.cmd_timeout_s = float(p("cmd_timeout_s").value)
        self.max_wheel_raw = int(p("max_wheel_raw").value)

        # -- state ---------------------------------------------------------- #
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_cov = np.zeros((3, 3))  # random-walk part, over (x, y, yaw)
        self.path_length = 0.0  # metres of travel, for the systematic error term
        self.abs_rotation = 0.0  # radians of |rotation|, likewise
        self.prev_ticks: dict[int, int] | None = None
        self.prev_sample_t: float | None = None
        self._last_tick_deltas: list[int] = [0, 0, 0]
        self.read_failures = 0
        self.samples = 0
        self.last_cmd: Twist | None = None
        self.last_cmd_t: float | None = None

        # -- bus ------------------------------------------------------------ #
        if bool(p("fake_bus").value):
            rates = [float(v) for v in p("fake_wheel_ticks_per_s").value]
            self.bus = FakeFeetechBus(self.motor_ids, dict(zip(self.motor_ids, rates)))
            self.get_logger().warn(f"fake_bus enabled, synthesising wheel ticks/s {rates}")
        else:
            self.bus = FeetechBus(
                port=str(p("port").value),
                baudrate=int(p("baudrate").value),
                timeout=float(p("read_timeout_s").value),
            )
        self.bus.connect()

        if self.enable_wheel_command:
            self._configure_wheels_for_velocity_mode()

        # -- publishers / subscribers --------------------------------------- #
        qos_depth = int(p("qos_depth").value)
        self.odom_pub = self.create_publisher(Odometry, str(p("odom_topic").value), qos_depth)
        camera_pose_topic = str(p("camera_pose_topic").value)
        self.camera_pose_pub = (
            self.create_publisher(PoseStamped, camera_pose_topic, qos_depth)
            if camera_pose_topic
            else None
        )
        self.camera_offset_xyz = np.asarray(
            [float(v) for v in p("camera_offset_xyz").value], dtype=float
        )
        if self.camera_offset_xyz.shape != (3,):
            raise ValueError(
                f"camera_offset_xyz must have 3 elements, got {self.camera_offset_xyz.tolist()}"
            )
        self.camera_frame = str(p("camera_frame").value)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        # base_link -> camera as a static TF too. Not needed by any node -- the
        # stack passes camera poses around in messages, not through TF -- but it
        # completes the chain for rviz, which otherwise cannot place anything
        # stamped in the `camera` frame: map_node broadcasts only world -> map and
        # nothing at all publishes world -> camera.
        self.static_tf_broadcaster = None
        if self.publish_tf and self.camera_frame:
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            self.static_tf_broadcaster.sendTransform(self._base_to_camera_tf())
        self.create_service(Empty, "~/reset", self._reset_srv)
        if self.enable_wheel_command:
            self.create_subscription(Twist, str(p("cmd_vel_topic").value), self._cmd_cb, 10)

        rate = float(p("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"wheel odometry up: ids={self.motor_ids} source={self.velocity_source} "
            f"r_wheel={self.kin.wheel_radius:.4f}m r_base={self.kin.base_radius:.4f}m "
            f"rate={rate:.1f}Hz -> {p('odom_topic').value} ({self.odom_frame} -> {self.base_frame})"
        )
        self.get_logger().info(
            f"camera pose -> {camera_pose_topic or '<disabled>'} "
            f"offset[fwd,left,up]={self.camera_offset_xyz.tolist()} "
            f"cmd={'/' + str(p('cmd_vel_topic').value).lstrip('/') if self.enable_wheel_command else '<disabled>'}"
        )

    # -- setup helpers ------------------------------------------------------ #

    def _configure_wheels_for_velocity_mode(self) -> None:
        """Put the three base servos in continuous-velocity mode with torque on.

        Only called when this node owns wheel commands. The EEPROM unlock dance
        and its read-back live in ``feetech_bus.configure_velocity_mode``, shared
        with the calibration and yaw-comparison tools -- see that docstring for
        why a locked write is acknowledged and then discarded.
        """
        configure_velocity_mode(self.bus, self.motor_ids)

    def _cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_t = self._mono()

    def _reset_srv(self, _req: Empty.Request, resp: Empty.Response) -> Empty.Response:
        self.x = self.y = self.theta = 0.0
        self.pose_cov = np.zeros((3, 3))
        self.path_length = 0.0
        self.abs_rotation = 0.0
        self.prev_ticks = None
        self.prev_sample_t = None
        self.get_logger().info("wheel odometry pose and covariance reset to origin")
        return resp

    def _mono(self) -> float:
        return time.monotonic()

    # -- main loop ---------------------------------------------------------- #

    def _tick(self) -> None:
        # rclpy's SIGINT/SIGTERM handler invalidates the context, but a timer
        # callback that has already been dispatched keeps running -- so
        # publishing below would raise RCLError("publisher's context is invalid")
        # out of the executor. That turns every ordinary shutdown into a
        # traceback, which hides real faults at exactly the moment a stop is in
        # progress; a shutdown that looks like a crash is one nobody trusts. The
        # wheels are still stopped by destroy_node(), which writes to the serial
        # port and needs no ROS context.
        if not rclpy.ok():
            return

        register = "Present_Position" if self.velocity_source == "position" else "Present_Velocity"
        try:
            result = self.bus.sync_read(register, self.motor_ids, num_retry=self.num_read_retries)
        except FeetechBusError as exc:
            # Narrow on purpose: a single corrupted status packet must not take
            # down navigation, but anything that is not a bus error still
            # propagates (fail fast, per AGENTS.md).
            self.read_failures += 1
            self.get_logger().warn(
                f"wheel {register} read failed ({self.read_failures} total): {exc}",
                throttle_duration_sec=2.0,
            )
            return

        # Map the monotonic sample instant onto the ROS clock, preserving the
        # measured read latency instead of stamping with "now".
        now_ros = self.get_clock().now()
        stamp = (now_ros - rclpy.duration.Duration(seconds=self._mono() - result.t_sample)).to_msg()

        if self.velocity_source == "position":
            body_delta, dt = self._body_delta_from_positions(result)
        else:
            body_delta, dt = self._body_delta_from_velocities(result)
        if body_delta is None:
            return

        dx, dy, dtheta = body_delta
        wheel_speeds_ticks = np.array([result.values[i] for i in self.motor_ids], dtype=float)
        if self.velocity_source == "position":
            wheel_speeds_ticks = np.array(self._last_tick_deltas, dtype=float) / dt

        twist_cov = self._twist_covariance(wheel_speeds_ticks, dt)
        self._propagate_pose(dx, dy, dtheta, twist_cov, dt)
        self.samples += 1

        twist = np.array([dx, dy, dtheta]) / dt
        self._publish(stamp, twist, twist_cov)
        if self.enable_wheel_command:
            self._drive_wheels()

    def _body_delta_from_positions(self, result):
        """Tick differencing path: returns (body_delta, dt) or (None, None)."""
        ticks = {i: int(result.values[i]) % self.ticks_per_rev for i in self.motor_ids}
        if self.prev_ticks is None or self.prev_sample_t is None:
            self.prev_ticks, self.prev_sample_t = ticks, result.t_sample
            return None, None

        dt = result.t_sample - self.prev_sample_t
        if dt <= 0.0:
            return None, None
        if dt > self.max_dt:
            self.get_logger().warn(
                f"sample gap {dt * 1e3:.0f}ms exceeds max_dt {self.max_dt * 1e3:.0f}ms, "
                "re-baselining encoders (tick deltas may have aliased)"
            )
            self.prev_ticks, self.prev_sample_t = ticks, result.t_sample
            return None, None

        deltas = [wrap_tick_delta(ticks[i], self.prev_ticks[i], self.ticks_per_rev) for i in self.motor_ids]
        self.prev_ticks, self.prev_sample_t = ticks, result.t_sample
        self._last_tick_deltas = deltas
        return self.kin.wheel_tick_delta_to_body_delta(deltas), dt

    def _body_delta_from_velocities(self, result):
        """Reported-velocity path: returns (body_delta, dt) or (None, None)."""
        if self.prev_sample_t is None:
            self.prev_sample_t = result.t_sample
            return None, None
        dt = result.t_sample - self.prev_sample_t
        if dt <= 0.0:
            return None, None
        if dt > self.max_dt:
            self.get_logger().warn(
                f"sample gap {dt * 1e3:.0f}ms exceeds max_dt {self.max_dt * 1e3:.0f}ms, skipping"
            )
            self.prev_sample_t = result.t_sample
            return None, None
        self.prev_sample_t = result.t_sample
        ticks_per_s = [result.values[i] for i in self.motor_ids]
        body_vel = self.kin.wheel_ticks_per_s_to_body(ticks_per_s)
        return body_vel * dt, dt

    # -- covariance --------------------------------------------------------- #

    def _twist_covariance(self, wheel_ticks_per_s: np.ndarray, dt: float) -> np.ndarray:
        """3x3 covariance of ``(vx, vy, omega)`` from per-wheel measurement noise.

        Per-wheel linear-speed noise has two parts:

        * a *reading* term, white.  In position mode the speed is a finite
          difference of two rounded tick counts, so its sigma is
          ``pos_quant_ticks / dt`` ticks/s -- note that this shrinks as dt grows,
          which is the real reason a slower loop dead-reckons more smoothly.  In
          velocity mode it is a fixed ``vel_noise_ticks``, which is much larger
          because the servo's internal velocity estimate is coarse and lagged.
        * a *slip* term proportional to the wheel's own speed
          (``wheel_slip_frac``).  Omniwheel rollers skid sideways under load; a
          faster wheel slips more.

        The interesting part is the projection.  Mapping wheel noise into the body
        frame with ``m_inv`` makes the geometry explicit: the yaw row of ``m_inv``
        is ``[1, 1, 1] / (3 * base_radius)``, so with base_radius = 0.125 m every
        metre-per-second of per-wheel error becomes 2.67 rad/s of yaw error, and
        the RSS over three independent wheels is 4.62.  A differential drive with
        a 0.5 m track has yaw = (v_r - v_l) / track, i.e. a per-wheel coefficient
        of 2.0 and an RSS of 2.83.  So the geometry alone makes this base ~1.6x
        noisier in yaw; the rest of the gap to a diff drive is slip, which on a
        kiwi drive is both continuous (the rollers skid sideways whenever the base
        turns) and completely unobservable (3 wheels, 3 DOF, no redundancy).

        Note also that ``m_inv`` gives vy twice the noise of vx (rows 1 and 2 have
        RSS 0.82 vs 0.71 -- the back wheel contributes to vy alone), which is why
        the published lateral variance is larger than the longitudinal one without
        any hand-tuning.
        """
        p = self.get_parameter
        if self.velocity_source == "position":
            read_ticks = float(p("pos_quant_ticks").value) / max(dt, 1e-6)
        else:
            read_ticks = float(p("vel_noise_ticks").value)
        slip_frac = float(p("wheel_slip_frac").value)

        # ticks/s -> m/s at the wheel contact point
        ticks_to_mps = self.kin.wheel_radius / self.kin.ticks_per_rad
        sigma_read = read_ticks * ticks_to_mps
        sigma_slip = slip_frac * np.abs(wheel_ticks_per_s) * ticks_to_mps
        sigma_wheel_sq = sigma_read**2 + sigma_slip**2

        return self.kin.m_inv @ np.diag(sigma_wheel_sq) @ self.kin.m_inv.T

    def _propagate_pose(
        self, dx: float, dy: float, dtheta: float, twist_cov: np.ndarray, dt: float
    ) -> None:
        """EKF-style dead-reckoning update of the pose and its random-walk covariance.

        ``P <- Gx P Gx^T + Gu Q Gu^T`` with the standard SE(2) Jacobians, where
        ``Q = twist_cov * dt^2`` is the per-sample *measurement* noise turned into
        an increment covariance.  Because that noise is white, ``P`` grows like a
        random walk: sigma ~ sqrt(number of samples).

        That is only half the story.  The systematic part (a miscalibrated
        ``wheel_radius`` or ``base_radius`` is the same error every single step,
        not a fresh random draw) grows *linearly* with distance and is added
        separately in :meth:`_systematic_covariance` from the accumulators
        updated here.  Folding it into ``Q`` as white noise would understate the
        real drift by a factor of sqrt(N) -- at 50 Hz over a 20 m run that is
        more than 30x.

        Consequence worth stating explicitly: the total covariance grows without
        bound.  There is no loop closure and no absolute reference here, so that
        is the honest answer; whoever consumes it must supply the correction.
        """
        theta0 = self.theta
        self.x, self.y, self.theta = integrate_se2(self.x, self.y, self.theta, dx, dy, dtheta)
        self.path_length += float(np.hypot(dx, dy))
        self.abs_rotation += abs(float(dtheta))

        q = twist_cov * dt**2

        # d(world displacement)/d(theta) = (-world_dy, world_dx)
        cos_t, sin_t = np.cos(theta0), np.sin(theta0)
        world_dx = cos_t * dx - sin_t * dy
        world_dy = sin_t * dx + cos_t * dy
        g_x = np.array([[1.0, 0.0, -world_dy], [0.0, 1.0, world_dx], [0.0, 0.0, 1.0]])
        g_u = np.array([[cos_t, -sin_t, 0.0], [sin_t, cos_t, 0.0], [0.0, 0.0, 1.0]])
        self.pose_cov = g_x @ self.pose_cov @ g_x.T + g_u @ q @ g_u.T

    def _systematic_covariance(self) -> np.ndarray:
        """Linear-in-distance error from miscalibrated geometry.

        Three contributions, all 1-sigma:

        * ``trans_scale_error * path_length`` -- a wheel_radius error scales every
          distance by the same fraction.  3% is what you get from an uncalibrated
          nominal radius on a compliant omniwheel.
        * ``rot_scale_error * abs_rotation`` -- a base_radius error scales every
          turn.  5% of a 360 degree spin is 18 degrees, which is why the spin
          calibration matters.
        * ``rot_bias_per_m * path_length`` -- yaw drift accumulated while driving
          straight, from asymmetric roller slip.  0.05 rad/m (2.9 deg/m) is
          roughly 5x what a well calibrated differential drive claims, because a
          kiwi drive slips sideways continuously and has a short (0.125 m) yaw
          lever arm.

        Reported as a diagonal.  The true error is correlated (a yaw bias also
        displaces position), so this understates the off-diagonals while being
        conservative on the diagonal -- acceptable for a term whose purpose is to
        stop a consumer from over-trusting the pose.
        """
        p = self.get_parameter
        sigma_trans = float(p("trans_scale_error").value) * self.path_length
        sigma_yaw = float(p("rot_scale_error").value) * self.abs_rotation + float(
            p("rot_bias_per_m").value
        ) * self.path_length
        return np.diag([sigma_trans**2, sigma_trans**2, sigma_yaw**2])

    # -- output ------------------------------------------------------------- #

    def _base_to_camera_tf(self) -> TransformStamped:
        """The fixed ``base_frame -> camera_frame`` extrinsic, in optical axes."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.camera_frame
        tf.transform.translation.x = float(self.camera_offset_xyz[0])
        tf.transform.translation.y = float(self.camera_offset_xyz[1])
        tf.transform.translation.z = float(self.camera_offset_xyz[2])
        qx, qy, qz, qw = QUAT_BASE_CAMERA
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        return tf

    def _publish(self, stamp, twist: np.ndarray, twist_cov: np.ndarray) -> None:
        # Re-checked here as well as in _tick: the signal can land between the
        # two, and this is the call that actually raises.
        if not rclpy.ok():
            return

        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self._fill_covariance(msg.pose.covariance, self.pose_cov + self._systematic_covariance())

        # Twist is expressed in child_frame_id, i.e. the body frame -- which is
        # what the kinematics natively produces, no rotation needed.
        msg.twist.twist.linear.x = float(twist[0])
        msg.twist.twist.linear.y = float(twist[1])
        msg.twist.twist.angular.z = float(twist[2])
        self._fill_covariance(msg.twist.covariance, twist_cov)
        self.odom_pub.publish(msg)

        if self.camera_pose_pub is not None:
            position, quaternion = base_pose_to_camera_pose(
                self.x, self.y, self.theta, self.camera_offset_xyz
            )
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = self.odom_frame
            pose_msg.pose.position.x = float(position[0])
            pose_msg.pose.position.y = float(position[1])
            pose_msg.pose.position.z = float(position[2])
            pose_msg.pose.orientation.x = float(quaternion[0])
            pose_msg.pose.orientation.y = float(quaternion[1])
            pose_msg.pose.orientation.z = float(quaternion[2])
            pose_msg.pose.orientation.w = float(quaternion[3])
            self.camera_pose_pub.publish(pose_msg)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def _fill_covariance(self, target, cov3: np.ndarray) -> None:
        """Scatter a 3x3 (x, y, yaw) covariance into a 6x6 row-major array."""
        target[COV_XX] = float(cov3[0, 0])
        target[COV_XY] = target[COV_YX] = float(cov3[0, 1])
        target[COV_YY] = float(cov3[1, 1])
        target[COV_XYAW] = target[COV_YAWX] = float(cov3[0, 2])
        target[COV_YYAW] = target[COV_YAWY] = float(cov3[1, 2])
        target[COV_YAWYAW] = float(cov3[2, 2])
        target[COV_ZZ] = target[COV_ROLL] = target[COV_PITCH] = self.planar_variance

    # -- optional wheel commands -------------------------------------------- #

    def _drive_wheels(self) -> None:
        """Push the latest Twist to Goal_Velocity, with a watchdog stop."""
        stale = (
            self.last_cmd is None
            or self.last_cmd_t is None
            or (self._mono() - self.last_cmd_t) > self.cmd_timeout_s
        )
        if stale:
            raw = np.zeros(3, dtype=int)
        else:
            cmd = self.last_cmd
            raw = self.kin.body_to_wheel_raw(
                cmd.linear.x, cmd.linear.y, cmd.angular.z, max_raw=self.max_wheel_raw
            )
        self.bus.sync_write("Goal_Velocity", dict(zip(self.motor_ids, (int(v) for v in raw))))

    # -- teardown ----------------------------------------------------------- #

    def destroy_node(self) -> bool:
        if self.enable_wheel_command and self.bus is not None:
            self.bus.sync_write("Goal_Velocity", dict.fromkeys(self.motor_ids, 0))
        self.get_logger().info(
            f"wheel odometry down after {self.samples} samples, {self.read_failures} read failures; "
            f"final pose x={self.x:.3f} y={self.y:.3f} yaw={np.degrees(self.theta):.2f}deg"
        )
        self.bus.disconnect()
        return super().destroy_node()


def main(args=None):
    # Configuration goes through ROS parameters (``--ros-args -p name:=value``)
    # rather than the argparse style used elsewhere in tinynav/core, because the
    # topic name and the geometry must be overridable from a launcher/script
    # without editing code.
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT and SIGTERM are how this node is normally stopped, not faults.
        pass
    finally:
        # Runs on every path, including an unexpected exception, because this is
        # what zeroes Goal_Velocity. It writes to the serial port and needs no
        # ROS context, so it still works after rclpy has shut down.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
