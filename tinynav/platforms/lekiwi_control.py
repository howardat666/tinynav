"""Turn a planned trajectory into base velocity commands for the LeKiwi base.

This node does **no hardware I/O**. It subscribes to ``/planning/trajectory_path``
and publishes ``geometry_msgs/Twist`` on ``/lekiwi_control/cmd_vel``; something
else puts those on the wire. That split is deliberate, for two reasons:

*Bus ownership.* The Feetech servo bus is half duplex and a serial port has
exactly one owner. ``tinynav/core/wheel_odometry_node.py`` already reads the
wheels continuously, so it is the natural owner; started with
``-p enable_wheel_command:=true`` it also consumes this node's Twist and writes
``Goal_Velocity``. Two processes opening the same port corrupt each other's
transactions.

*Dependencies.* This node used to drive the wheels via
``lerobot.robots.lekiwi.LeKiwi``, and ``lerobot[lekiwi]==0.3.3`` depends on
``torch>=2.4``. On the D-Robotics X5 inside the Looper camera that is not
available: MemTotal is 1307 MB with ~850 MB actually free and no swap, and
relocalization alone already peaks around 405 MB RSS. The torch-free
replacement is ``tinynav/platforms/feetech_bus.py`` plus
``tinynav/platforms/omni3_kinematics.py``.

So the runtime chain is::

    planning_node -> /planning/trajectory_path
                  -> lekiwi_control  (this node: path -> Twist)
                  -> /lekiwi_control/cmd_vel
                  -> wheel_odometry_node (enable_wheel_command:=true)
                  -> Goal_Velocity on the servo bus

FRAME CONVENTION -- READ THIS BEFORE CHANGING THE AXIS INDICES
--------------------------------------------------------------
The planned path is expressed in the repo's **camera** frame, not a base frame:
x right, y down, z forward. After rotating a position delta into the local frame,
forward is therefore component **[2]**, and the yaw rate is rotation about the
camera's **[1]** (down) axis. That is why the indices below are not the [0] and
[2] you would expect from a base frame. They are inherited from the version of
this node that drove the wheels through lerobot, and they are preserved here
unchanged.

One thing genuinely is unverified: because camera-y points *down* while base-yaw
is measured about *up*, the yaw rate plausibly needs negating. Nobody has driven
this on real hardware yet, so rather than silently guessing, the sign is exposed
as the ``yaw_sign`` parameter, defaulting to +1.0 to preserve existing
behaviour. Determining it is a one-minute test: put the chassis on blocks,
command a path that turns left, and check which way it spins. If it turns the
wrong way, set ``-p yaw_sign:=-1.0``.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node

from tinynav.platforms.omni3_kinematics import (
    DEFAULT_BASE_RADIUS,
    DEFAULT_TICKS_PER_REV,
    DEFAULT_WHEEL_RADIUS,
    Omni3Kinematics,
    quaternion_relative_rotvec,
    quaternion_rotate_inverse,
)

# Index of "forward" and of the yaw axis in the camera frame. See the module
# docstring; named rather than inlined so that they cannot be mistaken for
# base-frame indices and "corrected".
CAMERA_FORWARD_AXIS = 2
CAMERA_YAW_AXIS = 1


class LeKiwiControlNode(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_control")

        # -- trajectory sampling ------------------------------------------- #
        # The planner emits poses on a fixed grid; this must match it. It is a
        # parameter rather than the previous hardcoded 0.1 so that a change on
        # the planner side does not silently rescale every velocity here.
        self.declare_parameter("trajectory_dt", 0.1)
        self.declare_parameter("publish_rate_hz", 20.0)
        # After this long with no usable trajectory sample, publish zeros. Note
        # this is not the safety stop -- that lives in the node that owns the
        # bus, which cannot be talked out of it by this node crashing.
        self.declare_parameter("path_timeout_s", 0.5)

        # -- limits --------------------------------------------------------- #
        # Defaults are derived from the geometry and the servo command ceiling
        # rather than typed in, so they cannot drift away from what the hardware
        # can do. Override only downward.
        self.declare_parameter("max_wheel_raw", 3000)
        self.declare_parameter("wheel_radius", DEFAULT_WHEEL_RADIUS)
        self.declare_parameter("base_radius", DEFAULT_BASE_RADIUS)
        self.declare_parameter("ticks_per_rev", DEFAULT_TICKS_PER_REV)
        self.declare_parameter("max_linear_x", -1.0)  # <0 means "use the derived limit"
        self.declare_parameter("max_angular_z", -1.0)
        self.declare_parameter("yaw_sign", 1.0)

        # -- topics --------------------------------------------------------- #
        self.declare_parameter("path_topic", "/planning/trajectory_path")
        self.declare_parameter("cmd_vel_topic", "/lekiwi_control/cmd_vel")

        p = self.get_parameter
        self.trajectory_dt = float(p("trajectory_dt").value)
        if self.trajectory_dt <= 0.0:
            raise ValueError(f"trajectory_dt must be positive, got {self.trajectory_dt}")
        self.path_timeout_s = float(p("path_timeout_s").value)
        self.yaw_sign = float(p("yaw_sign").value)

        self.kin = Omni3Kinematics(
            wheel_radius=float(p("wheel_radius").value),
            base_radius=float(p("base_radius").value),
            ticks_per_rev=int(p("ticks_per_rev").value),
        )
        limits = self.kin.max_body_velocity(max_raw=int(p("max_wheel_raw").value))

        requested_linear = float(p("max_linear_x").value)
        requested_angular = float(p("max_angular_z").value)
        self.max_linear_x = limits["vx"] if requested_linear <= 0.0 else min(requested_linear, limits["vx"])
        self.max_angular_z = (
            limits["omega"] if requested_angular <= 0.0 else min(requested_angular, limits["omega"])
        )
        if requested_linear > limits["vx"]:
            self.get_logger().warn(
                f"max_linear_x:={requested_linear:.3f} m/s exceeds what the base can do "
                f"({limits['vx']:.3f} m/s); clamped to the hardware limit"
            )
        if requested_angular > limits["omega"]:
            self.get_logger().warn(
                f"max_angular_z:={requested_angular:.3f} rad/s exceeds the hardware limit "
                f"({limits['omega']:.3f} rad/s); clamped"
            )

        self.cmd_pub = self.create_publisher(Twist, str(p("cmd_vel_topic").value), 10)
        self.create_subscription(Path, str(p("path_topic").value), self.path_callback, 10)

        self.last_path: Path | None = None
        self.published_zero = False

        rate = float(p("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"lekiwi_control up: {p('path_topic').value} -> {p('cmd_vel_topic').value} at {rate:.0f} Hz; "
            f"limits {self.max_linear_x:.3f} m/s, {self.max_angular_z:.3f} rad/s "
            f"(yaw_sign={self.yaw_sign:+.0f}); this node does not touch the servo bus"
        )

    def path_callback(self, msg: Path) -> None:
        self.last_path = msg

    def _now_s(self) -> float:
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec * 1e-9

    def _sample_velocity(self) -> Twist | None:
        """Velocity for the current time from the stored path, or None if unusable.

        Evaluated on a timer against the current clock rather than only when a
        path arrives. A trajectory is time-parameterised, so re-sampling it as
        time advances is what following it means; computing once per path message
        instead leaves the base holding a stale velocity between messages.
        """
        path = self.last_path
        if path is None or len(path.poses) < 2:
            return None

        stamp = path.header.stamp
        elapsed = self._now_s() - (stamp.sec + stamp.nanosec * 1e-9)
        idx = round(elapsed / self.trajectory_dt)
        if idx < 0 or idx >= len(path.poses) - 1:
            return None

        p1 = path.poses[idx].pose.position
        p2 = path.poses[idx + 1].pose.position
        q1 = path.poses[idx].pose.orientation
        q2 = path.poses[idx + 1].pose.orientation
        quat1 = (q1.x, q1.y, q1.z, q1.w)
        quat2 = (q2.x, q2.y, q2.z, q2.w)

        # Equivalent to the previous scipy formulation, kept expression-for-
        # expression so behaviour does not change:
        #   linear  = R.from_quat(q1).inv().apply(dp / dt)
        #   angular = (R.from_quat(q2).inv() * R.from_quat(q1)).as_rotvec() / dt
        # Note the angular term is q2^-1 * q1, i.e. the *reverse* increment, so
        # its sign is opposite to the forward one. That is inherited, not chosen;
        # see the yaw_sign note in the module docstring.
        linear = quaternion_rotate_inverse(
            quat1, np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z]) / self.trajectory_dt
        )
        angular = quaternion_relative_rotvec(quat2, quat1) / self.trajectory_dt

        cmd = Twist()
        cmd.linear.x = float(
            np.clip(linear[CAMERA_FORWARD_AXIS], -self.max_linear_x, self.max_linear_x)
        )
        cmd.angular.z = float(
            np.clip(
                self.yaw_sign * angular[CAMERA_YAW_AXIS],
                -self.max_angular_z,
                self.max_angular_z,
            )
        )
        return cmd

    def _tick(self) -> None:
        cmd = self._sample_velocity()
        if cmd is None:
            # Publish zeros once, then stay quiet: a continuous stream of zeros
            # would keep the downstream watchdog fed, which is the opposite of
            # what a stalled planner should do. Going silent lets the bus owner's
            # timeout fire and stop the wheels on its own.
            if not self.published_zero:
                self.cmd_pub.publish(Twist())
                self.published_zero = True
                self.get_logger().warn(
                    "no usable trajectory sample for the current time; commanded stop and "
                    "stopped publishing, so the bus owner's watchdog takes over"
                )
            return

        self.published_zero = False
        self.cmd_pub.publish(cmd)

    def destroy_node(self) -> bool:
        # Best effort: if the bus owner is still alive it will see this and stop.
        # If it is not, its own teardown already zeroed Goal_Velocity.
        try:
            self.cmd_pub.publish(Twist())
        except RuntimeError as exc:
            # Publishing during teardown races rclpy's own shutdown; the bus
            # owner zeroes Goal_Velocity in its teardown regardless, so this is
            # worth noting and not worth failing over.
            self.get_logger().warn(f"could not publish the final stop command: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeKiwiControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
