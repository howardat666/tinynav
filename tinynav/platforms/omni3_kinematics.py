"""Three-wheel omnidirectional (kiwi drive) kinematics, numpy only.

Extracted from the upstream LeRobot LeKiwi driver
(``lerobot/robots/lekiwi/lekiwi.py``, methods ``_body_to_wheel_raw`` and
``_wheel_raw_to_body``, Apache-2.0, HuggingFace Inc.) so that it can be used
without importing ``lerobot`` -- and therefore without ``torch``, which does not
fit on the 1.3 GB rootfs of the D-Robotics X5.

Two deliberate differences from upstream:

* Angles are in **radians per second** everywhere, not deg/s.  Upstream returns
  ``theta.vel`` in deg/s, which is a trap when feeding ROS messages.
* The tick <-> SI conversions are exposed separately from the integer command
  rounding, so a float round-trip ``body -> wheel -> body`` is exact.

Geometry
--------
The three omniwheels sit at 120 degree intervals on a circle of radius
``base_radius``.  Upstream mounts them at ``[240, 0, 120]`` degrees for
(left, back, right) and applies a -90 degree offset, giving contact-direction
angles ``[150, -90, 30]`` degrees.

For a wheel whose rolling direction is at angle ``a`` from the body +x axis and
which sits at distance ``base_radius`` from the centre, the contact-point speed
along the rolling direction is::

    v_i = cos(a_i) * vx + sin(a_i) * vy + base_radius * omega

Stacking the three wheels gives ``v = M @ [vx, vy, omega]`` with::

    M = [[cos(a_i), sin(a_i), base_radius] for a_i in angles]

``M`` is square and (for 120 degree spacing) invertible, so the forward
kinematics used by odometry is simply ``[vx, vy, omega] = M^-1 @ v``.  There is
no least-squares/redundancy here: three wheels, three DOF, exactly determined.
That is also why this base has **no slip observability at all** -- any wheel slip
maps straight into a body velocity error with nothing to detect it.  See
``docs/x5/wheel_odometry.md``.

Sign conventions
----------------
ROS REP-103: +x forward, +y left, +z up, ``omega`` positive counter-clockwise
seen from above.  Whether a positive ``Present_Velocity`` on a given servo
actually corresponds to a positive ``v_i`` depends on how the motor is wired and
which way the wheel is bolted on.  ``wheel_signs`` flips individual wheels; get
it wrong and the odometry will look plausible but drift wildly.  The calibration
script (``tool/wheel_odom_calibrate.py``) checks the signs before anything else.
"""

from __future__ import annotations

import numpy as np

# LeKiwi factory geometry.  MEASURE THESE ON THE ACTUAL ROBOT -- see the docs.
DEFAULT_WHEEL_RADIUS = 0.05  # metres
DEFAULT_BASE_RADIUS = 0.125  # metres
DEFAULT_WHEEL_MOUNT_ANGLES_DEG = (240.0, 0.0, 120.0)  # (left, back, right)
DEFAULT_MOUNT_OFFSET_DEG = -90.0

# STS3215 encoder resolution: ticks per full wheel revolution.
DEFAULT_TICKS_PER_REV = 4096


class Omni3Kinematics:
    """Forward/inverse kinematics of a 3-wheel omni base.

    Args:
        wheel_radius: rolling radius of one omniwheel, metres.
        base_radius: distance from the base centre to each wheel contact, metres.
        mount_angles_deg: the three wheel mounting angles before the offset, in
            the order the motor IDs are given.
        mount_offset_deg: added to every mounting angle.
        ticks_per_rev: encoder counts per wheel revolution.
        wheel_signs: per-wheel +1/-1 to absorb reversed motor wiring.
    """

    def __init__(
        self,
        wheel_radius: float = DEFAULT_WHEEL_RADIUS,
        base_radius: float = DEFAULT_BASE_RADIUS,
        mount_angles_deg: tuple[float, float, float] = DEFAULT_WHEEL_MOUNT_ANGLES_DEG,
        mount_offset_deg: float = DEFAULT_MOUNT_OFFSET_DEG,
        ticks_per_rev: int = DEFAULT_TICKS_PER_REV,
        wheel_signs: tuple[float, float, float] = (1.0, 1.0, 1.0),
    ) -> None:
        if wheel_radius <= 0.0 or base_radius <= 0.0:
            raise ValueError("wheel_radius and base_radius must be positive")
        if ticks_per_rev <= 0:
            raise ValueError("ticks_per_rev must be positive")
        self.wheel_radius = float(wheel_radius)
        self.base_radius = float(base_radius)
        self.ticks_per_rev = int(ticks_per_rev)
        self.wheel_signs = np.asarray(wheel_signs, dtype=float)
        if self.wheel_signs.shape != (3,):
            raise ValueError("wheel_signs must have three entries")

        self.angles = np.radians(np.asarray(mount_angles_deg, dtype=float) + float(mount_offset_deg))
        self.m = np.array([[np.cos(a), np.sin(a), self.base_radius] for a in self.angles])
        det = float(np.linalg.det(self.m))
        if abs(det) < 1e-9:
            raise ValueError(f"degenerate wheel layout (det(M)={det:.3e}); check mount_angles_deg")
        self.m_inv = np.linalg.inv(self.m)

        # rad/s of a wheel -> encoder ticks/s.  rad/s * (rev/2pi rad) * ticks/rev
        self.ticks_per_rad = self.ticks_per_rev / (2.0 * np.pi)

    # -- SI level ----------------------------------------------------------- #

    def body_to_wheel_radps(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """Body velocity (m/s, m/s, rad/s) -> three wheel speeds in rad/s."""
        wheel_linear = self.m @ np.array([vx, vy, omega], dtype=float)
        return wheel_linear / self.wheel_radius

    def wheel_radps_to_body(self, wheel_radps) -> np.ndarray:
        """Three wheel speeds in rad/s -> body velocity ``[vx, vy, omega]``."""
        wheel_linear = np.asarray(wheel_radps, dtype=float) * self.wheel_radius
        return self.m_inv @ wheel_linear

    # -- encoder tick level ------------------------------------------------- #

    def body_to_wheel_ticks_per_s(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """Body velocity -> signed encoder ticks/s per wheel (float, unrounded)."""
        radps = self.body_to_wheel_radps(vx, vy, omega)
        return radps * self.ticks_per_rad * self.wheel_signs

    def wheel_ticks_per_s_to_body(self, ticks_per_s) -> np.ndarray:
        """Signed encoder ticks/s per wheel -> body velocity ``[vx, vy, omega]``."""
        radps = np.asarray(ticks_per_s, dtype=float) * self.wheel_signs / self.ticks_per_rad
        return self.wheel_radps_to_body(radps)

    def wheel_tick_delta_to_body_delta(self, tick_deltas) -> np.ndarray:
        """Per-wheel tick increments -> body-frame increment ``[dx, dy, dtheta]``.

        Valid because the map from wheel speeds to body twist is linear, so it
        applies unchanged to increments accumulated over one sampling interval
        (assuming the twist is constant over that interval, which is the usual
        dead-reckoning approximation).
        """
        wheel_rad = np.asarray(tick_deltas, dtype=float) * self.wheel_signs / self.ticks_per_rad
        wheel_linear = wheel_rad * self.wheel_radius
        return self.m_inv @ wheel_linear

    # -- integer command path (only needed when this process drives the base) - #

    def body_to_wheel_raw(
        self, vx: float, vy: float, omega: float, max_raw: int = 3000
    ) -> np.ndarray:
        """Body velocity -> integer ``Goal_Velocity`` per wheel.

        Matches upstream behaviour: if any wheel exceeds ``max_raw`` ticks/s all
        three are scaled down proportionally so the motion direction is kept.
        """
        ticks = self.body_to_wheel_ticks_per_s(vx, vy, omega)
        peak = float(np.max(np.abs(ticks))) if ticks.size else 0.0
        if peak > max_raw:
            ticks = ticks * (max_raw / peak)
        return np.round(ticks).astype(int)

    def max_body_velocity(self, max_raw: int = 3000) -> dict[str, float]:
        """What the base can actually do, given a per-wheel ``max_raw`` ticks/s.

        Reported per axis because a kiwi drive is not isotropic: with the
        standard [150, -90, 30] degree contact layout, two wheels share the load
        in pure +x motion but one wheel takes it all in pure +y, so forward is
        about 15% faster than sideways.

        These are *single-axis* maxima.  Combined motion is slower -- asking for
        max vx and max omega at once saturates, and ``body_to_wheel_raw`` scales
        the whole command down.  Use them to clamp a planner's output, not as a
        promise about diagonal motion.

        This exists because the numbers are easy to guess wrong by an order of
        magnitude: an earlier ``lekiwi_control.py`` clamped forward speed to
        2.0 m/s, roughly 7.5x what the hardware can do, which is the same as not
        clamping at all.
        """
        # max_raw ticks/s -> wheel rad/s -> contact-point speed in m/s
        wheel_linear = (max_raw / self.ticks_per_rad) * self.wheel_radius
        cos_gain = float(np.max(np.abs(self.m[:, 0])))
        sin_gain = float(np.max(np.abs(self.m[:, 1])))
        return {
            "wheel_linear": wheel_linear,
            "vx": wheel_linear / cos_gain if cos_gain > 0 else float("inf"),
            "vy": wheel_linear / sin_gain if sin_gain > 0 else float("inf"),
            "omega": wheel_linear / self.base_radius,
        }


def wrap_tick_delta(curr: int, prev: int, ticks_per_rev: int = DEFAULT_TICKS_PER_REV) -> int:
    """Shortest signed tick difference across the ``0 <-> ticks_per_rev-1`` seam.

    The STS3215 reports ``Present_Position`` as a single-turn value in
    ``[0, 4095]``, so a continuously spinning wheel wraps roughly once per
    revolution.  Differencing naively gives a -4095 spike at every wrap.

    The standard modular trick maps the difference into
    ``[-ticks_per_rev/2, ticks_per_rev/2)`` -- i.e. it always assumes the wheel
    took the *shorter* path::

        4090 -> 5     gives +11   (not -4085)
        5    -> 4090  gives -11   (not +4085)

    Aliasing limit: motion faster than half a revolution per sample is
    indistinguishable from motion the other way.  At 50 Hz that ceiling is
    ``2048 ticks / 0.02 s = 9000 deg/s``, versus an STS3215 no-load top speed
    around 270 deg/s -- a ~33x margin, so this is safe even at 5 Hz.  It does
    mean that if the sampling loop stalls for longer than ~0.6 s while a wheel
    is at full speed, the delta silently aliases.  The node therefore rejects
    intervals longer than ``max_dt``.
    """
    half = ticks_per_rev // 2
    return (int(curr) - int(prev) + half) % ticks_per_rev - half


def integrate_se2(
    x: float, y: float, theta: float, dx: float, dy: float, dtheta: float
) -> tuple[float, float, float]:
    """Compose an SE(2) pose with a body-frame increment, exactly.

    Uses the SE(2) exponential map rather than the naive
    ``x += dx*cos(theta) - dy*sin(theta)`` Euler step.  For a constant body
    twist over the interval, the true displacement in the starting body frame is
    ``V(dtheta) @ [dx, dy]`` with::

        V(w) = (1/w) * [[ sin(w), -(1 - cos(w))],
                        [ 1 - cos(w),  sin(w)  ]]

    which is the difference between a chord and an arc.  The Euler step
    systematically under-shoots turns; at 50 Hz the per-step error is tiny but it
    accumulates as a consistent bias over a long run, which is exactly the kind
    of drift wheel odometry can least afford.  ``V -> I`` as ``w -> 0``, so the
    straight-line case falls out of the same expression.
    """
    if abs(dtheta) < 1e-9:
        a, b = 1.0, 0.5 * dtheta  # 2nd-order limits of sin(w)/w and (1-cos w)/w
    else:
        a = np.sin(dtheta) / dtheta
        b = (1.0 - np.cos(dtheta)) / dtheta
    local_x = a * dx - b * dy
    local_y = b * dx + a * dy
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    return (
        x + cos_t * local_x - sin_t * local_y,
        y + sin_t * local_x + cos_t * local_y,
        wrap_angle(theta + dtheta),
    )


def wrap_angle(angle: float) -> float:
    """Wrap an angle into ``(-pi, pi]``."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Yaw-only rotation as ``(x, y, z, w)``.

    Done by hand instead of via ``scipy.spatial.transform`` because this module
    must stay import-light on the X5 (``tinynav/core/math_utils.py`` drags in
    numba, cv2 and fufpy).
    """
    return (0.0, 0.0, float(np.sin(0.5 * yaw)), float(np.cos(0.5 * yaw)))


# -- minimal quaternion helpers --------------------------------------------- #
# All use the ROS / scipy ordering ``(x, y, z, w)``.  They exist so that the
# LeKiwi control path does not need ``scipy.spatial.transform``: scipy is ~85 MB
# installed, and the board has ~850 MB of RAM free with no swap.  It is also the
# same reason this module avoids ``math_utils``.


def quaternion_matrix(q) -> np.ndarray:
    """Rotation matrix of a quaternion ``(x, y, z, w)``, normalised first."""
    x, y, z, w = (float(v) for v in q)
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError("zero-length quaternion has no rotation")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def quaternion_rotate_inverse(q, v) -> np.ndarray:
    """Rotate ``v`` by the inverse of ``q``; equals ``R.from_quat(q).inv().apply(v)``."""
    return quaternion_matrix(q).T @ np.asarray(v, dtype=float)


def quaternion_multiply(q1, q2) -> tuple[float, float, float, float]:
    """Hamilton product ``q1 * q2``, both ``(x, y, z, w)``."""
    x1, y1, z1, w1 = (float(v) for v in q1)
    x2, y2, z2, w2 = (float(v) for v in q2)
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_inverse(q) -> tuple[float, float, float, float]:
    """Inverse of a unit quaternion, i.e. its conjugate, normalised."""
    x, y, z, w = (float(v) for v in q)
    n2 = x * x + y * y + z * z + w * w
    if n2 == 0.0:
        raise ValueError("zero-length quaternion has no inverse")
    return (-x / n2, -y / n2, -z / n2, w / n2)


def quaternion_to_rotvec(q) -> np.ndarray:
    """Rotation vector (axis * angle, radians) of ``q``; equals ``as_rotvec()``.

    The sign is canonicalised the way scipy does it -- a quaternion with a
    negative scalar part is negated first, so the result always describes the
    shorter of the two rotations and the angle stays in ``[0, pi]``.  Without
    that, ``q`` and ``-q`` (the same rotation) would give opposite answers.
    """
    x, y, z, w = (float(v) for v in q)
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError("zero-length quaternion has no rotation vector")
    x, y, z, w = x / n, y / n, z / n, w / n
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w

    vec_norm = np.sqrt(x * x + y * y + z * z)
    if vec_norm < 1e-8:
        # angle = 2*atan2(vec_norm, w) -> 2*vec_norm as vec_norm -> 0, and the
        # axis is vec/vec_norm, so the product tends to 2*vec with no division.
        return 2.0 * np.array([x, y, z])
    angle = 2.0 * np.arctan2(vec_norm, w)
    return (angle / vec_norm) * np.array([x, y, z])


def quaternion_relative_rotvec(q_a, q_b) -> np.ndarray:
    """Rotation vector of ``q_a^-1 * q_b``."""
    return quaternion_to_rotvec(quaternion_multiply(quaternion_inverse(q_a), q_b))


# -- base pose -> camera pose ------------------------------------------------ #
# The rotation that takes camera-optical axes into REP-103 base axes:
#
#     camera +x (right)   = base -y      column 0 = ( 0, -1,  0)
#     camera +y (down)    = base -z      column 1 = ( 0,  0, -1)
#     camera +z (forward) = base +x      column 2 = ( 1,  0,  0)
#
# i.e. R_base_camera = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], whose quaternion is
# below.  Written as a literal rather than derived because it is a fixed
# convention constant, and ``test_omni3_kinematics`` checks it against the matrix.
QUAT_BASE_CAMERA = (-0.5, 0.5, -0.5, 0.5)


def base_pose_to_camera_pose(x, y, theta, offset_xyz):
    """Camera-optical pose from a planar base pose.

    Every pose consumer in the navigation stack -- ``planning_node``,
    ``cmd_vel_control``, ``looper_bridge_node`` -- works in the camera's *optical*
    convention (body ``+z`` forward, ``+x`` right, ``+y`` down) expressed in a
    gravity-aligned, ``z``-up ``world`` frame.  That is what the Looper VIO
    publishes on ``/camera/camera/vio_*``, and it is why ``cmd_vel_control``
    computes heading as ``atan2(R[1, 2], R[0, 2])`` (see its ``_control_loop``).

    Wheel odometry natively produces the *base* pose in REP-103 axes (``+x``
    forward, ``+y`` left, ``+z`` up), so handing ``Odometry.pose`` straight to
    those consumers makes them read the forward axis as "up" and the robot turns
    on the spot.  This converts, so that a wheel-odometry-navigated run is a
    topic swap rather than a rewrite.

    ``offset_xyz`` is ``base_link -> camera`` in REP-103 base axes, i.e.
    ``[forward, left, up]`` in metres.  Returns ``(position, quaternion)`` with
    the quaternion as ``(x, y, z, w)``.
    """
    offset = np.asarray(offset_xyz, dtype=float)
    if offset.shape != (3,):
        raise ValueError(f"offset_xyz must have 3 elements, got {offset.shape}")
    cos_t, sin_t = np.cos(float(theta)), np.sin(float(theta))
    position = np.array(
        [
            float(x) + cos_t * offset[0] - sin_t * offset[1],
            float(y) + sin_t * offset[0] + cos_t * offset[1],
            float(offset[2]),
        ]
    )
    quaternion = quaternion_multiply(yaw_to_quaternion(theta), QUAT_BASE_CAMERA)
    return position, quaternion
