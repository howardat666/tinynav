"""Robot geometry shared by the planner and the controller.

Its own module rather than living in ``planning_node`` because
``tinynav/platforms/cmd_vel_control.py`` needs the same numbers, and importing
``planning_node`` to get them would pull in the whole planning stack -- the
njit-compiled ESDF and theta* kernels, and a second ``Node`` subclass -- into a
process that runs on the D-Robotics X5 inside the Looper camera (~850 MB RAM free,
no swap).  Only ``dataclasses`` and ``numpy`` here.

Sharing them is not cosmetic.  Before this module existed the camera offset was
declared twice and disagreed: ``GO2_CONFIG.camera_x = 0.2`` against a literal
``camera_offset = [0.0, 0.0, 0.35]`` in ``cmd_vel_control._control_loop``, so the
planner and the controller placed the robot's control centre 150 mm apart.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class RobotConfig:
    """Robot geometry. Body frame: +x forward, +y left."""
    name: str = 'go2'
    shape: str = 'square'
    length: float = 0.7
    width: float = 0.3
    radius: float = 0.3
    camera_x: float = 0.35
    camera_y: float = 0.0
    control_x: float = 0.0
    control_y: float = 0.0
    safety_radius: float = 0.1
    # Round bases only: the shell's true reach. `radius` is the wheel-ground contact
    # circle, which is smaller. None means "same as radius".
    collision_radius: float | None = None
    # Round bases only: soft-penalty width past the hard limit, so a tight corridor
    # degrades into "prefer the middle" instead of "all forbidden".
    comfort_margin: float = 0.1
    # The z slice, relative to the camera, projected into 2D. Per-robot because GO2's
    # camera sits at z ~ 0 over a floor at -0.35 and LeKiwi's at 0.18 over a floor at 0.
    obstacle_z_bottom: float = -0.4
    obstacle_z_top: float = 0.4
    # front_clearance at or below which only turning in place is admissible. Measured
    # from the hull edge, so LeKiwi gates 0.50 m out from its control centre.
    front_blocked_m: float = 0.3
    # Obstacle inflation in cells. Measured x2.3 on the cell count, and scipy's default
    # cross element makes it directional: 0.100 m on the axes, 0.041 m on the diagonals.
    dilation_cells: int = 1
    # What the base can actually deliver, not what we would like. The planner used
    # to sample up to 0.5 m/s on a chassis that saturates at 0.268, so its
    # predictions ran 2x ahead of reality; see LEKIWI_CONFIG.
    max_vx: float = 0.5
    max_reverse_vx: float = 0.2
    max_yaw: float = 0.8
    # The occupancy grid is written only by forward raycasting, so a reverse move is
    # scored against cells the camera never looked at. Gated on front_blocked too.
    allow_reverse: bool = True

    @property
    def cam_offset_3d(self):
        """Control-center -> camera offset, expressed in *camera optical* axes.

        Consumed as ``center = T[:3, 3] - T[:3, :3] @ cam_offset_3d``, so the
        vector lives in the camera frame, whose axes are ``+x`` right, ``+y``
        down, ``+z`` forward.  ``camera_y`` is a body-frame *left* offset, so it
        enters the right-handed ``x`` slot negated.  Both previously shipped
        configs have ``camera_y = 0``, which is why the missing sign never showed
        up; ``LEKIWI_CONFIG`` has the camera 50 mm off-centre and would otherwise
        pick up a 100 mm lateral bias.
        """
        return np.array(
            [-(self.camera_y - self.control_y), 0.0, self.camera_x - self.control_x],
            dtype=np.float32,
        )

    @property
    def is_circle(self) -> bool:
        return self.shape == 'circle'

    @property
    def hull_radius(self) -> float:
        """Collision hull radius for a round base. Meaningless for a square one."""
        return float(self.radius if self.collision_radius is None else self.collision_radius)

    @property
    def hard_clearance(self) -> float:
        """ESDF below which a pose is a collision.

        A round base is one lookup at the centre, so the hull must be in the threshold.
        A square one carries its geometry in the sample offsets and keeps the historical
        "the sample cell is an obstacle cell" test, untouched.
        """
        return self.hull_radius + self.safety_radius if self.is_circle else 1e-3

    @property
    def soft_clearance(self) -> float:
        """ESDF above which a pose costs nothing at all."""
        return (
            self.hull_radius + self.safety_radius + self.comfort_margin
            if self.is_circle
            else self.safety_radius
        )

    @property
    def half_size(self):
        if self.is_circle:
            return (self.radius, self.radius)
        return (self.length / 2.0, self.width / 2.0)

    def footprint_from_control(self):
        """Returns (front_len, rear_len, half_w) relative to control center."""
        hl, hw = self.half_size
        return float(hl - self.control_x), float(hl + self.control_x), float(hw)

    def probe_geometry(self):
        """(front_len, half_w) for the forward clearance probe. A round base probes from
        its hull; footprint_from_control would give `radius` and understate the reach."""
        if self.is_circle:
            r = self.hull_radius
            return r, r
        fl, _, hw = self.footprint_from_control()
        return fl, hw

    def describe(self) -> str:
        size = (
            f"r={self.radius}m hull={self.hull_radius}m"
            if self.is_circle
            else f"{self.length}x{self.width}m"
        )
        return (
            f"{self.name} ({self.shape} {size}, "
            f"cam=({self.camera_x},{self.camera_y}), "
            f"ctrl=({self.control_x},{self.control_y}), "
            f"safety_r={self.safety_radius}m, "
            f"hard/soft={self.hard_clearance:.2f}/{self.soft_clearance:.2f}m, "
            f"z_band=[{self.obstacle_z_bottom:+.2f},{self.obstacle_z_top:+.2f}], "
            f"dilation={self.dilation_cells}, reverse={self.allow_reverse})"
        )


GO2_CONFIG = RobotConfig(
    name='go2', shape='square',
    length=0.4, width=0.3,
    camera_x=0.2, camera_y=0.0,
    control_x=0.0, control_y=0.0,
    safety_radius=0.2,
)

B2_CONFIG = RobotConfig(
    name='b2', shape='square',
    length=1.0, width=0.5,
    camera_x=0.5, camera_y=0.0,
    control_x=-0.5, control_y=0.0,
    safety_radius=0.1,
)

# LeKiwi three-wheel omni base carrying a Looper. Round footprint because it can
# rotate in place with no swept rectangle; radius is the chassis plate, a little
# outside the 127 mm wheel-centre base_radius the odometry calibration measured.
# camera_x/camera_y are the mount measured on the robot: 60 mm ahead of the base
# centre, 50 mm to its left. Note the distance from GO2's camera_x=0.2 -- running
# LeKiwi on the GO2 config puts the control centre 140 mm behind where it really
# is, on a robot 300 mm across, and 290 mm behind it if the controller's old 0.35
# literal is what you compare against.
LEKIWI_CONFIG = RobotConfig(
    name='lekiwi', shape='circle',
    radius=0.15,
    # Plate and Looper mount reach 0.20; see docs/x5/nav_obstacle_tuning.md section 3.
    collision_radius=0.20,
    comfort_margin=0.1,
    # Top matters, bottom does not. Capping at 0.38 m of world height halved the
    # obstacle count on a real floor (40.2 -> 17.4 cells) by dropping table tops the
    # 0.25 m robot drives under. Below the floor the layers measured empty.
    obstacle_z_bottom=-0.2,
    obstacle_z_top=0.2,
    dilation_cells=0,
    camera_x=0.06, camera_y=0.05,
    control_x=0.0, control_y=0.0,
    safety_radius=0.1,
    # Omni3Kinematics.max_body_velocity(3000) gives vx = 0.268 m/s, and 3000 ticks/s
    # is already the STS3215's no-load speed at 12 V. 0.22 leaves headroom so a yaw
    # component does not saturate body_to_wheel_raw and silently scale vx down.
    max_vx=0.22,
    # Only reachable through the front_blocked gate, so it is a way out of a dead end,
    # not a way to reach a POI behind the robot -- that case is the turn-in-place escape's.
    allow_reverse=True,
)

# ESP32-S3 + TB6612 differential-drive car carrying a Looper. Measured 2026-08-18:
# bounding box 240x120x210 mm, wheel diameter 70 mm, wheel separation 110 mm,
# camera optical centre 183 mm off the floor and laterally centred.
#
# Body origin is the bounding-box centre, because that is the frame the two measured
# offsets share: the camera sits 100 mm ahead of it and the drive axle 70 mm behind.
# control_x is the drive axle, not the box centre -- on a differential drive that is
# the only point with no lateral velocity, so it is what the trajectory rollout has
# to be about. footprint_from_control() then gives front 195 mm / rear 55 mm, i.e. the
# hull reaches 3.8x further ahead of the control centre than behind it.
#
# Rectangular rather than circular: a 240x120 box inscribed in a circle about the
# drive axle would need r = 190 mm, more than three times the true 60 mm half-width,
# and would refuse every real doorway.
#
# Rebuilt 2026-08-21 as front-wheel drive: the drive axle moved from 70 mm behind the
# box centre to 40 mm ahead of it, and the caster now trails 110 mm behind. Every
# number below is from that rebuild; the previous set described a 240x120 mm box with
# the axle at the back and is wrong in every field.
#
# The caster trails rather than leads now, which is the stable arrangement going
# forward, but an in-place turn still scrubs it sideways, so measured yaw lags the
# commanded one.
DIFFCAR_CONFIG = RobotConfig(
    name='diffcar', shape='square',
    # Wider than it is long: 350 mm across the body, 280 mm front to back. The 320 mm
    # drive-wheel track sits inside that, so the body is what the planner must clear.
    length=0.28, width=0.35,
    camera_x=0.09, camera_y=0.05,
    # The drive axle, which is what a differential base actually rotates about.
    control_x=0.04, control_y=0.0,
    # A rectangle takes its geometry from the sample offsets, so safety_radius is only
    # the soft-cost margin here (hard_clearance is 1e-3). LeKiwi uses 0.1 on a 300 mm
    # base; 0.1 keeps the same relative slack on a 350 mm one.
    safety_radius=0.1,
    # Camera is level, optical centre 0.18 m up. Widened downward rather than tightened
    # to the hull: the grid is 0.1 m per voxel and build_obstacle_map's span test needs
    # spare z layers to distinguish a wall from floor noise, so a narrow band turns the
    # floor into a wall (see docs/x5/diffcar.md).
    obstacle_z_bottom=-0.3,
    obstacle_z_top=0.2,
    dilation_cells=0,
    # Deliberately below what the base can do, not a measured ceiling: the firmware
    # accepts up to 0.8 m/s. Capped here so the planner's predictions stay inside what
    # the chassis delivers even loaded, which is the failure LeKiwi shipped once.
    max_vx=0.5,
    max_reverse_vx=0.2,
    # An in-place turn at this rate needs 0.128 m/s at each wheel on the 320 mm track,
    # against 0.044 on the old 110 mm one -- the wider base costs wheel speed for the
    # same yaw, so this is no longer nearly free.
    max_yaw=0.8,
    allow_reverse=True,
    front_blocked_m=0.2,
)

ROBOT_CONFIGS = {
    cfg.name: cfg
    for cfg in (GO2_CONFIG, B2_CONFIG, LEKIWI_CONFIG, DIFFCAR_CONFIG)
}


def robot_config(name: str) -> RobotConfig:
    """Look a config up by name, with the known names in the error message."""
    try:
        return ROBOT_CONFIGS[name]
    except KeyError:
        raise ValueError(
            f"unknown robot_type '{name}'; known: {sorted(ROBOT_CONFIGS)}"
        ) from None
