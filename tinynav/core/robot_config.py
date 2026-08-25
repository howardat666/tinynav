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

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ObstacleConfig:
    """How the occupancy grid becomes a 2D obstacle mask. Consumed by
    planning_node.build_obstacle_map.

    Lives here, nested in RobotConfig, because upstream's robot_specs.py does the same
    (#228) and because x5 previously wired only three of these five through -- so
    min_wall_span_m and occ_threshold could not be set per platform at all, which is
    exactly the pair that matters on a 0.2 m tall car.
    """
    # The z slice, relative to the camera, projected into 2D.
    robot_z_bottom: float = -0.4
    robot_z_top: float = 0.4
    occ_threshold: float = 0.1
    # A cell counts as an obstacle only if occupied voxels span at least this much in z.
    # Upstream replaced a plain height threshold with this test (#45) because the height
    # version could not filter stairs and floor bumps. The cost is a blind spot: at the
    # grid's 0.1 m voxels a span of 0.2 m needs three layers, so nothing shorter than
    # ~0.2 m is an obstacle at all.
    min_wall_span_m: float = 0.2
    # Inflation in cells. x2.3 on the cell count, and scipy's default cross element makes
    # it directional: 0.100 m on the axes, 0.041 m on the diagonals.
    dilation_cells: int = 2


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
    obstacle: ObstacleConfig = field(default_factory=ObstacleConfig)
    # front_clearance at or below which only turning in place is admissible. Measured
    # from the hull edge, so LeKiwi gates 0.50 m out from its control centre.
    front_blocked_m: float = 0.3
    # What the base can actually deliver, not what we would like. The planner used
    # to sample up to 0.5 m/s on a chassis that saturates at 0.268, so its
    # predictions ran 2x ahead of reality; see LEKIWI_CONFIG.
    max_vx: float = 0.5
    max_reverse_vx: float = 0.2
    max_yaw: float = 0.8
    # The actuator's own clamp, which is a different question from what the planner may
    # plan: max_vx is limited by the replan period (see DIFFCAR_CONFIG), while a human on
    # a joystick or the keyboard is the reaction budget and needs no such margin. None
    # means "same as max_vx", so a platform that has not thought about it cannot get a
    # higher limit by accident.
    chassis_max_vx: float | None = None
    chassis_max_yaw: float | None = None

    @property
    def actuator_max_vx(self) -> float:
        return self.max_vx if self.chassis_max_vx is None else self.chassis_max_vx

    @property
    def actuator_max_yaw(self) -> float:
        return self.max_yaw if self.chassis_max_yaw is None else self.chassis_max_yaw
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
        up.  Both ``LEKIWI_CONFIG`` and ``DIFFCAR_CONFIG`` now have the camera 50 mm
        off-centre -- on the diff car because the stack runs on ``infra1``, the
        rectified pair's left camera, not the centred colour lens -- and either
        would otherwise pick up a 100 mm lateral bias.
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
            f"z_band=[{self.obstacle.robot_z_bottom:+.2f},{self.obstacle.robot_z_top:+.2f}], "
            f"dilation={self.obstacle.dilation_cells} span>={self.obstacle.min_wall_span_m}m, "
            # 速度以前不在这一行里，所以日志看不出一次 run 是按什么上限跑的 —— 而这是最常调的参数。
            f"vx<={self.max_vx}/rev{self.max_reverse_vx} yaw<={self.max_yaw} "
            f"(clamp {self.actuator_max_vx}/{self.actuator_max_yaw}), "
            f"reverse={self.allow_reverse})"
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
    obstacle=ObstacleConfig(robot_z_bottom=-0.2, robot_z_top=0.2, dilation_cells=0),
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

# ESP32-S3 + TB6612 differential-drive car carrying a Looper. Re-measured 2026-08-21
# after the rebuild to front-wheel drive; every number the first version carried (a
# 240x120 mm box, a 110 mm track, an axle behind the centre) is wrong now, not stale.
#
# Body origin is the bounding-box centre, the frame the measured offsets share. control_x
# is the drive axle, not the box centre -- on a differential drive that is the only point
# with no lateral velocity, so it is what the rollout has to be about. The caster now
# trails 110 mm behind instead of leading, which is the stable arrangement going forward,
# but an in-place turn still scrubs it sideways so measured yaw lags the commanded one.
#
# Rectangular, not circular: a circle about the axle enclosing a 350 mm wide body needs
# r = 0.25 m against a true half-width of 0.175, and would refuse every real doorway.
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
    #
    # dilation 2, which is what every upstream platform runs (go2, go2w, b2, b2w, g1 all
    # take the default) and what this one wrongly overrode to 0. It is not a nicety here:
    # score_trajectories_by_ESDF samples the footprint at five points, and on a
    # 0.28x0.35 m body over 0.1 m cells that leaves 2.5 unsampled cells between the front
    # corners -- a 0.1 m obstacle can sit in the gap and every sample reads clear.
    # Inflation is how upstream covers that, and turning it off removed the cover without
    # replacing it. hard_clearance is 1e-3 on a square, so there was no other margin.
    # 2 格 = 20 cm，叠加车体半宽后足以把窄通道整条封死：2026-08-25 实测规划器只能选出
    # 原地不动的轨迹，而 cmd_vel_control 的到达判据是「离局部轨迹终点 <0.1 m」，于是
    # 每周期都报 endpoint reached 并发零速。
    obstacle=ObstacleConfig(robot_z_bottom=-0.3, robot_z_top=0.2, dilation_cells=1),
    # 0.6, and the reaction-budget argument that held it at 0.3 was mostly wrong: the
    # library samples 7 speeds from 0 to vx_max and collision-checks each over its whole
    # 3 s extent, so raising the ceiling adds fast options rather than forcing speed --
    # near an obstacle the fast ones are refused and a slower or turning one wins. The
    # 3 s check also looks 1.8 m ahead at 0.6, far past the 0.5 m front gate, so the gate
    # is the turn-only mode switch and not the primary guard. What is still true, and
    # narrower: an obstacle that only becomes visible after the car commits gets 0.70 m
    # of reaction distance instead of 0.35, and cmd_vel_control has no brake of its own --
    # a planning stall stops the car only when the trajectory expires.
    # 2026-08-25 收回 0.3：调到 0.6 时导航还没跑通（车因为「无目标 → 原地路径」根本没走
    # 起来），提速的收益无从验证，而重规划周期一旦跟不上，速度越高冲得越远。等重定位稳住
    # 再谈提速。注意下面那条注释里"导航的 0.3"说的就是这个值。
    max_vx=0.3,
    max_reverse_vx=0.2,
    # 1.05, matching the pi/3 the trajectory library samples: at 0.8 every turn the
    # planner scored was clipped 31% on its way to the wheels. Wheel speed is not the
    # limit either way (1.05 rad/s is 0.168 m/s per wheel on the 320 mm track, against
    # the 0.8 m/s the motors deliver). The real limit is that fast rotation is how VIO
    # loses tracking, and insight_full does not recover from that on its own -- watch
    # `ros2 topic info /camera/camera/vio_image` for Publisher count going to 0.
    max_yaw=1.05,
    # 手动开车（摇杆/键盘）的上限。导航的 0.3 是被重规划周期限住的，手上开车没有那个约束，
    # 固件本身收 0.8 m/s。
    # 0.8 = 固件自己的上限，当硬兜底用。它不是"平时开这么快"，摇杆和键盘各自有更低的默认值；
    # 这一条只保证限幅不会去截断上层真的想发的指令 —— 摇杆 0.5 被 clip 到 0.3 那种事。
    chassis_max_vx=0.8,
    # 1.2 而不是导航的 0.8：轮速不是瓶颈（0.8 rad/s 只要 0.128 m/s 每轮，电机跑 0.8），
    # 卡住导航的是快速旋转会让 VIO 丢跟踪，而丢了之后 insight_full 自恢复失败、要重启固件。
    # 手上开车看得见就停，所以这条约束不适用；导航仍留 0.8 直到量过旋转下的跟踪表现。
    chassis_max_yaw=1.2,
    allow_reverse=True,
    # Default 0.3, not the 0.2 the 240x120 mm car shipped with -- that value was chosen
    # because a 110 mm track "turns in place freely", and it does not any more. Measured
    # from the hull edge, so it guarantees 0.4 m about the axle, against the 0.251 m the
    # rear corner now sweeps in an in-place turn (the old car swept 0.078). At 0.2 the
    # gate would open with 0.049 m of slack, i.e. the pivot gets collision-refused before
    # the gate has bought anything, and the escape hatch runs instead.
    front_blocked_m=0.3,
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
