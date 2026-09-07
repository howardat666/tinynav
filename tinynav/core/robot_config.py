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

import os
from dataclasses import dataclass, field, replace

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
    # 🔴 只给【报表和仿真判据】用，规划器一律用 collision_radius。
    # 驱动轴不过圆心时这两者不是一回事：collision_radius 是绕控制点的【扫掠圆】（必然比
    # 车身大），而真正撞没撞要按【物理外壳】判。少了这两个字段，仿真会拿扫掠圆去判碰撞，
    # 除正前方外处处高报，「变胖之后压进障碍」就是这么来的假象。
    body_radius: float | None = None      # 物理外壳半径，None = 用 collision_radius
    body_offset_x: float = 0.0            # 外壳圆心在控制点【前】多少米
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
            # span/dilation 【不在这里打】：它们会被 TINYNAV_MIN_WALL_SPAN_M /
            # TINYNAV_DILATION_CELLS 覆盖，这行打的是覆盖【前】的值。2026-09-01 板上生效
            # 0.2 而这行印 0.1，我据此以为配置已生效，白查了一轮。生效值看 planning_node
            # 启动时的 "obstacle: raycast step=... span>=..." 那一行。

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
    # 🔴 圆形下 hard_clearance = collision_radius + safety_radius = 0.172，直接是硬门限
    # （方形时是 1e-3，safety_radius 只进软代价）。0.1 会变 0.222，把窄处全封死。
    safety_radius=0.05,
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
    # 🔴 圆形下 hard_clearance = collision_radius + safety_radius = 0.172，直接是硬门限
    # （方形时是 1e-3，safety_radius 只进软代价）。0.1 会变 0.222，把窄处全封死。
    safety_radius=0.05,
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
# 2026-09-01 起是圆盘。上面那条"方形优于圆形"的理由只对旧的 0.28x0.35 车体成立——
# 那时外接圆 r=0.25 远大于半宽 0.175；现在车体本身就是 Ø235 的盘，外接圆就是车体。
DIFFCAR_CONFIG = RobotConfig(
    # 圆盘底盘 Ø235（2026-09-01 换装），驱动轮后置 20 mm（2026-09-04 重装，
    # 见 3D/diff-car/diff_car_rear.scad 的 axle_y=-20）。
    #
    # 🔑 圆形碰撞模型仍然成立，但【圆心是驱动轴、不是盘心】：差速车原地转的瞬心一定在
    # 两轮接地点连线上，轴后移 d 之后车身绕轴扫出的是一个半径 = 盘半径 + d 的圆。把这个
    # 扫掠圆当作碰撞体，一次 ESDF 查表仍然成立，而且「原地转不改变占据区域」这条豁免
    # （_score_trajectories）也继续成立 —— 扫掠圆自转就是它自己。
    name='diffcar', shape='circle',
    length=0.28, width=0.35,   # 旧方车体，圆形下不参与碰撞，仅留作记录
    # radius 仍是盘半径（相对盘心），圆形路径上不参与碰撞，留作记录。
    # collision_radius = 绕【驱动轴】的扫掠半径 = 117.5 + 20 = 137.5 mm。轮子俯视投影的角
    # 在 sqrt(117.5^2+32.5^2)=121.9，被 137.5 包住；scad 的自检保证没有零件出盘沿。
    radius=0.1175, collision_radius=0.1375,
    # 物理外壳：盘 Ø235 的圆心在驱动轴前 20 mm，轮子俯视角伸到离盘心 121.9 mm。
    body_radius=0.122, body_offset_x=0.020,
    # camera_x: 光心在【盘心】前 67 mm。camera_y 是 infra1 相对模组中心的基线偏置，是相机
    # 自身属性，不随底盘变。相对控制点的偏置由 camera_x - control_x 算出（= 0.087）。
    camera_x=0.067, camera_y=0.05,
    # 控制点 = 旋转中心 = 驱动轴，在盘心【后】20 mm。node_manager 的 diffcar_control 启动
    # 参数是 camera_x - control_x 算出来的，所以改这一个数，执行器和规划器同时跟着走。
    control_x=-0.020, control_y=0.0,
    # 🔴 圆形下 hard_clearance = collision_radius + safety_radius = 0.1875，直接是硬门限
    # （方形时是 1e-3，safety_radius 只进软代价）。
    # ⚠️ 代价：最小可通过净宽 2x0.1875 = 0.375 m，而现场最窄 0.40 m —— 余量从 56 mm 掉到
    # 25 mm（每侧 12.5 mm）。这是后置换来的，不是可以再调小的，别动 safety_radius 去凑。
    safety_radius=0.05,
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
    # 0.4，即地面上方 0.58 m。取值卡在两个实物之间：椅子座面在地面 0.45 m（相机上方
    # 0.27 m）要看见，桌面在地面 0.70~0.75 m（相机上方 0.52 m 起）不能看见 —— 桌子车能
    # 从下面过，把它算成障碍会把整条走廊封掉。两边各留约 0.12 m 余量。
    # 1.0 试过，2026-08-25 实测把桌面之类的悬空物全投影成地面障碍：front_clearance 掉到
    # 0.11 m、106 条轨迹 101 条被拒，车原地左右打转 10 s 才脱困。0.2 又低于椅面。
    # 膨胀归零，配套 score_trajectories_by_ESDF 的取样点铺满（2026-08-26）。膨胀每格要多
    # 0.1 m 的过道净宽：仿真扫描下，膨胀 1 格时 0.9 m 过道抖动、1.1 m 才好用；归零之后
    # 0.7 m 一次穿过、零原地转零倒车。膨胀过去存在的唯一理由是盖住五点取样的空隙，
    # 那个洞现在被 4x5 铺满的取样点堵上了，所以这两处必须成对，别只改一个。
    # 2026-09-01 实测（深度图拟合地面平面，内点 91%，残差 RMS 5.4 mm）：光心离地 0.126，
    # 旧车是 0.18。波段相对相机，所以整体上移 0.054 才能保住原来的物理窗口：
    #   -0.20 -> 离地 -0.074   /   +0.40 -> 离地 +0.526（椅面 0.45 在内、桌面 0.70 在外）
    # 下界取 -0.20 而不是 -0.18：0.1m 栅格的层中心在 -0.25/-0.15/...，-0.20 正好落在两个
    # 中心之间的格子边界上，离两侧各 0.05；阈值压在边界上，谁也挤不进挤不出。
    # 🔴 min_wall_span_m 回到 0.20（2026-09-02 实测定案）。我 09-01 把它从 0.2 降到 0.10，
    # 理由是「0.2 m 在 0.1 m 栅格下是 3 层，0.10 m 在 0.05 m 下也是 3 层，抗噪层数不变」——
    # **这条推理是错的**：抗噪要的是米数，不是层数。同样的物理噪声在细栅格里覆盖更多层，
    # 所以「保持 3 层」等于把物理门限砍了一半。
    # 实测（车停在空地上、低障碍层已关）：单帧地面在 5x5 cm 格内的真实高度跨度 max 只有
    # 0.089 m，可量化到 0.05 m 的层上最多占 3 层 -> z_span 报成 0.10 m，正好等于门限。
    # 于是空地被标成障碍：最近「障碍」0.22 m（车体半径 0.122！）、blocked 90/110。
    # 门限提到 0.15 或 0.20，最近障碍立刻跳到 1.99 m（真墙）、blocked 变 False。
    # 取 0.20 而不是 0.15：量化最多多算 2 层，所以要留 2 层余量（0.089 + 2x0.05 ≈ 0.19）。
    # 仿真九个场景 0.10 与 0.20 逐场景结果完全相同（含正对 5 cm 椅腿、0.9 m 窄缝），
    # 也就是真结构的跨度远在门限之上，提门限不丢东西。
    # ⚠️ 顺带：robot_z_bottom=-0.20 在 0.05 m 栅格下对应世界 z = -0.076，而层边界在
    # -0.075 —— 只差 1 mm。地面往下那一层正好卡在波段边缘挤进来，这也是凑出第 3 层的
    # 原因之一。要动 z 波段的话记住这个数（当年这个 -0.20 是按 0.1 m 栅格的层中心选的）。
    # 🔴 2026-09-02 傍晚定案，两个实景 + 四个 z 相位扫出来的：
    #     robot_z_bottom -0.20 -> -0.11（离地 -0.076 -> +0.014 m）
    #     min_wall_span_m 0.20 -> 0.05
    # 思路是**先把地面剔掉，再谈跨度**，而不是靠抬高跨度门限去压地面噪声：
    # 波段起点抬到地面之上，地面就只占 1 层（跨度 0，永远不是障碍），而高出一层的东西
    # 就有 2 层 = 0.05 的跨度。于是门限可以降到 0.05，灵敏度反而上去了。
    # 判据（空地场景必须 0 误报 / 椅子场景要检出，各 250 帧、四个 z 相位）：
    #   离地-0.076 + span 0.05 : 空地 49~74 格误报            椅子 129~138
    #   离地-0.076 + span 0.20 : 空地 0                       椅子 49~58
    #   离地-0.016 + span 0.05 : 空地 0/23/0/0  <- 相位一变就误报，淘汰
    #   离地+0.014 + span 0.05 : 空地 0 0 0 0   椅子 91~119   <- 选它
    # 🔑 相位这一维必须扫：-0.016 那组只差一个相位就废了，单相位看不出来。
    # ⚠️ 旧注释说 -0.20 是「按 0.1 m 栅格的层中心选的、压在格子边界上」——
    # 分辨率降到 0.05 后那条理由已失效，反而让层【边界】压在地面上（见
    # obstacle-phantom-is-grid-z-phase）。
    # 顺带：span=0.05 能看见高出一层（约 5 cm）的东西，这正是原来 low_obs 那条通路
    # 想解决的问题，所以它关掉之后不留缺口。
    # 🔴 2026-09-02 二次定案：−0.11/0.05 → **−0.16/0.10**。理由不是"更少误报"（两者在
    # 空地上都是 4 个 z 相位全 0），而是**对 zbot 的容差宽一倍半**。离线扫 zbot × span：
    #   span=0.05 干净窗口 = zbot −0.09 ~ −0.13（4 cm 宽），−0.11 离边界只有 2 cm
    #   span=0.10 干净窗口 = zbot −0.09 ~ −0.19（10 cm 宽），−0.16 在正中间
    # 相机高度估错、地面不平、换厚地毯都会把地面相对波段推移，4 cm 的窗口撑不住
    # （实测 −0.16/0.05 在一个相位上误报 36 格，而 −0.16/0.10 全 0）。
    # 代价：椅脚「矮」检出 4 相位合计 56 → 63 格（略好），最近矮格 0.53 → 0.53~0.68 m；
    # 顺带总格子数从 149~181 降到 123~141，图更干净（这一趟 obstacle_cells p90 448/max 702）。
    obstacle=ObstacleConfig(robot_z_bottom=-0.16, robot_z_top=0.4, dilation_cells=0,
                            min_wall_span_m=0.10),
    # 0.6, and the reaction-budget argument that held it at 0.3 was mostly wrong: the
    # library samples 7 speeds from 0 to vx_max and collision-checks each over its whole
    # 3 s extent, so raising the ceiling adds fast options rather than forcing speed --
    # near an obstacle the fast ones are refused and a slower or turning one wins. The
    # 3 s check also looks 1.8 m ahead at 0.6, far past the 0.5 m front gate, so the gate
    # is the turn-only mode switch and not the primary guard. What is still true, and
    # narrower: an obstacle that only becomes visible after the car commits gets 0.70 m
    # of reaction distance instead of 0.35, and cmd_vel_control has no brake of its own --
    # a planning stall stops the car only when the trajectory expires.
    # 0.5。前一句"速度越高冲得越远"的担忧本身是对的，但它的解法不是压低上限 —— 轨迹库
    # 采 7 档速度（linspace(0, max_vx, 7)），现在每档按自己的速度要求视距
    # （v x reaction_time_s，见 planning_node._motion_gate_penalty），所以宽敞处才敢用
    # 高档、窄处自动落到低档。配套改的是探针量程 0.5 -> 1.2 m：0.5 时 front_clearance 的
    # p90 就是饱和值 0.50，看不到 0.5 m/s 需要的 0.80 m，也就无从判断敢不敢快。
    # 🔴 2026-09-01 撞击复盘后 0.5 -> 0.25。0.5 不是"担心冲得远"这种感觉问题，是算术不成立：
    # 实测总延迟 = 相机→bridge 0.10~0.21 + bridge→决策 0.20~0.25 + 重规划周期 0.21(p90 0.41)
    # + 执行器 0.19~0.26 = 0.70~1.13 s，乘 0.5 m/s 就是 0.35~0.57 m 的"已承诺、来不及改"
    # 行程，而 hard_clearance 只有 0.172 m。0.25 把它压到 0.18~0.28 m。
    # 配套前提：轨迹库的低档变成 0.042/0.083 m/s，这些档位在静摩擦击穿之前是执行不出来的
    # （实测增益 0.56），加了击穿之后 0.02 m/s 都有 0.88，低档才真正可用。
    max_vx=0.25,
    max_reverse_vx=0.2,
    # 1.05, matching the pi/3 the trajectory library samples: at 0.8 every turn the
    # planner scored was clipped 31% on its way to the wheels. Wheel speed is not the
    # limit either way (1.05 rad/s is 0.168 m/s per wheel on the 320 mm track, against
    # the 0.8 m/s the motors deliver). The real limit is that fast rotation is how VIO
    # loses tracking, and insight_full does not recover from that on its own -- watch
    # `ros2 topic info /camera/camera/vio_image` for Publisher count going to 0.
    # 🔴 1.05 -> 0.6（2026-09-01 实测两条独立理由）：
    # (1) 原始 VIO（不经地图校正）在持续 1.05 rad/s 原地转时**平移飞了 1 m**，而偏航是准的
    #     —— 上面那段注释早就预警"快速旋转是 VIO 丢跟踪的方式"，我们撞上了。
    # (2) max_yaw x 环路延迟(1.04 s) 是"来不及改"的转角：1.05 -> 63°，0.6 -> 36°。
    # 顺带把轨迹库的 omega 档位间隔从 0.150 收到 0.086 rad/s，脱困转向的比例控制更细。
    max_yaw=0.6,
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
    # 0.2：前进被**完全**禁掉的下限。以前 0.3 是"禁掉所有前进"的一刀切门限，而 0.3 比车
    # 一个周期加位姿滞后走的距离（约 0.42 m）还短，所以它拦不住车、只能在车已经楔进去之后
    # 报警。现在分档门限接管了"该多快"，这个值只回答"什么时候彻底别再往前"。
    # 0.2 和最慢一档自洽：0.5/6 = 0.083 m/s x 1.6 s = 0.13 m，取 max 后是 0.2。
    front_blocked_m=0.2,
)

ROBOT_CONFIGS = {
    cfg.name: cfg
    for cfg in (GO2_CONFIG, B2_CONFIG, LEKIWI_CONFIG, DIFFCAR_CONFIG)
}


# 只放「想在板上不改代码就 A/B 的」那几个。全部走这一个入口，规划和控制才拿到同一份数字
# —— 这个模块存在的理由本身就是那次相机偏置声明两遍、两边差 150 mm。
_ENV_OVERRIDES = {
    'TINYNAV_MAX_VX': 'max_vx',
    'TINYNAV_MAX_YAW': 'max_yaw',
}


def robot_config(name: str) -> RobotConfig:
    """Look a config up by name, with the known names in the error message."""
    try:
        cfg = ROBOT_CONFIGS[name]
    except KeyError:
        raise ValueError(
            f"unknown robot_type '{name}'; known: {sorted(ROBOT_CONFIGS)}"
        ) from None
    changes = {field_name: float(os.environ[env])
               for env, field_name in _ENV_OVERRIDES.items() if os.environ.get(env)}
    return replace(cfg, **changes) if changes else cfg
