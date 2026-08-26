import math
import array
from collections import deque
import json
import os
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointField
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from cv_bridge import CvBridge
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation
from dataclasses import dataclass
import message_filters
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool, Header, String
from codetiming import Timer
import cv2
from tinynav.core.math_utils import quat_to_matrix, matrix_to_quat, pose_msg2np
from tinynav.core.planning_kernels import (
    generate_trajectory_library_3d,
    run_raycasting_loopy,
    score_trajectories_by_ESDF,
)
# Re-exported so `from planning_node import GO2_CONFIG` keeps working
# (tool/planning_bag_viser.py does exactly that).
from tinynav.core.robot_config import (
    B2_CONFIG,
    GO2_CONFIG,
    LEKIWI_CONFIG,
    ObstacleConfig,
    ROBOT_CONFIGS,
    RobotConfig,
    robot_config,
)

__all__ = [
    "B2_CONFIG",
    "GO2_CONFIG",
    "LEKIWI_CONFIG",
    "ROBOT_CONFIGS",
    "RobotConfig",
    "robot_config",
]

# codetiming's default logger is the builtin print, and node_manager._launch_proc
# redirects stdout into an unrotated file on the board's eMMC. Eight of these fire per
# planning cycle -- ~40 lines/s at the 5 Hz depth rate -- unconditionally, with no level
# check and no way to turn them off, inside the loop that drives obstacle avoidance.
# logger=None makes codetiming record into Timer.timers without emitting anything, so
# the measurements survive and only the printing goes away.
_TIMER_LOGGER = print if os.environ.get('TINYNAV_VERBOSE_TIMER', '0') == '1' else None

# /planning/occupied_voxels_with_esdf has no subscriber anywhere in the repo -- the only
# reference is docs/vis.rviz. Producing it is not free: a 100x100 meshgrid, an
# applyColorMap over 10000 entries, a structured array and a ~160 KB PointCloud2, every
# cycle, for the whole ground plane regardless of occupancy. Same reasoning and same
# default as --publish-disparity-vis in looper_bridge_node. Its sibling
# /planning/occupied_voxels stays unconditional: node_manager subscribes to that one for
# the web UI.
_PUBLISH_ESDF_CLOUD = os.environ.get('TINYNAV_PUBLISH_ESDF_CLOUD', '0') == '1'

# Interpolate the footprint into an RViz-drawable outline instead of publishing its four
# corners. Off by default: see publish_footprint for the 8.49 ms this costs and why the
# app never sees the difference.
_FOOTPRINT_OUTLINE = os.environ.get('TINYNAV_FOOTPRINT_OUTLINE', '0') == '1'
# The app's local-view layers: obstacle mask, height map (ESDF heatmap) and footprint.
# One switch because they are one feature -- see the publish site for why grid_info ties
# them together. ON by default: the previous default of off left the UI blank, which read
# as "the frontend is broken" rather than "the publishes are disabled".
# This env var is the ceiling, not the switch: /planning/ui_active gates it at runtime,
# so an unattended robot skips the work without anyone having to set anything.
_PUBLISH_PLANNING_OVERLAYS = os.environ.get('TINYNAV_PUBLISH_PLANNING_OVERLAYS', '1') == '1'

# Pose topics whose stamps are byte-identical to the image stamps, so exact-stamp
# synchronisation against /slam/depth works. Kept in the same shape as
# looper_bridge_node._exact_pose_prefixes; the two must agree, because a pose source
# that is approximate for one of them is approximate for the other.
_EXACT_POSE_PREFIXES = ("/camera/camera/vio",)

# === Helper functions ===

def build_obstacle_map(occupancy_grid, origin, resolution, robot_z, config=None):
    """Obstacle = cells where occupied voxels span >= min_wall_span_m in z.
    Walls have large z-span; stair risers / ground bumps have small span."""
    config = config or ObstacleConfig()
    h, w, z_dim = occupancy_grid.shape
    z_world = origin[2] + (np.arange(z_dim) + 0.5) * resolution
    z_rel = z_world - robot_z
    z_mask = (z_rel >= config.robot_z_bottom) & (z_rel <= config.robot_z_top)

    obstacle = np.zeros((h, w), dtype=bool)
    if np.any(z_mask):
        band_occ = occupancy_grid[:, :, z_mask] > config.occ_threshold
        has_occ = np.any(band_occ, axis=2)
        n_z = band_occ.shape[2]
        z_idx = np.arange(n_z, dtype=np.float32)
        occ_high = np.where(band_occ, z_idx[np.newaxis, np.newaxis, :], -1).max(axis=2)
        occ_low = np.where(band_occ, z_idx[np.newaxis, np.newaxis, :], n_z).min(axis=2)
        z_span = (occ_high - occ_low) * resolution
        obstacle = has_occ & (z_span >= config.min_wall_span_m)

    if config.dilation_cells > 0 and np.any(obstacle):
        obstacle = binary_dilation(obstacle, iterations=config.dilation_cells)
    return obstacle


def generate_predefined_trajectory_vocabularies(
    duration=3.0, dt=0.1,
    init_p=np.zeros(3), init_q=np.array([0, 0, 0, 1])
):
    """
    Predefined trajectory vocabularies.
    """
    num_steps = int(duration / dt) + 1
    trajectories = []
    params = []

    # constant reverse trajectory
    # 0.06 m/s x 3 s = 0.18 m。原来是 0.2 m/s = 0.6 m，两个问题：
    # (1) 0.6 m 直退落到"没走过"的地方 —— 车卡住时朝向已偏离到达朝向 40~60°，
    #     0.6 m 的落点离历史轨迹 46~54 cm，_retreat_index 的 20 cm 容差必然拒绝；
    #     0.18 m 在任何朝向下的偏差都只有约 15 cm，能过（2026-08-25 实测几何）。
    # (2) 退 0.10 m 就足以让 esdf_at_robot 从 0.22 越过原地转的扫过半径 0.251 m，
    #     即"退一点点就能转了"，根本不需要 0.6 m。
    # 速度小还有一层：车后方的格子从来没被观测过，0.06 m/s 撞上去的动能是 0.2 m/s 的 1/11。
    reverse_speed = 0.06
    p = init_p.copy()
    q = quat_to_matrix(init_q)
    # omega is zero for this vocabulary, so the orientation never changes: the
    # quaternion and the world velocity are loop invariants. They used to be
    # recomputed on every step to produce the same value 31 times.
    quat = matrix_to_quat(q)
    step = (q @ np.array([0.0, 0.0, -reverse_speed])) * dt
    traj = np.empty((num_steps, 7), dtype=np.float64)
    for i in range(num_steps):
        p += step
        traj[i, :3] = p
        traj[i, 3:] = quat
    traj[:, 2] = traj[0, 2]
    trajectories.append(traj)
    params.append(np.array([-reverse_speed, 0.0], dtype=np.float64))

    return np.asarray(trajectories), np.asarray(params)


def normalize_pose_trajectories(trajectories):
    if trajectories.ndim != 3:
        return np.zeros((0, 0, 7), dtype=np.float64)
    if trajectories.shape[2] >= 7:
        return trajectories[:, :, :7].astype(np.float64)
    return np.zeros((0, 0, 7), dtype=np.float64)


def roll_occupancy_grid(occupancy_grid, old_origin, new_origin, resolution):
    shift_m = new_origin - old_origin
    shift_voxels = np.round(shift_m / resolution).astype(int)
    if np.all(shift_voxels == 0):
        return occupancy_grid, old_origin
    rolled = np.roll(occupancy_grid, shift=tuple(-shift_voxels), axis=(0, 1, 2))
    x, y, z = occupancy_grid.shape
    if shift_voxels[0] > 0:
        rolled[-shift_voxels[0]:, :, :] = 0
    elif shift_voxels[0] < 0:
        rolled[:-shift_voxels[0], :, :] = 0
    if shift_voxels[1] > 0:
        rolled[:, -shift_voxels[1]:, :] = 0
    elif shift_voxels[1] < 0:
        rolled[:, :-shift_voxels[1], :] = 0
    if shift_voxels[2] > 0:
        rolled[:, :, -shift_voxels[2]:] = 0
    elif shift_voxels[2] < 0:
        rolled[:, :, :-shift_voxels[2]] = 0
    updated_origin = old_origin + shift_voxels * resolution
    return rolled, updated_origin


# === PlanningNode class ===
class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        self.declare_parameter('robot_type', 'go2')
        self.robot = robot_config(str(self.get_parameter('robot_type').value))
        self.get_logger().info(f"Robot: {self.robot.describe()}")
        self.bridge = CvBridge()
        self.path_pub = self.create_publisher(Path, '/planning/trajectory_path', 10)
        # Throttled JSON for the app's diagnostics panel; see _publish_diagnostics.
        self.diag_pub = self.create_publisher(String, '/planning/diagnostics', 1)
        self.height_map_pub = self.create_publisher(Image, "/planning/height_map", 10)
        self.obstacle_mask_pub = self.create_publisher(OccupancyGrid, '/planning/obstacle_mask', 10)
        self.footprint_pub = self.create_publisher(PointCloud, '/planning/footprint', 10)
        self.occupancy_cloud_pub = self.create_publisher(PointCloud2, '/planning/occupied_voxels', 10)
        self.occupancy_cloud_esdf_pub = self.create_publisher(PointCloud2, '/planning/occupied_voxels_with_esdf', 10)
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, '/planning/occupancy_grid', 10)
        self.depth_sub = message_filters.Subscriber(self, Image, '/slam/depth')
        # Where this node takes the robot pose from. A parameter rather than a
        # literal for two reasons: /insight/vio_20hz does not exist on current
        # Looper firmware (it is /camera/camera/vio_image, measured 19.99 Hz), and
        # pointing this at a wheel-odometry topic is exactly how a VIO-mapped but
        # odometry-navigated run gets configured, with no code change.
        self.declare_parameter('pose_topic', '/camera/camera/vio_image')
        pose_topic = str(self.get_parameter('pose_topic').value)
        self.get_logger().info(f"planning pose source: {pose_topic}")
        self.pose_sub = message_filters.Subscriber(self, PoseStamped, pose_topic)

        # Exact vs approximate matching, and why it cannot be a constant.
        #
        # The firmware stamps /camera/camera/vio_image with the *image's* timestamp,
        # so depth and that pose carry byte-identical (sec, nanosec) pairs and exact
        # matching works. A wheel-odometry pose has no such relationship:
        # wheel_odometry_node stamps with its own clock minus the measured bus read
        # latency, so its nanoseconds never coincide with an image's.
        #
        # Measured 2026-08-07 on one boot, same code, only the scheme changed:
        # scheme a (/camera/camera/vio_image) ran 746 planning loops, scheme b
        # (/wheel/camera_pose) ran ZERO -- and reported nothing at all. Both streams
        # published normally the whole time (pose 22.7 Hz, /slam/depth 3.3 Hz) and
        # `ros2 topic hz` was green on both, so from outside the node looked healthy
        # while it silently produced no trajectory. That is the same silent-failure
        # shape looper_bridge_node already guards against with the same rule; this
        # node was the one place that still hard-coded TimeSynchronizer.
        self.declare_parameter('pose_sync', 'auto')
        # Sized from the pose rate actually observed rather than the configured one:
        # the LeKiwi's servo bus loses reads, so /wheel/camera_pose arrives in bursts
        # well below its nominal 50 Hz. Same default as the bridge's --pose-sync-slop.
        self.declare_parameter('pose_sync_slop', 0.06)
        pose_sync_mode = str(self.get_parameter('pose_sync').value)
        pose_sync_slop = float(self.get_parameter('pose_sync_slop').value)
        if pose_sync_mode == 'exact':
            use_exact_pose_sync = True
        elif pose_sync_mode == 'approx':
            use_exact_pose_sync = False
        else:
            use_exact_pose_sync = pose_topic.startswith(_EXACT_POSE_PREFIXES)

        # Deep on purpose, and NOT a latency setting. This is an exact-stamp
        # TimeSynchronizer over two streams at different rates -- /slam/depth at ~5 Hz
        # against a ~20 Hz pose -- and message_filters applies one queue_size to both.
        # Slots therefore buy different amounts of *time* per topic: 30 slots is 6 s of
        # depth but only 1.5 s of pose, and the pose side is what has to still be in the
        # buffer when its depth twin finally arrives after the bridge's ~1.33 MB decode
        # and DDS hop.
        #
        # Measured, because I got this wrong once: cutting this to 3 (0.15 s of pose
        # history) to "fix latency" collapsed the publish interval from p50 1.17 s /
        # max 2.68 s to p50 3.2 s / max 215.6 s, and since a trajectory only lives 3.0 s
        # it then expired before its successor arrived -- `trajectory expired` in
        # cmd_vel_control went from 1 occurrence to 342. The robot stopped moving.
        #
        # Depth and freshness are separate concerns and the fix for each is separate.
        # The queue's job is to not miss matches; max_input_age_s below is what refuses
        # stale data, and it does it by measuring age rather than by hoping a shallow
        # queue implies freshness. The age check is also cheap and runs before any work,
        # so a backlog is walked through quickly instead of being planned on.
        if use_exact_pose_sync:
            self.ts = message_filters.TimeSynchronizer(
                [self.depth_sub, self.pose_sub], queue_size=30
            )
        else:
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.depth_sub, self.pose_sub], queue_size=30, slop=pose_sync_slop
            )
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info(
            "planning pose sync: "
            + ("exact" if use_exact_pose_sync else f"approximate slop={pose_sync_slop}s")
        )
        # Planning on old geometry is worse than not planning: the robot reacts to
        # obstacles that have moved and misses ones that have not. map_node guards its
        # keyframes the same way (max_keyframe_age_s); planning had no age check at all.
        self.max_input_age_s = 0.5
        self._last_stale_log_ns = 0
        self.camerainfo_sub = self.create_subscription(CameraInfo, '/camera/camera/infra2/camera_info', self.info_callback, 10)

        # z 是 9 层而不是对称的 10：栅格以相机为中心时只覆盖相机 ±0.5 m，robot_z_top 再大
        # 也会被栅格自己截断。9 层 + 上抬后覆盖相机 -0.3 .. +0.6 m，够放 robot_z_top 到 0.5。
        # 曾经开到 13 层（上限 1.0 m），结果桌面之类的悬空物全投影成地面障碍，走廊被关死：
        # front_clearance 掉到 0.11 m、106 条轨迹里 101 条被拒，车只能原地打转（2026-08-25）。
        self.grid_shape = (100, 100, 9)
        self.resolution = 0.1
        # 地面在相机下方 0.18 m，往下留 1 层余量够了；省下的层全给上方。
        self.grid_offset = np.array([0.0, 0.0, 0.15])
        self.origin = np.array(self.grid_shape) * self.resolution / -2. + self.grid_offset
        self.step = 10
        self.occupancy_grid = np.zeros(self.grid_shape)
        # 每个 2D 格子最后一次"是障碍"的时刻。占据栅格没有任何时间衰减 —— 只有射线穿过时
        # 才 -0.05，所以一个走出视野的格子会一直留着。热力图上"障碍物停留很久"就是这个，
        # 而以前没有任何数字能区分"它确实还在"和"它只是没人再看它一眼"。
        self._obstacle_first_seen = np.zeros(self.grid_shape[:2], dtype=np.float64)
        self._obstacle_last_seen = np.zeros(self.grid_shape[:2], dtype=np.float64)
        self.K = None
        self.baseline = None
        self.last_T = None
        self.last_param = (0.0, 0.0) # acc and gyro
        # 脱困转向锁定的方向与起始时刻。见 _pick_escape_turn 里的滞回说明。
        self._escape_turn_sign = None
        self._escape_turn_ns = 0
        self._escape_lock_max_s = 4.0
        # 后退脱困。占据栅格只由前向射线写入，所以车后方的格子从来没被看过 —— 盲退曾经
        # 造成 40 s 的 vx=-0.2 直接撞上去（2026-08-10 19:19），这也是 reverse 一直被门掉的
        # 原因。这里换一个能证明安全的判据：只退到**自己刚刚待过**的位置去。
        self._centre_history = deque()      # (t_ns, x, y)，按路程保留，见下
        # 按**累计路程**保留而不是按时间。2026-08-25 19:12 实测教训：车原地转了 58 s，
        # 8 秒的时间窗里全是卡住那一个点，"开进来的那条路"早被裁掉 —— 后退恰好在最需要
        # 的时候不可用。同时"车不动就不记"，否则卡住时重复点会把有用的历史挤出去。
        self._retreat_history_m = 2.0
        self._retreat_history_max_s = 60.0   # 太久的不能用：这段时间里后面可能有人走过
        self._retreat_min_step_m = 0.02
        self._retreat_match_m = 0.20        # 采样点离历史轨迹这么近才算"待过"。车体半宽 0.175，
                                            # 即后退路径不得偏离车身实际压过的范围一个身位以上
        self._retreat_max_s = 4.0           # 上限。0.06 m/s x 4 s = 0.24 m，够让原地转重新可行
        self._retreat_start_ns = 0
        # 脱困持续这么久还没脱出去就别再摆了，直接退。2026-08-25 19:12 实测：58 s 的
        # turn-only 段里 escape_clear 在 >0.50m 和 0.1x m 之间来回跳，于是"净空够就转"
        # 这一条让它一直选转向，yaw 在 -59° 到 -131° 之间摆了近一分钟也没出来。
        self._escape_before_retreat_s = 3.0
        self._escape_episode_ns = 0
        # The robot's own, whole. Rebuilding it field by field wired three of five
        # through, so min_wall_span_m and occ_threshold silently kept the class default
        # no matter what a platform asked for.
        self.obstacle_config = self.robot.obstacle
        self.stamp = None
        self.current_pose = None  # Store the latest pose from odometry

        self.smoothed_velocity = 0.0
        self.dt = 0.1
        self._last_static_log_ns = {}
        self._last_target_rx_ns = 0
        self._last_cycle_ns = 0
        self._last_diag_ns = 0
        # How far the forward probe looks. Past this it reports the
        # sentinel max+1.0, which is not a distance -- see _clearance_along.
        # 0.5 -> 1.2 m。0.5 时 front_clearance 的 p90 就是 0.50，是饱和值 —— 看不到更远，
        # 也就无法支撑一个随速度变大的门限。1.2 m 覆盖 max_vx=0.5 需要的 0.80 m。
        self.front_probe_max_m = 1.2
        # 从"承诺一条轨迹"到"能改指令"之间车会走 v x reaction_time_s。
        # 这个值由闭环仿真定，不是由"周期 + 滞后"直接推 —— tool/sim_front_gate.py 用真的
        # run_raycasting_loopy + build_obstacle_map 让相机以 0.5 m/s 撞向一堵墙，扫出来：
        #   reaction<=1.0 撞上 / 1.2 剩 0.03 m / 1.6 剩 0.09 m / 2.0 剩 0.11 m
        # 而把控制滞后调到 0.8 s（板子排队时的 p90~max）后，1.6 只剩 0.03 m、2.0 仍有 0.09 m。
        # 取 2.0：代价是全速要 1.0 m 净空（探针 1.2 m 够），换来负载高时余量不塌。
        # ⚠️ 别用"实测周期 0.30 + 滞后 0.37 = 0.67"去设它 —— 仿真里 0.67 直接撞上。
        self.reaction_time_s = 2.0
        self._last_loop_ns = 0
        self._loop_period_s = None
        # How long /control/target_pose must go quiet before proximity to it counts as
        # arrival rather than as passing over a waypoint. map_node republishes roughly
        # once a second while it is navigating and stops entirely once the POI list is
        # done, so anything comfortably above its period separates the two.
        # Rotate-first escape hatch: how much closer the best trajectory must get
        # than standing still to count as progress, and how far from the target it
        # must be before turning is preferred over closing the last few centimetres.
        self.min_progress_m = 0.05
        self.rotate_first_min_dist_m = 0.5
        # The primary deadlock detector, taken from main's force_turn (#152). A gain
        # threshold was the wrong instrument: it measures the symptom, and a run with
        # the target 172 deg behind produced best_gain = 0.05 m against a 0.05 m
        # threshold, so the escape missed by an epsilon and the robot stood still for
        # 69 s. Heading error measures the cause and has no scale to get wrong.
        self.force_turn_heading_rad = math.radians(80.0)
        # A heading counts as an escape only if the probe finds nothing within this
        # much. Above robot.front_blocked_m so the turn actually releases the gate
        # instead of handing back a heading that re-triggers it next cycle.
        self.escape_min_clearance_m = max(0.4, self.robot.front_blocked_m + 0.1)

        # TRANSIENT_LOCAL because this topic carries state -- where the robot is going --
        # not a stream. map_node publishes it once per POI transition, so a volatile
        # subscriber that joins late never learns the target and this node publishes
        # "No target pose, publishing static path" forever: trajectory drawn, cmd_vel
        # zero, nothing logged as wrong. That is exactly how the POI target was lost on
        # /mapping/cmd_pois, and this node is a late joiner every time
        # cmd_restart_nav_nodes restarts it while map_node keeps running.
        # Both ends must be TRANSIENT_LOCAL: a latched writer and a volatile reader still
        # connect, but the reader gets no history.
        self.create_subscription(
            Odometry, '/control/target_pose', self.target_pose_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.target_pose = None
        # 0.5 m at the 5 Hz lookahead rate would need 2.5 m/s, well past max_vx, so
        # anything above this is the path moving rather than the robot advancing.
        self.target_jump_warn_m = float(os.environ.get('TINYNAV_TARGET_JUMP_WARN_M', '0.5'))

        self.poi_change_sub = self.create_subscription(Odometry, "/mapping/poi_change", self.poi_change_callback, 10)


        # Whether a browser is actually looking at the local view. The overlay layers and
        # the voxel cloud exist only to be drawn, and together they measured 25.9 ms of a
        # 250 ms cycle -- the single largest stage -- so an unattended robot pays a tenth
        # of its planning budget to render for nobody.
        #
        # A latched Bool rather than get_subscription_count(): node_manager subscribes for
        # the whole life of the backend regardless of who is connected, so the count is
        # always 1 and says nothing. Latched so a planning_node restarted mid-session
        # inherits the current answer instead of guessing.
        #
        # Defaults to True, and stays True if node_manager never publishes: the failure
        # mode of guessing wrong is a blank local view, and that reads as a broken
        # frontend rather than as a disabled publish -- the same trap the env var above
        # was moved off of.
        self._ui_active = True
        self.create_subscription(
            Bool, '/planning/ui_active', self._ui_active_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self._traj_warmup_done = threading.Event()
        self._traj_warmup_ms = None
        self._start_trajectory_warmup()

    def _start_trajectory_warmup(self):
        """Compile the trajectory kernels before the first target, not after it.

        generate_trajectory_library_3d and score_trajectories_by_ESDF sit past the
        "No target pose" early return, so on a cold cache the first target starts a
        72 s compile instead of navigation. Same background-thread shape as
        map_node._start_nav_path_search_warmup.
        """

        def _run():
            t0 = time.monotonic()
            try:
                # Must match the real call sites or numba compiles an unused signature.
                init_p = np.zeros(3)
                init_q = np.array([0.0, 0.0, 0.0, 1.0])
                trajectories, params = generate_trajectory_library_3d(
                    init_p=init_p, init_q=init_q, dt=self.dt, vx_max=self.robot.max_vx,
                    omega_max=self.robot.max_yaw
                )
                trajectories = normalize_pose_trajectories(trajectories)
                vocab_trajs, vocab_params = generate_predefined_trajectory_vocabularies(
                    init_p=init_p, init_q=init_q, dt=self.dt
                )
                vocab_trajs = normalize_pose_trajectories(vocab_trajs)
                if len(vocab_trajs) > 0:
                    trajectories = np.concatenate([trajectories, vocab_trajs], axis=0)
                    params = np.concatenate([params, vocab_params], axis=0)
                # 2-D float32 clearance field, as built in sync_callback.
                esdf = np.full(self.grid_shape[:2], 10.0, dtype=np.float32)
                self._score_trajectories(trajectories, esdf)
            except Exception as e:  # noqa: BLE001 - a warmup thread must never die silently
                self.get_logger().error(f"trajectory warmup failed: {e}")
            finally:
                self._traj_warmup_ms = (time.monotonic() - t0) * 1000.0
                self._traj_warmup_done.set()
                self.get_logger().info(
                    f"trajectory kernels ready in {self._traj_warmup_ms:.0f} ms"
                )

        threading.Thread(target=_run, name="trajectory_warmup", daemon=True).start()

    def poi_change_callback(self, msg):
        # This silently discards the target, and it is a prime suspect for the
        # remaining stalls: in one run planning held a target for about 2 s out of a
        # 72 s window and then reported "No target pose" 563 times, with only one
        # genuine arrival. /mapping/poi_change is published by map_node when POIs are
        # cleared and again on every POI advance, and by the backend to cancel -- three
        # senders, no payload distinguishing them, and nothing logged at either end.
        had = self.target_pose is not None
        self.target_pose = None
        if had:
            self.get_logger().info(
                "target cleared by /mapping/poi_change (POI advance, POI clear, or a "
                "cancel from the backend -- the message does not say which)"
            )

    def target_pose_callback(self, msg):
        new_target = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        now_ns = self.get_clock().now().nanoseconds
        # /control/target_pose is a ROLLING LOOKAHEAD, so it is supposed to move -- which
        # is exactly why a jump needs its own line. The `navigating:` log is throttled to
        # 1 Hz, so at 5 Hz four out of five targets are never recorded and a jump has to be
        # inferred by diffing two surviving lines. Report the step and the gap it happened
        # over; map_node's "nav path changed" tells you whether the path moved under it.
        if self.target_pose is not None and self._last_target_rx_ns:
            step = float(np.linalg.norm(new_target - self.target_pose))
            dt = (now_ns - self._last_target_rx_ns) / 1e9
            if step > self.target_jump_warn_m:
                self.get_logger().warning(
                    f"target jumped {step:.2f}m in {dt:.2f}s: "
                    f"[{self.target_pose[0]:.2f},{self.target_pose[1]:.2f},{self.target_pose[2]:.2f}]"
                    f" -> [{new_target[0]:.2f},{new_target[1]:.2f},{new_target[2]:.2f}]"
                )
        self.target_pose = new_target
        # When the target last arrived, which is what separates "standing on a rolling
        # waypoint" from "the run is over". See the arrival test in the planning loop.
        self._last_target_rx_ns = now_ns

    def _ui_active_callback(self, msg: Bool):
        if bool(msg.data) != self._ui_active:
            self._ui_active = bool(msg.data)
            self.get_logger().info(
                f"local-view publishes {'on' if self._ui_active else 'off'} "
                f"({'a client is watching' if self._ui_active else 'nobody is watching'})"
            )

    def info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            # P[0,3] = -fx * baseline
            fx = self.K[0, 0]
            Tx = msg.p[3] # From the right camera's projection matrix
            self.baseline = -Tx / fx
            self.get_logger().info(f"Camera intrinsics and baseline received. Baseline: {self.baseline:.4f}m")
            self.destroy_subscription(self.camerainfo_sub)

    def camera_to_robot_center(self, T):
        """World control-center position derived from camera pose T_cam->world."""
        return T[:3, 3] - T[:3, :3] @ self.robot.cam_offset_3d

    # Reads as a circle at the app's scale, and clear of node_manager._on_footprint's
    # `n >= 84 and n % 21 == 0` rectangle-outline branch.
    _CIRCLE_SEGMENTS = 16

    def publish_footprint(self, T, stamp):
        """Footprint outline as a PointCloud for RViz and the app. Both consumers close
        the polygon themselves, so a round base is just a 16-gon and needs no change."""
        forward = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
        left    = T[:3, :3] @ np.array([1.0, 0.0, 0.0])
        center  = self.camera_to_robot_center(T)
        if self.robot.is_circle:
            r = self.robot.hull_radius
            corners = [
                center + forward * (r * math.cos(a)) + left * (r * math.sin(a))
                for a in np.linspace(0.0, 2.0 * math.pi, self._CIRCLE_SEGMENTS, endpoint=False)
            ]
        else:
            fl, rl, hw = self.robot.footprint_from_control()
            corners = [
                center + forward * fl + left * hw,
                center + forward * fl - left * hw,
                center - forward * rl - left * hw,
                center - forward * rl + left * hw,
            ]
        # Four corners by default, not 84 interpolated points along the edges.
        #
        # The interpolation exists so RViz's PointCloud display draws a rectangle
        # outline, since it has no line primitive. But the only runtime consumer is
        # node_manager._on_footprint, which detects the 21-per-edge pattern and keeps
        # exactly the corners -- it throws 80 of the 84 points away. Each Point32 is a
        # rosidl message construction costing ~100 us, so those discarded points were
        # 8.49 ms of the planning loop, measured, for nothing.
        #
        # node_manager's else-branch already handles a short point list, so publishing
        # corners needs no backend change. The flag is here only so someone debugging
        # with docs/vis.rviz can get the outline back; RViz shows four dots without it.
        if _FOOTPRINT_OUTLINE:
            points = []
            for i in range(len(corners)):
                a, b = corners[i], corners[(i + 1) % len(corners)]
                for k in range(21):
                    t = k / 20
                    p = (1.0 - t) * a + t * b
                    points.append(Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        else:
            points = [Point32(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in corners]
        msg = PointCloud()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.points = points
        self.footprint_pub.publish(msg)

    @staticmethod
    def _is_turn_in_place(param):
        return abs(param[0]) < 1e-6 and abs(param[1]) > 1e-6

    def _score_trajectories(self, trajectories, esdf, params=None):
        """Collision scores, with in-place turns exempted on a round base.

        A circle rotating in place sweeps the area it already occupies, so the verdict is
        about the present, not the motion, and is unactionable. Measured 2026-08-10: it
        killed 12 of 14 turns, and the escape hatch then failed silently for 23 s.
        """
        front_len, rear_len, half_w = self.robot.footprint_from_control()
        scores, occ_points = score_trajectories_by_ESDF(
            trajectories, esdf, self.origin, self.resolution,
            self.robot.hard_clearance, self.robot.soft_clearance,
            front_len, rear_len, half_w, self.robot.is_circle,
        )
        if params is not None and self.robot.is_circle:
            for i in range(len(params)):
                if self._is_turn_in_place(params[i]):
                    scores[i] = 0.0
        return scores, occ_points

    def _front_gate_m(self):
        """前进被完全禁掉的净空阈值。取"最慢一档前进所需视距"和配置下限的较大者。"""
        return max(self.robot.front_blocked_m,
                   (self.robot.max_vx / 6.0) * self.reaction_time_s)

    def _motion_gate_penalty(self, param, front_clearance, front_blocked):
        """1e9 on any motion this sensor cannot vouch for, 0.0 otherwise.

        The occupancy grid is written only by forward raycasting, so a reverse
        trajectory cannot be rejected for collision -- it is scored against cells the
        camera never looked at. Measured 2026-08-10 19:19: 40 s of vx=-0.2 straight
        into an obstacle while up to 100 of the 106 trajectories were collision-free,
        because the old gate made reverse the *only* admissible option whenever the
        front was blocked and the library holds exactly one reverse.

        前进不再一刀切。每条轨迹按**它自己的速度**要求视距：承诺它到能改指令之间车会走
        v x reaction_time_s，那段距离内不能有东西。这才是"宽敞跑快、窄处爬慢"—— 旧的
        二值门限在 0.3 m 处把所有速度一起禁掉，于是要么全速要么完全不许前进。"""
        is_reverse = param[0] < 0.0
        if is_reverse:
            return 0.0 if (front_blocked and self.robot.allow_reverse) else 1e9
        if self._is_turn_in_place(param):
            # Turning in place is the only motion whose swept volume the camera has
            # already observed, so the front probe does not gate it.
            return 0.0
        return 0.0 if front_clearance >= param[0] * self.reaction_time_s else 1e9

    def _record_centre(self, centre, now_ns):
        """车体中心的近期轨迹，供 _retreat_index 判断"后面走过没有"。

        车不动就不记，且按累计路程而不是时间裁剪 —— 2026-08-25 19:12 实测：车原地转了
        58 s，按时间保留 8 s 的窗口里全是卡住那一个点，"开进来的那条路"恰好在最需要它的
        时候被裁掉了。"""
        h = self._centre_history
        x, y = float(centre[0]), float(centre[1])
        if not h or math.hypot(x - h[-1][1], y - h[-1][2]) >= self._retreat_min_step_m:
            h.append((now_ns, x, y))
        while len(h) > 1 and now_ns - h[0][0] > self._retreat_history_max_s * 1e9:
            h.popleft()
        # 点已按 2 cm 去重，2 m 预算下最多约 100 点，每周期重算一次路程可以忽略不计。
        while len(h) > 2:
            pts = list(h)
            total = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                        for a, b in zip(pts, pts[1:]))
            if total <= self._retreat_history_m:
                break
            h.popleft()

    def _retreat_index(self, params, trajectories, now_ns):
        """直线后退轨迹的下标，若此刻后退不可证明安全则 None。

        占据栅格只由前向射线写入，所以车后方的格子从来没被观测过 —— 对后退做碰撞检查是
        拿没看过的格子在打分，这正是 _motion_gate_penalty 里那段历史（40 s 的 vx=-0.2 直接
        撞上去）的由来。这里不检查栅格，而是检查**这条后退路径的每个采样点，车体中心是否
        在最近几秒真的待过那里** —— 待过就说明能走。"""
        if (self._retreat_start_ns
                and (now_ns - self._retreat_start_ns) / 1e9 > self._retreat_max_s):
            return None                     # 退够久了就停手，不能一路倒回起点
        if not self._centre_history:
            return None
        hist = list(self._centre_history)
        for i, param in enumerate(params):
            if not (param[0] < 0.0 and abs(param[1]) < 1e-6):
                continue
            pts = trajectories[i][:, :2]
            # 每 5 个点抽一个：0.1 s 步长下相邻点只差 2 cm，逐点查是白花钱。
            for x, y in pts[::5]:
                if not any(math.hypot(x - hx, y - hy) <= self._retreat_match_m
                           for _, hx, hy in hist):
                    return None
            if not self._retreat_start_ns:
                self._retreat_start_ns = now_ns
            return i
        return None

    def _escape_reason(self, front_blocked, stand_dist, heading_err_abs, best_gain):
        """Why the robot should turn in place instead of following the ranking, or "".

        A method rather than an inline condition so tool/x5_board/replay_escape_decision.py
        exercises this exact test: it used to keep its own copy, which agreed right up
        until the copies diverged and the replay reported an escape the robot did not
        take."""
        if front_blocked:
            return "blocked"
        if stand_dist <= self.rotate_first_min_dist_m:
            return ""
        # Heading before progress. The cost function scores only how close a
        # trajectory's endpoint gets to the target, and an in-place turn ends where it
        # started, so turning can never outscore standing still while the omega
        # continuity term makes it strictly worse. With the target behind, standing
        # still is the global optimum -- not a tuning failure, and not something a
        # threshold on progress can be set around: the run this fixes had best_gain
        # 0.05 m against a 0.05 m threshold and stood still for 69 s.
        if heading_err_abs > self.force_turn_heading_rad:
            return "heading"
        if best_gain < self.min_progress_m:
            return "no-progress"
        return ""

    def _pick_escape_turn(self, center, turns, trajectories, yaw_to_target, obstacle_mask,
                          params, now_ns):
        """Index of the in-place turn to commit to, plus the clearance it buys.

        Lexicographic, not a weighted sum: a heading the camera can vouch for beats a
        better-aimed one it cannot. Falls back to the freest heading when every
        candidate is tight, so this always yields a turn rather than a standstill."""
        end_yaws = np.array([self._yaw_of(trajectories[i][-1, 3:7]) for i in turns])
        clear = np.array([self._clearance_along(center, math.cos(y), math.sin(y), obstacle_mask)
                          for y in end_yaws])
        head_err = np.array([abs(self._wrap(yaw_to_target - y)) for y in end_yaws])
        usable = np.flatnonzero(clear >= self.escape_min_clearance_m)
        cand = usable if len(usable) else np.arange(len(turns))

        # 方向滞回。上面这两条规则在 usable 空/非空之间来回切换，而"最对准目标"和"最空
        # 方向"经常指反方向 —— 2026-08-25 实测车原地左右摆了 10 s，变号时刻和 escape_clear
        # 跨过阈值的时刻逐条对齐。一旦开始转就锁定转向，除到期或同方向已无可选项。
        # 锁有时限：锁死一个永远转不出去的方向会变成一直自转。
        if (self._escape_turn_sign is not None
                and (now_ns - self._escape_turn_ns) / 1e9 < self._escape_lock_max_s):
            same = [int(k) for k in cand
                    if math.copysign(1.0, params[turns[k]][1]) == self._escape_turn_sign]
            if same:
                cand = np.array(same)
        else:
            self._escape_turn_sign = None

        k = int(cand[int(np.argmin(head_err[cand]))]) if len(usable) \
            else int(cand[int(np.argmax(clear[cand]))])
        omega = params[turns[k]][1]
        if omega != 0.0:
            if self._escape_turn_sign is None:
                self._escape_turn_ns = now_ns
            self._escape_turn_sign = math.copysign(1.0, omega)
        return turns[k], float(clear[k])

    @staticmethod
    def _fmt_clearance(d, max_dist=0.5):
        """">0.50m" rather than "1.50m": the probe stops at max_dist, so the sentinel
        max_dist+1.0 is not a distance and reading it as one is misleading."""
        if math.isnan(d):
            return "n/a"
        return f">{max_dist:.2f}m" if d > max_dist else f"{d:.2f}m"

    def _clearance_along(self, center, fx, fy, obstacle_mask, max_dist=0.5):
        """Clearance from the footprint edge to the nearest obstacle along (fx, fy).

        Returns max_dist + 1.0 when the corridor is clear, which is a "nothing found"
        marker rather than a measured distance -- do not print it as one."""
        fl, hw = self.robot.probe_geometry()
        # Project every obstacle cell onto the heading, rather than sampling points along
        # it. Sampling cannot be made gap-free at any density: the sample lattice rotates
        # with the robot while the grid does not, so at an oblique heading a step of one
        # cell skips cells diagonally. Measured 2026-08-12 -- three lines at (-hw, 0, +hw)
        # missed 40 placements inside the hull's own width even head-on, and one line per
        # grid column still missed 4 of 20 in-corridor cells at 55 deg, the worst of them
        # 0.03 m past the hull face. This has no sampling in it, so it cannot have holes.
        #
        # Windowed to what the corridor can reach first. Projecting is O(obstacle cells)
        # where sampling was O(steps x lines), and on the X5 the whole 100x100 mask cost
        # 3.4 ms when every cell was an obstacle against 0.19 ms for the sampled loop --
        # times ~16 calls a cycle. The window is at most ~15x15 cells, so the cost stops
        # depending on how cluttered the rest of the 10x10 m grid is.
        reach = fl + max_dist
        i0 = max(0, int((center[0] - reach - self.origin[0]) / self.resolution))
        j0 = max(0, int((center[1] - reach - self.origin[1]) / self.resolution))
        i1 = min(obstacle_mask.shape[0],
                 int((center[0] + reach - self.origin[0]) / self.resolution) + 2)
        j1 = min(obstacle_mask.shape[1],
                 int((center[1] + reach - self.origin[1]) / self.resolution) + 2)
        if i0 >= i1 or j0 >= j1:
            return max_dist + 1.0
        oi, oj = np.nonzero(obstacle_mask[i0:i1, j0:j1])
        if oi.size == 0:
            return max_dist + 1.0
        dx = self.origin[0] + (oi + i0 + 0.5) * self.resolution - center[0]
        dy = self.origin[1] + (oj + j0 + 0.5) * self.resolution - center[1]
        along = dx * fx + dy * fy
        lateral = dx * -fy + dy * fx
        # Half a cell of slack on the width: `lateral` is measured to the cell's centre,
        # and a cell whose centre sits just outside the corridor still overlaps it. The old
        # code counted such cells too -- its +-hw sample point landed in them -- so this
        # keeps the corridor the same width rather than quietly narrowing it, and leaves
        # front_blocked_m meaning what it was tuned to mean.
        in_corridor = (np.abs(lateral) <= hw + 0.5 * self.resolution) & (along >= fl)
        if not np.any(in_corridor):
            return max_dist + 1.0
        d_from_face = float(np.min(along[in_corridor])) - fl
        return d_from_face if d_from_face <= max_dist else max_dist + 1.0

    def _front_obstacle_dist(self, T, obstacle_mask, max_dist=1.2):
        """Clearance in the corridor the robot is currently facing."""
        center = self.camera_to_robot_center(T)
        fwd = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
        n = (fwd[0] ** 2 + fwd[1] ** 2) ** 0.5
        fx, fy = (fwd[0] / n, fwd[1] / n) if n > 1e-6 else (1.0, 0.0)
        return self._clearance_along(center, fx, fy, obstacle_mask, max_dist)

    def _track_obstacle_age(self, mask):
        """障碍格子的存活时长。判"它还在不在"要看的是这个，不是格子总数。"""
        now = time.time()
        new = mask & (self._obstacle_last_seen == 0.0)
        self._obstacle_first_seen[new] = now
        self._obstacle_last_seen[mask] = now
        # 掉出掩码的格子清零，这样年龄是"连续存在的时长"而不是"第一次见到有多久"
        self._obstacle_first_seen[~mask] = 0.0
        self._obstacle_last_seen[~mask] = 0.0
        if not mask.any():
            return
        age = now - self._obstacle_first_seen[mask]
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_static_log_ns.get("obstacle_age", 0) < 5_000_000_000:
            return
        self._last_static_log_ns["obstacle_age"] = now_ns
        self.get_logger().info(
            f"obstacle age s: cells={int(mask.sum())} "
            f"p50={float(np.median(age)):.1f} p90={float(np.percentile(age, 90)):.1f} "
            f"max={float(age.max()):.1f} "
            f"older_than_5s={int((age > 5.0).sum())} older_than_20s={int((age > 20.0).sum())}"
        )

    def publish_obstacle_mask(self, mask, stamp):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.info.resolution = self.resolution
        msg.info.width = mask.shape[1]
        msg.info.height = mask.shape[0]
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        # 画在机器人所在高度，而不是栅格的几何中心 -- 栅格在 z 上是偏置的。
        msg.info.origin.position.z = (self.origin[2] - self.grid_offset[2]
                                      + self.grid_shape[2] * self.resolution / 2)
        msg.info.origin.orientation.w = 1.0
        # array.array, not .tolist(). OccupancyGrid.data is int8[], and rclpy's fast
        # path for a primitive sequence is an array.array with the matching typecode;
        # .tolist() instead materialises 10000 Python ints (100x100 grid) for rosidl to
        # then convert one at a time. Measured: this publish was 13.70 ms, the single
        # most expensive thing in the planning loop's vis stage -- more than the ESDF
        # heatmap's applyColorMap -- and none of it was arithmetic.
        data = np.where(mask, 100, 0).astype(np.int8).ravel(order="F")
        msg.data = array.array('b', data.tobytes())
        self.obstacle_mask_pub.publish(msg)

    def publish_height_map(self, origin, esdf_map, header):
        height_normalized = np.clip(esdf_map / 2.0 * 255, 0, 255).astype(np.uint8)
        color_image = cv2.applyColorMap(height_normalized, cv2.COLORMAP_JET)
        img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        img_msg.header = header
        self.height_map_pub.publish(img_msg)

    def publish_2d_occupancy_grid(self, ESDF_map, origin, resolution, stamp, z_offset=0.0):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = stamp
        occupancy_grid_msg.header.frame_id = "world"
        occupancy_grid_msg.info.resolution = resolution
        occupancy_grid_msg.info.width = ESDF_map.shape[1]
        occupancy_grid_msg.info.height = ESDF_map.shape[0]
        occupancy_grid_msg.info.origin.position.x = origin[0]
        occupancy_grid_msg.info.origin.position.y = origin[1]
        occupancy_grid_msg.info.origin.position.z = origin[2] + z_offset
        occupancy_grid_msg.info.origin.orientation.w = 1.0
        flat_data = np.where(ESDF_map <= 0.00, 100, np.clip(((1-ESDF_map/0.5) * 120).astype(int), 0, 120)).ravel(order="F").tolist()
        occupancy_grid_msg.data = flat_data
        self.occupancy_grid_pub.publish(occupancy_grid_msg)

    def publish_3d_occupancy_cloud(self, grid3d, resolution=0.1, origin=(0, 0, 0)):
        occupied = np.argwhere(grid3d > 0.1)
        # vectorized operation to avoid for loop
        if len(occupied) == 0:
            points = []
        else:
            origin_np = np.array(origin)
            world_coords = origin_np + occupied * resolution
            points = world_coords.tolist()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"
        pc2_msg = pc2.create_cloud_xyz32(header, points)
        self.occupancy_cloud_pub.publish(pc2_msg)

    def publish_3d_occupancy_cloud_with_esdf(self, grid3d, ESDF_map, resolution=0.1, origin=(0, 0, 0), max_dist=1.0):
        X, Y, Z = grid3d.shape
        # ground
        gx, gy = np.meshgrid(np.arange(X), np.arange(Y), indexing='ij')
        ground = np.stack([gx.ravel(), gy.ravel(), np.zeros_like(gx).ravel()+2], axis=-1)
        coords = ground * resolution + np.asarray(origin)
        # query ESDF
        ix, iy = ground[:, 0].astype(int), ground[:, 1].astype(int)
        valid = (0 <= ix) & (ix < ESDF_map.shape[0]) & (0 <= iy) & (iy < ESDF_map.shape[1])
        dist = np.full(len(ground), max_dist, dtype=np.float32)
        dist[valid] = np.clip(ESDF_map[ix[valid], iy[valid]], 0, max_dist)
        # map color
        v = np.uint8((1 - dist / max_dist) * 255)
        colors = cv2.applyColorMap(v.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
        rgb = (colors[:, 2].astype(np.uint32) << 16) | (colors[:, 1].astype(np.uint32) << 8) | colors[:, 0].astype(np.uint32)
        # build point cloud
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
        points = np.zeros(coords.shape[0], dtype=dtype)
        points['x'], points['y'], points['z'] = coords[:, 0], coords[:, 1], coords[:, 2]
        points['rgb'] = rgb
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="world")
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        self.occupancy_cloud_esdf_pub.publish(pc2.create_cloud(header, fields, points))

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _yaw_of(quat_xyzw):
        """Ground-plane heading of a pose. Body +z is forward; see omni3_kinematics."""
        fwd = quat_to_matrix(np.asarray(quat_xyzw, dtype=np.float64)) @ np.array([0.0, 0.0, 1.0])
        return math.atan2(float(fwd[1]), float(fwd[0]))

    def _publish_diagnostics(self, front_clearance, obstacle_mask, esdf_map, T, stamp):
        """The decision log's numbers, on a topic, for the app's diagnostics panel.

        Throttled to 2 Hz: the panel is read by eye and the payload is small, but this
        runs inside the loop that already drops a fifth of its frames.
        """
        now_ns = self.get_clock().now().nanoseconds
        # Every call, before the throttle: _last_cycle_ns only advances on the decision
        # path, so with no target the panel would show no rate at all -- which is when
        # you most want to know whether the loop is even turning.
        period_s = (now_ns - self._last_loop_ns) / 1e9 if self._last_loop_ns else None
        self._last_loop_ns = now_ns
        if period_s is not None:
            self._loop_period_s = (0.7 * self._loop_period_s + 0.3 * period_s
                                   if self._loop_period_s else period_s)
        if now_ns - self._last_diag_ns < 500_000_000:
            return
        self._last_diag_ns = now_ns
        centre = self.camera_to_robot_center(T)
        esdf_at_robot = self._esdf_at(esdf_map, centre)
        msg = String()
        msg.data = json.dumps({
            # None rather than the sentinel. _clearance_along returns max+1.0 when it
            # finds nothing, which is finite, so an isfinite() guard silently reports
            # "1.50 m" for "nothing within 0.5 m" -- the exact misreading its docstring
            # warns about, and the one this panel shipped with.
            'frontClearanceM': (None if front_clearance > self.front_probe_max_m
                                else round(float(front_clearance), 2)),
            'frontProbeMaxM': round(float(self.front_probe_max_m), 2),
            'frontBlocked': bool(front_clearance <= self._front_gate_m()),
            'frontBlockedAtM': round(float(self._front_gate_m()), 2),
            'obstacleCells': int(np.count_nonzero(obstacle_mask)),
            'esdfAtRobotM': (None if not np.isfinite(esdf_at_robot)
                             else round(float(esdf_at_robot), 2)),
            'cycleS': (round(self._loop_period_s, 3) if self._loop_period_s else None),
            'stampLagS': round(now_ns / 1e9 - stamp, 2),
        }, separators=(',', ':'))
        self.diag_pub.publish(msg)

    def _esdf_at(self, esdf_map, p):
        """Clearance under a world point, or nan if it is off the local grid."""
        i = int((p[0] - self.origin[0]) / self.resolution)
        j = int((p[1] - self.origin[1]) / self.resolution)
        if 0 <= i < esdf_map.shape[0] and 0 <= j < esdf_map.shape[1]:
            return float(esdf_map[i, j])
        return float('nan')

    def _make_static_path(self, init_p, init_q, header, base_time, num_steps):
        """Build a fresh zero-motion path so controllers receive an explicit stop."""
        path = Path()
        path.header = Header()
        path.header.stamp = header.stamp
        path.header.frame_id = "world"
        num_steps = max(2, int(num_steps))
        static_traj = np.empty((num_steps, 7), dtype=np.float64)
        static_traj[:, :3] = init_p
        static_traj[:, 3:7] = init_q
        for j in range(num_steps):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = (base_time + Duration(seconds=float(j) * self.dt)).to_msg()
            pose.header.frame_id = "world"
            pose.pose.position.x = float(init_p[0])
            pose.pose.position.y = float(init_p[1])
            pose.pose.position.z = float(init_p[2])
            pose.pose.orientation.x = float(init_q[0])
            pose.pose.orientation.y = float(init_q[1])
            pose.pose.orientation.z = float(init_q[2])
            pose.pose.orientation.w = float(init_q[3])
            path.poses.append(pose)
        return path, static_traj

    def _publish_static_path(self, init_p, init_q, header, base_time, num_steps, reason, log_key=None):
        path, _ = self._make_static_path(init_p, init_q, header, base_time, num_steps)
        self.path_pub.publish(path)

        now_ns = self.get_clock().now().nanoseconds
        # 这条分支也算一个走完的周期。不更新的话，下一次真决策的 cycle 会把整段放弃期间
        # 都算进去 —— 实测报出过 14.94 s 的假值，而那 15 s 是连续的 No admissible motion。
        self._last_cycle_ns = now_ns
        key = log_key or reason
        last_ns = self._last_static_log_ns.get(key, 0)
        if now_ns - last_ns >= 1_000_000_000:
            self._last_static_log_ns[key] = now_ns
            self.get_logger().info(f"{reason}, publishing static path.")

    def _depth_meters(self, depth_msg: Image) -> np.ndarray:
        """Metres from /slam/depth, whichever encoding the publisher chose.

        looper_bridge_node forwards the camera's mono16 millimetres unchanged rather
        than doubling the topic to 32FC1; perception_node still publishes 32FC1 in
        RealSense mode. Same branch as planning_bag_viser._decode_depth.
        """
        if depth_msg.encoding in ("mono16", "16UC1"):
            raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            return np.asarray(raw).astype(np.float32) / 1000.0
        return self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

    @Timer(name="Planning Loop", text="\n\n[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER)
    def sync_callback(self, depth_msg, pose_msg):
        if self.K is None:
            return
        age_s = (self.get_clock().now() - Time.from_msg(pose_msg.header.stamp)).nanoseconds / 1e9
        if age_s > self.max_input_age_s:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_stale_log_ns >= 1_000_000_000:
                self._last_stale_log_ns = now_ns
                self.get_logger().warning(
                    f"dropping stale synced set: {age_s:.2f}s old (limit {self.max_input_age_s:.2f}s) -- "
                    "planning is not keeping up with the depth rate"
                )
            return
        with Timer(name='preprocess', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            depth = self._depth_meters(depth_msg)
            stamp = Time.from_msg(pose_msg.header.stamp).nanoseconds / 1e9
            T = pose_msg2np(pose_msg)
            if self.last_T is None:
                self.last_T = T.copy()
                self.smoothed_velocity = 0.0
                self.last_stamp = 0
                self.smoothed_velocity = 0.0
            velocity_estimated = np.linalg.norm(T[:3, 3] - self.last_T[:3, 3]) / (stamp - self.last_stamp)
            self.smoothed_velocity = 0.9 * self.smoothed_velocity + 0.1 * velocity_estimated
            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]

        with Timer(name='raycasting', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            # 减掉 grid_offset 才是「栅格当前对准的机器人位置」。不减的话 z 上永远差
            # 0.35 m，每个周期都判定需要重定心并整体 roll 一次栅格。
            center = (self.origin + np.array(self.grid_shape) * self.resolution / 2
                      - self.grid_offset)
            robot_pos = T[:3, 3]
            delta = robot_pos - center
            if np.linalg.norm(delta) > .1:
                new_center = robot_pos
                new_origin = (new_center - np.array(self.grid_shape) * self.resolution / 2
                              + self.grid_offset)
                self.occupancy_grid, self.origin = roll_occupancy_grid(self.occupancy_grid, self.origin, new_origin, self.resolution)
            new_occ = run_raycasting_loopy(depth, T, self.grid_shape, fx, fy, cx, cy, self.origin, self.step, self.resolution)
            self.occupancy_grid *= 0.99
            self.occupancy_grid += new_occ
            # In place: np.clip without out= allocated a fresh grid every cycle and
            # dropped the old one. On this board that is pure DDR traffic in the loop
            # that is already the latency bottleneck, and the bus loss measurements
            # (docs/x5/servo_bus.md) make memory bandwidth a suspect in its own right.
            np.clip(self.occupancy_grid, -0.2, 0.2, out=self.occupancy_grid)

            # Nothing but the app's 3D local view consumes this, so it follows the same
            # gate as the overlays below rather than get_subscription_count() -- see
            # _ui_active for why the count cannot answer the question.
            if self._ui_active:
                self.publish_3d_occupancy_cloud(self.occupancy_grid, self.resolution, self.origin)

        # 每周期都看，包括没有目标的时候 —— 卡住之后要回溯的正是卡住之前走过的地方。
        self._record_centre(self.camera_to_robot_center(T),
                            self.get_clock().now().nanoseconds)

        with Timer(name='obstacle map', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            obstacle_mask = build_obstacle_map(
                self.occupancy_grid, self.origin, self.resolution,
                robot_z=T[2, 3], config=self.obstacle_config,
            )
            self._track_obstacle_age(obstacle_mask)
            ESDF_map = distance_transform_edt(~obstacle_mask).astype(np.float32) * self.resolution
            # Before the no-target return below, so the UI keeps reading a clearance
            # while the robot is parked -- which is exactly when you want to know
            # whether it thinks something is in front of it.
            front_clearance = self._front_obstacle_dist(
                T, obstacle_mask, self.front_probe_max_m)
            self._publish_diagnostics(front_clearance, obstacle_mask, ESDF_map, T, stamp)

        with Timer(name='vis', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            if _PUBLISH_ESDF_CLOUD:
                self.publish_3d_occupancy_cloud_with_esdf(self.occupancy_grid, ESDF_map, self.resolution, self.origin)
            # These three are the app's local-view layers, and they are one group, not
            # three independent switches: node_manager derives grid_info -- the
            # world-to-canvas transform every other layer is drawn through -- from the
            # obstacle mask's OccupancyGrid metadata. Publish the mask without the height
            # map and the ESDF heatmap is blank; publish neither and grid_info is null, so
            # the trajectory has thousands of points and nowhere to put them.
            #
            # All three had been commented out for cost while node_manager kept
            # subscribing, which made the overlay silently blank and, for the mask
            # specifically, made "All trajectories in collision" unanswerable -- it says
            # the footprint is inside a dilated obstacle cell, not what put it there.
            # On by default because the UI is unusable without them; the escape hatch is
            # for when the board is starved.
            # Timed individually because 'vis' as a whole measured 25.9 ms -- 34% of the
            # planning loop, the largest single stage -- and which of the three that is
            # decides the fix. If one dominates it can be made cheaper on its own; if the
            # cost is spread evenly the only answer is to stop producing them when no UI
            # client is looking, which is a much bigger change.
            if _PUBLISH_PLANNING_OVERLAYS and self._ui_active:
                with Timer(name='vis:mask', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
                    self.publish_obstacle_mask(obstacle_mask, depth_msg.header.stamp)
                with Timer(name='vis:heightmap', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
                    self.publish_height_map(T[:3, 3], ESDF_map, depth_msg.header)
                with Timer(name='vis:footprint', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
                    self.publish_footprint(T, depth_msg.header.stamp)
            # Left off deliberately: nothing subscribes to /planning/project_3d_to_2d.
            #self.publish_2d_occupancy_grid(ESDF_map, self.origin, self.resolution, depth_msg.header.stamp, z_offset=self.grid_shape[2]*self.resolution/2)

        with Timer(name='traj gen', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            # Straight from the wheels, like upstream main. This used to seed from the
            # previous cycle's own trajectory to cover the sensing latency, but that fed
            # each prediction into the next with no feedback: measured 2026-08-12 at a
            # 0.29 m median offset while driving, and unbounded in yaw because the
            # fallback compared positions only, which an in-place turn never changes.
            # cmd_vel_control already indexes the path by wall clock, so the latency is
            # compensated there, downstream, where a stale start is self-correcting.
            init_p = self.camera_to_robot_center(T)
            init_q = np.array([
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ])
            self.last_T = T
            self.last_stamp = stamp
            base_time = Time(seconds=stamp)
            static_steps = max(2, int(round(2.0 / self.dt)) + 1)
            if self.target_pose is None:
                self._publish_static_path(
                    init_p, init_q, depth_msg.header, base_time, static_steps,
                    "No target pose"
                )
                return

            if not self._traj_warmup_done.is_set():
                # Says so explicitly, so a JIT stall is not inferred from a hole in the log.
                self.get_logger().warning(
                    "first target arrived before the trajectory kernels finished "
                    "compiling -- this callback will block until they do"
                )
            target_pose = self.target_pose.copy()
            # Components 0 and 1 are the ground plane and this is correct -- do not
            # "fix" it to [0, 2]. It was changed to [0, 2] on 2026-08-10 and reverted
            # the same day. The reasoning that led there was: this stack carries poses
            # in the camera-optical convention, the trajectory library above puts its
            # forward speed sample into component 2, therefore forward is z therefore
            # the ground plane is (x, z). Every step is true except the conclusion.
            #
            # base_pose_to_camera_pose (omni3_kinematics.py) is the authority, since it
            # is what produces these poses, and it says: body +z forward / +x right /
            # +y down, "expressed in a gravity-aligned, z-up world frame". Only the
            # ORIENTATION is camera-optical. The POSITION is z-up world, and its third
            # component is literally assigned the constant camera height. So (x, y) is
            # the ground plane and component 2 carries no horizontal information.
            #
            # The lesson is about where to look, not about axes: the convention of a
            # pose is fixed by whatever publishes it, and inferring it from a consumer
            # -- even a consumer in this same file -- gets a plausible wrong answer.
            target_dist_xy = float(np.linalg.norm(init_p[:2] - target_pose[:2]))
            # No arrival test here, deliberately. main's planning_node has none: the
            # target is cleared only by /mapping/poi_change, and map_node owns arrival --
            # it holds the POI list, its own 0.5 m radius, and publishes /mapping/nav_done
            # when the list is done. x5 added a "close to the target AND map_node has gone
            # quiet for 2 s" heuristic on the premise that map_node only goes quiet when
            # finished. That premise is false on this board: map_node also goes quiet for
            # 7-40 s whenever keyframes starve. Measured 2026-08-24, one 227 s run declared
            # arrival six times on a rolling lookahead 2 m down a 9.9 m path -- 9 of its 11
            # published targets were intermediate -- and stopped the car for 138 s, 61% of
            # the run.
            #
            # Nothing replaces it because nothing needs to: standing on the lookahead makes
            # target_dist_xy small, the cost function stops rewarding forward motion, and
            # the robot coasts to a halt there until the next target arrives. The POI
            # advance that ends the leg comes from map_node either way.

            # While the robot was actually navigating this node logged nothing at all,
            # so a run
            # that drove badly and a run that never started were indistinguishable in
            # the log. Diagnosing the axis bug above needed init_p, and init_p had to
            # be reconstructed by solving backwards from the reported distance --
            # guesswork standing in for evidence. Both positions go in raw for that
            # reason, and the distance is labelled with the axes it was taken over so
            # the next reader does not have to work out which plane was meant.
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_static_log_ns.get("navigating", 0) >= 1_000_000_000:
                self._last_static_log_ns["navigating"] = now_ns
                self.get_logger().info(
                    f"navigating: ground_dist_xy={target_dist_xy:.3f}m "
                    # One position now: planned-from and measured are the same thing
                    # since the trajectory seed went away.
                    f"robot=[{init_p[0]:.2f},{init_p[1]:.2f},{init_p[2]:.2f}] "
                    f"target=[{target_pose[0]:.2f},{target_pose[1]:.2f},{target_pose[2]:.2f}]"
                )

            trajectories, params = generate_trajectory_library_3d(
                init_p = init_p,
                init_q = init_q,
                dt = self.dt,
                vx_max = self.robot.max_vx,
                omega_max = self.robot.max_yaw,
            )
            trajectories = normalize_pose_trajectories(trajectories)
            vocab_trajs, vocab_params = generate_predefined_trajectory_vocabularies(init_p=init_p, init_q=init_q, dt=self.dt)
            vocab_trajs = normalize_pose_trajectories(vocab_trajs)
            if len(vocab_trajs) > 0:
                trajectories = np.concatenate([trajectories, vocab_trajs], axis=0)
                params = np.concatenate([params, vocab_params], axis=0)

        with Timer(name='traj score', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            scores, occ_points = self._score_trajectories(trajectories, ESDF_map, params)

        with Timer(name='pub', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            # "前方堵住" = 连最慢的那档前进都过不了视距门限。轨迹库把速度采成
            # linspace(0, max_vx, 7)，所以最慢的非零档是 max_vx/6。留一个下限：低于它就
            # 别再试前进了，交给转向/后退。
            front_blocked = front_clearance <= self._front_gate_m()

            def cost_function(traj, param, score, target_pose):
                gate_penalty = self._motion_gate_penalty(
                    param, front_clearance, front_blocked)

                # regular trajectory penalty
                traj_end = np.array(traj[-1,:3])
                target_end = target_pose if target_pose is not None else traj_end
                # xy only. The target carries a camera height ~0.7 m above the
                # trajectory plane, and sqrt(dxy^2 + 0.7^2) compresses the ranking to
                # nothing near the goal: 0.1 m and 0.3 m of real error scored 0.707
                # against 0.762, so continuity outweighed goal-seeking.
                dist = np.linalg.norm(traj_end[:2] - target_end[:2])

                return score * 100000 + 100 * dist + 40 * abs(self.last_param[0] - param[0]) + 10 * abs(self.last_param[1] - param[1]) + gate_penalty

            # path
            path = Path()
            path.header = Header()
            path.header.stamp = depth_msg.header.stamp
            path.header.frame_id = "world"

            if self.target_pose is None:
                self._publish_static_path(
                    init_p, init_q, depth_msg.header, base_time, len(trajectories[0]),
                    "No target pose"
                )
                return

            n_blocked = int(sum(1 for s in scores if s == float('inf')))
            if n_blocked == len(scores):
                self._publish_static_path(
                    init_p, init_q, depth_msg.header, base_time, len(trajectories[0]),
                    f"All {len(scores)} trajectories in collision "
                    f"(front_clearance={self._fmt_clearance(front_clearance, self.front_probe_max_m)}, gate="
                    f"{'turn only' if front_blocked else 'forward only'}, "
                    f"obstacle_cells={int(np.count_nonzero(obstacle_mask))}, "
                    f"esdf_at_robot={self._esdf_at(ESDF_map, init_p):.2f}m, "
                    f"safety_r={self.robot.safety_radius:.2f}m)",
                    log_key="all in collision",
                )
                return

            stand_dist = target_dist_xy
            yaw_now = self._yaw_of(init_q)
            to_t = target_pose[:2] - init_p[:2]
            yaw_to_target = math.atan2(float(to_t[1]), float(to_t[0]))

            top_k = 1
            costs = np.array([cost_function(trajectories[i], params[i], scores[i], self.target_pose)
                              for i in range(len(trajectories))])
            top_indices = np.argsort(costs, kind='stable')[:top_k]
            turned_in_place = False

            # TURN-IN-PLACE ESCAPE HATCH, for the two states where the cost function
            # cannot steer. cost_function scores only the endpoint POSITION, so (a) a
            # target behind the robot makes every forward option score worse than
            # standing still and the continuity term hands the tie to "do nothing"
            # (91 s of vx=0 omega=0, 2026-08-10 18:12), and (b) with the front blocked
            # every turn has the same stationary endpoint, so the tie goes to whatever
            # omega was last commanded -- including zero.
            #
            # It picks among pure turns only. The previous version picked the lowest
            # heading error over the whole admissible set, which under a blocked front
            # was the single hard-coded reverse: 40 s of vx=-0.200 omega=+0.000 into an
            # obstacle, 2026-08-10 19:19.
            admissible = np.flatnonzero(costs < 1e9)
            escape_clear = float('nan')
            best_gain = float('nan')
            turns = [int(i) for i in admissible if self._is_turn_in_place(params[i])]

            # Before trusting argsort. A gated trajectory costs 1e9 and a collided one
            # costs inf, so 1e9 < inf makes argsort hand back the gated one -- the
            # banned reverse -- whenever every ungated option is in collision.
            now_ns = self.get_clock().now().nanoseconds
            retreating = False
            if len(admissible) == 0 or (front_blocked and not turns):
                # 走到这里说明前进被门掉、原地转也全撞（方形底盘的原地转是要做碰撞检查的，
                # 圆形才豁免）。2026-08-25 18:50 实测这个状态连续 15 s，车就站着不动 ——
                # 而它其实是从后面开进来的，退回去必然有路。
                retreat_idx = self._retreat_index(params, trajectories, now_ns)
                if retreat_idx is None:
                    self._publish_static_path(
                        init_p, init_q, depth_msg.header, base_time, len(trajectories[0]),
                        f"No admissible motion: {n_blocked}/{len(scores)} in collision and the rest "
                        f"gated (front_clearance={self._fmt_clearance(front_clearance, self.front_probe_max_m)}, "
                        f"gate={'turn only' if front_blocked else 'forward only'}, "
                        f"turns_left={len(turns)}, "
                        f"esdf_at_robot={self._esdf_at(ESDF_map, init_p):.2f}m, "
                        f"retreat=unavailable) -- holding still "
                        f"rather than moving somewhere the camera has not looked",
                        log_key="no admissible motion",
                    )
                    return
                top_indices = np.array([retreat_idx])
                retreating = True
                escape_reason = "retreat"
                best_gain = float('nan')
                self._escape_turn_sign = None
            else:
                ends_xy = np.array([trajectories[i][-1, :2] for i in admissible])
                best_gain = stand_dist - float(np.min(np.linalg.norm(
                    ends_xy - target_pose[None, :2], axis=1)))
                escape_reason = self._escape_reason(
                    front_blocked, stand_dist,
                    abs(self._wrap(yaw_to_target - yaw_now)), best_gain)

                if escape_reason and turns:
                    if not self._escape_episode_ns:
                        self._escape_episode_ns = now_ns
                    escape_age_s = (now_ns - self._escape_episode_ns) / 1e9
                    pick, escape_clear = self._pick_escape_turn(
                        self.camera_to_robot_center(T), turns, trajectories,
                        yaw_to_target, obstacle_mask, params, now_ns)
                    # 有转向可选 != 转了能出去。2026-08-25 18:50 实测：turns 只剩 1~2 条、
                    # 每条的净空只有 0.04~0.08 m，车在那儿摆了 42 s。净空不够就别摆了，
                    # 退回自己刚走过的地方 —— main 对「前方堵住」的答案本来就是倒车
                    # （硬互斥门 front_clearance<=0.3 时只准倒车），它缺的只是「后面能不能
                    # 走」的凭据，那正是 _retreat_index 提供的。
                    if (escape_clear < self.escape_min_clearance_m
                            or escape_age_s > self._escape_before_retreat_s):
                        retreat_idx = self._retreat_index(params, trajectories, now_ns)
                        if retreat_idx is not None:
                            top_indices = np.array([retreat_idx])
                            retreating = True
                            escape_reason = "retreat"
                            self._escape_turn_sign = None
                    if not retreating:
                        top_indices = np.array([pick])
                        turned_in_place = True
                        self._retreat_start_ns = 0
                else:
                    # 脱困结束就松锁，否则下一次脱困会沿用上一次的方向。
                    self._escape_turn_sign = None
                    self._retreat_start_ns = 0
                    self._escape_episode_ns = 0

            self.last_param = params[top_indices[0]]
            best_idx = int(top_indices[0])
            if now_ns - self._last_static_log_ns.get("decision", 0) >= 1_000_000_000:
                cycle_s = (now_ns - self._last_cycle_ns) / 1e9 if self._last_cycle_ns else float('nan')
                self._last_static_log_ns["decision"] = now_ns
                self.get_logger().info(
                    f"decision: {'RETREAT ' if retreating else ''}"
                    f"{'TURN-IN-PLACE ' if turned_in_place else ''}chose vx={params[best_idx][0]:+.3f} omega={params[best_idx][1]:+.3f} "
                    f"(cap {self.robot.max_vx:.2f}) blocked={n_blocked}/{len(scores)} "
                    f"front_clearance={self._fmt_clearance(front_clearance, self.front_probe_max_m)} "
                    f"gate={'turn-only' if front_blocked else 'forward'} "
                    f"escape={escape_reason or 'off'} turns={len(turns)} "
                    f"escape_age={((now_ns - self._escape_episode_ns) / 1e9) if self._escape_episode_ns else 0.0:.1f}s "
                    f"hist={len(self._centre_history)}pts "
                    f"best_gain={best_gain:+.2f}m escape_clear={self._fmt_clearance(escape_clear)} "
                    # Was only logged on the two give-up branches, so a run could not
                    # be read for whether the z band and dilation did what they claim.
                    f"obstacle_cells={int(np.count_nonzero(obstacle_mask))} "
                    f"esdf_at_robot={self._esdf_at(ESDF_map, init_p):.2f}m "
                    f"yaw={math.degrees(yaw_now):+.0f}deg to_target={math.degrees(yaw_to_target):+.0f}deg "
                    f"heading_err={math.degrees(self._wrap(yaw_to_target - yaw_now)):+.0f}deg "
                    f"cycle={cycle_s:.2f}s stamp_lag={(now_ns / 1e9 - stamp):.2f}s"
                )
            self._last_cycle_ns = now_ns

            for i in top_indices:
                for j in range(0, len(trajectories[i]), 1):
                    x,y,z,qx,qy,qz,qw = trajectories[i][j, :7]
                    pose = PoseStamped()
                    pose.header = Header()
                    pose.header.stamp = (base_time + Duration(seconds=float(j) * self.dt)).to_msg()
                    pose.header.frame_id = "world"
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = z
                    pose.pose.orientation.x = qx
                    pose.pose.orientation.y = qy
                    pose.pose.orientation.z = qz
                    pose.pose.orientation.w = qw
                    path.poses.append(pose)
            self.path_pub.publish(path)
            if now_ns - self._last_static_log_ns.get("traj_pub", 0) >= 1_000_000_000:
                self._last_static_log_ns["traj_pub"] = now_ns
                pts = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z]
                                for q in path.poses], dtype=np.float64)
                gaps = np.linalg.norm(np.diff(pts, axis=0), axis=1) if len(pts) > 1 else np.zeros(1)
                self.get_logger().info(
                    f"traj published: n={len(pts)} "
                    f"first=[{pts[0][0]:.2f},{pts[0][1]:.2f},{pts[0][2]:.2f}] "
                    f"last=[{pts[-1][0]:.2f},{pts[-1][1]:.2f},{pts[-1][2]:.2f}] "
                    f"span={np.linalg.norm(pts[-1][:2] - pts[0][:2]):.2f}m max_gap={gaps.max():.3f}m"
                )

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGTERM, which is how the app's node manager stops this process. rclpy's
        # signal handler invalidates the context before spin() returns, so without
        # this every ordinary stop prints a traceback -- noise that matters when the
        # logs are what you read to compare runs.
        pass

if __name__ == '__main__':
    main()
