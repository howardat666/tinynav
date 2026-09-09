import math
import array
from collections import deque
import json
import os
import dataclasses
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool, Header, String
from codetiming import Timer
import cv2
from tinynav.core.lat_stats import LatStats
from tinynav.core.math_utils import quat_to_matrix, matrix_to_quat, pose_msg2np, rotvec_to_matrix
from tinynav.core.raycast_split import run_raycasting_split
from tinynav.core.planning_kernels import (
    generate_trajectory_library_3d,
    low_obstacle_hits,
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
_PUBLISH_VOXEL_CLOUD = os.environ.get('TINYNAV_PUBLISH_VOXELS', '1') == '1'

# Pose topics whose stamps are byte-identical to the image stamps, so exact-stamp
# synchronisation against /slam/depth works. Kept in the same shape as
# looper_bridge_node._exact_pose_prefixes; the two must agree, because a pose source
# that is approximate for one of them is approximate for the other.
_EXACT_POSE_PREFIXES = ("/camera/camera/vio",)

# === Helper functions ===

# 离地高度的固定配色，和 tool/x5_board/low_obs_debug.py 逐字一致 —— 两边能直接对着看
# 才有意义。固定是重点：前端原来的 3D 体素按每帧 5%~95% 分位自适应，同一个高度每帧
# 颜色都不同。边界之外的第 7 项是无效深度。
_HEIGHT_BINS = np.array([0.02, 0.05, 0.15, 0.25, 0.60], dtype=np.float32)
_HEIGHT_LUT = np.array([          # BGR，给 bgr8
    [105, 96, 90],                # <2cm    地面
    [150, 80, 40],                # 2~5cm   噪声带
    [60, 60, 235],                # 5~15cm  矮障碍
    [40, 140, 245],               # 15~25cm
    [60, 210, 240],               # 25~60cm
    [215, 205, 200],              # >60cm
    [22, 18, 16],                 # 无效
], dtype=np.uint8)

# z 跨度判据的逐像素判决配色。回答的是「图里哪块被判成障碍、哪块没有、以及为什么没有」——
# 离地高度那套配色答不了「为什么没识别」，因为它不看格子的跨度，只看单点的高度。
_VERDICT_LUT = np.array([         # BGR，给 bgr8
    [25, 20, 18],                 # 0 无效深度
    [90, 90, 90],                 # 1 栅格外（超出 4m 框）
    [50, 50, 240],                # 2 🔴 该格【是障碍】
    [60, 220, 250],               # 3 🟡 该格波段内有占据，但 z 跨度不够 —— 关键调试项
    [90, 190, 90],                # 4 🟢 该格波段内没有占据（真空）
    [200, 140, 70],               # 5 🔵 本像素的 z 在波段外（地面 / 太高），本来就不参与
    [230, 60, 230],               # 6 🟣 跨度不够，但该格最高占据层【明显离地】= 有个矮东西被否了
    [70, 70, 40],                 # 7 ⬛ 同上，但那个体素的累加值是【负的】= 栅格主动认定它是空的
], dtype=np.uint8)
# 3 和 6 必须分开：一个格子"有占据但跨度不够"最常见的原因就是【它只有地面】。
# 板上实测 223 个"跨度不够"的格子，混在一起这个数没有意义 —— 真正要找的是 6。
_VERDICT_NAMES = ('无效', '栅格外', '障碍', '只有地面', '栅格不认', '波段外', '矮物被否', '被刻空')
# 逐类薄涂强度。平涂（全 1.0）实测读不出来：判决色是大块纯色、没有任何实物轮廓，
# 对不上现实里哪个东西被判成了什么。要看的两类（障碍 / 矮物被否）涂重，其余压到 0.16~0.35
# 让底图的纹理透出来。
_VERDICT_ALPHA = np.array([0.90, 0.35, 0.72, 0.16, 0.30, 0.16, 0.92, 0.34],
                          dtype=np.float32)


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


def min_visible_height_m(origin_z, robot_z, grid_nz, resolution, camera_height_m, config):
    """离地多高的物体才够 min_wall_span_m 的 z 跨度（地面自己占的层算在内）。

    🔴 必须传【运行时真实的】origin[2]，不能从 grid_offset 反推：初值 origin 里不含
    位姿（= -shape*res/2 + grid_offset），而 roll_occupancy_grid 返回的是
    old_origin + 整数格，所以 origin 永远停在那个初始格上 —— 实机是 -0.100 而不是
    反推出来的 -0.076。差 24 mm，正好差一整档检出高度，我按反推算错过一次。

    模块级函数，和 classify_verdict 一样给 tool/verdict_tuner.py 共用。
    """
    zc = origin_z + (np.arange(grid_nz) + 0.5) * resolution
    h = zc[((zc - robot_z) >= config.robot_z_bottom)
           & ((zc - robot_z) <= config.robot_z_top)] - (robot_z - camera_height_m)
    lo, hi = h - resolution / 2, h + resolution / 2
    for H in np.arange(0.0, 1.0, 0.001):
        occ = np.flatnonzero((lo < H + 1e-9) & (hi > -1e-9))   # 地面(0) 到 H
        if occ.size and (occ[-1] - occ[0]) * resolution >= config.min_wall_span_m - 1e-9:
            return float(H)
    return float('inf')


def classify_verdict(occupancy_grid, origin, resolution, robot_z, camera_height_m,
                     obstacle_mask, d, pw_x, pw_y, pw_z, low_h, config):
    """逐像素判决：这块地方被 z 跨度判据判成了什么、没判成障碍的话卡在哪一步。

    模块级函数，planning_node 和 tool/verdict_tuner.py 共用 —— 两边各写一份的话会
    静默漂移，这一条我已经踩过两次（离线复现和板上给出不同的类别分布）。

    判决按格子(i,j)优先，因为"这块地方是不是障碍"本来就是按格子决定的。
    返回 (verdict uint8 HxW, stats dict)。
    """
    nx, ny, nz = occupancy_grid.shape
    res = resolution
    # 和 build_obstacle_map 同一套算法，重算一遍（80x80x14 的布尔运算，亚毫秒），
    # 免得为了拿中间量去改 build_obstacle_map 的签名。
    zc = origin[2] + (np.arange(nz) + 0.5) * res
    zm = ((zc - robot_z) >= config.robot_z_bottom) & ((zc - robot_z) <= config.robot_z_top)
    band = occupancy_grid[:, :, zm] > config.occ_threshold
    has_occ = band.any(axis=2)
    n = band.shape[2]
    zi = np.arange(n, dtype=np.float32)
    hi = np.where(band, zi[None, None, :], -1).max(axis=2)
    lo = np.where(band, zi[None, None, :], n).min(axis=2)
    span_ok = has_occ & (((hi - lo) * res) >= config.min_wall_span_m)

    i = np.floor((pw_x - origin[0]) / res).astype(np.int32)
    j = np.floor((pw_y - origin[1]) / res).astype(np.int32)
    inside = (d > 0) & (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
    ic = np.clip(i, 0, nx - 1); jc = np.clip(j, 0, ny - 1)

    zrel = pw_z - robot_z
    px_in_band = (zrel >= config.robot_z_bottom) & (zrel <= config.robot_z_top)

    verdict = np.full(d.shape, 1, dtype=np.uint8)          # 1 栅格外
    verdict[d <= 0] = 0                                     # 0 无效
    # 🔴 波段外要先整片涂掉，而且下面每一条按格子的赋值都必须带 judged。以前"波段外"
    # 只在格子没占据时才可能出现，于是桌面/椅背（离地 >0.52 m，本来不参与判断）继承了
    # 它那一列的颜色 —— 板上真数据实测 69% 的波段外像素被涂错，桌子一片黄就是这个。
    judged = inside & px_in_band
    verdict[inside & ~px_in_band] = 5                        # 波段外
    cell_occ = has_occ[ic, jc]
    # 🔴 这一档原来叫"空（真空）"，名字是错的：有像素就说明这一帧真有回波，judged 又
    # 保证回波落在波段内 —— 所以"该列没占据"只能意味着【栅格不认本帧的观测】。图里根本
    # 不存在"确认是空"这一类，因为没东西反光就不会有像素。
    # 再按本像素自己那个体素的累加值分两档：负 = 栅格主动刻空了它（掠射线把地面自己的
    # 回波抹掉；板上真数据实测 1.5~2 m 的地面像素 87% 落这一档）；0~门限 = 只是还没攒够
    # 两帧。前者是系统性的，后者等一帧就好。
    kz = np.clip(np.floor((pw_z - origin[2]) / res).astype(np.int32), 0, nz - 1)
    own = occupancy_grid[ic, jc, kz]
    verdict[judged & ~cell_occ] = 4                           # 栅格不认（还在攒）
    verdict[judged & ~cell_occ & (own < 0.0)] = 7             # 栅格不认（被刻空）
    # 该格最高占据层的离地高度，用来把"只有地面"和"有个矮东西被否了"分开。
    # zc[zm] 是波段内各层的中心（世界 z），hi 是波段内的层【序号】。
    zc_band = zc[zm]
    top_h = np.where(has_occ, np.take(zc_band, np.clip(hi.astype(np.int32), 0,
                                                       len(zc_band) - 1)),
                     -1e3) - (robot_z - camera_height_m)
    low_obj = has_occ & (top_h > low_h)
    verdict[judged & cell_occ] = 3                            # 只有地面
    verdict[judged & low_obj[ic, jc]] = 6                     # 矮物被否
    verdict[judged & obstacle_mask[ic, jc]] = 2               # 障碍（最高优先）

    stats = {
        'obstacle_cells': int(obstacle_mask.sum()),
        'low_rejected_cells': int((low_obj & ~span_ok).sum()),
        'ground_only_cells': int((has_occ & ~span_ok & ~low_obj).sum()),
        'empty_cols': int((~has_occ).sum()),
        'band_layers': int(n),
        'band_z_lo': float(zc[zm][0]) if n else float('nan'),
        'band_z_hi': float(zc[zm][-1]) if n else float('nan'),
    }
    return verdict, stats


def tint_verdict(base_bgr, verdict):
    """判决色薄涂在底图上 + 底部色标条。

    平涂（全 1.0）实测读不出来：判决色是大块纯色、没有任何实物轮廓，对不上现实里
    哪个东西被判成了什么。要看的两类涂重，其余压轻让底图纹理透出来。
    """
    a = _VERDICT_ALPHA[verdict][..., None]
    img = np.clip(base_bgr.astype(np.float32) * (1.0 - a)
                  + _VERDICT_LUT[verdict].astype(np.float32) * a, 0, 255).astype(np.uint8)
    k = len(_VERDICT_LUT)
    w = img.shape[1]
    bar = np.zeros((12, w, 3), dtype=np.uint8)
    for t in range(k):
        bar[:, t * w // k:(t + 1) * w // k] = _VERDICT_LUT[t]
    return np.vstack([img, bar])


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
    # 倒车不再只有直退一条。车被楔住时朝向已经偏离到达朝向 40~60 度(2026-08-25 实测)，
    # 直退等于沿着"歪掉的"朝向退，落点离来路越退越远 —— 而 _retreat_index 要求整条轨迹都
    # 压在走过的地方，于是直退经常是唯一的候选却又必然被拒。人把方形小车从角落里弄出来
    # 靠的正是打着方向倒。3 s x 0.25 rad/s = 43 度、x 0.5 = 86 度，覆盖那个偏离区间。
    R_init = quat_to_matrix(init_q)
    for omega in (0.0, 0.25, -0.25, 0.5, -0.5):
        p = init_p.copy()
        q = R_init
        dq = rotvec_to_matrix(np.array([0.0, omega * dt, 0.0]))
        v_body = np.array([0.0, 0.0, -reverse_speed])
        traj = np.empty((num_steps, 7), dtype=np.float64)
        for i in range(num_steps):
            q = q @ dq
            p = p + (q @ v_body) * dt
            traj[i, :3] = p
            traj[i, 3:] = matrix_to_quat(q)
        traj[:, 2] = traj[0, 2]
        trajectories.append(traj)
        params.append(np.array([-reverse_speed, omega], dtype=np.float64))

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


def _roll_2d(grid, shift_voxels):
    """xy roll 的 2D 版本，语义与 roll_occupancy_grid 一致：挪出去的边清零。"""
    if np.all(shift_voxels == 0):
        return grid
    rolled = np.roll(grid, shift=tuple(-shift_voxels), axis=(0, 1))
    sx, sy = int(shift_voxels[0]), int(shift_voxels[1])
    if sx > 0:
        rolled[-sx:, :] = 0.0
    elif sx < 0:
        rolled[:-sx, :] = 0.0
    if sy > 0:
        rolled[:, -sy:] = 0.0
    elif sy < 0:
        rolled[:, :-sy] = 0.0
    return rolled


def build_route_fields(route_xy, shape, origin, resolution):
    """把全局路线栅格化成两张查表图，供 DWA 打分一次索引搞定。

    路线本身已经是绕开静态障碍的（map_node 在地图的 SDF 走廊里搜出来的），这里只是让它
    可查询。返回 (path_dist_map, remaining_map, has_route)：前者是每格到最近路线格的距离，
    后者是该格"路线还剩多长"，不在路线上的格子继承最近路线格的剩余量。
    """
    path_dist_map = np.full(shape, 1e3, dtype=np.float32)
    remaining_map = np.full(shape, 1e3, dtype=np.float32)
    if route_xy is None or len(route_xy) < 2:
        return path_dist_map, remaining_map, False

    rows, cols = shape
    route = np.asarray(route_xy, dtype=float)[:, :2]
    node_arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))))
    arc = float(node_arc[-1])
    if arc < 1e-9:
        return path_dist_map, remaining_map, False

    # 按半格重采样，否则栅格化出来的线有洞，距离变换会从洞里穿过去
    sample_arc = np.linspace(0.0, arc, int(np.ceil(arc / (0.5 * resolution))) + 1)
    r = ((np.interp(sample_arc, node_arc, route[:, 0]) - origin[0]) / resolution).astype(np.int64)
    c = ((np.interp(sample_arc, node_arc, route[:, 1]) - origin[1]) / resolution).astype(np.int64)
    inside = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
    if not np.any(inside):
        return path_dist_map, remaining_map, False

    route_mask = np.zeros(shape, dtype=bool)
    arc_map = np.zeros(shape, dtype=np.float32)
    route_mask[r[inside], c[inside]] = True
    arc_map[r[inside], c[inside]] = sample_arc[inside]   # 被经过两次的格子留后一次的弧长
    dist_cells, (near_r, near_c) = distance_transform_edt(~route_mask, return_indices=True)
    return ((dist_cells * resolution).astype(np.float32),
            (arc - arc_map[near_r, near_c]).astype(np.float32), True)


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
        # 🔴 可视化话题一律 BEST_EFFORT + depth 1。默认 QoS 是 RELIABLE：订阅方是 web
        # 后端（uvicorn 45~62% CPU）跟不上时，publish() 会把【规划环】卡住。
        # 2026-09-02 实测 A/B：可视化开 → 规划周期 0.76 Hz（p90 2.37s）；关 → 4.92 Hz
        # （p90 0.22s），快 6.5 倍。判据是关掉后 planning 的 CPU 反而从 35.6% 升到
        # 52.8% —— 它之前是【被阻塞】而不是算不过来。
        # ⚠️ 两端必须一起改：BEST_EFFORT 发布 + RELIABLE 订阅是 QoS 不兼容，零投递且完全静默。
        # trajectory_path 不在此列 —— 那是控制指令，必须可靠。
        self._viz_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.height_map_pub = self.create_publisher(Image, "/planning/height_map", self._viz_qos)
        # 相机视角的离地高度着色图，给前端和 infra1 并排对比用。只在有人订阅时才算。
        self.height_color_pub = self.create_publisher(Image, "/planning/height_color", self._viz_qos)
        self._height_color_uv = None
        self._height_color_last_ns = 0
        self.height_color_hz = float(os.environ.get('TINYNAV_HEIGHT_COLOR_HZ', '2.0'))
        self.height_color_stride = int(os.environ.get('TINYNAV_HEIGHT_COLOR_STRIDE', '2'))
        # obstacle = 逐像素打「这块被 z 跨度判成障碍了吗、没有的话卡在哪一步」；
        # height  = 旧的离地高度配色（它答不了「为什么没识别」）。
        self.height_color_mode = os.environ.get('TINYNAV_HEIGHT_COLOR_MODE', 'obstacle')
        # 判决图里区分「只有地面」和「有个矮东西被否了」的离地高度界。
        self.verdict_low_h = float(os.environ.get('TINYNAV_VERDICT_LOW_H', '0.03'))
        # 判决图的底图。infra = 红外图（能看清实物纹理，对得上现实），depth = 深度当亮度
        # （只有轮廓没纹理，但【免费】—— 深度图本来就在手里）。
        # ⚠️ infra 要多订阅一个 20 Hz 的 640x544 话题，板上实测一个什么都不做的裸订阅者
        # 就要【13.9% 一个核】，而判决图只用其中 1 Hz 一帧。这笔钱是刻意付的：depth 当
        # 底图只有轮廓没纹理，对不上现实里到底哪个东西被判成了什么，而这张图的全部用处
        # 就是让人做那个判断。CPU 再紧就退 depth。
        # 没有更便宜的红外源：keyframe_image 只有 0.66 Hz，运动时底图会比判决图旧 1.5 s。
        self.verdict_base = os.environ.get('TINYNAV_VERDICT_BASE', 'infra')
        self.infra_topic = os.environ.get(
            'TINYNAV_VERDICT_BASE_TOPIC', '/camera/camera/infra1/image_rect_raw')
        self._infra_sub = None
        self._infra_msg = None
        self.obstacle_mask_pub = self.create_publisher(OccupancyGrid, '/planning/obstacle_mask', self._viz_qos)
        self.footprint_pub = self.create_publisher(PointCloud, '/planning/footprint', self._viz_qos)
        self.occupancy_cloud_pub = self.create_publisher(PointCloud2, '/planning/occupied_voxels', self._viz_qos)
        self.occupancy_cloud_esdf_pub = self.create_publisher(PointCloud2, '/planning/occupied_voxels_with_esdf', self._viz_qos)
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, '/planning/occupancy_grid', self._viz_qos)
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
        # 取用策略：直接在回调里干活(FIFO，永远处理最旧的未过期集合)，还是回调只存、
        # 定时器取最新。默认沿用旧行为 —— 09-02 的仿真套件在这个改动下从 9/9 变 7/9，
        # 但那两个失败场景【单跑都过】，套件本身不可复现，所以既不能定罪也不能放行。
        # 要 A/B 就 TINYNAV_PLAN_LATEST_ONLY=1，判据见 in_age= 那一列。
        self._plan_latest_only = os.environ.get('TINYNAV_PLAN_LATEST_ONLY', '0') == '1'
        self._pending_set = None
        if self._plan_latest_only:
            # 20 Hz：必须快过深度帧(4.8 Hz)，否则 pending 会白等一拍。单线程执行器 +
            # 默认互斥回调组，所以它和 sync_callback 不会重入。
            self.create_timer(0.05, self._plan_tick)
        self.get_logger().info(
            "planning pose sync: "
            + ("exact" if use_exact_pose_sync else f"approximate slop={pose_sync_slop}s")
            + (", latest-only intake" if self._plan_latest_only else ", FIFO intake")
        )
        # Planning on old geometry is worse than not planning: the robot reacts to
        # obstacles that have moved and misses ones that have not. map_node guards its
        # keyframes the same way (max_keyframe_age_s); planning had no age check at all.
        self.max_input_age_s = float(os.environ.get('TINYNAV_MAX_INPUT_AGE_S', '0.5'))
        self._last_entry_age_s = 0.0
        self._lat = LatStats('plan', self.get_logger().info)
        self._last_stale_log_ns = 0
        self.camerainfo_sub = self.create_subscription(CameraInfo, '/camera/camera/infra2/camera_info', self.info_callback, 10)

        # z 是 9 层而不是对称的 10：栅格以相机为中心时只覆盖相机 ±0.5 m，robot_z_top 再大
        # 也会被栅格自己截断。9 层 + 上抬后覆盖相机 -0.3 .. +0.6 m，够放 robot_z_top 到 0.5。
        # 曾经开到 13 层（上限 1.0 m），结果桌面之类的悬空物全投影成地面障碍，走廊被关死：
        # front_clearance 掉到 0.11 m、106 条轨迹里 101 条被拒，车只能原地打转（2026-08-25）。
        # 🔴 2026-09-01：0.1 m -> 0.05 m，同时把框从 10 m 缩到 4 m，总格数几乎不变。
        # 起因是一个可复现的现场：椅子之间物理净宽 0.50 m，而车需要 0.344 m —— 本该能过，
        # 但过道相对栅格轴斜 54°，0.1 m 轴对齐栅格对斜过道每侧最多吃 0.1*(|cos|+|sin|)
        # = 0.140 m，净宽被栅格化成 0.30 m 就判死了。同一帧用原始深度独立重建，逐点得到
        # 一样的 0.30 m ——**地图是忠实的，是分辨率不够**（详见 docs/x5/grid_resolution.md）。
        # 只改这个局部避障栅格；全局地图（map_node）保持 0.1 m，两者本来就解耦。
        #   格数 80x80x14 = 89,600  vs  100x100x9 = 90,000  (1.00x)
        #   2D 平面 6,400 vs 10,000 (0.64x，EDT 反而更便宜)
        #   z 覆盖相机 -0.20 .. +0.475 m；band 是 [-0.20,+0.40]，正好用掉 12 层、上方留余量
        # 2 m 半径够用：3 s 轨迹 @max_vx 0.25 只有 0.75 m，前向探针 1.2 m，
        # 最远需求 0.75+0.122+0.05 = 0.92 m << 2.0 m。
        self.grid_shape = (80, 80, 14)
        self.resolution = 0.05
        # 地面在相机下方约 0.124 m。上抬 0.15 让栅格中心的 z 正好落在相机上（见下面 center
        # 的算法），否则每周期都会判定需要重定心。
        # z 分量决定体素层的水平切面落在地面哪个位置（"z 相位"）。位姿源是轮速里程计、
        # z 是固定偏置，所以相位是【冻结】的一个值，不像 VIO 那样会漂。0.15 时地面所在
        # 层的上沿离地 +24 mm，物体要够到再上一层（+74 mm）才有 0.10 的跨度 —— 6~7 cm
        # 的椅脚正好差这几毫米。降它就整体下移：0.135 -> 门限 60 mm。代价是等量的地面
        # 噪声容差（两者是同一条边界），改前必须在实际场地量一次空地误报。
        self.grid_offset = np.array(
            [0.0, 0.0, float(os.environ.get('TINYNAV_GRID_OFFSET_Z', '0.15'))])
        self.origin = np.array(self.grid_shape) * self.resolution / -2. + self.grid_offset
        # 深度图取样步长。10 时前方 1~3 m 的占据格单帧只召回 42~56%（以 step=2 为参考真值，
        # 2026-08-26 板上实测），而一个格子要连续两帧才越过阈值 —— 于是墙上全是洞。
        # 5 把召回提到 68~72%，代价 4.3 -> 14.0 ms（250 ms 周期的 4%）。这是填洞的正路：
        # 用膨胀填洞每格要付 0.1 m/侧的过道净宽，而这里付的是 CPU。
        self.step = int(os.environ.get('TINYNAV_RAYCAST_STEP', '5'))
        # 刻空闲的取样步长，和标命中分开。一条射线要写约 40 个体素，其中只有 1 个是命中 ——
        # 97% 的算力在刻空，而两件事需要的密度完全不同：命中稀了漏矮东西，刻空则相邻射线
        # 刻的几乎是同一批体素。板上真录数据实测 3/8 vs 3/3：耗时 1/3.4，椅子那侧障碍格
        # 94->208，空地近场假障碍仍 0。代价是清除延迟 —— 每帧被刻到的体素只少 13%，但
        # "一帧就能清"的从 97% 降到 67%，即 1/3 的体素清除从 0.2 s 变 0.4 s。
        # 设成和 step 相同就退回原版 run_raycasting_loopy（数学等价，实测差 3e-14）。
        self.carve_step = int(os.environ.get('TINYNAV_CARVE_STEP', '8'))
        # 轨迹库的采样密度：omega 取 n 个、vx 取 max(3, n//2) 个，共 n*max(3,n//2) 条
        # 前进弧 + 5 条倒车。打分是 O(条数 x 31 步 x 20 个车体取样点)，加密要线性付钱。
        self.traj_samples = int(os.environ.get('TINYNAV_TRAJ_SAMPLES', '15'))
        # 梯度脱困：当前位姿本身已在碰撞里时用的兜底。速度压得很低，且累计位移有预算 ——
        # 后方没有任何传感，长时间盲退比楔住更危险。
        self.escape_speed = float(os.environ.get('TINYNAV_ESCAPE_SPEED', '0.08'))
        self.escape_grad_min = float(os.environ.get('TINYNAV_ESCAPE_GRAD_MIN', '0.30'))
        # 倒车总开关。默认关：车后方没有任何传感器，而 2026-08-27 之前控制器把倒车指令
        # 一路执行成前进（见 tests/test_reverse_v_ref.py），所以整套"退"的阈值从来没有在
        # 真的会退的车上验证过。关掉之后楔住就停住并报无解，这比朝障碍开安全。
        self.allow_reverse = (self.robot.allow_reverse
                              and os.environ.get('TINYNAV_ALLOW_REVERSE', '0') == '1')
        # 一次脱困里允许倒的**净位移**上限，退够了就不再退。这是结构上的上限，不是阈值：
        # 无论哪条分支选了倒车、无论时钟怎么算，一次脱困最多只能倒这么远。
        self.reverse_budget_m = float(os.environ.get('TINYNAV_REVERSE_BUDGET_M', '0.30'))
        self._reverse_anchor = None
        self._reverse_spent_m = 0.0
        self._grad_escapes = 0
        self.occupancy_grid = np.zeros(self.grid_shape)
        # 每个 2D 格子最后一次"是障碍"的时刻。栅格确实每周期 x0.99，但那是慢的：饱和在
        # 0.2 的格子要 69 个周期(4.3 Hz 下约 16 s)才掉到阈值 0.1 以下。相比之下被射线穿过
        # 的格子一帧就清掉(0.2x0.99-0.1=0.098)。所以"障碍物停留很久"= 没人再看它一眼，
        # 而这两种情况以前没有任何数字能区分。
        # 🔴 衰减必须按**时间**折算，不能按周期。栅格原来每周期 x0.99，注释里的"约 16 s
        # 遗忘时间"隐含了 4.3 Hz 这个前提 —— 于是地图的新鲜度和 CPU 负载**静默耦合**：
        # 2026-09-01 把 raycast step 5->3 之后 cycle 从 0.23 涨到 0.40 s（p90 0.77），
        # 遗忘时间跟着变成 27.6 s。当天在过道里实测：mask 有 60% 的格子超过 20 s 没被再
        # 看到、中位年龄 96 s、最老 358 s，把物理 0.50 m 的净宽收成 0.30 m，车不敢走。
        # 折算到这个参考周期，遗忘时间就固定在秒上，不再随负载漂。
        self._decay_ref_dt = 0.23
        self._last_decay_stamp = None
        self._min_vis_logged = False
        # 规划环各段耗时的聚合。5 s 一行，不是每拍一行 —— TINYNAV_VERBOSE_TIMER=1 每拍
        # 打 5~10 行，那本身就会干扰它要测的东西，而且静止时测出来的数不代表跑起来。
        # codetiming 的 Timer.timers 一直在静默累计（logger=None 也累计），所以这里
        # 只是读出来 + 清零，不用改那 10 个 with Timer 的调用点。
        self.stage_log_s = float(os.environ.get('TINYNAV_STAGE_LOG_S', '5.0'))
        self._stage_win_ns = 0
        self._depth_dts = []
        self._last_depth_stamp = None
        self._stale_in_win = 0
        self._obstacle_first_seen = np.zeros(self.grid_shape[:2], dtype=np.float64)
        self._obstacle_last_seen = np.zeros(self.grid_shape[:2], dtype=np.float64)
        # 矮障碍（办公椅星形底盘、门槛、趴着的狗）的独立 2D 证据。z 跨度判据结构上看不见
        # 它们：0.1 m 的体素里 4~8 cm 的椅子腿和地面同层，跨度恒为 0，而体素层和地面的
        # 相位由 VIO 的 z 决定、会漂。所以这一路直接拿「离地高度」判，绕开体素量化。
        self.camera_height_m = float(os.environ.get('TINYNAV_CAMERA_HEIGHT_M', '0.18'))
        self.low_obs_h_lo = float(os.environ.get('TINYNAV_LOW_OBS_H_LO', '0.05'))
        # 0.25 以上交给跨度判据，这一路只管它看不见的那一段。
        self.low_obs_h_hi = float(os.environ.get('TINYNAV_LOW_OBS_H_HI', '0.25'))
        # 1.5 m 不是保守，是量出来的：2 m 之外拟合地面自己就抬高 4.5 cm，门限会开始误报。
        self.low_obs_max_range_m = float(os.environ.get('TINYNAV_LOW_OBS_RANGE_M', '1.5'))
        # 每格最少命中点数。2 而不是 5：一个格子收到的采样点数按 1/d^2 掉，而 4 cm 的椅子
        # 腿本来就只占格子的一小块 —— 1.0 m 处约 6 点、1.2 m 处 4.3 点、1.5 m 处 2.7 点，
        # 定值 5 等于在 1.1 m 之外把椅子腿整个丢掉。实测那一帧里 0.75~1.25 m 的触发格有
        # 36~42% 不足 5 点。空地那边不需要这个余量：71 个空地格在 1~8 点各档下误报全是 0，
        # 而「连续两帧才算」本身已经把单点飞点滤掉了。
        self.low_obs_min_pts = int(os.environ.get('TINYNAV_LOW_OBS_MIN_PTS', '2'))
        # 0 关掉这一路。
        # 🔴 默认关（2026-09-02 定案）。0.05 m 栅格下 z 跨度单独就够：仿真里 5 cm x
        # 0.35 m 的椅腿开/关都是 3 个障碍格，这一层贡献 0。而板上它贡献 185/243 格
        # (76%) —— 加的全是噪声：判据下限 0.05 m 比误差预算还小（实测俯仰固定偏置
        # 1.38 度在它自己 1.5 m 的量程上就是 0.036 m，再加地毯绒毛）。
        self.low_obs_enabled = os.environ.get('TINYNAV_LOW_OBS', '0') != '0'
        # 衰减 0.9 + 每帧 +0.1，判据 >0.1：和占据栅格一样要连续两帧，但会自己清掉 ——
        # 相机在 0.18 m 处向下只能看到身前 0.18 m 以外，车头前 13 cm 是盲区，靠这段记忆
        # （0.9^n，约 1.4 s / 0.3 m/s 下 0.42 m）盖过去。
        self._low_obstacle = np.zeros(self.grid_shape[:2], dtype=np.float64)
        self.K = None
        self.baseline = None
        self.last_T = None
        self.last_param = (0.0, 0.0) # acc and gyro
        # 锁定的是**朝向**，不是转向符号。锁符号的版本会被"这一拍恰好没有同号的可行原地转"
        # 打断，然后立刻反向重锁 —— 2026-08-27 实测每 0.9 s 变一次向、累计只转 13° 就回头，
        # 而当时车周围 12 个方向有 10 个在 0.5 m 内是空的，只要一直转 40° 就有路。
        self._escape_goal_yaw = None
        self._escape_goal_ns = 0
        # 一圈都没有能证明是空的朝向时，扫哪一边。必须在整个脱困过程里保持不变：按
        # "目标在左还是右"每次重算的话，车一转过目标方位这一侧就翻，于是又变成摆头 ——
        # 仿真 dead_end 场景一次跑出 20 次变向。
        self._escape_sweep_side = None
        self._escape_goal_max_s = 15.0          # 锁死一个转不出去的朝向的上限
        self._escape_goal_reached_rad = math.radians(12.0)
        # 从"发出转向指令"到"能改这条指令"之间的总环路延迟。2026-09-01 板上逐段实测：
        # 位姿数据龄 stamp_lag p50 0.59 + 重规划周期 0.23 + 执行器 0.22 = 1.04 s。
        # _pick_turn_toward 用它把档位按剩余误差选 —— 不做这件事就是 bang-bang 控制配
        # 1 秒延迟，必然发散（实测 112 s 转了 3147°，净转角只有 1142°）。
        self.escape_turn_delay_s = 1.0
        self._escape_scan_step_deg = 15
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
        self._escape_last_ns = 0            # 上一个脱困周期的时刻，用来判断 episode 断没断
        self._escape_episode_gap_s = 1.5    # 规划周期约 1.1 s，隔一拍就算新的一次脱困
        # The robot's own, whole. Rebuilding it field by field wired three of five
        # through, so min_wall_span_m and occ_threshold silently kept the class default
        # no matter what a platform asked for.
        # 三个最常调的门限做成环境变量，好在板上直接 A/B，不用改代码重推。
        self.obstacle_config = dataclasses.replace(
            self.robot.obstacle,
            min_wall_span_m=float(os.environ.get(
                'TINYNAV_MIN_WALL_SPAN_M', self.robot.obstacle.min_wall_span_m)),
            dilation_cells=int(os.environ.get(
                'TINYNAV_DILATION_CELLS', self.robot.obstacle.dilation_cells)),
            # z 波段下界也做成开关：它决定「地面那一层算不算进来」。相对相机，
            # 0.05 m 栅格下 -0.20 对应世界 z=-0.076，比地面还低 7.6 cm ——
            # 地面自己就在波段里，跨度全靠它凑（见 robot_config 里那段实测）。
            robot_z_bottom=float(os.environ.get(
                'TINYNAV_ROBOT_Z_BOTTOM', self.robot.obstacle.robot_z_bottom)),
        )
        self.get_logger().info(
            f"obstacle: raycast hit_step={self.step} carve_step={self.carve_step} "
            f"span>={self.obstacle_config.min_wall_span_m} "
            f"zbot={self.obstacle_config.robot_z_bottom:+.2f}(离地"
            f"{self.obstacle_config.robot_z_bottom + self.camera_height_m:+.3f}m) "
            f"dilation={self.obstacle_config.dilation_cells} "
            f"verdict_base={self.verdict_base} "
            f"low_obs={'on' if self.low_obs_enabled else 'off'} "
            f"({self.low_obs_h_lo}~{self.low_obs_h_hi} m, <{self.low_obs_max_range_m} m, "
            f">={self.low_obs_min_pts} pts, cam_h={self.camera_height_m})")
        # 单独打一行：robot.describe() 里的 reverse= 是平台配置，不是这次实际生效的值。
        self.get_logger().info(
            f"reverse: {'ENABLED' if self.allow_reverse else 'disabled'} "
            f"(TINYNAV_ALLOW_REVERSE, 一次脱困上限 {self.reverse_budget_m:.2f}m) —— "
            f"{'楔住时会倒车脱困' if self.allow_reverse else '楔住时停住并报无解，绝不倒车'}")
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
        # 一格。障碍图已经膨胀过 1 格(dilation_cells=1)，这一格是在那之上的余量。
        # 方形底盘用：几何在取样偏置里，这只是纯余量。圆盘走 _prefix_gate_m。
        self.prefix_margin_m = 0.10
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
        # 80 -> 120（2026-09-01 实测）。前方开着时前进的靠拢速率是 v*cos(误差)，90° 以内
        # 恒为正，而弧线还同时在把误差压小 —— 所以 80~120° 之间"停下来原地转"严格劣于走弧线。
        # 实测这一档触发了 46 次原地转里的 39 次，且当时 blocked 中位只有 42/110（前方是开的）。
        # 120° 以上目标真在身后，前进会走远，那时原地转才是对的。
        # 🔴 代价函数的终点朝向项。没有它的时候，14 条原地转在所有位置类代价上**完全同值**
        # （终点位置就是起点、路线剩余相同、贴合度相同、圆盘的碰撞分恒 0），唯一的区分项是
        # smoothness = 10*|上一拍 omega - 本拍 omega| —— 纯惯性，方向一旦错就一直错。
        # 2026-09-01 实测：目标在左 117°、前方大开（blocked 19/110、front_clearance>1.2 m）时
        # 代价函数选原地右转（远路 243°），heading_err 从 +104 一路涨到 +123，直到越过
        # force_turn_heading_rad 才被 escape=heading 拉回来，两者来回拉锯。
        # 权重 20：smoothness 的上限是 10*2*max_yaw = 12，20*1 rad 就压得住；而前进轨迹的
        # 位置项是 100*dist（0.75 m 给 75），所以「有路可进就别光对朝向」这个优先级不变。
        self.w_heading = float(os.environ.get('TINYNAV_W_HEADING', '20.0'))
        # 低于这个距离就不再谈"朝向目标"。0.3 m 与 map_node 的到达半径 0.4 m 同量级，
        # 且大于相机光心到控制中心的 0.067 m 偏置。
        self.heading_min_dist_m = float(os.environ.get('TINYNAV_HEADING_MIN_DIST_M', '0.3'))
        # 障碍势垒的权重。原来是写死的 100000 配 1/(d-hard)：在 soft 边界上外侧 0 分、
        # 内侧 99 万分，而「多推进 1 m」只值 100 分 —— 于是车一旦离障碍不到 soft，
        # 站着不动就是全局最优，实测前 5 名候选全是 vx=0（2026-09-01）。
        # 现在的量纲：1.0 表示「净空比 hard 多 0.03 m」约等于「少走 0.25 m」。
        self.w_obstacle = float(os.environ.get('TINYNAV_W_OBSTACLE', '1.0'))
        # 静止本身的代价。代价函数里三项都在惩罚运动（势垒、从 0 起步的 smoothness、朝向），
        # 而最慢那档 0.042 m/s 在 3 s 里只推进 0.125 m、进展奖励只有 13 分，加起来必输 ——
        # 于是 0.40 m 的窄缝前车永远站着（2026-09-01 实测：不动 207 分，最好前进 220 分）。
        # 目标近了要归零：到达是靠"奖励消失让车自然停住"实现的，见上面 target_dist_xy 那段。
        self.w_idle = float(os.environ.get('TINYNAV_W_IDLE', '40.0'))
        # 位姿瞬移检测。固件 VIO 跟丢后会【内部】重启并把原点归零（进程不重启，我们这边
        # 毫无察觉），世界系就此改了意义 —— 存下来的占据格记的都是旧世界的坐标，全部作废。
        # 不处理的后果不是"地图不准"而是"地图看着一片开阔"：2026-09-01 实测重置后一拍就
        # blocked=0/110、fwd_ok=90/90、front_clearance=>1.20m，规划器立刻发出 vx=0.250
        # 满速冲进一张空白地图。门限按物理极限定，1.0 m/s 是 max_vx 的 4 倍。
        self._pose_prev = None
        self._pose_jump_speed = float(os.environ.get('TINYNAV_POSE_JUMP_SPEED', '1.0'))
        # 占据要连续两帧才过阈值，5 Hz 下 0.4 s；留到 1.2 s 才敢再往前走。
        self._blind_s = float(os.environ.get('TINYNAV_BLIND_AFTER_JUMP_S', '1.2'))
        self._blind_until = 0.0
        self.idle_free_m = float(os.environ.get('TINYNAV_IDLE_FREE_M', '0.5'))
        # 「多慢算站着不动」。默认 1e-6 = 只罚严格 0，于是最慢那条 vx=0.067 完全躲过
        # 惩罚。上游 xiaole/planning-cost-and-s-bend 用的是 min_linear_vel 这一档门限，
        # 且权重 4000（我们 40 = 只值 0.4 m 路线进展）。留成旋钮，先在仿真里 A/B。
        self.idle_vx_eps = float(os.environ.get('TINYNAV_IDLE_VX_EPS', '1e-6'))
        self.force_turn_heading_rad = math.radians(120.0)
        # no-progress 脱困的时间预算与之后的冷却，见 _noprogress_budget。
        self._noprog_start_ns = 0
        self._noprog_block_until_ns = 0
        self._noprog_max_s = float(os.environ.get('TINYNAV_NOPROGRESS_ESCAPE_S', '5.0'))
        self._noprog_cooldown_s = float(os.environ.get('TINYNAV_NOPROGRESS_COOLDOWN_S', '5.0'))
        # A heading counts as an escape only if the probe finds nothing within this
        # much. Above robot.front_blocked_m so the turn actually releases the gate
        # instead of handing back a heading that re-triggers it next cycle.
        self.escape_min_clearance_m = max(0.4, self.robot.front_blocked_m + 0.1)
        # front_blocked 的解封门限：可行前进轨迹要到这么多条才算"前方不堵了"。
        # 1 条不算 —— 见 _blocked_latched。0 = 关掉滞回，回到旧的瞬时判据。
        self.blocked_release_ok = int(os.environ.get('TINYNAV_BLOCKED_RELEASE_OK', '5'))
        self._blocked_latch = False

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

        # 全局路线。订的是 map_node 已经转到 world 帧的那条（/mapping/global_plan 是 map 帧，
        # 而导航模式下没有任何节点广播 map->world 的 TF）。
        self.create_subscription(Path, '/mapping/global_plan_odom', self._on_global_route, 1)
        self._route_xy = None
        self.route_cost_enabled = os.environ.get('TINYNAV_ROUTE_COST', '1') == '1'
        # 障碍项的 1e5 压倒一切；剩下的里，progress 决定走多快，follow 拦住"横切到路线上
        # 更近的一点"这种抄近道。
        self.w_route_progress = float(os.environ.get('TINYNAV_W_ROUTE_PROGRESS', '100.0'))
        self.w_path_follow = float(os.environ.get('TINYNAV_W_PATH_FOLLOW', '80.0'))
        # remaining_map 在到达前就饱和到 0，最后一段要靠这项把车拉到精确目标上
        self.w_goal_terminal = float(os.environ.get('TINYNAV_W_GOAL_TERMINAL', '100.0'))
        self.route_terminal_band = float(os.environ.get('TINYNAV_ROUTE_TERMINAL_BAND', '0.5'))
        # 决策日志的节流，Hz。0 = 每个规划周期都打。
        # 🔴 默认曾是写死的 1 Hz，而规划周期是 0.25 s —— 于是 5 拍里只看得到 1 拍，
        # 从日志量"障碍首次出现在多远"必然偏小 v*1.15s（0.4 m/s 时 0.46 m），我据此
        # 给出过一版偏低的探测距离。复盘要用的那几个量必须逐拍可见。
        self.decision_log_hz = float(os.environ.get('TINYNAV_DECISION_LOG_HZ', '0'))
        # 沿全局路线往前探多远，用来判"路线自己撞进障碍里了"。
        self.route_probe_m = float(os.environ.get('TINYNAV_ROUTE_PROBE_M', '2.0'))
        # 路线自己穿进障碍时，本拍不再为【贴合】它罚分（进展项照旧保留）。
        # 判据是 2026-09-03 板上实测，不是调出来的：route_clear >= hard 的 382 拍里
        # esdf_at_robot 中位 0.56 m、可行前进 85/90；< hard 的 325 拍（占全程 46%）
        # 中位掉到 0.27、可行 26/90，而那一趟 3 次真碰撞 + 17 次全速贴着走【全部】
        # 落在后者。所以阈值就取碰撞门限本身。
        # 🔴 只清 w_path_follow，不清 w_route_progress —— 整条路线丢掉会退回贪心终点
        # 距离，那正是 PR #226 的路线打分修掉的绕角抖动（doorway 138 次变向）。
        self.route_unpin_on_block = os.environ.get('TINYNAV_ROUTE_UNPIN_ON_BLOCK', '1') == '1'
        # 势垒是否把起点算进最小净空。默认 0 = 不算，见 clear_min 处的注释。
        self.barrier_include_start = os.environ.get('TINYNAV_BARRIER_INCLUDE_START', '0') == '1'


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
                    num_samples=self.traj_samples,
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
                self._score_trajectories(trajectories, esdf)   # 预热 numba
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
        self._route_xy = None           # 缓存的路线是通往旧目标的
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

    def _on_global_route(self, msg: Path):
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self._route_xy = np.asarray(pts, dtype=float) if len(pts) >= 2 else None

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

    def _score_trajectories(self, trajectories, esdf, params=None,
                            path_dist_map=None, remaining_map=None):
        """Collision scores, with in-place turns exempted on a round base.

        A circle rotating in place sweeps the area it already occupies, so the verdict is
        about the present, not the motion, and is unactionable. Measured 2026-08-10: it
        killed 12 of 14 turns, and the escape hatch then failed silently for 23 s.

        🔴 这条豁免的前提是【碰撞圆的圆心就是旋转中心】。驱动轮后置 20 mm 之后车身不再
        以盘心为圆心，所以 collision_radius 取的是绕驱动轴的扫掠半径（0.1375 = 盘半径
        0.1175 + 后移 0.020）。谁要是把它改回盘半径，这条豁免就静默地变成错的。
        """
        front_len, rear_len, half_w = self.robot.footprint_from_control()
        if path_dist_map is None:
            path_dist_map = np.full(esdf.shape, 1e3, dtype=np.float32)
            remaining_map = np.full(esdf.shape, 1e3, dtype=np.float32)
        scores, occ_points, path_costs, end_remainings = score_trajectories_by_ESDF(
            trajectories, esdf, path_dist_map, remaining_map,
            self.origin, self.resolution,
            self.robot.hard_clearance, self.robot.soft_clearance,
            front_len, rear_len, half_w, self.robot.is_circle,
        )
        if params is not None and self.robot.is_circle:
            for i in range(len(params)):
                if self._is_turn_in_place(params[i]):
                    scores[i] = 0.0
        return scores, occ_points, path_costs, end_remainings

    def _front_gate_m(self):
        """界面提示用的净空阈值。规划器已不用它，见 _prefix_clearance。"""
        return max(self.robot.front_blocked_m,
                   (self.robot.max_vx / 6.0) * self.reaction_time_s)

    def _prefix_gate_m(self):
        """承诺段的可行门限。圆盘：车体在门限里，和碰撞判据同一个数，两处不会再打架。"""
        return self.robot.hard_clearance if self.robot.is_circle else self.prefix_margin_m

    def _prefix_clearance(self, trajectories, esdf):
        """每条轨迹在它**将要执行的那一段**上的最小 ESDF（圆盘查中心，方形查车体五点）。

        为什么不是沿当前朝向的一条直线：转开的弧线在自己那条路上是空的，直线探针却因为
        正前方有东西把它一起禁掉 —— 2026-08-26 闭环仿真里，0.9 m 宽的缝在直线探针下
        「转 102 次、变向 25 次、超时」，换成这个判据后「13.5 s 到达，0 转 0 退 0 变向」，
        其余四个场景逐位不变。

        只取前 reaction_time_s 那一段而不是整条：重规划周期 p50 0.28 s，承诺出去的只有
        到下一次能改指令为止的那一截，再往后的部分下个周期会重新判。"""
        n_pref = max(2, min(trajectories.shape[1],
                            int(round(self.reaction_time_s / self.dt)) + 1))
        P = trajectories[:, :n_pref, :]
        if self.robot.is_circle:
            # 圆盘查中心一次就够，车体半径在门限里（见 hard_clearance）。按外接方形取四角
            # 是把半径算第二遍：角在 0.166 m 处，再要 0.10 m 净空等于要求过道净宽
            # 2x(0.1175+0.10)=0.435 m，而真实需求 2x0.172=0.344 m。实测 0.40 m 的过道
            # 因此被判成堵，90 条前进轨迹只放行 6 条（改后 30 条）。
            # score_trajectories_by_ESDF 早就是 is_circle -> 单点，这里当初漏了。
            return self._esdf_lookup(esdf, P[..., 0], P[..., 1]).min(axis=1)
        fl, rl, hw = self.robot.footprint_from_control()
        qx, qy, qz, qw = P[..., 3], P[..., 4], P[..., 5], P[..., 6]
        fx = 2.0 * (qx * qz + qw * qy)
        fy = 2.0 * (qy * qz - qw * qx)
        nrm = np.hypot(fx, fy)
        nrm[nrm < 1e-6] = 1.0
        fx, fy = fx / nrm, fy / nrm
        lx, ly = -fy, fx
        xs = np.stack([P[..., 0],
                       P[..., 0] + fx * fl + lx * hw, P[..., 0] + fx * fl - lx * hw,
                       P[..., 0] - fx * rl + lx * hw, P[..., 0] - fx * rl - lx * hw], axis=-1)
        ys = np.stack([P[..., 1],
                       P[..., 1] + fy * fl + ly * hw, P[..., 1] + fy * fl - ly * hw,
                       P[..., 1] - fy * rl + ly * hw, P[..., 1] - fy * rl - ly * hw], axis=-1)
        return self._esdf_lookup(esdf, xs, ys).min(axis=(1, 2))

    _STAGES = ('preprocess', 'raycasting', 'obstacle map', 'vis',
               'vis:mask', 'vis:heightmap', 'vis:footprint', 'vis:verdict', 'vis:voxels',
               'pub:prefix', 'pub:cost', 'pub:emit', 'pub:log',
               'traj gen', 'traj score', 'pub')

    def _log_stage_timing(self, now_ns, depth_stamp_s):
        """规划环的逐段耗时 + 实际消费到的深度间隔。一行一个窗口。

        depth_dt 是【真正被规划消费的】相邻两帧的间隔，不是话题频率 —— bridge 丢帧、
        执行器排不上都会体现在这里，而话题频率看不出来。"""
        if depth_stamp_s is not None:
            if self._last_depth_stamp is not None:
                dt = depth_stamp_s - self._last_depth_stamp
                if 0.0 < dt < 10.0:
                    self._depth_dts.append(dt)
            self._last_depth_stamp = depth_stamp_s
        if self.stage_log_s <= 0.0:
            return
        if self._stage_win_ns == 0:
            self._stage_win_ns = now_ns
            return
        win_s = (now_ns - self._stage_win_ns) / 1e9
        if win_s < self.stage_log_s:
            return
        self._stage_win_ns = now_ns

        def q(name):
            if name not in Timer.timers:
                return None
            return (Timer.timers.median(name) * 1e3, Timer.timers.max(name) * 1e3)
        loop = q('Planning Loop')
        parts = []
        for st in self._STAGES:
            v = q(st)
            if v is not None:
                parts.append(f"{st.replace(' ', '')} {v[0]:.0f}/{v[1]:.0f}")
        n = Timer.timers.count('Planning Loop') if 'Planning Loop' in Timer.timers else 0
        Timer.timers.clear()

        dts = sorted(self._depth_dts); self._depth_dts = []
        if dts:
            d50 = dts[len(dts) // 2]
            d90 = dts[min(len(dts) - 1, int(0.9 * len(dts)))]
            gaps = sum(1 for d in dts if d > 0.30)
            dtxt = (f"depth_dt p50={d50:.2f}s p90={d90:.2f}s >0.30s={gaps}/{len(dts)} "
                    f"({1.0 / d50:.1f} Hz)")
        else:
            dtxt = "depth_dt n/a"
        stale = self._stale_in_win; self._stale_in_win = 0
        self.get_logger().info(
            f"stage ms (n={n}, {win_s:.1f}s): "
            + (f"total {loop[0]:.0f}/{loop[1]:.0f} | " if loop else "")
            + " ".join(parts) + f" | {dtxt} | stale={stale}")

    def _min_visible_height_m(self, origin_z, robot_z):
        return min_visible_height_m(origin_z, robot_z, self.grid_shape[2],
                                    self.resolution, self.camera_height_m,
                                    self.obstacle_config)

    def _esdf_lookup(self, esdf, xs, ys):
        """格外的点返回 inf（当作没约束），不是 0 —— 否则栅格边缘会把轨迹全禁掉。"""
        ii = ((xs - self.origin[0]) / self.resolution).astype(np.int32)
        jj = ((ys - self.origin[1]) / self.resolution).astype(np.int32)
        inb = ((ii >= 0) & (ii < esdf.shape[0]) & (jj >= 0) & (jj < esdf.shape[1]))
        return np.where(inb, esdf[np.clip(ii, 0, esdf.shape[0] - 1),
                                  np.clip(jj, 0, esdf.shape[1] - 1)], np.inf)

    def _route_block_ahead(self, esdf, route_xy, init_p):
        """沿全局路线往前找第一个被挡点，返回 (弧长 m, 这段路线上的最小净空 m)。

        全局搜索只认建图那一刻的占据图（map_node 一次性 np.load，全程不更新），建图之后
        搬进来的东西它完全看不见 —— 而局部这边还在为贴合这条路线付 w_path_follow。
        没有这个量，"卡在障碍前"和"绕不过去"在日志里长得一模一样。"""
        nan = float('nan')
        if route_xy is None or len(route_xy) < 2:
            return nan, nan
        r = np.asarray(route_xy, dtype=float)[:, :2]
        seg = r[int(np.argmin(np.linalg.norm(r - init_p[:2], axis=1))):]
        if len(seg) < 2:
            return nan, nan
        arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))))
        keep = arc <= self.route_probe_m
        d = self._esdf_lookup(esdf, seg[keep, 0], seg[keep, 1])
        if d.size == 0:
            return nan, nan
        hit = np.flatnonzero(d < self.robot.hard_clearance)
        return (float(arc[keep][hit[0]]) if hit.size else float('inf')), float(d.min())

    def _motion_gate_penalty(self, param, idx, forward_ok, front_blocked):
        """1e9 on any motion this sensor cannot vouch for, 0.0 otherwise.

        The occupancy grid is written only by forward raycasting, so a reverse
        trajectory cannot be rejected for collision -- it is scored against cells the
        camera never looked at. Measured 2026-08-10 19:19: 40 s of vx=-0.2 straight
        into an obstacle while up to 100 of the 106 trajectories were collision-free,
        because the old gate made reverse the *only* admissible option whenever the
        front was blocked and the library holds exactly one reverse."""
        is_reverse = param[0] < 0.0
        if is_reverse:
            return 0.0 if (front_blocked and self.allow_reverse) else 1e9
        if self._is_turn_in_place(param):
            # Turning in place is the only motion whose swept volume the camera has
            # already observed, so the front probe does not gate it.
            return 0.0
        return 0.0 if forward_ok[idx] else 1e9

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
        """最贴合来路的那条后退轨迹的下标，若此刻后退不可证明安全则 None。

        占据栅格只由前向射线写入，所以车后方的格子从来没被观测过 —— 对后退做碰撞检查是
        拿没看过的格子在打分，这正是 _motion_gate_penalty 里那段历史（40 s 的 vx=-0.2 直接
        撞上去）的由来。这里不检查栅格，而是检查**这条后退路径的每个采样点，车体中心是否
        在最近几秒真的待过那里** —— 待过就说明能走。"""
        if not self.allow_reverse:
            return None
        if (self._retreat_start_ns
                and (now_ns - self._retreat_start_ns) / 1e9 > self._retreat_max_s):
            return None                     # 退够久了就停手，不能一路倒回起点
        if not self._centre_history:
            return None
        hist = list(self._centre_history)
        # 词汇表现在有 5 条后退(直退 + 4 条带转向)，所以要**逐条评分再挑最好的**，
        # 不能像以前那样第一条不合格就整个放弃 —— 那时只有一条，两种写法等价。
        best_i, best_worst = None, None
        for i, param in enumerate(params):
            if param[0] >= 0.0:
                continue
            pts = trajectories[i][:, :2]
            # 每 5 个点抽一个：0.1 s 步长下相邻点只差 2 cm，逐点查是白花钱。
            worst = 0.0
            for x, y in pts[::5]:
                d = min((math.hypot(x - hx, y - hy) for _, hx, hy in hist), default=1e9)
                if d > worst:
                    worst = d
                if worst > self._retreat_match_m:
                    break
            if worst > self._retreat_match_m:
                continue
            if best_worst is None or worst < best_worst:
                best_i, best_worst = i, worst
        if best_i is None:
            return None
        if not self._retreat_start_ns:
            self._retreat_start_ns = now_ns
        return best_i

    def _reverse_ok(self, init_p):
        """这一拍还能不能倒。总开关关着、或本次脱困已经倒够了，就不能。"""
        if not self.allow_reverse:
            return False
        if self._reverse_anchor is None:
            return True
        self._reverse_spent_m = float(np.linalg.norm(
            np.asarray(init_p[:2], dtype=np.float64) - self._reverse_anchor))
        return self._reverse_spent_m <= self.reverse_budget_m

    def _reverse_mark(self, init_p):
        """记下这次脱困的起点；预算按离它的净位移算，不按发出去的路径长度。"""
        if self._reverse_anchor is None:
            self._reverse_anchor = np.asarray(init_p[:2], dtype=np.float64).copy()
            self._reverse_spent_m = 0.0

    def _reverse_release(self):
        """有非倒车的动作可做了，预算复位。"""
        self._reverse_anchor = None
        self._reverse_spent_m = 0.0

    def _should_retreat(self, front_blocked, escape_clear, escape_age_s):
        """原地转脱困该不该改成倒车。

        `front_blocked` 是硬前提：倒车治不了朝向错和没进展，只能治"前面堵住"。少了这个
        前提，2026-08-27 实测出现连续 36 s 的倒车，其间 front_clearance 一路涨到 >1.20 m
        —— 目标在身后、正确动作是原地转，结果车倒着开向目标。
        """
        if not front_blocked:
            return False
        return (escape_clear < self.escape_min_clearance_m
                or escape_age_s > self._escape_before_retreat_s)

    def _blocked_latched(self, raw_blocked, n_fwd_ok):
        """给 front_blocked 加滞回。

        🔴 原来它是瞬时判据：90 条前进轨迹里只要有 1 条可行就翻成 False，于是
        `escape_reason` 掉回 ""、`escape_age` 归零。2026-09-02 板上卡死 12.5 s 那段
        `fwd_ok` 在 0 和 1~2 之间每拍来回，`escape_age` 因此在 0.2/1.4/0.5/0.2 之间
        重置、**从没累积到 `_escape_before_retreat_s`**，倒车永不触发；13 秒里净位移
        0.04 m，最后要人去搬。
        判据是量出来的：那段里"解封"的那些拍 `fwd_ok` 从没超过 **2/90**，所以
        「要 5 条才算解封」在整段里都保持锁定，而真的走得动时是 71~90 条。"""
        if raw_blocked:
            self._blocked_latch = True
        elif n_fwd_ok >= self.blocked_release_ok:
            self._blocked_latch = False
        return self._blocked_latch

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

    def _noprogress_budget(self, escape_reason, now_ns):
        """给 no-progress 脱困一个时间预算，超了就放手让排序自己走。

        这一支唯一的出口是"真的选了 vx>0 的轨迹"，可脱困模式恒发原地转，出口永远够不着。
        2026-09-02 板上：33 拍里脱困目标在 +169°/+148° 之间换边 15 次，朝向误差反而从 39°
        涨到 100°，35 s 一步没走，其间可行前进轨迹有 88/90 条、前方净空 >1.2 m。
        放手是安全的：这一支的前提就是前方没堵（front_blocked 先被 "blocked" 截走），
        而 120° 以内前进的靠拢速率 v*cos(误差) 为正、弧线同时在把朝向误差压小。"""
        if escape_reason != "no-progress":
            self._noprog_start_ns = 0
            return escape_reason
        if now_ns < self._noprog_block_until_ns:
            return ""
        if not self._noprog_start_ns:
            self._noprog_start_ns = now_ns
        if (now_ns - self._noprog_start_ns) / 1e9 <= self._noprog_max_s:
            return escape_reason
        self._noprog_start_ns = 0
        self._noprog_block_until_ns = now_ns + int(self._noprog_cooldown_s * 1e9)
        self.get_logger().info(
            f"no-progress 脱困用满 {self._noprog_max_s:.0f}s 仍无进展，"
            f"放手 {self._noprog_cooldown_s:.0f}s 让排序走弧线")
        return ""

    def _open_heading(self, center, obstacle_mask, yaw_now, yaw_to_target, min_deg=0):
        """最近的一个「走廊在探针内是空的」朝向，平手时偏目标那一侧。

        扫的是整整一圈的候选朝向，而不是那 1~3 条可行原地转的**终点朝向** —— 前方 0.2 m
        有墙时可行的原地转只剩最小的几条(±13~26°)，它们的终点朝向全都还对着墙，净空一律
        0.05~0.14 m，于是"哪个更空"完全是噪声，左右都一样。

        返回 (朝向, 净空)；一圈都没有能证明是空的就返回 (None, 最好的那个净空)。
        """
        prefer = 1.0 if self._wrap(yaw_to_target - yaw_now) >= 0.0 else -1.0
        best_c = float('nan')
        for step in range(int(min_deg), 181, self._escape_scan_step_deg):
            for sgn in ((prefer, -prefer) if step else (1.0,)):
                y = self._wrap(yaw_now + sgn * math.radians(step))
                c = float(self._clearance_along(center, math.cos(y), math.sin(y),
                                                obstacle_mask))
                if c >= self.escape_min_clearance_m:
                    return y, c
                if math.isnan(best_c) or c > best_c:
                    best_c = c
        return None, best_c

    @staticmethod
    def _world_yaw_rate(param):
        """轨迹库参数里的角速度换成**世界系** yaw 角速度。

        🔴 `params[..][1]` 是绕**相机 Y 轴**的角速度（`generate_trajectory_library_3d` 里
        `rotvec_to_matrix([0, omega*dt, 0])`），而相机光学约定 +y **朝下** —— 绕 +y 正转
        是俯视顺时针，也就是世界 yaw **负**向。所以两者恒为反号。

        2026-09-01 实测佐证：同一次运行的 79 条决策里，日志打的 `omega`（params 那个）
        和跟踪器实收的 `w_ref`（世界 wz）**96% 反号、63/72 对绝对值相等**。

        `_pick_turn_toward` 原来直接拿 `params[i][1]` 的符号去和世界系的 `goal_err` 比，
        于是**恒定选反方向**：车朝着锁定朝向的反面转，一路转到对面 180° 附近才因为 wrap
        翻号而卡住 —— 实测 `|goal_err|` 中位 158°、**一次都没进过 12° 的到位窗口**。
        三个现场症状（能走不走一直摆头 / 摆头收敛到与 POI 完全反向 / POI 在反向更不走）
        全部是这一个符号错。

        车身接近水平时就是取负（俯仰 1° 的量级可以忽略）。"""
        return -float(param[1])

    def _fastest_within_budget(self, cand, params, err_abs):
        """剩余误差 err_abs 下转过去不会冲出到位窗口的最快一档。

        全都太快时取**最慢**的那一档而不是最快的：站着不动会让 escape_age 一直涨，
        而低于轨迹库最小档的角速度本来也发不出来。"""
        budget = (max(err_abs - self._escape_goal_reached_rad * 0.5, 0.0)
                  / self.escape_turn_delay_s)
        mag = lambda i: abs(float(params[i][1]))
        ok = [i for i in cand if mag(i) <= budget]
        return max(ok, key=mag) if ok else min(cand, key=mag)

    def _pick_turn_toward(self, turns, params, err):
        """转向锁定朝向的那条可行原地转，(下标, 是否走远路) 或 (None, False)。

        近路是 |err|，远路是 2*pi-|err|，**两条都收敛到同一个锁定朝向** —— 朝向是固定的，
        所以允许走远路不会来回摆。少了这条会死锁：仿真里出现过 `goal_err=-170deg turns=7`
        而 7 条可行原地转全是反号，于是每一拍都"保持朝向不动"，永远不动。

        🔴 档位按剩余误差选，不是永远选最快的那一档（2026-09-01 撞击后第二次实测定案）。
        原来是 bang-bang：不管还差 90° 还是差 15° 都发 max_yaw。而 max_yaw x 环路延迟
        = 1.05 x 1.04 = 62.6°，是 12° 到位窗口的 5.2 倍 —— 每次都冲过头，reached 触发后
        重扫 _open_heading 得到一个**不同**的朝向（实测 111 s 里换了 12 个），于是车一直
        转下去：累计 3147°，净转角 1142°，vx=0 占 65% 的决策，而 gate=forward 有 75/78
        次是开着的（**根本没被堵住**）。按误差选档让"来不及改的转角 <= 剩余误差"自动成立。
        """
        # err 是世界系的角度差，所以比的必须是世界系的 yaw 角速度，见 _world_yaw_rate
        want = math.copysign(1.0, err)
        near = [int(i) for i in turns
                if params[i][1] != 0.0
                and math.copysign(1.0, self._world_yaw_rate(params[i])) == want]
        if near:
            return self._fastest_within_budget(near, params, abs(err)), False
        far = [int(i) for i in turns if params[i][1] != 0.0]
        if not far:
            return None, False
        return self._fastest_within_budget(
            far, params, 2.0 * math.pi - abs(err)), True

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

    def _publish_height_color(self, depth, T, fx, fy, cx, cy, floor_z, stamp,
                              obstacle_mask=None):
        """相机视角的逐像素判决图，和 infra1 并排看。

        默认 obstacle 模式：每个像素投到栅格，报它所在格子被 z 跨度判据判成什么。
        `TINYNAV_HEIGHT_COLOR_MODE=height` 回到旧的离地高度配色。
        """
        if self.height_color_pub.get_subscription_count() == 0:
            return
        if (self._infra_sub is None and self.height_color_mode == 'obstacle'
                and self.verdict_base == 'infra'):
            self._infra_sub = self.create_subscription(
                Image, self.infra_topic, self._infra_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.get_logger().info(f"判决图底图: 订阅 {self.infra_topic}")
        now = self.get_clock().now().nanoseconds
        if now - self._height_color_last_ns < 1e9 / max(self.height_color_hz, 0.1):
            return
        self._height_color_last_ns = now
        # 半分辨率：预览链路本来就把帧缩到 320 px 边长再编码，全分辨率算完全是白花的。
        d = depth[::self.height_color_stride, ::self.height_color_stride]
        if self._height_color_uv is None or self._height_color_uv[0].shape != d.shape:
            st = self.height_color_stride
            v, u = np.mgrid[0:depth.shape[0]:st, 0:depth.shape[1]:st]
            self._height_color_uv = ((u.astype(np.float32) - cx) / fx,
                                     (v.astype(np.float32) - cy) / fy)
        gu, gv = self._height_color_uv
        R = T[:3, :3]
        pw_z = d * (R[2, 0] * gu + R[2, 1] * gv + R[2, 2]) + T[2, 3]

        if self.height_color_mode != 'obstacle' or obstacle_mask is None:
            idx = np.searchsorted(_HEIGHT_BINS, pw_z - floor_z).astype(np.uint8)
            idx[d <= 0] = len(_HEIGHT_LUT) - 1
            img = _HEIGHT_LUT[idx]
        else:
            img = self._verdict_image(d, gu, gv, T, pw_z, obstacle_mask)
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        self.height_color_pub.publish(msg)

    def _infra_callback(self, msg):
        # 只存引用，转换留给 2 Hz 的判决图去做 —— 这个回调要尽量便宜。
        self._infra_msg = msg

    def _verdict_base(self, d):
        """判决图的底图：红外图优先，没有就用深度当亮度（能看出轮廓但没纹理）。"""
        msg = self._infra_msg
        if msg is not None:
            try:
                g = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                g = g[::self.height_color_stride, ::self.height_color_stride]
                if g.shape == d.shape:
                    return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            except Exception:
                pass
        v = np.clip((3.0 - d) / 2.8, 0.0, 1.0) * 0.70 + 0.30
        v[d <= 0] = 0.22
        return cv2.cvtColor((v * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)

    def _verdict_image(self, d, gu, gv, T, pw_z, obstacle_mask):
        R = T[:3, :3]
        pw_x = d * (R[0, 0] * gu + R[0, 1] * gv + R[0, 2]) + T[0, 3]
        pw_y = d * (R[1, 0] * gu + R[1, 1] * gv + R[1, 2]) + T[1, 3]
        verdict, st = classify_verdict(
            self.occupancy_grid, self.origin, self.resolution, T[2, 3],
            self.camera_height_m, obstacle_mask, d, pw_x, pw_y, pw_z,
            self.verdict_low_h, self.obstacle_config)
        if self.stage_log_s > 0.0:
            tot = verdict.size
            cnt = np.bincount(verdict.ravel(), minlength=len(_VERDICT_NAMES))
            self.get_logger().info(
                "verdict %: " + " ".join(
                    f"{nm}={100.0 * c / tot:.0f}" for nm, c in zip(_VERDICT_NAMES, cnt))
                + f" | 格子: 障碍={st['obstacle_cells']}"
                  f" 矮物被否={st['low_rejected_cells']}"
                  f" 只有地面={st['ground_only_cells']}"
                  f" 空列={st['empty_cols']}"
                  f" 门限={self.obstacle_config.min_wall_span_m:.2f}m"
                  f" 离地界={self.verdict_low_h:.2f}m",
                throttle_duration_sec=self.stage_log_s)
        return tint_verdict(self._verdict_base(d), verdict)

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
            # 只是给界面看的提示，**不再是规划器的判据** —— 规划器现在按每条轨迹自己
            # 那段路判(见 _prefix_clearance)，「堵住」等于一条前进轨迹都不可行。这里保留
            # 直线探针的口径是因为诊断在轨迹库生成之前就发布了，那时还没有轨迹可判。
            'frontBlocked': bool(front_clearance <= self._front_gate_m()),
            'frontBlockedAtM': round(float(self._front_gate_m()), 2),
            'obstacleCells': int(np.count_nonzero(obstacle_mask)),
            'esdfAtRobotM': (None if not np.isfinite(esdf_at_robot)
                             else round(float(esdf_at_robot), 2)),
            'cycleS': (round(self._loop_period_s, 3) if self._loop_period_s else None),
            'stampLagS': round(now_ns / 1e9 - stamp, 2),
        }, separators=(',', ':'))
        self.diag_pub.publish(msg)

    def _gradient_escape(self, esdf_map, init_p, init_q, header, base_time, num_steps):
        """当前位姿本身已在碰撞里时，从 ESDF 场解出脱困方向，而不是从 110 条预制弧里挑
        （那时全是 inf，挑不出东西 -> 发静态路径 -> 控制器判"到达" -> 零速 -> 下一周期
        还是全灭，自我维持）。方向是解出来的不是选出来的，所以永远有答案。

        只做纯平移：楔住时旋转会把车角扫进障碍（方形底盘扫过半径 0.202 m > 前缘 0.10 m）。
        """
        pt = self._worst_footprint_point(esdf_map, init_p, init_q)
        gx, gy = self._esdf_gradient(esdf_map, pt)
        if gx is None:
            return None
        fwd = self._forward_of(init_q)
        # 上坡方向离障碍更远；取它在车头方向的投影决定进还是退。投影太小说明障碍在正侧方，
        # 平移帮不上，仍选倒车 —— 它是方形底盘唯一不扫角的动作。
        s = gx * fwd[0] + gy * fwd[1]
        v = self.escape_speed if s > self.escape_grad_min else -self.escape_speed
        if v < 0.0:
            if not self._reverse_ok(init_p):
                if self.allow_reverse:
                    self.get_logger().warning(
                        f"gradient escape: 已倒 {self._reverse_spent_m:.2f}m "
                        f"(上限 {self.reverse_budget_m:.2f}m) 仍未脱困，保持静止")
                return None
            self._reverse_mark(init_p)
        num_steps = max(2, int(num_steps))
        path = Path()
        path.header = Header()
        path.header.stamp = header.stamp
        path.header.frame_id = "world"
        for j in range(num_steps):
            d = v * float(j) * self.dt
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = (base_time + Duration(seconds=float(j) * self.dt)).to_msg()
            pose.header.frame_id = "world"
            pose.pose.position.x = float(init_p[0] + fwd[0] * d)
            pose.pose.position.y = float(init_p[1] + fwd[1] * d)
            pose.pose.position.z = float(init_p[2])
            pose.pose.orientation.x = float(init_q[0])
            pose.pose.orientation.y = float(init_q[1])
            pose.pose.orientation.z = float(init_q[2])
            pose.pose.orientation.w = float(init_q[3])
            path.poses.append(pose)
        self._grad_escapes += 1
        if self._grad_escapes % 10 == 1:
            self.get_logger().warning(
                f"gradient escape #{self._grad_escapes}: "
                f"{'forward' if v > 0 else 'reverse'} {abs(v):.2f}m/s "
                f"uphill.fwd={s:+.2f} esdf_at_robot={self._esdf_at(esdf_map, init_p):.2f}m "
                f"spent={self._reverse_spent_m:.2f}m/{self.reverse_budget_m:.2f}m")
        return path

    def _forward_of(self, q):
        x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        fx = 2.0 * (x * z + w * y)
        fy = 2.0 * (y * z - w * x)
        n = math.hypot(fx, fy)
        return (fx / n, fy / n) if n > 1e-6 else (1.0, 0.0)

    def _worst_footprint_point(self, esdf_map, init_p, init_q):
        """车体取样点里 ESDF 最小的那个 —— 侵入最深处，梯度在那里取才有意义。
        取样铺满方式和 score_trajectories_by_ESDF 一致，别让两处判据不同。"""
        if self.robot.is_circle:
            return (float(init_p[0]), float(init_p[1]))
        fl, rl, hw = self.robot.footprint_from_control()
        fwd = self._forward_of(init_q)
        left = (-fwd[1], fwd[0])
        n_a = max(2, int(math.ceil((fl + rl) / self.resolution)) + 1)
        n_c = max(2, int(math.ceil(2.0 * hw / self.resolution)) + 1)
        best, best_d = None, float('inf')
        for ia in range(n_a):
            oa = -rl + (fl + rl) * ia / (n_a - 1)
            for ic in range(n_c):
                oc = -hw + 2.0 * hw * ic / (n_c - 1)
                px = float(init_p[0]) + fwd[0] * oa + left[0] * oc
                py = float(init_p[1]) + fwd[1] * oa + left[1] * oc
                d = self._esdf_at(esdf_map, (px, py))
                if not math.isnan(d) and d < best_d:
                    best_d, best = d, (px, py)
        return best

    def _esdf_gradient(self, esdf_map, p):
        """中心差分并归一化。返回 (None, None) 表示点不在格内、贴边、或场是平的。"""
        if p is None:
            return None, None
        i = int((p[0] - self.origin[0]) / self.resolution)
        j = int((p[1] - self.origin[1]) / self.resolution)
        if not (1 <= i < esdf_map.shape[0] - 1 and 1 <= j < esdf_map.shape[1] - 1):
            return None, None
        gx = (float(esdf_map[i + 1, j]) - float(esdf_map[i - 1, j])) / (2.0 * self.resolution)
        gy = (float(esdf_map[i, j + 1]) - float(esdf_map[i, j - 1])) / (2.0 * self.resolution)
        n = math.hypot(gx, gy)
        return (gx / n, gy / n) if n >= 1e-6 else (None, None)

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

    def sync_callback(self, depth_msg, pose_msg):
        """只存最新的一对，真正的规划在 _plan_tick 里做。

        🔴 message_filters 是 FIFO：在回调里直接干活，就永远在处理队列头那个【最旧的、
        还没过期的】集合，即使后面已经躺着一个新 0.2 s 的。实测 /slam/depth 到手时
        数据龄 163 ms，而决策做出时 stamp_lag 已经 590 ms —— 中间 ~230 ms 纯粹是排队。
        队列深度不能动（30 改 3 那次把发布间隔从 1.17 s 打到 3.2 s，车停了），
        所以改的是取用策略：回调只赋值，让队列全速排空，定时器永远拿最新那一个。
        """
        if not self._plan_latest_only:
            self._plan_measured(depth_msg, pose_msg)
            return
        self._pending_set = (depth_msg, pose_msg)

    def _plan_tick(self):
        pending, self._pending_set = self._pending_set, None
        if pending is not None:
            self._plan_measured(*pending)

    def _lat_age(self, stamp) -> float:
        now = self.get_clock().now().nanoseconds
        return (now - (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9

    def _plan_measured(self, depth_msg, pose_msg):
        """深度和位姿是两条独立的戳，必须分开记。

        位姿由 diffcar_control 在【发布时】打 now，所以 p_in 不含任何传感器延迟，
        纯粹是 ROS 侧的传输+排队；深度的戳是相机采集时刻，d_in 含双目计算。
        混成一个数会把这两件完全不同的事算到一起 —— 我就栽过。
        """
        if not self._lat.enabled:
            self._plan_once(depth_msg, pose_msg)
            return
        d_in = self._lat_age(depth_msg.header.stamp)
        p_in = self._lat_age(pose_msg.header.stamp)
        self._lat.add('d_in', d_in)
        self._lat.add('p_in', p_in)
        self._lat.add('skew', d_in - p_in)
        self._plan_once(depth_msg, pose_msg)
        self._lat.add('d_out', self._lat_age(depth_msg.header.stamp))
        self._lat.tick()

    @Timer(name="Planning Loop", text="\n\n[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER)
    def _plan_once(self, depth_msg, pose_msg):
        if self.K is None:
            return
        age_s = (self.get_clock().now() - Time.from_msg(pose_msg.header.stamp)).nanoseconds / 1e9
        self._last_entry_age_s = age_s
        if age_s > self.max_input_age_s:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_stale_log_ns >= 1_000_000_000:
                self._last_stale_log_ns = now_ns
                self.get_logger().warning(
                    f"dropping stale synced set: {age_s:.2f}s old (limit {self.max_input_age_s:.2f}s) -- "
                    "planning is not keeping up with the depth rate"
                )
            self._stale_in_win += 1
            return
        self._log_stage_timing(self.get_clock().now().nanoseconds,
                               Time.from_msg(depth_msg.header.stamp).nanoseconds / 1e9)
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
            if self._pose_prev is not None:
                pdt = max(stamp - self._pose_prev[0], 1e-3)
                pmv = float(np.linalg.norm(robot_pos[:2] - self._pose_prev[1]))
                if pmv > self._pose_jump_speed * pdt:
                    self.get_logger().error(
                        f"位姿瞬移 {pmv:.2f}m/{pdt:.2f}s = {pmv / pdt:.2f} m/s "
                        f"(上限 {self._pose_jump_speed:.2f}) —— 世界系变了，整张障碍图作废，"
                        f"接下来 {self._blind_s:.1f}s 不前进")
                    self.occupancy_grid[:] = 0.0
                    self._low_obstacle[:] = 0.0
                    self._obstacle_first_seen[:] = 0.0
                    self._last_decay_stamp = None
                    self._blind_until = stamp + self._blind_s
            self._pose_prev = (stamp, robot_pos[:2].copy())
            delta = robot_pos - center
            if np.linalg.norm(delta) > .1:
                new_center = robot_pos
                new_origin = (new_center - np.array(self.grid_shape) * self.resolution / 2
                              + self.grid_offset)
                shift_xy = np.round((new_origin[:2] - self.origin[:2]) / self.resolution).astype(int)
                self.occupancy_grid, self.origin = roll_occupancy_grid(self.occupancy_grid, self.origin, new_origin, self.resolution)
                self._low_obstacle = _roll_2d(self._low_obstacle, shift_xy)
                # 🔴 年龄数组也得跟着挪。以前没挪：车一走栅格就整体平移，年龄数组留在
                # 原索引上，于是"连续存在时长"在移动时恒为 0 —— 是记账假象，不是图在闪。
                self._obstacle_first_seen = _roll_2d(self._obstacle_first_seen, shift_xy)
                self._obstacle_last_seen = _roll_2d(self._obstacle_last_seen, shift_xy)
            if self.carve_step == self.step:
                new_occ = run_raycasting_loopy(depth, T, self.grid_shape, fx, fy, cx, cy,
                                               self.origin, self.step, self.resolution)
            else:
                new_occ = run_raycasting_split(depth, T, self.grid_shape, fx, fy, cx, cy,
                                               self.origin, self.step, self.carve_step,
                                               self.resolution)
            # 夹在 [1, 8] 个参考周期内：丢帧后 dt 可能很大，不夹会把整张图一次抹掉；
            # 而 dt 反常地小（时间戳倒退）时至少衰减一次。
            dt_decay = (self._decay_ref_dt if self._last_decay_stamp is None
                        else min(max(stamp - self._last_decay_stamp, self._decay_ref_dt),
                                 8.0 * self._decay_ref_dt))
            self._last_decay_stamp = stamp
            n_ref = dt_decay / self._decay_ref_dt
            self.occupancy_grid *= 0.99 ** n_ref
            self.occupancy_grid += new_occ
            # In place: np.clip without out= allocated a fresh grid every cycle and
            # dropped the old one. On this board that is pure DDR traffic in the loop
            # that is already the latency bottleneck, and the bus loss measurements
            # (docs/x5/servo_bus.md) make memory bandwidth a suspect in its own right.
            np.clip(self.occupancy_grid, -0.2, 0.2, out=self.occupancy_grid)

            if self.low_obs_enabled:
                hits = low_obstacle_hits(
                    depth, T, self.grid_shape, fx, fy, cx, cy, self.origin,
                    self.step, self.resolution, T[2, 3] - self.camera_height_m,
                    self.low_obs_h_lo, self.low_obs_h_hi, self.low_obs_max_range_m)
                self._low_obstacle *= 0.9 ** n_ref
                self._low_obstacle[hits >= self.low_obs_min_pts] += 0.1
                np.clip(self._low_obstacle, 0.0, 0.2, out=self._low_obstacle)
            # Nothing but the app's 3D local view consumes this, so it follows the same
            # gate as the overlays below rather than get_subscription_count() -- see
            # _ui_active for why the count cannot answer the question.
            # 只要局部视图开着就发，而它只在 3D 模式下被画 —— 「要不要 3D」那个标志
            # (_want_voxels) 只在后端，planning 不知道。所以给一个总开关：3D 视图用得少，
            # 关掉是纯削减。TINYNAV_PUBLISH_VOXELS=0 关。
            if self._ui_active and _PUBLISH_VOXEL_CLOUD:
                with Timer(name='vis:voxels', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
                    self.publish_3d_occupancy_cloud(self.occupancy_grid, self.resolution, self.origin)

        # 每周期都看，包括没有目标的时候 —— 卡住之后要回溯的正是卡住之前走过的地方。
        self._record_centre(self.camera_to_robot_center(T),
                            self.get_clock().now().nanoseconds)

        with Timer(name='obstacle map', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            obstacle_mask = build_obstacle_map(
                self.occupancy_grid, self.origin, self.resolution,
                robot_z=T[2, 3], config=self.obstacle_config,
            )
            if self.low_obs_enabled:
                obstacle_mask = obstacle_mask | (self._low_obstacle > 0.1)
            if not self._min_vis_logged:
                self._min_vis_logged = True
                self.get_logger().info(
                    f"obstacle 最小可见高度: "
                    f"{self._min_visible_height_m(self.origin[2], T[2, 3]) * 1000:.0f} mm "
                    f"(离地; 低于它的东西 z 跨度不够，恒不是障碍) "
                    f"origin_z={self.origin[2]:+.3f} grid_offset_z={self.grid_offset[2]:.4f} "
                    f"cam_h={self.camera_height_m:.3f}")
            self._track_obstacle_age(obstacle_mask)
            # 有符号：障碍外为正、内部为负。无符号版在障碍内部恒为 0，邻域也是 0，于是
            # _esdf_gradient 的中心差分退化成 0，_gradient_escape 拿不到"往哪出去"。
            ESDF_map = ((distance_transform_edt(~obstacle_mask)
                         - distance_transform_edt(obstacle_mask))
                        * self.resolution).astype(np.float32)
            # Before the no-target return below, so the UI keeps reading a clearance
            # while the robot is parked -- which is exactly when you want to know
            # whether it thinks something is in front of it.
            front_clearance = self._front_obstacle_dist(
                T, obstacle_mask, self.front_probe_max_m)
            self._publish_diagnostics(front_clearance, obstacle_mask, ESDF_map, T, stamp)

        with Timer(name='vis', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            # 判决图要 obstacle_mask，所以必须在 build_obstacle_map 之后 —— 以前它在
            # raycasting 段里、拿不到 mask。放在 vis 段里计时归属也才对。
            with Timer(name='vis:verdict', logger=None):
                self._publish_height_color(depth, T, fx, fy, cx, cy,
                                           T[2, 3] - self.camera_height_m,
                                           depth_msg.header.stamp, obstacle_mask)
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
                num_samples = self.traj_samples,
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
            route_xy = self._route_xy if self.route_cost_enabled else None
            path_dist_map, remaining_map, has_route = build_route_fields(
                route_xy, ESDF_map.shape, self.origin, self.resolution)
            _rb_arc, _rb_min = self._route_block_ahead(ESDF_map, route_xy, init_p)
            route_unpinned = bool(
                self.route_unpin_on_block and has_route
                and _rb_min == _rb_min and _rb_min < self.robot.hard_clearance)
            w_path_follow = 0.0 if route_unpinned else self.w_path_follow
            scores, occ_points, path_costs, end_remainings = self._score_trajectories(
                trajectories, ESDF_map, params, path_dist_map, remaining_map)

        with Timer(name='pub', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=_TIMER_LOGGER):
            # "前方堵住" = 连最慢的那档前进都过不了视距门限。轨迹库把速度采成
            # linspace(0, max_vx, 7)，所以最慢的非零档是 max_vx/6。留一个下限：低于它就
            # 别再试前进了，交给转向/后退。
            # 「前方堵住」= 一条前进轨迹都不可行，不再是一条直线探针跨过某个数字。
            # 这同时去掉了一个抖动源：旧判据在阈值附近来回跨越，堵/不堵两个状态的可行集
            # 完全不同，于是车在两套动作之间跳（2026-08-25 实测摆了 10 s）。
            _t_prefix = Timer(name='pub:prefix', logger=None); _t_prefix.start()
            prefix_clear = self._prefix_clearance(trajectories, ESDF_map)
            forward_ok = prefix_clear >= self._prefix_gate_m()
            fwd_idx = [i for i in range(len(params))
                       if params[i][0] > 0.0 and not self._is_turn_in_place(params[i])]
            n_fwd_ok = sum(1 for i in fwd_idx if forward_ok[i])
            front_blocked = self._blocked_latched(
                not any(forward_ok[i] for i in fwd_idx), n_fwd_ok)
            _t_prefix.stop()

            def heading_cost(traj, target_pose):
                """轨迹终点处「车头朝向」与「终点到目标的方位」之差的绝对值（弧度）。

                对位移轨迹这一项几乎不变（弧线终点朝向≈行进方向），对原地转却是唯一有意义
                的判据 —— 见 self.w_heading 的注释。"""
                if target_pose is None:
                    return 0.0
                ex, ey = float(traj[-1, 0]), float(traj[-1, 1])
                qx, qy, qz, qw = (float(traj[-1, 3]), float(traj[-1, 4]),
                                  float(traj[-1, 5]), float(traj[-1, 6]))
                fx = 2.0 * (qx * qz + qw * qy)
                fy = 2.0 * (qy * qz - qw * qx)
                if fx * fx + fy * fy < 1e-12:
                    return 0.0
                yaw_end = math.atan2(fy, fx)
                dx, dy = float(target_pose[0]) - ex, float(target_pose[1]) - ey
                # 目标很近时"到目标的方位"纯是噪声，1 mm 的门限等于没有门限：
                # 2026-09-02 实测目标只差 0.067 m（相机光心与控制中心的偏置）时，
                # 这一项仍是原地转的唯一区分项，把车带得持续慢转 127 拍。
                if dx * dx + dy * dy < self.heading_min_dist_m ** 2:
                    return 0.0
                return abs(self._wrap(math.atan2(dy, dx) - yaw_end))

            # 分项留痕。「为什么选了这条」以前完全不可观测，只能靠离线复算猜，而内部 ESDF
            # 和发布出去的 obstacle_mask 不是同一份，复算必然对不上。
            cost_parts = [None] * len(trajectories)
            # 圆盘的整条轨迹中心最小净空 —— 和内核的碰撞判据同源（内核对圆也是中心单点）。
            # 🔴 从第 1 步起，不含起点。轨迹是先积分再存，所以第 0 点是 0.1 s 之后而不是
            # 当前位姿本身 —— 但最快那档一步只走 0.04 m，不到一格（0.05 m），而 ESDF 是
            # 按格量化的，于是只要车已经贴着障碍（当前格就是全程最紧的点），min 取到的
            # 就是同一格的同一个值，对 110 条候选完全相同 —— 势垒退化成常数。
            # 圆盘才会这样：碰撞判据是中心单点。main 的矩形底盘查 4 个会转的角点，
            # 第 0 步在候选之间本来就不同，所以那边这个退化被天然稀释了。
            # 2026-09-03 仿真实测：route_thru_wall 卡住时 110 条候选全是
            # `clr=0.200 obst=25`，唯一还在区分的是朝向项，于是车原地转到超时（121 次变向）。
            # 势垒本该回答"往哪走更安全"，含起点时它只能回答"我现在危不危险"。
            _c0 = 0 if self.barrier_include_start else 1
            clear_min = (self._esdf_lookup(ESDF_map, trajectories[:, _c0:, 0],
                                           trajectories[:, _c0:, 1]).min(axis=1)
                         if self.robot.is_circle else None)

            def obstacle_cost(idx, score):
                """soft 处连续归零、往 hard 发散的势垒。方形底盘保持原样不动。"""
                if score == float('inf'):
                    return float('inf')
                if clear_min is None:
                    return score * 100000
                d = float(clear_min[idx])
                soft, hard = self.robot.soft_clearance, self.robot.hard_clearance
                if not math.isfinite(d) or d >= soft:
                    return 0.0
                return self.w_obstacle * (1.0 / (max(d, hard) - hard + 1e-3)
                                          - 1.0 / (soft - hard + 1e-3))

            idle_scale = min(1.0, max(0.0,
                (target_dist_xy - self.idle_free_m) / max(self.idle_free_m, 1e-6)))

            def cost_function(traj, param, score, target_pose, idx):
                gate_penalty = self._motion_gate_penalty(
                    param, idx, forward_ok, front_blocked)
                # ⚠️ 40 是 6754818「Add planning controller debug tools」里顺带进来的，
                # 提交信息没提平滑度，所以它没有依据（main 是 10/10）。两个已知问题：
                # (1) abs 让加速和减速罚一样多 —— 「别猛加速」有理，「别减速」没理；
                # (2) omega 只罚 10，左右翻一次 3.4 分 ≈ 3.4 cm 路程，压不住摆头。
                smoothness = (40 * abs(self.last_param[0] - param[0])
                              + 10 * abs(self.last_param[1] - param[1]))
                # 原地转和完全停住拿到同一个值，所以它们内部的相对排名不变。
                idle = self.w_idle * idle_scale if abs(param[0]) < self.idle_vx_eps else 0.0

                if not has_route:
                    # 没有路线就退回原来的贪心终点距离。
                    # xy only. The target carries a camera height ~0.7 m above the
                    # trajectory plane, and sqrt(dxy^2 + 0.7^2) compresses the ranking to
                    # nothing near the goal: 0.1 m and 0.3 m of real error scored 0.707
                    # against 0.762, so continuity outweighed goal-seeking.
                    traj_end = np.array(traj[-1, :3])
                    target_end = target_pose if target_pose is not None else traj_end
                    dist = np.linalg.norm(traj_end[:2] - target_end[:2])
                    hd = self.w_heading * heading_cost(traj, target_pose)
                    ob = obstacle_cost(idx, score)
                    cost_parts[idx] = (ob, 100 * dist, 0.0, smoothness, gate_penalty, hd, idle)
                    return ob + 100 * dist + smoothness + gate_penalty + hd + idle

                # 有路线：按"沿路线还剩多远"算进展，按"离路线最远多少"算贴合度。
                # 直线距离在绕障时是个局部极小，这两项没有 —— 路线本身已经绕过去了。
                terminal = 0.0
                if end_remainings[idx] < self.route_terminal_band and target_pose is not None:
                    terminal = self.w_goal_terminal * float(
                        np.linalg.norm(traj[-1, :2] - target_pose[:2]))
                hd = self.w_heading * heading_cost(traj, target_pose)
                ob = obstacle_cost(idx, score)
                cost_parts[idx] = (ob,
                                   self.w_route_progress * end_remainings[idx] + terminal,
                                   w_path_follow * path_costs[idx],
                                   smoothness, gate_penalty, hd, idle)
                return (ob
                        + self.w_route_progress * end_remainings[idx]
                        + w_path_follow * path_costs[idx]
                        + terminal
                        + smoothness
                        + gate_penalty
                        + hd
                        + idle)

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

            if stamp < self._blind_until:
                self._publish_static_path(
                    init_p, init_q, depth_msg.header, base_time, len(trajectories[0]),
                    f"位姿刚瞬移，障碍图重建中（还剩 {self._blind_until - stamp:.1f}s）"
                    f"—— 这时候的空白地图不代表没有障碍",
                    log_key="blind after jump",
                )
                return

            n_blocked = int(sum(1 for s in scores if s == float('inf')))
            if n_blocked == len(scores):
                # 预制弧全灭 != 无路可走。从 ESDF 场解一个脱困方向出来再说。
                esc = self._gradient_escape(ESDF_map, init_p, init_q, depth_msg.header,
                                            base_time, len(trajectories[0]))
                if esc is not None:
                    self.path_pub.publish(esc)
                    self._last_cycle_ns = self.get_clock().now().nanoseconds
                    return
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

            # 走到这里说明至少有一条轨迹可选。真正的复位放到选定动作之后 —— 这里还不知道
            # 选出来的会不会又是倒车。
            if self._reverse_anchor is not None and self._reverse_spent_m > 0.0:
                self.get_logger().info(
                    f"gradient escape 结束：共退 {self._reverse_spent_m:.2f}m，"
                    f"现在有 {len(scores) - n_blocked}/{len(scores)} 条可行轨迹")
            stand_dist = target_dist_xy
            yaw_now = self._yaw_of(init_q)
            to_t = target_pose[:2] - init_p[:2]
            yaw_to_target = math.atan2(float(to_t[1]), float(to_t[0]))

            top_k = 1
            with Timer(name='pub:cost', logger=None):
                costs = np.array([cost_function(trajectories[i], params[i], scores[i],
                                                self.target_pose, i)
                                  for i in range(len(trajectories))])
            top_indices = np.argsort(costs, kind='stable')[:top_k]
            turned_in_place = False
            long_way = False

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
                retreat_idx = (self._retreat_index(params, trajectories, now_ns)
                               if self._reverse_ok(init_p) else None)
                if retreat_idx is None:
                    # 车站得越久，来路证据越少（_centre_history 收缩成一个点），最需要退的
                    # 时候恰好退不了。ESDF 梯度不依赖来路，所以这里也走一次脱困。
                    esc = self._gradient_escape(ESDF_map, init_p, init_q,
                                                depth_msg.header, base_time,
                                                len(trajectories[0]))
                    if esc is not None:
                        self.path_pub.publish(esc)
                        self._last_cycle_ns = self.get_clock().now().nanoseconds
                        return
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
                self._reverse_mark(init_p)
            else:
                if has_route:
                    # 有路线时进展要沿路线算。直线距离在绕障时本来就会"越走越远"，
                    # 用它判 no-progress 会在正确绕行的路上误触发脱困。
                    here = np.asarray(init_p[:2], dtype=np.float64)
                    hx = int((here[0] - self.origin[0]) / self.resolution)
                    hy = int((here[1] - self.origin[1]) / self.resolution)
                    if (0 <= hx < remaining_map.shape[0] and 0 <= hy < remaining_map.shape[1]
                            and remaining_map[hx, hy] < 1e3):
                        best_gain = float(remaining_map[hx, hy]) - float(
                            min(end_remainings[i] for i in admissible))
                    else:
                        best_gain = 0.0
                else:
                    ends_xy = np.array([trajectories[i][-1, :2] for i in admissible])
                    best_gain = stand_dist - float(np.min(np.linalg.norm(
                        ends_xy - target_pose[None, :2], axis=1)))
                escape_reason = self._escape_reason(
                    front_blocked, stand_dist,
                    abs(self._wrap(yaw_to_target - yaw_now)), best_gain)
                escape_reason = self._noprogress_budget(escape_reason, now_ns)

                if escape_reason and turns:
                    # 连续计时，不是"上次脱困以来"。全灭/无解那两条分支是提前 return 的，
                    # 不经过下面的清零，于是这个时钟能跨过 22 分钟的冻结一直涨 —— 实测涨到
                    # 1360 s，车一解冻就直接判"摆太久了，退"。
                    if (not self._escape_episode_ns
                            or (now_ns - self._escape_last_ns) / 1e9 > self._escape_episode_gap_s):
                        self._escape_episode_ns = now_ns
                    self._escape_last_ns = now_ns
                    escape_age_s = (now_ns - self._escape_episode_ns) / 1e9
                    centre = self.camera_to_robot_center(T)
                    expired = (self._escape_goal_ns and
                               (now_ns - self._escape_goal_ns) / 1e9 > self._escape_goal_max_s)
                    goal_err = (self._wrap(self._escape_goal_yaw - yaw_now)
                                if self._escape_goal_yaw is not None else 0.0)
                    reached = abs(goal_err) < self._escape_goal_reached_rad
                    if self._escape_goal_yaw is None or expired or reached:
                        # 🔴 escape=heading 要转向**目标**，不能用 _open_heading（2026-09-01
                        # 实测定案）。_open_heading 是从当前朝向往外扫、返回第一个够空的方向，
                        # 是为"前方堵住、找最近的出口"设计的；而 heading 这一支恰恰是**前方
                        # 开着**（实测 39 次里 blocked 中位只有 42/110），于是 step=0 就命中，
                        # 返回的就是当前朝向本身 —— 日志里 `goal=-1deg yaw=-1deg
                        # to_target=+127deg` 就是它。goal_err 立刻为 0 -> reached -> 带
                        # min_deg=12° 重扫 -> 得到 yaw±12° -> 转 12° -> 又 reached，成了一个
                        # 朝目标那侧的 12° 棘轮：111 s 累计转 3147°、净转 1142°，而 vx=0
                        # 占了 65% 的决策。转向目标之后 heading_err 落回门限内，脱困自然退出。
                        if escape_reason == "heading":
                            g = yaw_to_target
                            escape_clear = float(self._clearance_along(
                                centre, math.cos(g), math.sin(g), obstacle_mask))
                        else:
                            # 转到位了但前面还是堵着，说明这个朝向选错了 —— 重新扫，且要求
                            # 新朝向至少偏出"已到位"的容差，否则会反复选中同一个朝向。
                            g, escape_clear = self._open_heading(
                                centre, obstacle_mask, yaw_now, yaw_to_target,
                                min_deg=(math.degrees(self._escape_goal_reached_rad)
                                         if reached and self._escape_goal_yaw is not None
                                         else 0))
                        if g is None:
                            # 一圈都没有能证明是空的朝向。朝一边扫 90°，别站着。方向只在
                            # 本次脱困第一次进到这里时定，之后一直沿用。
                            if self._escape_sweep_side is None:
                                self._escape_sweep_side = (
                                    1.0 if self._wrap(yaw_to_target - yaw_now) >= 0.0
                                    else -1.0)
                            g = self._wrap(yaw_now
                                           + self._escape_sweep_side * math.pi / 2.0)
                        self._escape_goal_yaw = g
                        self._escape_goal_ns = now_ns
                        goal_err = self._wrap(g - yaw_now)
                    else:
                        escape_clear = float(self._clearance_along(
                            centre, math.cos(self._escape_goal_yaw),
                            math.sin(self._escape_goal_yaw), obstacle_mask))
                    pick, long_way = self._pick_turn_toward(turns, params, goal_err)
                    # 有转向可选 != 转了能出去。2026-08-25 18:50 实测：turns 只剩 1~2 条、
                    # 每条的净空只有 0.04~0.08 m，车在那儿摆了 42 s。净空不够就别摆了，
                    # 退回自己刚走过的地方 —— main 对「前方堵住」的答案本来就是倒车
                    # （硬互斥门 front_clearance<=0.3 时只准倒车），它缺的只是「后面能不能
                    # 走」的凭据，那正是 _retreat_index 提供的。
                    if (self._should_retreat(front_blocked, escape_clear, escape_age_s)
                            and self._reverse_ok(init_p)):
                        retreat_idx = self._retreat_index(params, trajectories, now_ns)
                        if retreat_idx is not None:
                            top_indices = np.array([retreat_idx])
                            retreating = True
                            escape_reason = "retreat"
                            self._reverse_mark(init_p)
                    if not retreating and pick is not None:
                        top_indices = np.array([pick])
                        turned_in_place = True
                        self._retreat_start_ns = 0
                    elif not retreating:
                        # 朝锁定方向的原地转这一拍都撞。原地不动一拍，下一拍再试 —— 绝不
                        # 反向：反向正是把累计转角清零、变成左右摆头的那个动作。
                        self._publish_static_path(
                            init_p, init_q, depth_msg.header, base_time,
                            len(trajectories[0]),
                            f"escape turn toward {math.degrees(self._escape_goal_yaw):+.0f}deg "
                            f"blocked this cycle (goal_err={math.degrees(goal_err):+.0f}deg, "
                            f"turns={len(turns)}) -- no admissible in-place turn either way",
                            log_key="escape turn blocked")
                        self._last_cycle_ns = now_ns
                        return
                else:
                    self._retreat_start_ns = 0
                    self._escape_episode_ns = 0
                    # 朝向锁**不**在这里松。门是逐轨迹判的，车一转它就在 blocked/forward
                    # 之间抖，于是 escape_reason 每隔一拍就空一次；在这里松锁等于每隔一拍
                    # 重选一个朝向，而墙正前方时 ±83° 是对称的，重选就会换边 —— 仿真
                    # wall_ahead 场景实测就是 -83/+82 交替。锁只在车**真的往前走了**之后松，
                    # 见下面 best_idx 处。

            self.last_param = params[top_indices[0]]
            best_idx = int(top_indices[0])
            if params[best_idx][0] >= 0.0:
                self._reverse_release()
            if params[best_idx][0] > 1e-6:
                # 真的选了一条往前走的轨迹，这次脱困才算结束。纯转向（vx=0）不算 ——
                # 它没有把车带离那个位置，松锁只会让下一拍重新挑边。
                if self._escape_goal_yaw is not None:
                    self.get_logger().info(
                        f"escape done: forward motion available again "
                        f"(vx={params[best_idx][0]:+.3f}), releasing heading lock "
                        f"{math.degrees(self._escape_goal_yaw):+.0f}deg")
                self._escape_goal_yaw = None
                self._escape_goal_ns = 0
                self._escape_sweep_side = None
            _dec_gap_ns = 0 if self.decision_log_hz <= 0 else int(1e9 / self.decision_log_hz)
            _t_log = Timer(name='pub:log', logger=None)
            if now_ns - self._last_static_log_ns.get("decision", 0) >= _dec_gap_ns:
                _t_log.start()
                cycle_s = (now_ns - self._last_cycle_ns) / 1e9 if self._last_cycle_ns else float('nan')
                self._last_static_log_ns["decision"] = now_ns
                self.get_logger().info(
                    f"decision: {'RETREAT ' if retreating else ''}"
                    # omega 打**世界系**的值。原来打的是 params 里绕相机 Y 轴那个，
                    # 和同一行的 yaw/to_target/heading_err/goal（全是世界系）差一个负号
                    # —— 2026-09-01 我据此误判了两轮，以为是跟踪器把符号弄反了。
                    f"{'TURN-IN-PLACE ' if turned_in_place else ''}chose vx={params[best_idx][0]:+.3f} "
                    f"omega={self._world_yaw_rate(params[best_idx]):+.3f} "
                    f"(cap {self.robot.max_vx:.2f}) blocked={n_blocked}/{len(scores)} "
                    f"front_clearance={self._fmt_clearance(front_clearance, self.front_probe_max_m)} "
                    f"gate={'turn-only' if front_blocked else 'forward'} "
                    f"fwd_ok={int(np.count_nonzero(forward_ok[fwd_idx]))}/{len(fwd_idx)} "
                    f"escape={escape_reason or 'off'} turns={len(turns)} "
                    f"goal={'-' if self._escape_goal_yaw is None else f'{math.degrees(self._escape_goal_yaw):+.0f}deg'}"
                    f"{'(long way)' if turned_in_place and long_way else ''} "
                    f"rev={'on' if self.allow_reverse else 'OFF'}"
                    f"{'' if self._reverse_anchor is None else f'({self._reverse_spent_m:.2f}/{self.reverse_budget_m:.2f}m)'} "
                    f"escape_age={((now_ns - self._escape_episode_ns) / 1e9) if self._escape_episode_ns else 0.0:.1f}s "
                    f"hist={len(self._centre_history)}pts "
                    f"best_gain={best_gain:+.2f}m escape_clear={self._fmt_clearance(escape_clear)} "
                    # Was only logged on the two give-up branches, so a run could not
                    # be read for whether the z band and dilation did what they claim.
                    f"obstacle_cells={int(np.count_nonzero(obstacle_mask))} "
                    f"esdf_at_robot={self._esdf_at(ESDF_map, init_p):.2f}m "
                    f"yaw={math.degrees(yaw_now):+.0f}deg to_target={math.degrees(yaw_to_target):+.0f}deg "
                    f"heading_err={math.degrees(self._wrap(yaw_to_target - yaw_now)):+.0f}deg "
                    f"cycle={cycle_s:.2f}s stamp_lag={(now_ns / 1e9 - stamp):.2f}s "
                    # 进回调时就已经这么老 = 相机+bridge+排队；stamp_lag 减它 = 本节点计算
                    f"pose_age={self._last_entry_age_s:.2f}s "
                    # 路线自己被挡在前方多远（inf=没挡）、以及这段路线上的最小净空。
                    f"route_block={_rb_arc:.2f}m route_clear={_rb_min:.2f}m"
                    f"{' ROUTE-UNPINNED' if route_unpinned else ''}"
                )
                def _cand(i):
                    o, pr, pf, sm, g, h, idl = cost_parts[i]
                    cl = (f" clr={clear_min[i]:.3f}" if clear_min is not None else "")
                    return (f"vx={params[i][0]:+.3f} w={self._world_yaw_rate(params[i]):+.3f}"
                            f"{cl} obst={o:.0f} prog={pr:.0f} path={pf:.0f} sm={sm:.1f}"
                            f" gate={g:.0e} head={h:.1f} idle={idl:.0f} = {costs[i]:.0f}")
                rank = [int(i) for i in np.argsort(costs, kind='stable')
                        if cost_parts[i] is not None]
                self.get_logger().info(
                    "candidates: " + " | ".join(_cand(i) for i in rank[:4]))
                # 前进轨迹单独排一次：整体前几名常常被 vx=0 那一排 15 条占满，看不到
                # 「最好的前进选项差在哪一项」——而那才是「该走不走」唯一要看的东西。
                fwd_rank = [i for i in rank if params[i][0] > 0.0][:4]
                if fwd_rank:
                    self.get_logger().info(
                        "best forward: " + " | ".join(_cand(i) for i in fwd_rank))
                _t_log.stop()
            self._last_cycle_ns = now_ns

            _t_emit = Timer(name='pub:emit', logger=None); _t_emit.start()
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
            _t_emit.stop()
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
