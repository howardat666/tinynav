import rclpy
import os
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
import numpy as np
import math
import sys
import json

import heapq
from tinynav.core.nav_search_kernels import (
    search_close_to_sdf_map_numba,
    search_within_sdf_map_numba,
)
from tinynav.core.math_utils import matrix_to_quat, msg2np, np2msg, estimate_pose, np2tf, se3_inv
from sensor_msgs.msg import Image, CameraInfo
from message_filters import TimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import cv2
from codetiming import Timer
import argparse

from tinynav.tinynav_cpp_bind import pose_graph_solve
from tinynav.core.models_trt import (
    DBoW3Engine,
    Dinov2TRT,
    LightGlueTRT,
    ORBFeatureTRTCompatible,
    ORBMatcher,
    SuperPointMatcher,
    SuperPointTRT,
    make_sp_extractor,
)
import logging
import asyncio
import threading
import time
from tf2_ros import TransformBroadcaster
from tinynav.core.build_map_node import DEFAULT_VLAD_CENTRES, LOOP_CLOSURE_DEFAULTS, load_vlad_centres, TinyNavDB
from tinynav.core.build_map_node import solve_pose_graph
import einops
from tinynav.core.build_map_node import OdomPoseRecorder, LoopClosure
logger = logging.getLogger(__name__)



def draw_image_match_origin(prev_image: np.ndarray, curr_image: np.ndarray, prev_keypoints: np.ndarray, curr_keypoints: np.ndarray, matches: np.ndarray):
    cv_matches = [cv2.DMatch(_queryIdx=matches[index, 0].item(), _trainIdx=matches[index, 1].item(), _imgIdx=0, _distance=0) for index in range(matches.shape[0])]
    # convert kpts_prev and kpts_curr to cv2.KeyPoint
    cv_kpts_prev = [cv2.KeyPoint(x=prev_keypoints[index, 0].item(), y=prev_keypoints[index, 1].item(), size=20) for index in range(prev_keypoints.shape[0])]
    cv_kpts_curr = [cv2.KeyPoint(x=curr_keypoints[index, 0].item(), y=curr_keypoints[index, 1].item(), size=20) for index in range(curr_keypoints.shape[0])]
    output_image = cv2.drawMatches(prev_image, cv_kpts_prev, curr_image, cv_kpts_curr, cv_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    return output_image

def depth_to_cloud(depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """
    Convert depth image to point cloud.
    :param depth: (H, W) depth image.
    :param K: (3, 3) camera intrinsic matrix.
    :return: (N, 3) point cloud in camera coordinates.
    """
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth.flatten()

    x = (u.flatten() - K[0, 2]) * z / K[0, 0]
    y = (v.flatten() - K[1, 2]) * z / K[1, 1]

    points_3d = np.vstack((x, y, z)).T
    return points_3d[~np.isnan(points_3d).any(axis=1)]

def transform_point_cloud(point_cloud: np.ndarray, T: np.ndarray) -> np.ndarray:
    """
    Transform a point cloud with a transformation matrix.
    :param point_cloud: (N, 3) numpy array of points in the point cloud.
    :param T: (4, 4) transformation matrix.
    :return: (N, 3) transformed point cloud.
    """
    assert point_cloud.shape[1] == 3, "Point cloud must be of shape (N, 3)"
    assert T.shape == (4, 4), "Transformation matrix must be of shape (4, 4)"

    # Convert to homogeneous coordinates
    ones = np.ones((point_cloud.shape[0], 1))
    homogeneous_points = np.hstack((point_cloud, ones))
    # Apply transformation
    transformed_points = homogeneous_points @ T.T
    return transformed_points[:, :3]

def heuristic(start, goal, resolution):
    vec_start = np.array(start)
    vec_goal = np.array(goal)
    return np.linalg.norm((vec_start - vec_goal) * resolution) + 20 * np.abs(vec_start[2] - vec_goal[2]) * resolution

def reconstruct_path_sdf(parent:dict, current:tuple):
    path = []
    while current in parent:
        path.append(current)
        if current == parent[current]:
            break
        current = parent[current]
    return path[::-1]

def search_close_to_sdf_map(start_index:tuple, sdf_map:np.ndarray, occupancy_map:np.ndarray, stop_distance:np.ndarray):
    start_index = tuple(start_index.flatten()) if isinstance(start_index, np.ndarray) else start_index
    open_heap = [(sdf_map[start_index], start_index)]
    open_heap_set = set()
    open_heap_set.add(start_index)
    parent = {start_index: start_index}
    visited = set()
    while len(open_heap) > 0:
        current_sdf, current = heapq.heappop(open_heap)
        open_heap_set.remove(current)
        visited.add(current)
        if current_sdf < stop_distance:
            return reconstruct_path_sdf(parent, current)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                    if (0 <= neighbor[0] < sdf_map.shape[0] and
                            0 <= neighbor[1] < sdf_map.shape[1] and
                            0 <= neighbor[2] < sdf_map.shape[2]):
                        if neighbor not in open_heap_set and neighbor not in visited and occupancy_map[neighbor] != 2:
                            open_heap_set.add(neighbor)
                            heapq.heappush(open_heap, (sdf_map[neighbor], neighbor))
                            parent[neighbor] = current
    return []

def search_within_sdf_map( start:tuple, goal:tuple, sdf_map:np.ndarray, occupancy_map:np.ndarray, resolution: float):
    start = tuple(start.flatten()) if isinstance(start, np.ndarray) else start
    goal = tuple(goal.flatten()) if isinstance(goal, np.ndarray) else goal
    sdf_bins = [0.2, 0.5, 1.0, 2.0, 5.0, 10.0]

    def get_queue_index(sdf_value: float) -> int:
        for idx, threshold in enumerate(sdf_bins):
            if sdf_value < threshold:
                return idx
        return len(sdf_bins)

    open_heaps = [[] for _ in range(len(sdf_bins) + 1)]
    open_sets = [set() for _ in range(len(sdf_bins) + 1)]
    start_queue_idx = get_queue_index(float(sdf_map[start]))
    heapq.heappush(open_heaps[start_queue_idx], (heuristic(start, goal, resolution), start))
    open_sets[start_queue_idx].add(start)
    parent = {start: start}
    visited = set()

    while True:
        queue_idx = -1
        for i, q in enumerate(open_heaps):
            if len(q) > 0:
                queue_idx = i
                break
        if queue_idx == -1:
            break

        current_cost, current = heapq.heappop(open_heaps[queue_idx])
        open_sets[queue_idx].remove(current)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            return reconstruct_path_sdf(parent, current)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                    if (0 <= neighbor[0] < sdf_map.shape[0] and
                            0 <= neighbor[1] < sdf_map.shape[1] and
                            0 <= neighbor[2] < sdf_map.shape[2]):
                        if neighbor in visited or occupancy_map[neighbor] == 2:
                            continue
                        neighbor_sdf = float(sdf_map[neighbor])
                        neighbor_queue_idx = get_queue_index(neighbor_sdf)
                        if neighbor in open_sets[neighbor_queue_idx]:
                            continue
                        open_sets[neighbor_queue_idx].add(neighbor)
                        heapq.heappush(
                            open_heaps[neighbor_queue_idx],
                            (heuristic(neighbor, goal, resolution), neighbor),
                        )
                        if neighbor not in parent:
                            parent[neighbor] = current
    return []


class DummyEmbeddingEngine:
    async def infer(self, _image: np.ndarray) -> np.ndarray:
        return np.zeros((1, 768), dtype=np.float32)


# 重定位的三道关卡原本都按 ORB 的数量级定：ORB 一帧几百个点、匹配上百个。SuperPoint 配
# cross-check 匹配是"少而准"，实测只有 12-16 个匹配，连第一关都过不去，而路标数不可能超过
# 匹配数，所以第二关的 40 是连坐卡死。换了特征就得换判据；几何退化另有
# _observations_constrain_pose 把关，不靠数量。
_RELOC_GATES = {
    "bow": (20, 40, 20),
    # 内点门槛回调到 12：8 是在「匹配数只有 12-16」的观测下定的，而那批数字来自车停在
    # 一个视角很差的位置。真实运行实测匹配 52-83 个，8 会放行内点率 17%、重投影中位
    # 4.1 px 的解。12 挡掉这些，同时保留内点率 35% 以上的。
    "vlad": (12, 12, 12),
    "embedding": (20, 40, 20),
}


class MapNode(Node):
    def __init__(
        self,
        tinynav_db_path: str,
        tinynav_map_path: str,
        extractor,
        matcher,
        embedding_extractor,
        loop_closure_mode: str = "embedding",
        vlad_centres_path: str | None = None,
        loop_closure_use_bow: bool = False,
        dbow3_vocabulary_path: str | None = None,
        verbose_timer: bool = True,
        warmup_nav_path_search: bool = True,
    ):
        """Initialization

        Args:
            tinynav_db_path (str): Directory to store output data.
            tinynav_map_path (str): Directory to load the pre-built map.
            verbose_timer (bool): Whether to use verbose timer output.
            warmup_nav_path_search (bool): Precompile the numba path-search
                kernels on a background thread. Set False on relocalization-only
                deployments, which never call them; the first path search then
                compiles them inline.
        """
        super().__init__('map_node')
        self._warmup_nav_path_search_enabled = bool(warmup_nav_path_search)
        self._nav_warmed = False
        # Throttle bookkeeping; see the constants above keyframe_callback. Both are
        # keyed on message stamps rather than wall clock so that a bag replayed
        # faster or slower than real time throttles the same way.
        self._dropped_stale_keyframes = 0
        self._last_relocalization_stamp_ns = 0

        # Runtime relocalization accounting.
        #
        # "Is relocalization working, and if not which layer is failing" was not
        # answerable while the robot was running. The information existed only as
        # individual log lines, so answering it meant grepping afterwards -- and the
        # obvious grep is wrong, because failure lines carry solvePnPRansac's own
        # `success=` as well as the relocalization result, which over-counts successes
        # by roughly 2x. Counting here removes both problems: the rate and the failing
        # layer are available live, in the log and over the backend's status API.
        #
        # Two windows. Cumulative answers "how has this run gone", the rolling window
        # answers "what is happening now" -- which is the one that matters when you are
        # standing next to a robot that has stopped, since a good first minute hides a
        # bad current minute in the cumulative figure.
        self._reloc_totals = self._new_reloc_bucket()
        self._reloc_window = self._new_reloc_bucket()
        # monotonic throughout: this board has no RTC, so a wall-clock span would jump.
        self._reloc_start_monotonic = time.monotonic()
        self._reloc_window_t0 = self._reloc_start_monotonic
        self._reloc_last_window = None
        # 上次重定位成功时最佳候选帧在地图中的位置。用来判断检索指得对不对：车连续走几米，
        # 候选也该连续移动几米；跳到几十米外就是检索错了，而那种错会被记成 pnp_inliers 不足。
        self._last_reloc_ref_xyz = None
        # 地图加载只是"点 nav 到能用"的一部分，真正的等待要算到第一次重定位成功为止。
        self._first_reloc_wall = None
        self._reloc_last_success_monotonic = None
        # Paired wall clock for the same event, for the status panel only: monotonic
        # cannot be aged by another process, and the panel wants a live "N s ago".
        self._reloc_last_success_epoch = None
        self._reloc_last_failure_code = ""
        self._reloc_stats_pub = self.create_publisher(
            String, '/map/relocalization_stats',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        # Latched: arrival is state. A browser that reloads after the robot arrived
        # must still learn that it arrived.
        self._poi_status_pub = self.create_publisher(
            String, '/mapping/poi_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._nav_warmup_lock = threading.Lock()
        self._nav_warmup_thread = None
        self.logger = logging.getLogger(__name__)
        self.timer_logger = self.logger.info if verbose_timer else self.logger.debug
        self.extractor = extractor
        self.matcher = matcher
        self.embedding_extractor = embedding_extractor
        self.loop_closure_use_bow = bool(loop_closure_use_bow)
        self.loop_closure_mode = "bow" if self.loop_closure_use_bow else loop_closure_mode
        # Distinct from self.vlad_centres below, which is an evaluation hook that
        # overrides the query embedding while the index stays in embedding mode.
        self.frozen_vlad_centres = load_vlad_centres(vlad_centres_path)
        self.dbow3_vocabulary_path = dbow3_vocabulary_path
        self.tinynav_db_path = tinynav_db_path

        self.bridge = CvBridge()

        # subs
        # /slam/keyframe_depth is deliberately NOT subscribed here. keyframe_callback
        # never read the pixels, only header.stamp, and only to compute a sync skew
        # that is identically zero: TimeSynchronizer matches stamps exactly, so the
        # three headers are equal by construction. Subscribing cost a 1.39 MB
        # deserialization per keyframe to learn nothing. The topic itself must keep
        # being published -- build_map_node consumes it offline.
        self.keyframe_image_sub = Subscriber(self, Image, '/slam/keyframe_image')
        self.keyframe_odom_sub = Subscriber(self, Odometry, '/slam/keyframe_odom')
        self.continuous_odom_sub = self.create_subscription(Odometry, '/slam/odometry', self.continuous_odom_callback, 100)
        # TRANSIENT_LOCAL to match the publisher. The nav target is state, published
        # once per user click, and this node is always the late joiner: it is started
        # by the same request that then sends the POI, and needs ~15 s to get here --
        # 11 s to load the map keyframes plus kernel compilation. With a volatile
        # subscription the target published inside that window is lost for good and
        # every nav-path attempt logs skip_no_poi from then on.
        self.pois_sub = self.create_subscription(
            String, '/mapping/cmd_pois', self.pois_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        # pubs
        self.pose_graph_trajectory_pub = self.create_publisher(Path, "/mapping/pose_graph_trajectory", 10)
        self.relocation_pub = self.create_publisher(Odometry, '/map/relocalization', 10)
        self.current_pose_in_map_pub = self.create_publisher(Odometry, "/mapping/current_pose_in_map", 10)

        # Add stop signal subscription and data saved publisher
        self.localization_stop_sub = self.create_subscription(Bool, '/benchmark/stop', self.localization_stop_callback, 10)
        self.localization_data_saved_pub = self.create_publisher(Bool, '/benchmark/data_saved', 10)
        self.ts = TimeSynchronizer([self.keyframe_image_sub, self.keyframe_odom_sub], 10)
        self.ts.registerCallback(self.keyframe_callback)

        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/infra2/camera_info', self.info_callback, 10)
        self.K = None
        self.baseline = None
        self.last_keyframe_image = None
        self.continuous_odom_recorder = OdomPoseRecorder(tinynav_db_path, "localization")

        self.odom = {}
        self.pose_graph_used_pose = {}
        self.relative_pose_constraint = []
        # 每次成功重定位的「质量」：(候选跨度秒, 内点率)。存下来才能在解算那一步说清
        # 100 条约束里有几条是干净的 —— 2026-08-27 的跳变分析全靠事后从日志里重建，
        # 而日志里既没有位姿也没有质量，只能推断。
        self.relocalization_pose_quality = {}
        self._last_kf_odom = None
        self._odom_resets = 0
        # VIO 自恢复时会把里程计原点重置，而这一侧的位姿是喂给 map->odom 约束的：重置之后
        # 旧约束用的是另一个坐标系，混在一起解出来的 T 必然是错的。2026-08-27 实测：原地
        # 快速旋转 21.6 s 里重定位全失败（内点 0~12），恢复时位姿跳 0.97 m、其中 z 从
        # -0.76 跳到 +0.00，随后 T 的朝向阶跃 94 度。
        # 判据用 z 而不是速度：平地上轮式车 z 逐帧应当不动（实测正常行驶 <=0.02 m），
        # 而重置那次是 0.75 m；速度判据反而漏（那次跨了 21.6 s，等效速度很小）。
        self.odom_reset_dz_m = float(os.environ.get('TINYNAV_ODOM_RESET_DZ_M', '0.15'))
        self.odom_reset_speed = float(os.environ.get('TINYNAV_ODOM_RESET_SPEED', '2.0'))
        self._last_T_map_to_odom_logged = None
        self.last_keyframe_timestamp = None

        (self.loop_similarity_threshold, self.loop_top_k,
         self.relocalization_threshold, self.relocalization_loop_top_k) = \
            LOOP_CLOSURE_DEFAULTS[self.loop_closure_mode]
        # 这一行以前不存在，检索方案只能靠日志里冒出 "ORBMatcher flann" 反推 —— 而一次
        # 用错方案的 run 和一次用对的 run 在日志里长得一模一样，直到重定位开始莫名其妙地失败。
        # 阈值一起打出来：它们是按方案取的，两套的量纲完全不同。
        (self.reloc_min_matches, self.reloc_min_landmarks,
         self.reloc_min_inliers) = _RELOC_GATES.get(self.loop_closure_mode, (20, 40, 20))
        # 实测通过的解占比 28-53%，被挡的 14-21%。0.25 卡在这条缝里。
        self.reloc_min_inlier_ratio = float(os.environ.get("TINYNAV_RELOC_MIN_INLIER_RATIO", "0.25"))
        self.get_logger().info(
            f"retrieval: {self.loop_closure_mode} "
            f"({'DBoW3 over ORB' if self.loop_closure_mode == 'bow' else 'VLAD over BPU SuperPoint' if self.loop_closure_mode == 'vlad' else 'DINOv2 embedding + LightGlue'}), "
            f"loop_sim>={self.loop_similarity_threshold} top_k={self.loop_top_k}, "
            f"reloc_sim>={self.relocalization_threshold} top_k={self.relocalization_loop_top_k}"
            + (f", vocab={self.dbow3_vocabulary_path}" if self.loop_closure_mode == "bow" else "")
        )
        # Straight-line pursuit radius for /control/target_pose. 2.0 m sits at the
        # 1.82 m median the old arc-length walk was actually producing, so a healthy
        # path aims where it always did; the change is that a folded one no longer
        # collapses the target onto the robot.
        self.nav_lookahead_m = 2.0

        # There are two bow LoopClosures below (nav + map). Read the vocabulary
        # from disk once and hand the same object to both: each Database still
        # deep-copies it, but this drops the two redundant Python-side copies.
        # Measured steady-state cost of the two engines' vocabularies, x86_64:
        # ORBvoc 1189 -> 628 MiB, self-trained k10L5 123 -> 63 MiB.
        shared_dbow3_vocabulary = None
        if self.loop_closure_mode == "bow" and self.dbow3_vocabulary_path is not None:
            shared_dbow3_vocabulary = DBoW3Engine.load_vocabulary(self.dbow3_vocabulary_path)

        os.makedirs(f"{tinynav_db_path}/nav_temp", exist_ok=True)
        # No image videos in the scratch db. This one exists to hold live keyframes for
        # nav-side loop closure, and it never gets an image written to it -- but with the
        # default it still opened two h264 encoders at every nav start, for a directory
        # nothing ever reads back.
        self._node_start_wall = time.monotonic()
        _t_load0 = time.perf_counter()
        self.nav_temp_db = TinyNavDB(
            f"{tinynav_db_path}/nav_temp", is_scratch=True,
            save_infra1_video=False, save_rgb_video=False,
        )
        _t_navdb = time.perf_counter()
        self.nav_loop_closure = LoopClosure(
            db=self.nav_temp_db,
            timestamps=[],
            mode=self.loop_closure_mode,
            dbow3_vocabulary_path=self.dbow3_vocabulary_path,
            embedding_similarity_threshold=self.loop_similarity_threshold,
            embedding_top_k=self.loop_top_k,
            dbow3_vocabulary=shared_dbow3_vocabulary,
            vlad_centres=self.frozen_vlad_centres,
        )
        _t_navlc = time.perf_counter()
        self.map_poses = np.load(f"{tinynav_map_path}/poses.npy", allow_pickle=True).item()
        self.map_K = np.load(f"{tinynav_map_path}/intrinsics.npy")
        self.db = TinyNavDB(tinynav_map_path, is_scratch=False)
        _t_mapdb = time.perf_counter()
        self.map_loop_closure = LoopClosure(
            db=self.db,
            timestamps=list(self.map_poses.keys()),
            mode=self.loop_closure_mode,
            dbow3_vocabulary_path=self.dbow3_vocabulary_path,
            embedding_similarity_threshold=self.relocalization_threshold,
            embedding_top_k=self.relocalization_loop_top_k,
            dbow3_vocabulary=shared_dbow3_vocabulary,
            vlad_centres=self.frozen_vlad_centres,
        )
        _t_maplc = time.perf_counter()
        # Both databases hold their own copy now; release ours.
        del shared_dbow3_vocabulary

        self.occupancy_map = np.load(f"{tinynav_map_path}/occupancy_grid.npy")
        self.occupancy_map_meta = np.load(f"{tinynav_map_path}/occupancy_meta.npy")
        self.sdf_map = np.load(f"{tinynav_map_path}/sdf_map.npy")
        _t_grids = time.perf_counter()
        # 「点击 nav 到能用」慢在哪，此前只能靠猜。地图加载全在 __init__ 里，所以这一行就是
        # 那段等待的全部构成；vlad 模式下 map_loop_closure 要为每个关键帧现算 VLAD，是大头。
        self.get_logger().info(
            f"map load timing ms: total={(_t_grids - _t_load0) * 1e3:.0f}, "
            f"nav_temp_db={(_t_navdb - _t_load0) * 1e3:.0f}, "
            f"nav_loop_closure={(_t_navlc - _t_navdb) * 1e3:.0f}, "
            f"poses+map_db={(_t_mapdb - _t_navlc) * 1e3:.0f}, "
            f"map_loop_closure={(_t_maplc - _t_mapdb) * 1e3:.0f} "
            f"({len(self.map_poses)} keyframes, mode={self.loop_closure_mode}), "
            f"grids={(_t_grids - _t_maplc) * 1e3:.0f}")
        self._map_load_s = _t_grids - _t_load0
        # Starts here, *after* the two LoopClosures, and must not be moved ahead of
        # them. It looks like free real estate: the warmup needs nothing from the
        # arrays above but their dtypes and one scalar, so starting it earlier
        # would seem to hide its ~2.7 s of JIT inside the rebuild's ~18 s. Measured
        # on the X5, k10L5 vocabulary, 1161-keyframe map, it does the opposite:
        #
        #   warmup started after the rebuild  : __init__ 21205 / 20068 ms
        #   warmup started before the rebuild : __init__ 44481 / 32345 ms
        #
        # The DBoW3 rebuild does not release the GIL, so numba's compiler and the
        # rebuild thrash against each other rather than overlapping -- in one run
        # the JIT itself stretched from 2.7 s to 21.7 s. Serial beats contending.
        self._start_nav_path_search_warmup()

        print(f"sdf_map.shape: {self.sdf_map.shape}")
        print(f"occupancy_map.shape: {self.occupancy_map.shape}")

        self.relocalization_poses = {}
        self.relocalization_pose_weights = {}
        self.failed_relocalizations = []
        self.last_relocalization_failure_reason = ""
        # Successes carry the same candidate detail failures already carried, so the two
        # distributions are comparable from one grep over the log.
        self.last_relocalization_detail = ""
        self.last_relocalization_timing = {}
        self._last_nav_path_sig = None
        # The same numbers last_relocalization_detail carries, kept numeric. The string is
        # for humans reading one case; offline evaluation aggregates thousands of queries,
        # and re-parsing that string is how the wrong 58% success rate got reported once.
        self.last_relocalization_stats = {}
        # Optional evaluation hooks, both off in normal operation. See their use in
        # relocalize_with_depth for why retrieval and matching may need to differ.
        self.retrieval_extractor = None
        self.alt_map_features = None
        # VLAD ranks by dense descriptor, not by an inverted index, so it needs the query
        # encoded the same way the map was rather than the embedding engine's output.
        self.vlad_centres = None

        self.T_from_map_to_odom = None

        self.pois = {}
        self.poi_index = -1
        # Per-leg progress accounting for /mapping/nav_progress. A "leg" is one POI:
        # the initial path length is captured the first time a path to it is planned,
        # and percent/ETA are derived from how much of that has been consumed. The
        # speed estimate deliberately survives POI transitions so the ETA keeps
        # improving instead of restarting from "unknown" at every waypoint.
        self._nav_completed = False
        self._leg_initial_length: float | None = None
        self._leg_start_time: float | None = None
        self._speed_estimate: float | None = None

        self.poi_pub = self.create_publisher(Odometry, "/mapping/poi", 10)
        self.poi_change_pub = self.create_publisher(Odometry, "/mapping/poi_change", 10)
        self.nav_done_pub = self.create_publisher(Bool, '/mapping/nav_done', 10)
        self.nav_progress_pub = self.create_publisher(String, '/mapping/nav_progress', 10)

        self.current_pose_pub = self.create_publisher(Odometry, "/mapping/current_pose", 10)
        self.global_plan_pub = self.create_publisher(Path, '/mapping/global_plan', 10)
        # Latched for the same reason as /mapping/cmd_pois above: this is state, not a
        # stream. It is published once when a POI becomes the active target, and
        # planning_node is restarted independently of this node (cmd_restart_nav_nodes),
        # so it is routinely the late joiner. A volatile pair leaves it with
        # target_pose=None and it publishes "No target pose, publishing static path"
        # indefinitely -- a trajectory appears, cmd_vel stays zero, and nothing is
        # logged as an error. Measured: 224 s of that in one run.
        self.target_pose_pub = self.create_publisher(
            Odometry, "/control/target_pose",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self._save_completed = False

    def _start_nav_path_search_warmup(self):
        """Kick off the path-search JIT warmup without blocking the constructor.

        Compiling the ten @njit path-search kernels costs ~33 s on the X5 with a
        cold numba cache, and none of them are on the relocalization path -- a
        deployment that only relocalizes used to pay all of it before the node
        could receive its first keyframe.

        The warmup is therefore moved onto a background thread instead of being
        made lazy outright: `generate_nav_path_in_map` runs inside the keyframe
        callback, so compiling there on first use would just move the same stall
        into the control loop. Navigation additionally cannot start until a
        relocalization has succeeded *and* the planner has sent POIs, which in
        practice leaves the thread far more than 33 s to finish. Should a path
        search still get there first, `_ensure_nav_path_search_warm` blocks on
        the thread, which is no worse than today's behaviour.
        """
        if not self._warmup_nav_path_search_enabled:
            return

        def _run():
            t0 = time.perf_counter()
            try:
                self._warmup_nav_path_search()
            except Exception as e:  # noqa: BLE001 - a warmup thread must never die silently
                # The only symptom would otherwise be an unexplained stall in the
                # first path search, which then recompiles inline.
                self.get_logger().error(f"nav path search warmup failed: {e}")
                return
            self.get_logger().info(
                f"nav path search kernels ready in {(time.perf_counter() - t0) * 1000.0:.0f} ms"
            )

        self._nav_warmup_thread = threading.Thread(
            target=_run, name="nav_path_search_warmup", daemon=True
        )
        self._nav_warmup_thread.start()

    def _warmup_nav_path_search(self):
        with self._nav_warmup_lock:
            if self._nav_warmed:
                return
            small_sdf = np.ones((3, 3, 3), dtype=self.sdf_map.dtype)
            small_sdf[1, 1, 1] = 0.0
            small_occupancy = np.zeros((3, 3, 3), dtype=self.occupancy_map.dtype)
            search_close_to_sdf_map_numba(np.array([0, 0, 0], dtype=np.int32), small_sdf, small_occupancy, 0.2)
            # NOT float(...): occupancy_meta is float32, and the cast made this a
            # float64 signature the real call never hits, so the first path search
            # recompiled from scratch -- 17.9 s, logged as search time.
            search_within_sdf_map_numba(
                np.array([0, 0, 0], dtype=np.int32),
                np.array([2, 2, 2], dtype=np.int32),
                small_sdf,
                small_occupancy,
                self.occupancy_map_meta[3],
            )
            self._nav_warmed = True

    def _ensure_nav_path_search_warm(self):
        """Block until the path-search kernels are compiled. Idempotent."""
        if self._nav_warmed:
            return
        thread = self._nav_warmup_thread
        if thread is not None and thread.is_alive():
            thread.join()
        if not self._nav_warmed:
            # Warmup was disabled, or the background thread raised. Compile here;
            # the njit calls below would do it anyway, just without the log line.
            self.get_logger().info("Compiling nav path search kernels inline")
            self._warmup_nav_path_search()

    def pois_callback(self, msg: String):
        self.get_logger().info("Received POIs from planner: " + msg.data)
        try:
            self.pois = json.loads(msg.data)

            # 保持发布方给的键顺序，不要按编号 sorted()。后端的 cmd_send_pois 用原始 POI
            # 编号做键、按用户勾选顺序写 JSON，排序会把它重排成建图时的保存顺序 —— 而前端
            # 在每个 POI 上显示勾选序号徽章，于是界面承诺的顺序和实际走的顺序不一致。
            # tool/pub_pois.py 不受影响：它已经把键重编成 0,1,2,...，而它的
            # `--pois "2,1,0"` 本来就是"按我给的顺序走"的意思。
            pois_dict = {}
            order = list(self.pois.keys())
            for index, key in enumerate(order):
                pois_dict[index] = np.array(self.pois[key]["position"])
            self.pois = pois_dict

            if not self.pois:
                self.poi_index = -1
                # Signal planning_node to clear target_pose so it stops publishing paths
                dummy_pose = np.eye(4)
                self.poi_change_pub.publish(np2msg(dummy_pose, self.get_clock().now().to_msg(), "world", "map"))
                self.get_logger().info("POIs cleared, navigation cancelled")
                return

            self.poi_index = min(0, len(self.pois) - 1)
            self._nav_completed = False
            self._leg_initial_length = None
            self._leg_start_time = None
            self._speed_estimate = None
            self.get_logger().info(
                f"Parsed POIs (visit order = as received): keys={order} -> {self.pois}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse POIs JSON: {e}")
            self.pois = {}

    def info_callback(self, msg:CameraInfo):
        if self.K is None:
            self.get_logger().info("Camera intrinsics received.")
            self.K = np.array(msg.k).reshape(3, 3)
            fx = self.K[0, 0]
            Tx = msg.p[3]
            self.baseline = -Tx / fx
            self.destroy_subscription(self.camera_info_sub)

    def continuous_odom_callback(self, odom_msg: Odometry):
        self.continuous_odom_recorder.record_odometry_msg(odom_msg)

    def localization_stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Received benchmark stop signal, starting save process...")
            try:
                self.save_relocalization_poses()
                self.get_logger().info("Localization save completed successfully")

                # Publish save finished signal
                save_finished_msg = Bool()
                save_finished_msg.data = True
                self.localization_data_saved_pub.publish(save_finished_msg)
                self.get_logger().info("Published data save finished signal")

            except Exception as e:
                self.get_logger().error(f"Error during localization save: {e}")
                # Still publish completion signal even if there was an error
                save_finished_msg = Bool()
                save_finished_msg.data = False
                self.localization_data_saved_pub.publish(save_finished_msg)

    # Two independent throttles, because the node cannot keep up with the camera
    # and the two ways of falling behind need different cures.
    #
    # Measured on the X5: keyframes arrive at up to 4.83 Hz (the bridge's exact
    # stamp sync is capped by the 5 Hz depth stream, and with a 3 cm keyframe
    # threshold anything above ~0.15 m/s makes every synced frame a keyframe),
    # while one relocalization costs 374 ms p50. 200 ms in, 374 ms out: the queue
    # grows without bound until the middleware starts dropping, by which point the
    # pose being published is seconds stale.
    #
    # `max_keyframe_age_s` is the safety net. Note a "busy" flag would not work
    # here: main() uses rclpy.spin(), a single-threaded executor, so callbacks are
    # serialised and never re-enter -- by the time one returns, the backlog is
    # already sitting in the executor queue. Rejecting on *age* does work, because
    # each stale item is discarded in microseconds and the queue drains almost
    # instantly, leaving the node working on the newest data.
    #
    # `min_relocalization_interval_s` attacks the cause rather than the symptom.
    # Relocalization is the only expensive stage; the pose-graph bookkeeping either
    # side of it is cheap. Running it once a second rather than five times keeps
    # every keyframe in the graph while removing the overload -- and the
    # relocalization budget for this robot is 5 s, so 1 Hz is already 5x margin.
    # Raised from 0.5 s: that was sized for 4.83 Hz keyframes against 374 ms
    # relocalization. The bridge now delivers 0.71 Hz with a p90 publish lag of
    # 0.83 s, so 0.5 s was rejecting 19% of perfectly usable frames to protect
    # against an overload that no longer exists.
    max_keyframe_age_s = 1.0
    min_relocalization_interval_s = 1.0

    def keyframe_callback(self, keyframe_image_msg:Image, keyframe_odom_msg:Odometry):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = int(keyframe_image_msg.header.stamp.sec * 1e9) + int(keyframe_image_msg.header.stamp.nanosec)
        age_s = (now_ns - stamp_ns) / 1e9
        self._reloc_tally('keyframes')
        self._report_reloc_stats()
        if self.max_keyframe_age_s > 0.0 and age_s > self.max_keyframe_age_s:
            self._reloc_tally('dropped_stale')
            # Logged as a running count rather than per drop: a message per
            # discarded keyframe would itself cost time in the loop that is
            # already behind, and the useful signal is the rate, not the events.
            self._dropped_stale_keyframes += 1
            if self._dropped_stale_keyframes % 20 == 1:
                self.get_logger().warning(
                    f"dropping stale keyframes: {self._dropped_stale_keyframes} so far, "
                    f"latest was {age_s:.2f}s old (limit {self.max_keyframe_age_s:.2f}s). "
                    "The node is not keeping up with the keyframe rate."
                )
            return

        t_start = time.perf_counter()
        stage_timings = {}

        def mark_stage(name: str, previous_t: float) -> float:
            now = time.perf_counter()
            stage_timings[name] = (now - previous_t) * 1000.0
            return now

        # NOTE: keyframe_mapping / keyframe_mapping_with_timer below are not called
        # from anywhere -- mapping is done offline by build_map_node. That used to be
        # announced with an INFO line here, once per keyframe, which is a fact about
        # the source that a running log has no reason to repeat. Left as a comment.
        image = self.bridge.imgmsg_to_cv2(keyframe_image_msg, desired_encoding="mono8")
        t_stage = mark_stage("image_decode", t_start)

        keyframe_image_timestamp_ns = int(keyframe_image_msg.header.stamp.sec * 1e9) + int(keyframe_image_msg.header.stamp.nanosec)
        t_stage = mark_stage("timestamp_parse", t_stage)

        since_last_s = (stamp_ns - self._last_relocalization_stamp_ns) / 1e9
        if self._last_relocalization_stamp_ns == 0 or since_last_s >= self.min_relocalization_interval_s:
            self._last_relocalization_stamp_ns = stamp_ns
            self._reloc_tally('attempts')
            success, pose_in_world = self.keyframe_relocalization(keyframe_image_msg.header.stamp, image)
            if success:
                self._reloc_tally('success')
                self._reloc_last_success_monotonic = time.monotonic()
                self._reloc_last_success_epoch = time.time()
        else:
            # Skipped, not failed. The keyframe still goes into the pose graph
            # below; only the expensive relocalization is rate-limited. Counted
            # separately because a skip landing in the same success=False bucket as a
            # real failure is exactly what made the measured rate wrong.
            self._reloc_tally('skipped_rate_limit')
            success, pose_in_world = False, np.eye(4)
        t_stage = mark_stage("relocalization", t_stage)

        odom, _ = msg2np(keyframe_odom_msg)
        t_stage = mark_stage("odom_msg_decode", t_stage)

        prev = self._last_kf_odom
        if prev is not None:
            dt_s = max(1e-3, (keyframe_image_timestamp_ns - prev[0]) / 1e9)
            step = odom[:3, 3] - prev[1][:3, 3]
            dz = abs(float(step[2]))
            speed = float(np.linalg.norm(step)) / dt_s
            if dz > self.odom_reset_dz_m or speed > self.odom_reset_speed:
                self._odom_resets += 1
                self.get_logger().error(
                    f"odometry discontinuity #{self._odom_resets}: dz={dz:.2f}m "
                    f"speed={speed:.2f}m/s over {dt_s:.1f}s "
                    f"({prev[1][0,3]:+.2f},{prev[1][1,3]:+.2f},{prev[1][2,3]:+.2f}) -> "
                    f"({odom[0,3]:+.2f},{odom[1,3]:+.2f},{odom[2,3]:+.2f}) -- "
                    f"dropping {len(self.relocalization_poses)} relocalization constraints and "
                    "refitting map->odom from scratch"
                )
                self.relocalization_poses.clear()
                self.relocalization_pose_weights.clear()
                self.relocalization_pose_quality.clear()
                self.T_from_map_to_odom = None
                self._last_T_map_to_odom_logged = None
        self._last_kf_odom = (keyframe_image_timestamp_ns, odom.copy())

        self.pose_graph_used_pose[keyframe_image_timestamp_ns] = odom
        self.odom[keyframe_image_timestamp_ns] = odom
        t_stage = mark_stage("pose_cache_update", t_stage)

        if success:
            # 和上面的 reloc pose 配对：同一个时间戳下 VIO 说车在哪。两行相减就是这一次
            # 观测给出的 map->odom，不必再从 T 反推。
            self.get_logger().info(
                f"reloc odom: t={keyframe_image_timestamp_ns} "
                f"cam_odom=[{odom[0,3]:+.2f},{odom[1,3]:+.2f},{odom[2,3]:+.2f}] "
                f"yaw_odom={self._ground_yaw_deg(odom):+.1f}deg"
            )
            self.compute_transform_from_map_to_odom()
        t_stage = mark_stage("tf_update", t_stage)

        with Timer(name = "nav path", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.timer_logger):
            self.try_publish_nav_path(keyframe_image_timestamp_ns)
            # timer or queue for publish the nav path
            # and record the map pose
            # compute the coordinate transform from the map pose to the keyframe pose
            # publish the nav path from the map pose to the keyframe pose with the cost map
        t_stage = mark_stage("nav_path", t_stage)

        total_ms = (t_stage - t_start) * 1000.0
        stage_parts = ", ".join(f"{name}={ms:.1f}" for name, ms in stage_timings.items())
        relocal_parts = ", ".join(f"relocal_{name}={ms:.1f}" for name, ms in self.last_relocalization_timing.items())
        msg = (
            f"Keyframe callback benchmark ms: timestamp={keyframe_image_timestamp_ns}, "
            f"total={total_ms:.1f}, {stage_parts}, success={success}"
        )
        if relocal_parts:
            msg += f", {relocal_parts}"
        if total_ms > 500.0:
            self.get_logger().warning(msg)
        else:
            self.get_logger().info(msg)

    def keyframe_mapping_with_timer(self, keyframe_image_msg:Image, keyframe_odom_msg:Odometry, depth_msg:Image):
        with Timer(name="Mapping Loop", text="\n\n[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.timer_logger):
            self.keyframe_mapping(keyframe_image_msg, keyframe_odom_msg, depth_msg)

    def keyframe_mapping(self, keyframe_image_msg:Image, keyframe_odom_msg:Odometry, depth_msg:Image):
        if self.K is None:
            return
        keyframe_image_timestamp = int(keyframe_image_msg.header.stamp.sec * 1e9) + int(keyframe_image_msg.header.stamp.nanosec)
        keyframe_odom_timestamp = int(keyframe_odom_msg.header.stamp.sec * 1e9) + int(keyframe_odom_msg.header.stamp.nanosec)
        depth_timestamp = int(depth_msg.header.stamp.sec * 1e9) + int(depth_msg.header.stamp.nanosec)
        assert keyframe_image_timestamp == keyframe_odom_timestamp
        assert keyframe_image_timestamp == depth_timestamp
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        odom, _ = msg2np(keyframe_odom_msg)
        image = self.bridge.imgmsg_to_cv2(keyframe_image_msg, desired_encoding="mono8")
        rgb_image_place_holder = einops.repeat(image, "h w -> h w c", c = 3)

        self.nav_temp_db.set_entry(keyframe_image_timestamp, depth = depth, infra1_image = image, rgb_image = rgb_image_place_holder)
        embedding = self.get_embeddings(image)
        self.nav_temp_db.set_entry(keyframe_image_timestamp, embedding = embedding)
        features = asyncio.run(self.extractor.infer(image))
        self.nav_temp_db.set_entry(keyframe_image_timestamp, features = features)

        if len(self.odom) == 0 and self.last_keyframe_timestamp is None:
            self.odom[keyframe_odom_timestamp] = odom
            self.pose_graph_used_pose[keyframe_odom_timestamp] = odom
            self.nav_loop_closure.add_timestamp(keyframe_odom_timestamp)
        else:
            last_keyframe_odom_pose = self.odom[self.last_keyframe_timestamp]
            T_prev_curr = se3_inv(last_keyframe_odom_pose) @ odom
            self.relative_pose_constraint.append((keyframe_image_timestamp, self.last_keyframe_timestamp, T_prev_curr))
            self.pose_graph_used_pose[keyframe_image_timestamp] = odom
            self.odom[keyframe_image_timestamp] = odom
            def find_loop_and_pose_graph(timestamp):
                    valid_timestamp = [t for t in self.pose_graph_used_pose.keys() if t + 10 * 1e9 < timestamp]
                    if len(valid_timestamp) == 0:
                        return
                    target_embedding = self.nav_temp_db.get_embedding(timestamp)
                    _, _, curr_features, _, _ = self.nav_temp_db.get_depth_embedding_features_images(timestamp)
                    curr_kp = curr_features["kpts"][0] if curr_features["kpts"].ndim == 3 else curr_features["kpts"]
                    curr_desc = curr_features["descps"][0] if curr_features["descps"].ndim == 3 else curr_features["descps"]
                    with Timer(name = "find loop", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.timer_logger):
                        loop_list = self.nav_loop_closure.find_candidate_timestamps(
                            curr_kp,
                            curr_desc,
                            target_embedding,
                            top_k=self.loop_top_k,
                            allowed_timestamps=set(valid_timestamp),
                        )
                    with Timer(name = "Relative pose estimation", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.timer_logger):
                        for candidate in loop_list:
                            prev_timestamp = candidate["timestamp"]
                            curr_timestamp = timestamp
                            similarity = float(candidate["similarity"])
                            self.logger.info(
                                f"Loop candidate curr={curr_timestamp} prev={prev_timestamp} similarity={float(similarity):.4f}"
                            )
                            prev_depth, _, prev_features, _, _ = self.nav_temp_db.get_depth_embedding_features_images(prev_timestamp)
                            curr_depth, _, curr_features, _, _ = self.nav_temp_db.get_depth_embedding_features_images(curr_timestamp)
                            prev_matched_keypoints, curr_matched_keypoints, matches = self.match_keypoints(prev_features, curr_features)
                            success, T_prev_curr, _, _, inliers = estimate_pose(prev_matched_keypoints, curr_matched_keypoints, curr_depth, self.K)
                            if success and len(inliers) >= 100:
                                self.relative_pose_constraint.append((curr_timestamp, prev_timestamp, T_prev_curr))
                                self.logger.info(
                                    f"Loop accepted curr={curr_timestamp} prev={prev_timestamp} similarity={float(similarity):.4f} inliers={len(inliers)}"
                                )
                    with Timer(name = "solve pose graph", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.timer_logger):
                        self.pose_graph_used_pose = solve_pose_graph(self.pose_graph_used_pose, self.relative_pose_constraint, max_iteration_num = 5)
            find_loop_and_pose_graph(keyframe_image_timestamp)
            self.nav_loop_closure.add_timestamp(keyframe_image_timestamp)
            self.pose_graph_trajectory_publish(keyframe_image_timestamp)
        self.last_keyframe_timestamp = keyframe_odom_timestamp
        self.last_keyframe_image = image


    def get_embeddings(self, image: np.ndarray) -> np.ndarray:
        # shape: (1, 768)
        return asyncio.run(self.embedding_extractor.infer(image))

    def match_keypoints(self, feats0:dict, feats1:dict, image_shape = np.array([848, 480], dtype = np.int64)) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        match_result = asyncio.run(self.matcher.infer(feats0["kpts"], feats1["kpts"], feats0['descps'], feats1['descps'], feats0['mask'], feats1['mask'], image_shape, image_shape))
        match_indices = match_result["match_indices"][0]
        if feats0["kpts"].ndim != 3 or feats1["kpts"].ndim != 3:
            return np.zeros((0, 2), dtype=np.float32), np.zeros((0, 2), dtype=np.float32), np.zeros((0, 2), dtype=np.int64)
        if feats0["kpts"].shape[0] == 0 or feats1["kpts"].shape[0] == 0:
            return np.zeros((0, 2), dtype=np.float32), np.zeros((0, 2), dtype=np.float32), np.zeros((0, 2), dtype=np.int64)
        # Guard against invalid indices returned by matcher.
        match_indices = match_indices.copy()
        invalid = (match_indices < 0) | (match_indices >= feats1["kpts"][0].shape[0]) | (np.arange(match_indices.shape[0]) >= feats0["kpts"][0].shape[0])
        match_indices[invalid] = -1
        valid_mask = match_indices != -1
        keypoints0 = feats0["kpts"][0][valid_mask]
        keypoints1 = feats1["kpts"][0][match_indices[valid_mask]]
        matches = []
        for i, index in enumerate(match_indices):
            if index != -1:
                matches.append([i, index])
        return keypoints0, keypoints1, np.array(matches, dtype=np.int64)

    def pose_graph_trajectory_publish(self, timestamp):
        path_msg = Path()
        path_msg.header.stamp.sec = int(timestamp / 1e9)
        path_msg.header.stamp.nanosec = int(timestamp % 1e9)
        path_msg.header.frame_id = "world"
        for t, pose_in_world in self.pose_graph_used_pose.items():
            pose = PoseStamped()
            pose.header = path_msg.header
            t = pose_in_world[:3, 3]
            quat = matrix_to_quat(pose_in_world[:3, :3])
            pose.pose.position.x = t[0]
            pose.pose.position.y = t[1]
            pose.pose.position.z = t[2]
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            path_msg.poses.append(pose)
        self.pose_graph_trajectory_pub.publish(path_msg)

    # Rolling-window length for the runtime stats. Long enough that a 1 Hz attempt rate
    # gives a meaningful denominator, short enough to reflect where the robot is now.
    _RELOC_STATS_INTERVAL_S = 10.0

    @staticmethod
    def _new_reloc_bucket() -> dict:
        return {
            'keyframes': 0,          # keyframe sets that arrived
            'dropped_stale': 0,      # too old to be worth relocalizing against
            'skipped_rate_limit': 0, # inside min_relocalization_interval_s
            'attempts': 0,           # relocalization actually run
            'success': 0,
            'failure': 0,
            'by_code': {},           # failure layer -> count
        }

    def _reloc_tally(self, key: str, code: str | None = None) -> None:
        for bucket in (self._reloc_totals, self._reloc_window):
            bucket[key] += 1
            if code is not None:
                bucket['by_code'][code] = bucket['by_code'].get(code, 0) + 1

    @staticmethod
    def _reloc_rates(bucket: dict, span_s: float) -> dict:
        attempts = bucket['attempts']
        return {
            **{k: v for k, v in bucket.items() if k != 'by_code'},
            'byCode': dict(bucket['by_code']),
            # Two different frequencies, and conflating them is how a healthy stack
            # looks broken: attemptHz is how often relocalization runs at all (capped
            # by min_relocalization_interval_s), successHz is how often it produces a
            # pose. successRate relates the two.
            'attemptHz': round(attempts / span_s, 3),
            'successHz': round(bucket['success'] / span_s, 3),
            'successRate': round(bucket['success'] / attempts, 3) if attempts else None,
            'spanS': round(span_s, 1),
        }

    def _reloc_stats_snapshot(self) -> dict:
        """Both windows plus the derived rates, as the backend and the log both want."""
        now = time.monotonic()
        rates = self._reloc_rates
        # The last *completed* window, not the one filling up: this snapshot goes out on
        # every keyframe now, and a partial window would make attemptHz swing from 0 to
        # several Hz between publishes. Falls back to the partial one before the first
        # window closes, so the panel is not blank for the first 10 s.
        window = self._reloc_last_window or rates(
            self._reloc_window, max(1e-6, now - self._reloc_window_t0))

        uptime_s = max(1e-6, now - self._reloc_start_monotonic)
        return {
            'window': window,
            'total': rates(self._reloc_totals, uptime_s),
            'lastFailureCode': self._reloc_last_failure_code or None,
            # The numbers behind the code, minus the candidate dump: that part runs to
            # several hundred characters and belongs in the log, not on a status panel.
            'lastFailureReason':
                self.last_relocalization_failure_reason.split(', candidates=')[0][:160] or None,
            'secondsSinceLastSuccess': (
                round(now - self._reloc_last_success_monotonic, 1)
                if self._reloc_last_success_monotonic is not None else None
            ),
            # Wall clock, so the backend can age it on every /device/status instead of
            # serving whatever this 10 s snapshot happened to freeze.
            'lastSuccessEpoch': self._reloc_last_success_epoch,
        }

    def _report_reloc_stats(self) -> None:
        """Publish on every keyframe; log and roll the window every _RELOC_STATS_INTERVAL_S.

        These were one operation, and that made lastSuccessEpoch as stale as the 10 s
        publish period: the backend re-ages it against the current wall clock on every
        /device/status, so a frozen epoch showed up as "last ok" sawtoothing from 1 s
        to 11 s and back while relocalization was in fact succeeding at 0.8 Hz.
        """
        now = time.monotonic()
        if now - self._reloc_window_t0 >= self._RELOC_STATS_INTERVAL_S:
            self._reloc_last_window = self._reloc_rates(
                self._reloc_window, max(1e-6, now - self._reloc_window_t0))
            self._log_reloc_window(self._reloc_last_window)
            self._reloc_window = self._new_reloc_bucket()
            self._reloc_window_t0 = now
        try:
            msg = String()
            msg.data = json.dumps(self._reloc_stats_snapshot(), separators=(',', ':'))
            self._reloc_stats_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"could not publish relocalization stats: {e}")

    def _log_reloc_window(self, w: dict) -> None:
        codes = ', '.join(f'{k}={v}' for k, v in sorted(w['byCode'].items())) or 'none'
        rate = 'n/a' if w['successRate'] is None else f"{100.0 * w['successRate']:.0f}%"
        self.get_logger().info(
            f"relocalization {w['spanS']:.0f}s window: "
            f"{w['keyframes']} keyframes -> {w['attempts']} attempts "
            f"({w['attemptHz']:.2f} Hz) -> {w['success']} ok ({rate}, {w['successHz']:.2f} Hz); "
            f"skipped_rate_limit={w['skipped_rate_limit']}, dropped_stale={w['dropped_stale']}; "
            f"failures: {codes}"
        )

    # Arrival is decided here and was reported nowhere. The advance below is the only
    # place that knows the robot reached a POI, and its one output was
    # /mapping/poi_change -- a topic the backend *publishes* (to cancel a target) and
    # never subscribes to. get_status has no arrival field either, and the frontend
    # renders navStatus, which is 'navigating' for as long as the backend's own state
    # machine says so. So the robot could arrive, map_node could log "All POIs have been
    # visited", and the UI would still read "Navigating..." indefinitely -- observed.
    #
    # A dedicated latched topic rather than more traffic on /mapping/poi_change, which
    # already carries two different meanings in two directions and would make the
    # backend hear its own cancels.
    # 0.25 m was below the pose uncertainty: relocalization alone moved the reported
    # position 0.15 m while the wheels stood still, and the path can only end on a free
    # cell, which sat 0.13 m from the POI. Closest approach was 0.276 m, so arrival never
    # fired. There is no z condition: this robot drives on one floor, and a second
    # condition that always passes is a trap waiting for the day it does not.
    POI_ARRIVAL_RADIUS_XY_M = 0.4

    def _publish_poi_status(self, pose_in_map_position: np.ndarray, advanced: int) -> None:
        total = len(self.pois)
        idx = self.poi_index
        active = self.pois[idx] if 0 <= idx < total else None
        payload = {
            'total': total,
            'index': idx,
            'visited': max(0, min(idx, total)),
            'allVisited': idx >= total,
            # Distance to the *current* target, so the UI can show closing-in rather
            # than only the binary arrival.
            'distanceXyM': (
                round(float(np.linalg.norm(active[:2] - pose_in_map_position[:2])), 3)
                if active is not None else None
            ),
            'arrivalRadiusXyM': self.POI_ARRIVAL_RADIUS_XY_M,
            'advancedThisTick': advanced,
        }
        try:
            msg = String()
            msg.data = json.dumps(payload, separators=(',', ':'))
            self._poi_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"could not publish POI status: {e}")

    def _relocalization_failed(self, reason: str, code: str = "unknown") -> tuple[bool, np.ndarray, float]:
        # code is a short stable slug for the failing layer, so the counts stay
        # aggregatable while `reason` keeps the numbers that explain the individual case.
        self.last_relocalization_failure_reason = reason
        self._reloc_last_failure_code = code
        self.last_relocalization_stats["fail_code"] = code
        self._reloc_tally('failure', code=code)
        return False, np.eye(4), -np.inf

    def _log_relocalization_timing(self, timestamp_ns: int, success: bool, timings: dict[str, float], extra: str = ""):
        total_ms = sum(timings.values())
        parts = ", ".join(f"{name}={ms:.1f}" for name, ms in timings.items())
        msg = f"Relocalization stage timing ms: timestamp={timestamp_ns}, total={total_ms:.1f}, {parts}, success={success}"
        if extra:
            msg += f", {extra}"
        if total_ms > 500.0:
            self.get_logger().warning(msg)
        else:
            self.get_logger().info(msg)

    def relocalize_with_depth(self, keyframe: np.ndarray, keyframe_features: dict, K: np.ndarray | None, timings: dict[str, float] | None = None) -> tuple[bool, np.ndarray, float]:
        if timings is None:
            timings = {}
        self.last_relocalization_failure_reason = ""
        self.last_relocalization_detail = ""
        # Mutated in place as the pipeline advances, so whichever return fires leaves the
        # stats describing exactly how far it got.
        stats = self.last_relocalization_stats = {
            "query_kpts": 0, "candidates": 0, "top_sim": 0.0, "cand_ts": [],
            "cand_sims": [], "cand_matches": [], "cand_valid_depth": [],
            "landmarks": 0, "pnp_inliers": 0,
        }
        if K is None:
            return self._relocalization_failed("camera intrinsics unavailable", "no_intrinsics")
        t0 = time.perf_counter()
        query_embedding = self.get_embeddings(keyframe)
        query_embedding_norm = np.linalg.norm(query_embedding)
        if query_embedding_norm > 0:
            query_embedding = query_embedding / query_embedding_norm
        timings["embedding"] = timings.get("embedding", 0.0) + (time.perf_counter() - t0) * 1000.0

        query_kp = keyframe_features["kpts"][0] if keyframe_features["kpts"].ndim == 3 else keyframe_features["kpts"]
        query_desc = keyframe_features["descps"][0] if keyframe_features["descps"].ndim == 3 else keyframe_features["descps"]
        # Retrieval and matching do not have to run on the same descriptor. The DBoW3
        # vocabulary is trained on ORB, so a SuperPoint front end cannot query it, and
        # retraining is a bigger change than it looks. Setting retrieval_extractor keeps
        # retrieval on ORB while matching uses whatever self.extractor produced.
        retrieval_kp, retrieval_desc = query_kp, query_desc
        if self.retrieval_extractor is not None:
            rf = asyncio.run(self.retrieval_extractor.infer(keyframe))
            retrieval_kp = rf["kpts"][0] if rf["kpts"].ndim == 3 else rf["kpts"]
            retrieval_desc = rf["descps"][0] if rf["descps"].ndim == 3 else rf["descps"]
        t0 = time.perf_counter()
        if self.vlad_centres is not None:
            from tinynav.core.vlad import compute_vlad
            query_embedding = compute_vlad(retrieval_desc, self.vlad_centres)
        candidates = self.map_loop_closure.find_candidate_timestamps(
            retrieval_kp,
            retrieval_desc,
            query_embedding,
            top_k=self.relocalization_loop_top_k,
        )
        timings["candidate_search"] = timings.get("candidate_search", 0.0) + (time.perf_counter() - t0) * 1000.0
        max_similarity = max([c["similarity"] for c in candidates]) if len(candidates) > 0 else 0
        stats["query_kpts"] = int(len(query_kp))
        stats["candidates"] = len(candidates)
        stats["top_sim"] = float(max_similarity)
        stats["cand_ts"] = [int(c["timestamp"]) for c in candidates]
        stats["cand_sims"] = [float(c["similarity"]) for c in candidates]
        if len(candidates) > 0:
            point_3d_in_world_arrays = []
            point_2d_in_keyframe_arrays = []
            candidate_summaries = []
            candidate_timing_summaries = []
            for candidate in candidates:
                timestamp_in_map = int(candidate["timestamp"])
                similarity = float(candidate["similarity"])
                reference_keyframe_pose = self.map_poses[timestamp_in_map]
                cand_xyz = np.asarray(reference_keyframe_pose)[:3, 3]
                cand_jump = (float(np.linalg.norm(cand_xyz - self._last_reloc_ref_xyz))
                             if self._last_reloc_ref_xyz is not None else float("nan"))
                t_db = time.perf_counter()
                reference_depth, _, reference_features, _, _ = self.db.get_depth_embedding_features_images(timestamp_in_map)
                # A map built with ORB has only ORB descriptors, so evaluating a
                # different feature layer means supplying its descriptors for the same
                # keyframes rather than rebuilding the map -- which also keeps the
                # keyframe set and poses identical between the two, so the comparison
                # measures the descriptor and nothing else. The depth stays the map's.
                if self.alt_map_features is not None:
                    alt = self.alt_map_features.get(timestamp_in_map)
                    if alt is None:
                        continue
                    reference_features = alt
                db_ms = (time.perf_counter() - t_db) * 1000.0
                timings["db_load"] = timings.get("db_load", 0.0) + db_ms
                t_match = time.perf_counter()
                reference_matched_keypoints, keyframe_matched_keypoints, matches = self.match_keypoints(reference_features, keyframe_features)
                match_ms = (time.perf_counter() - t_match) * 1000.0
                timings["match"] = timings.get("match", 0.0) + match_ms
                depth_ms = 0.0
                if len(matches) >= self.reloc_min_matches:
                    t_depth = time.perf_counter()
                    point_3d_in_world, inliers = self.keypoint_with_depth_to_3d(reference_matched_keypoints, reference_depth, reference_keyframe_pose, self.map_K)
                    depth_ms = (time.perf_counter() - t_depth) * 1000.0
                    timings["depth3d"] = timings.get("depth3d", 0.0) + depth_ms
                    point_3d_in_world_arrays.append(point_3d_in_world[inliers])
                    point_2d_in_keyframe_arrays.append(keyframe_matched_keypoints[inliers])
                    candidate_summaries.append(
                        f"{timestamp_in_map}:sim={similarity:.3f},matches={len(matches)},"
                        f"valid_depth={int(np.count_nonzero(inliers))},jump={cand_jump:.1f}m"
                    )
                    stats["cand_valid_depth"].append(int(np.count_nonzero(inliers)))
                else:
                    candidate_summaries.append(
                        f"{timestamp_in_map}:sim={similarity:.3f},matches={len(matches)}"
                        f"<{self.reloc_min_matches},jump={cand_jump:.1f}m"
                    )
                    stats["cand_valid_depth"].append(0)
                stats["cand_matches"].append(int(len(matches)))
                candidate_timing_summaries.append(
                    f"{timestamp_in_map}:db={db_ms:.1f},match={match_ms:.1f},depth3d={depth_ms:.1f}"
                )

            landmark_count = int(sum(points.shape[0] for points in point_3d_in_world_arrays))
            stats["landmarks"] = landmark_count
            if landmark_count > self.reloc_min_landmarks:
                point_3d_in_world_list = np.concatenate(point_3d_in_world_arrays, axis=0)
                point_2d_in_keyframe_list = np.concatenate(point_2d_in_keyframe_arrays, axis=0)

                # A correspondence count says nothing about whether the geometry
                # constrains a pose. If the 2D observations are all (nearly) the
                # same pixel, every 3D point projects to that pixel from a camera
                # at effectively infinite distance, so PnP fits *perfectly* with
                # zero reprojection error and a 1.0 inlier ratio while the
                # translation is completely unobservable. Counting inliers cannot
                # detect that; measuring the spread of the observations can.
                spread_ok, spread_detail = self._observations_constrain_pose(point_2d_in_keyframe_list)
                if not spread_ok:
                    self.get_logger().info(f"Relocalization candidate timing ms: {'; '.join(candidate_timing_summaries)}")
                    return self._relocalization_failed(
                        f"degenerate 2D observations: {spread_detail}, "
                        f"landmarks={len(point_3d_in_world_list)}, "
                        f"candidates=[{'; '.join(candidate_summaries)}]",
                        "degenerate_2d",
                    )

                t_pnp = time.perf_counter()
                success, rvec, tvec, inliers = cv2.solvePnPRansac(point_3d_in_world_list, point_2d_in_keyframe_list, self.map_K, None)
                timings["pnp"] = timings.get("pnp", 0.0) + (time.perf_counter() - t_pnp) * 1000.0
                stats["pnp_inliers"] = 0 if inliers is None else int(len(inliers))
                # 判断降阈值是否站得住的唯一依据：少而准的匹配应当给出高内点率和小重投影误差。
                if success and inliers is not None and len(inliers) > 0:
                    idx = np.asarray(inliers).ravel()
                    proj, _ = cv2.projectPoints(point_3d_in_world_list[idx], rvec, tvec, self.map_K, None)
                    err = np.linalg.norm(proj.reshape(-1, 2) - point_2d_in_keyframe_list[idx], axis=1)
                    self.get_logger().info(
                        f"PnP quality: inliers={len(idx)}/{len(point_3d_in_world_list)} "
                        f"({len(idx) / len(point_3d_in_world_list):.0%}), gate>={self.reloc_min_inliers}, "
                        f"reproj px p50={float(np.median(err)):.2f} p90={float(np.percentile(err, 90)):.2f} "
                        f"max={float(err.max()):.2f}")
                # 绝对数之外再看占比。重投影误差不能当门槛用：它只在内点上算，内点越少越
                # 容易好看 —— 实测 7/49(14%) 的 p50 是 2.66 px，比 31/58(53%) 的 4.00 px
                # "更好"，那是 RANSAC 过拟合到少数点的症状，不是解更准。占比才能挡住它。
                inlier_ratio = (0.0 if inliers is None or len(point_3d_in_world_list) == 0
                                else len(inliers) / len(point_3d_in_world_list))
                if (success and len(inliers) >= self.reloc_min_inliers
                        and inlier_ratio >= self.reloc_min_inlier_ratio):
                    R, _ = cv2.Rodrigues(rvec)
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = tvec.reshape(3)
                    # Last line of defence: the camera has to end up somewhere the
                    # map actually covers. This catches any remaining ill-posed
                    # solve regardless of how it arose.
                    if self._first_reloc_wall is None:
                        self._first_reloc_wall = time.monotonic()
                        self.get_logger().info(
                            "time to first relocalization: "
                            f"{self._first_reloc_wall - self._node_start_wall:.1f}s since map_node start "
                            f"(map load was {self._map_load_s:.1f}s of it)")
                    if candidates:
                        self._last_reloc_ref_xyz = np.asarray(
                            self.map_poses[int(candidates[0]["timestamp"])])[:3, 3].copy()
                    pose_ok, pose_detail = self._pose_is_within_map(T)
                    if not pose_ok:
                        self.get_logger().info(f"Relocalization candidate timing ms: {'; '.join(candidate_timing_summaries)}")
                        return self._relocalization_failed(
                            f"relocalized pose outside the map: {pose_detail}, "
                            f"inliers={len(inliers)}/{len(point_2d_in_keyframe_list)}, "
                            f"candidates=[{'; '.join(candidate_summaries)}]",
                            "outside_map",
                        )
                    self.get_logger().info(f"Relocalization candidate timing ms: {'; '.join(candidate_timing_summaries)}")
                    # Successes carried only pose_weight, so there was no way to compare
                    # them against failures: the candidate similarities and the inlier
                    # margin appeared in the failure reason and nowhere else. That made
                    # the obvious question -- is the DBoW3 similarity discriminative
                    # enough to reject bad candidates before paying for the match --
                    # unanswerable from the logs, because only one side of the
                    # distribution was recorded. Failures show sim 0.003-0.051; without
                    # the same numbers on successes, any threshold is a guess.
                    self.last_relocalization_detail = (
                        f"inliers={len(inliers)}/{len(point_2d_in_keyframe_list)}, "
                        f"landmarks={len(point_3d_in_world_list)}, "
                        f"candidates=[{'; '.join(candidate_summaries)}]"
                    )
                    return True, T, len(inliers) / len(point_2d_in_keyframe_list)
                inlier_count = 0 if inliers is None else len(inliers)
                self.get_logger().info(f"Relocalization candidate timing ms: {'; '.join(candidate_timing_summaries)}")
                return self._relocalization_failed(
                    f"solvePnPRansac failed or insufficient inliers: success={success}, "
                    f"inliers={inlier_count}<{self.reloc_min_inliers}, landmarks={len(point_3d_in_world_list)}, "
                    f"candidates=[{'; '.join(candidate_summaries)}]",
                    "pnp_inliers",
                )
            else:
                self.get_logger().info(f"Relocalization candidate timing ms: {'; '.join(candidate_timing_summaries)}")
                # 报错要指对阶段。matches<20 时上面的深度分支根本不执行，valid_depth=0 是构造
                # 出来的，说"深度地标不够"会把人带去查深度 —— 实测失败的都是这一类：候选
                # matches 11~23（成功时约 180），sim 0.051~0.061（成功时 0.078~0.085）。
                #
                # cand_spread 是检索对不对的干净判据：检索对的时候前 3 名是地图里相邻的关键帧
                # （实测跨度 <1 s），错的时候散在地图各处（实测 173 s）—— 那是"没有真匹配、
                # 只是噪声排序"的典型形状。
                cand_ts = stats.get("cand_ts") or []
                spread_s = ((max(cand_ts) - min(cand_ts)) / 1e9) if len(cand_ts) > 1 else 0.0
                best_matches = max(stats["cand_matches"]) if stats["cand_matches"] else 0
                kept = sum(stats["cand_valid_depth"]) if stats["cand_valid_depth"] else 0
                # 归因给深度只有在深度真的丢掉了东西时才成立。实测有一例 23 个匹配全部带有效
                # 深度、短缺纯粹是匹配太少 —— 那还是检索的问题，标成 depth 会把人带错方向。
                if best_matches < 20:
                    reason_kind = (f"retrieval: no candidate reached {self.reloc_min_matches} matches "
                                   f"(best {best_matches})")
                elif kept >= landmark_count and landmark_count == best_matches:
                    reason_kind = (f"retrieval: depth kept all {kept} matches, there were "
                                   f"just too few (best candidate {best_matches})")
                else:
                    reason_kind = (f"depth: {best_matches} matches on the best candidate but "
                                   f"only {landmark_count} usable 3D landmarks")
                return self._relocalization_failed(
                    f"{reason_kind} (need >{self.reloc_min_landmarks} landmarks, got {landmark_count}), "
                    f"cand_spread={spread_s:.1f}s, "
                    f"candidates=[{'; '.join(candidate_summaries)}]",
                    "retrieval_miss" if reason_kind.startswith("retrieval") else "few_landmarks",
                )
        else:
            return self._relocalization_failed(
                f"no loop candidates above threshold: candidates={len(candidates)}, max_similarity={max_similarity:.3f}",
                "no_candidates",
            )
        return self._relocalization_failed("unknown relocalization failure")

    # A pose is only determined if the bearings to the landmarks actually differ.
    # These two numbers are deliberately loose -- they are there to reject the
    # pathological cases (everything on one pixel, or a handful of pixels), not to
    # second-guess a genuine solve. A healthy relocalization spreads its
    # observations over ~100 px of a 544x640 image.
    min_distinct_observations = 20
    min_observation_spread_px = 8.0

    def _observations_constrain_pose(self, points_2d: np.ndarray) -> tuple[bool, str]:
        """Reject 2D correspondence sets too concentrated to determine a pose."""
        if points_2d.shape[0] == 0:
            return False, "no observations"
        # Round to a tenth of a pixel before counting: the failure mode is exact
        # duplication (many reference keypoints matched to one query keypoint),
        # and rounding keeps float noise from hiding it.
        distinct = np.unique(np.round(points_2d[:, :2], 1), axis=0).shape[0]
        # RMS distance from the centroid, i.e. how wide a patch the observations
        # cover. Zero means they are all the same point.
        spread = float(np.sqrt(points_2d[:, :2].var(axis=0).sum()))
        detail = f"distinct={distinct}, spread={spread:.2f}px"
        if distinct < self.min_distinct_observations:
            return False, f"{detail} (need distinct >= {self.min_distinct_observations})"
        if spread < self.min_observation_spread_px:
            return False, f"{detail} (need spread >= {self.min_observation_spread_px}px)"
        return True, detail

    # How far outside the mapped area a relocalized camera may still land. The
    # map is metric and bounded, so this only has to be generous enough never to
    # reject a plausible pose; it exists to catch the absurd ones.
    max_distance_outside_map_m = 50.0

    def _map_bounds(self) -> tuple[np.ndarray, float] | None:
        """Centroid and radius of the mapped keyframe positions, computed once."""
        cached = getattr(self, "_map_bounds_cache", None)
        if cached is not None:
            return cached
        if not self.map_poses:
            return None
        positions = np.array([pose[:3, 3] for pose in self.map_poses.values()])
        centre = positions.mean(axis=0)
        radius = float(np.linalg.norm(positions - centre, axis=1).max())
        self._map_bounds_cache = (centre, radius)
        return self._map_bounds_cache

    def _pose_is_within_map(self, pose_world_to_camera: np.ndarray) -> tuple[bool, str]:
        """Check the solved camera centre sits somewhere the map covers."""
        bounds = self._map_bounds()
        if bounds is None:
            return True, "map bounds unavailable"
        centre, radius = bounds
        # solvePnPRansac returns world->camera; the camera position in world is
        # -R^T t.
        rotation = pose_world_to_camera[:3, :3]
        translation = pose_world_to_camera[:3, 3]
        camera_in_world = -rotation.T @ translation
        if not np.all(np.isfinite(camera_in_world)):
            return False, "camera position is not finite"
        distance = float(np.linalg.norm(camera_in_world - centre))
        limit = radius + self.max_distance_outside_map_m
        detail = f"|camera - map_centre|={distance:.2f}m, map_radius={radius:.2f}m, limit={limit:.2f}m"
        return distance <= limit, detail

    def keypoint_with_depth_to_3d(self, keypoints:np.ndarray, depth:np.ndarray, pose_from_camera_to_world:np.ndarray, K:np.ndarray):
        point_in_camera = []
        inliers = []
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        for kp in keypoints:
            u = int(kp[0])
            v = int(kp[1])
            Z = depth[v, u]
            if Z > 0 and Z < 50:
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                inliers.append(True)
            else:
                X = 0
                Y = 0
                inliers.append(False)
            point_in_camera.append(np.array([X, Y, Z]))
        # shape: (N, 3)
        point_in_camera = np.array(point_in_camera)
        inliers = np.array(inliers)
        rotation = pose_from_camera_to_world[:3, :3]
        translation = pose_from_camera_to_world[:3,3]

        point_in_world = (rotation @ point_in_camera.T).T + translation
        return point_in_world, inliers

    @Timer(name="Relocalization loop", text="\n\n[{name}] Elapsed time: {milliseconds:.0f} ms")
    def keyframe_relocalization(self, timestamp, image:np.ndarray) -> tuple[bool, np.ndarray]:
        timestamp_ns = int(timestamp.sec * 1e9) + int(timestamp.nanosec)
        loop_t0 = time.perf_counter()
        timings = {}
        t0 = time.perf_counter()
        features = asyncio.run(self.extractor.infer(image))
        timings["feature_extract"] = (time.perf_counter() - t0) * 1000.0
        res, pose_in_camera, pose_cov_weight = self.relocalize_with_depth(image, features, self.K, timings)
        if res:
            t0 = time.perf_counter()
            # publish the relocalization pose for debug
            pose_in_world = se3_inv(pose_in_camera)
            self.relocation_pub.publish(np2msg(pose_in_world, timestamp, "world", "camera"))
            self.relocalization_poses[timestamp_ns] = pose_in_world
            self.relocalization_pose_weights[timestamp_ns] = pose_cov_weight
            # 候选跨度：检索对不对的干净判据（对的时候前 3 名是地图里相邻的关键帧）。
            st = self.last_relocalization_stats or {}
            cand_ts = st.get("cand_ts") or []
            span_s = ((max(cand_ts) - min(cand_ts)) / 1e9) if len(cand_ts) > 1 else 0.0
            self.relocalization_pose_quality[timestamp_ns] = (span_s, float(pose_cov_weight))
            # 位姿本身以前从来没进过日志，所以「停着时是不是定位对了」只能靠推断 ——
            # 2026-08-27 分析那次 5.34 m 跳变时就卡在这里。ref 是最佳候选在地图里的位置，
            # 它和 cam_map 的距离就是「解出来的位置离它参考的关键帧有多远」。
            ref_txt = "-"
            if cand_ts and st.get("cand_matches"):
                best_i = int(np.argmax(st["cand_matches"]))
                ref_pose = (self.map_poses.get(cand_ts[best_i])
                            if best_i < len(cand_ts) else None)
                if ref_pose is not None:
                    rp = np.asarray(ref_pose)[:3, 3]
                    ref_txt = f"[{rp[0]:+.2f},{rp[1]:+.2f},{rp[2]:+.2f}]"
            cm = pose_in_world[:3, 3]
            self.get_logger().info(
                f"reloc pose: t={timestamp_ns} "
                f"cam_map=[{cm[0]:+.2f},{cm[1]:+.2f},{cm[2]:+.2f}] "
                f"yaw_map={self._ground_yaw_deg(pose_in_world):+.1f}deg "
                f"ref_map={ref_txt} cand_span={span_s:.1f}s ratio={pose_cov_weight:.2f}"
            )
            timings["publish"] = (time.perf_counter() - t0) * 1000.0
            timings["unaccounted"] = max(0.0, (time.perf_counter() - loop_t0) * 1000.0 - sum(timings.values()))
            self.last_relocalization_timing = dict(timings)
            self._log_relocalization_timing(
                timestamp_ns,
                True,
                timings,
                extra=f"pose_weight={pose_cov_weight:.3f}, {self.last_relocalization_detail}",
            )
            return True, pose_in_world
        else:
            self.failed_relocalizations.append(timestamp)
            reason = self.last_relocalization_failure_reason or "unknown relocalization failure"
            timings["unaccounted"] = max(0.0, (time.perf_counter() - loop_t0) * 1000.0 - sum(timings.values()))
            self.last_relocalization_timing = dict(timings)
            self._log_relocalization_timing(timestamp_ns, False, timings, extra=f"reason={reason}")
            self.get_logger().warning(
                f"Relocalization failed at timestamp={timestamp_ns}: {reason}"
            )
            return False, np.eye(4)

    def save_relocalization_poses(self):
        if self._save_completed:
            self.get_logger().info("Relocalization data already saved, skipping duplicate save")
            return

        print("saving localization data...")
        self.continuous_odom_recorder.save_to_disk()

        if len(self.relocalization_poses) == 0:
            self.get_logger().warning("No relocalization poses found - not saving")
            return

        np.save(f"{self.tinynav_db_path}/relocalization_poses.npy", self.relocalization_poses, allow_pickle=True)
        np.save(f"{self.tinynav_db_path}/relocalization_pose_weights.npy", self.relocalization_pose_weights, allow_pickle=True)
        np.save(f"{self.tinynav_db_path}/failed_relocalizations.npy", self.failed_relocalizations, allow_pickle=True)
        np.save(f"{self.tinynav_db_path}/poses.npy", self.pose_graph_used_pose, allow_pickle=True)

        logging.info(f"Saved {len(self.relocalization_poses)} relocalization poses to {self.tinynav_db_path}")
        logging.info(f"Failed relocalizations count: {len(self.failed_relocalizations)}")

        self._save_completed = True

    def destroy_node(self):
        try:
            self.save_relocalization_poses()
            self.nav_temp_db.close()
            self.db.close()
            super().destroy_node()
        except Exception:
            # Ignore errors during destruction as resources may already be freed
            pass


    @staticmethod
    def _ground_yaw_deg(T):
        """Ground-plane heading of a camera-optical pose. Body +z is forward."""
        fwd = np.asarray(T)[:3, :3] @ np.array([0.0, 0.0, 1.0])
        return math.degrees(math.atan2(float(fwd[1]), float(fwd[0])))

    def compute_transform_from_map_to_odom(self):
        """
        Solve the optmization problem.
        """
        relative_pose_constraint = []
        optimized_parameters = {
            0 : np.eye(4) if self.T_from_map_to_odom is None else self.T_from_map_to_odom,
            1 : np.eye(4),
        }
        constant_pose_index_dict = { 1: True }
        used_timestamps = []
        for timestamp, pose in self.relocalization_poses.items():
            if timestamp in self.pose_graph_used_pose:
                used_timestamps.append(timestamp)
                camera_in_map_world = pose
                camera_in_odom_world = self.pose_graph_used_pose[timestamp]
                observation_T_from_map_to_odom =  camera_in_odom_world @ se3_inv(camera_in_map_world)
                weight = self.relocalization_pose_weights[timestamp]

                relative_pose_constraint.append((0, 1, observation_T_from_map_to_odom, weight * np.array([10.0, 10.0, 10.0]), weight * np.array([10.0, 10.0, 10.0])))
        relative_pose_constraint = relative_pose_constraint[-100:]
        optimized_parameters = pose_graph_solve(optimized_parameters, relative_pose_constraint, constant_pose_index_dict, max_iteration_num = 1000)
        self.T_from_map_to_odom = optimized_parameters[0]

        # 这条是位姿跳变的直接证据：跳的是 T，不是位姿，而 T 只在这里变。附上约束成分是
        # 因为解由成分决定 —— 只按时间取最近 100 条、又没有鲁棒核，陈旧低质约束占多数时
        # 解会在两个盆地之间翻（2026-08-27 实测 5.34 m + 51 度）。
        # 解完之后立刻拿解去核对它自己的约束。2026-08-27 实测：81 条观测彼此一致
        # （朝向偏差中位 0.4 度、p90 2.9 度），而解出来的 T 在 45~60 条约束时把 odom 位姿
        # 投回地图的位置残差中位 9.46 m —— 也就是解根本没落在观测的中心。所以这条残差
        # 必须和 T 一起打出来，否则「约束不好」和「解算不对」在日志里长得一模一样。
        #
        # 同时算一个闭式对照：这里只有一个自由参数块（节点 1 固定为单位阵），本质是
        # 「把 N 个 SE(3) 观测平均起来」，不是位姿图。闭式解是确定性的、不会跑偏，
        # 拿它当基准就能判断非线性求解器有没有必要留着。
        T = self.T_from_map_to_odom
        obs = [c[2] for c in relative_pose_constraint]
        w = np.array([float(c[3][0]) for c in relative_pose_constraint])
        if obs:
            def _resid(Tx):
                te, re_ = [], []
                for Ob in obs:
                    E = se3_inv(Tx) @ Ob
                    te.append(float(np.linalg.norm(E[:3, 3])))
                    ct = (np.trace(E[:3, :3]) - 1.0) / 2.0
                    re_.append(math.degrees(math.acos(max(-1.0, min(1.0, ct)))))
                return np.array(te), np.array(re_)
            # 闭式：平移按权重加权平均，旋转用 SVD 投影回 SO(3)
            W = w / max(w.sum(), 1e-9)
            t_cf = np.sum([W[i] * obs[i][:3, 3] for i in range(len(obs))], axis=0)
            R_acc = np.sum([W[i] * obs[i][:3, :3] for i in range(len(obs))], axis=0)
            U, _, Vt = np.linalg.svd(R_acc)
            R_cf = U @ np.diag([1.0, 1.0, float(np.linalg.det(U @ Vt))]) @ Vt
            T_cf = np.eye(4); T_cf[:3, :3] = R_cf; T_cf[:3, 3] = t_cf
            te_s, re_s = _resid(T)
            te_c, re_c = _resid(T_cf)
            gap = float(np.linalg.norm(T[:3, 3] - T_cf[:3, 3]))
            self.get_logger().info(
                f"map->odom fit: solver resid t_p50={np.median(te_s):.2f}m "
                f"r_p50={np.median(re_s):.1f}deg | closed-form resid t_p50={np.median(te_c):.2f}m "
                f"r_p50={np.median(re_c):.1f}deg | gap={gap:.2f}m "
                f"cf_yaw={self._ground_yaw_deg(T_cf):+.1f}deg"
            )
        prev = self._last_T_map_to_odom_logged
        d_t = float(np.linalg.norm(T[:3, 3] - prev[:3, 3])) if prev is not None else 0.0
        d_yaw = (self._ground_yaw_deg(T) - self._ground_yaw_deg(prev)) if prev is not None else 0.0
        d_yaw = (d_yaw + 180.0) % 360.0 - 180.0
        self._last_T_map_to_odom_logged = T.copy()
        used = [self.relocalization_pose_quality.get(ts) for ts in used_timestamps[-100:]]
        used = [q for q in used if q is not None]
        clean = sum(1 for sp, r in used if sp <= 2.0 and r >= 0.70)
        spans = sorted(sp for sp, _ in used) or [0.0]
        ratios = sorted(r for _, r in used) or [0.0]
        self.get_logger().info(
            f"map->odom: t=[{T[0,3]:+.2f},{T[1,3]:+.2f},{T[2,3]:+.2f}] "
            f"yaw={self._ground_yaw_deg(T):+.1f}deg "
            f"tilt={math.degrees(math.acos(max(-1.0, min(1.0, float(T[2,2]))))):.1f}deg "
            f"d_t={d_t:.3f}m d_yaw={d_yaw:+.1f}deg "
            f"constraints={len(relative_pose_constraint)} clean={clean}/{len(used)} "
            f"span_p50={spans[len(spans)//2]:.1f}s ratio_p50={ratios[len(ratios)//2]:.2f}"
        )

    def try_publish_nav_path(self, timestamp: int):
        t_start = time.perf_counter()
        stage_timings = {}

        def mark_stage(name: str, previous_t: float) -> float:
            now = time.perf_counter()
            stage_timings[name] = (now - previous_t) * 1000.0
            return now

        def log_nav_timing(result: str, path_count: int | None = None):
            total_ms = (time.perf_counter() - t_start) * 1000.0
            parts = ", ".join(f"{name}={ms:.1f}" for name, ms in stage_timings.items())
            msg = f"Nav path timing ms: timestamp={timestamp}, total={total_ms:.1f}, result={result}"
            if path_count is not None:
                msg += f", path_count={path_count}"
            if parts:
                msg += f", {parts}"
            if total_ms > 500.0:
                self.get_logger().warning(msg)
            else:
                self.get_logger().info(msg)

        t_stage = t_start
        self.get_logger().info(f"try_publish_nav_path, timestamp: {timestamp}")
        t_stage = mark_stage("start_log", t_stage)
        if self.T_from_map_to_odom is None:
            self.get_logger().info("Relocalization not successful yet, skip publishing nav path")
            log_nav_timing("skip_no_relocalization")
            return

        pose_in_map = se3_inv(self.T_from_map_to_odom) @ self.pose_graph_used_pose[timestamp]
        self.current_pose_in_map_pub.publish(np2msg(pose_in_map, self.get_clock().now().to_msg(), "world", "map"))
        pose_in_map_position = pose_in_map[:3, 3]
        t_stage = mark_stage("pose_in_map_publish", t_stage)

        if self.poi_index == -1:
            self.get_logger().info("No POI found, skip publishing nav path")
            log_nav_timing("skip_no_poi")
            return

        if self.poi_index >= len(self.pois):
            self.get_logger().info("All POIs have been visited, skip publishing nav path")
            log_nav_timing("skip_all_pois_visited")
            return

        poi = self.pois[self.poi_index]
        print(f"poi: {poi}")
        poi_pose = np.eye(4)
        poi_pose[:3, 3] = poi
        self.poi_pub.publish(np2msg(poi_pose, self.get_clock().now().to_msg(), "world", "map"))
        t_stage = mark_stage("poi_publish", t_stage)

        advanced_poi_count = 0
        while self.poi_index < len(self.pois):
            poi = self.pois[self.poi_index]
            diff_position_norm_xy = np.linalg.norm(poi[:2] - pose_in_map_position[:2])
            dz = float(poi[2] - pose_in_map_position[2])  # logged only, not a condition
            # The one number that decides whether a run counts as arrived, and it was
            # nowhere in the log -- the closest approach had to be reconstructed from
            # robot_map minus the POI by hand.
            self.get_logger().info(
                f"poi {self.poi_index}/{len(self.pois)}: dist_xy={diff_position_norm_xy:.3f}m "
                f"(radius {self.POI_ARRIVAL_RADIUS_XY_M:.2f}m) dz={dz:+.2f}m(ignored) "
                f"robot_map=[{pose_in_map_position[0]:.2f},{pose_in_map_position[1]:.2f}] "
                f"poi_map=[{poi[0]:.2f},{poi[1]:.2f}]",
                throttle_duration_sec=1.0,
            )
            if diff_position_norm_xy < self.POI_ARRIVAL_RADIUS_XY_M:
                # Emit the 100% frame *before* advancing the index, otherwise the UI's
                # last observed progress for this POI is whatever partial value the
                # previous keyframe happened to publish, and the bar never fills.
                if self._leg_initial_length is not None:
                    arrived_msg = String()
                    arrived_msg.data = json.dumps({
                        "poi_index": self.poi_index,
                        "percent": 100.0,
                        "path_remaining_m": 0.0,
                        "path_total_m": round(self._leg_initial_length, 2),
                        "estimated_remaining_s": 0.0,
                    })
                    self.nav_progress_pub.publish(arrived_msg)
                self.poi_index += 1
                advanced_poi_count += 1
                self._leg_initial_length = None
                self._leg_start_time = None
                dummy_pose = np.eye(4)

                stamp_msg = self.get_clock().now().to_msg()
                stamp_msg.sec = int(timestamp / 1e9)
                stamp_msg.nanosec = int(timestamp % 1e9)
                self.poi_change_pub.publish(np2msg(dummy_pose, stamp_msg, "world", "map"))
                continue
            else:
                break
        t_stage = mark_stage("poi_advance", t_stage)
        self._publish_poi_status(pose_in_map_position, advanced_poi_count)

        if self.poi_index >= len(self.pois):
            # One-shot, guarded by the flag: this branch runs on every keyframe once
            # the last POI is reached, and the backend turns each Bool into a state
            # transition back to idle.
            if not self._nav_completed:
                self._nav_completed = True
                self.nav_done_pub.publish(Bool(data=True))
            self.get_logger().info("All POIs have been visited, skip publishing nav path")
            log_nav_timing(f"skip_all_pois_visited_after_advance:{advanced_poi_count}")
            return

        target_poi = self.pois[self.poi_index]
        with Timer(name = "generate nav path in map", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.timer_logger):
            paths_in_map = self.generate_nav_path_in_map(pose_in_map = pose_in_map, target_poi = target_poi)
        t_stage = mark_stage("generate_path", t_stage)

        if paths_in_map is not None:
            # xy only: the map's path points span 0.4 m in z, which inflated both the
            # progress percentage and the ETA on a robot that cannot change height.
            remaining_length = sum(
                np.linalg.norm(paths_in_map[i + 1][:2] - paths_in_map[i][:2])
                for i in range(len(paths_in_map) - 1)
            ) if len(paths_in_map) > 1 else 0.0

            # monotonic, NOT time.time(): the X5 has no battery-backed RTC, so the wall
            # clock jumps ~344 days the moment sync_board_time.sh runs. A wall-clock
            # elapsed would go negative or enormous and the speed estimate with it.
            now = time.monotonic()
            if self._leg_initial_length is None:
                self._leg_initial_length = remaining_length
                self._leg_start_time = now

            covered = self._leg_initial_length - remaining_length
            elapsed = now - self._leg_start_time
            if covered > 0.1 and elapsed > 1.0:
                self._speed_estimate = covered / elapsed

            initial = self._leg_initial_length
            percent = max(0.0, min(100.0, covered / initial * 100.0)) if initial > 0 else 0.0
            estimated_remaining_s = remaining_length / self._speed_estimate if self._speed_estimate else -1.0

            progress_msg = String()
            progress_msg.data = json.dumps({
                "poi_index": self.poi_index,
                "percent": round(percent, 1),
                "path_remaining_m": round(remaining_length, 2),
                "path_total_m": round(initial, 2),
                "estimated_remaining_s": round(estimated_remaining_s, 1),
            })
            self.nav_progress_pub.publish(progress_msg)

            # Pure pursuit: the first path point that is far enough from the robot in a
            # straight line. Walking the path by arc length instead put the target 0.20 m
            # from the robot after spending 2.23 m of a 3.98 m path (2026-08-12): the
            # global path doubles back on itself -- see nav_obstacle_tuning.md section 18
            # -- so arc length gets eaten by a fold that goes nowhere, and the planner's
            # 100*dist term then throttles vx to zero. Straight-line distance is immune:
            # a fold does not get further away, so the scan simply walks past it.
            with Timer(name = "Find target position", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.timer_logger):
                lookahead_m = self.nav_lookahead_m
                path_arr_xy = np.asarray(paths_in_map, dtype=float)[:, :2]
                robot_xy = np.asarray(pose_in_map_position[:2], dtype=float)
                radial = np.linalg.norm(path_arr_xy - robot_xy, axis=1)
                beyond = np.flatnonzero(radial >= lookahead_m)
                # Earliest qualifying point, not the nearest one: order along the path is
                # what makes this follow the route rather than cut to whatever end is
                # closest. Nothing qualifies only when the whole path is inside the
                # radius, and then the far end is the best available aim point.
                chosen_index = int(beyond[0]) if len(beyond) else len(paths_in_map) - 1
                target_position = paths_in_map[chosen_index]
                accumulated_distance = float(np.sum(np.linalg.norm(
                    np.diff(path_arr_xy[:chosen_index + 1], axis=0), axis=1
                ))) if chosen_index > 0 else 0.0
                target_position_in_map = np.array([target_position[0], target_position[1], target_position[2]])
                pose_in_origin_odom = self.odom[timestamp]
                T = pose_in_origin_odom @ se3_inv(pose_in_map)
                target_position_in_odom = T[:3, :3] @ target_position_in_map + T[:3, 3]
                dummy_pose = np.eye(4)
                dummy_pose[:3, 3] = target_position_in_odom
                # Instrumented 2026-08-10 because the published target kept landing
                # 0.1-0.2 m from the robot with 27 points in the path and this block
                # reported none of the quantities that would say why. arc_to is the arc
                # length spent reaching the chosen point: comparing it against
                # robot_to_target_xy is what exposes a folded path, since a straight
                # path makes them nearly equal and a fold makes arc_to much larger.
                # The world frame is z-up, so the third component is height and carries
                # no horizontal information -- reading it as "forward" gives a plausible
                # wrong answer, which is what these labels exist to prevent.
                path_arr = np.asarray(paths_in_map, dtype=float)
                path_len_m = (
                    float(np.sum(np.linalg.norm(np.diff(path_arr[:, :3], axis=0), axis=1)))
                    if len(path_arr) > 1 else 0.0
                )
                self.get_logger().info(
                    f"nav target: points={len(path_arr)} path_len={path_len_m:.2f}m "
                    f"lookahead={lookahead_m:.1f}m arc_to={accumulated_distance:.2f}m "
                    f"index={chosen_index}/{len(path_arr) - 1} "
                    f"is_final={chosen_index == len(paths_in_map) - 1} "
                    f"robot_to_target_xy="
                    f"{float(np.linalg.norm((target_position_in_map - pose_in_map_position)[:2])):.3f}m "
                    f"robot_map={np.round(pose_in_map_position[:3], 2)} "
                    f"target_map={np.round(target_position_in_map, 2)} "
                    f"target_odom={np.round(target_position_in_odom, 2)}",
                    throttle_duration_sec=1.0,
                )
            t_stage = mark_stage("target_select", t_stage)

            self.target_pose_pub.publish(np2msg(dummy_pose, self.get_clock().now().to_msg(), "world", "camera"))
            t_stage = mark_stage("target_pose_publish", t_stage)

            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            for x, y, z in paths_in_map:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            t_stage = mark_stage("global_path_msg_build", t_stage)

            self.global_plan_pub.publish(path_msg)
            # 判"target 跳变"的分水岭:/control/target_pose 本来就是沿路径滚动的前视点，所以
            # 它动是正常的。只有这条路径自己变了，跳变才是缺陷。记首尾点和长度就够对上。
            head = paths_in_map[0] if len(paths_in_map) else None
            tail = paths_in_map[-1] if len(paths_in_map) else None
            sig = (len(paths_in_map), None if head is None else tuple(round(v, 2) for v in head),
                   None if tail is None else tuple(round(v, 2) for v in tail))
            if sig != self._last_nav_path_sig:
                prev = self._last_nav_path_sig
                self._last_nav_path_sig = sig
                self.get_logger().info(
                    f"nav path changed: n={sig[0]} head={sig[1]} tail={sig[2]}"
                    + (f"  (was n={prev[0]} head={prev[1]} tail={prev[2]})" if prev else "  (first)")
                )
            t_stage = mark_stage("global_path_publish", t_stage)

            self.tf_broadcaster.sendTransform(np2tf(T, self.get_clock().now().to_msg(), "world", "map"))
            mark_stage("tf_publish", t_stage)
            log_nav_timing("published", path_count=len(paths_in_map))
        else:
            logging.info("No path found in map")
            log_nav_timing("skip_no_path")

    def generate_nav_path_in_map(self, pose_in_map: np.ndarray, target_poi: np.ndarray) -> np.ndarray:
        t_start = time.perf_counter()
        stage_timings = {}

        def mark_stage(name: str, previous_t: float) -> float:
            now = time.perf_counter()
            stage_timings[name] = (now - previous_t) * 1000.0
            return now

        def log_generate_timing(
            result: str,
            start_path_count: int = 0,
            goal_path_count: int = 0,
            path_sdf_count: int = 0,
            path_count: int = 0,
        ):
            total_ms = (time.perf_counter() - t_start) * 1000.0
            parts = ", ".join(f"{name}={ms:.1f}" for name, ms in stage_timings.items())
            msg = (
                f"Generate nav path timing ms: total={total_ms:.1f}, result={result}, "
                f"start_path_count={start_path_count}, goal_path_count={goal_path_count}, "
                f"path_sdf_count={path_sdf_count}, path_count={path_count}"
            )
            if parts:
                msg += f", {parts}"
            if total_ms > 500.0:
                self.get_logger().warning(msg)
            else:
                self.get_logger().info(msg)

        t_stage = t_start
        dummy_poi_pose = np.eye(4)
        dummy_poi_pose[:3, 3] = target_poi
        self.poi_pub.publish(np2msg(dummy_poi_pose, self.get_clock().now().to_msg(), "world", "map"))
        t_stage = mark_stage("poi_publish", t_stage)

        occupancy_map_origin = self.occupancy_map_meta[:3]
        resolution = self.occupancy_map_meta[3]
        start_idx = np.array([
            int((pose_in_map[0, 3] - occupancy_map_origin[0]) / resolution),
            int((pose_in_map[1, 3] - occupancy_map_origin[1]) / resolution),
            int((pose_in_map[2, 3] - occupancy_map_origin[2]) / resolution)
        ], dtype=np.int32)
        poi_goal_idx = np.array([
            int((target_poi[0] - occupancy_map_origin[0]) / resolution),
            int((target_poi[1] - occupancy_map_origin[1]) / resolution),
            int((target_poi[2] - occupancy_map_origin[2]) / resolution)
        ], dtype=np.int32)
        t_stage = mark_stage("index_convert", t_stage)

        if (
            start_idx[0] < 0
            or start_idx[0] >= self.occupancy_map.shape[0]
            or start_idx[1] < 0
            or start_idx[1] >= self.occupancy_map.shape[1]
            or start_idx[2] < 0
            or start_idx[2] >= self.occupancy_map.shape[2]
            or poi_goal_idx[0] < 0
            or poi_goal_idx[0] >= self.occupancy_map.shape[0]
            or poi_goal_idx[1] < 0
            or poi_goal_idx[1] >= self.occupancy_map.shape[1]
            or poi_goal_idx[2] < 0
            or poi_goal_idx[2] >= self.occupancy_map.shape[2]
        ):
            log_generate_timing("out_of_bounds")
            return None 

        # Normally a no-op: the warmup thread started in __init__ has long since
        # finished by the time a POI arrives. Present so that a path search which
        # does get here first waits for the JIT rather than racing it.
        self._ensure_nav_path_search_warm()
        t_stage = mark_stage("nav_warmup_wait", t_stage)

        sdf_start_path = search_close_to_sdf_map_numba(start_idx, self.sdf_map, self.occupancy_map, 0.2)
        t_stage = mark_stage("search_start_close", t_stage)
        sdf_goal_path = search_close_to_sdf_map_numba(poi_goal_idx, self.sdf_map, self.occupancy_map, 0.2)
        t_stage = mark_stage("search_goal_close", t_stage)

        if len(sdf_start_path) == 0 or len(sdf_goal_path) == 0:
            self.get_logger().warning(
                f"search_close_to_sdf_map returned empty path: start_count={len(sdf_start_path)}, goal_count={len(sdf_goal_path)}"
            )
            log_generate_timing("empty_close_path", start_path_count=len(sdf_start_path), goal_path_count=len(sdf_goal_path))
            return None

        sdf_start_sdf = sdf_start_path[-1]
        sdf_goal_sdf = sdf_goal_path[-1]
        path_sdf = search_within_sdf_map_numba(sdf_start_sdf, sdf_goal_sdf, self.sdf_map, self.occupancy_map, resolution)
        t_stage = mark_stage("search_within", t_stage)
        if len(path_sdf) == 0:
            self.get_logger().warning(
                f"search_within_sdf_map returned empty path: start_idx={tuple(sdf_start_sdf)}, goal_idx={tuple(sdf_goal_sdf)}"
            )
        path = np.vstack((sdf_start_path, path_sdf, sdf_goal_path[::-1]))
        t_stage = mark_stage("path_stack", t_stage)
        if len(path) > 0:
            converted_path = path.astype(np.float64) * resolution + occupancy_map_origin
            mark_stage("path_convert", t_stage)
            log_generate_timing(
                "ok",
                start_path_count=len(sdf_start_path),
                goal_path_count=len(sdf_goal_path),
                path_sdf_count=len(path_sdf),
                path_count=len(path),
            )
            return converted_path
        log_generate_timing(
            "empty_path",
            start_path_count=len(sdf_start_path),
            goal_path_count=len(sdf_goal_path),
            path_sdf_count=len(path_sdf),
        )
        return None

def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(filename)s:%(lineno)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--tinynav_db_path", type=str, default="tinynav_temp")
    parser.add_argument("--tinynav_map_path", type=str, required=True)
    # Default off: codetiming's default logger is print(), node_manager redirects stdout
    # into an unrotated file on the board's eMMC, and these fire per keyframe.
    parser.add_argument("--verbose_timer", action="store_true", default=False, help="Enable verbose timer output")
    parser.add_argument("--no_verbose_timer", dest="verbose_timer", action="store_false", help="Disable verbose timer output")
    parser.add_argument("--loop-closure-mode", type=str, default="embedding",
                        choices=["embedding", "bow", "vlad"])
    parser.add_argument("--vlad-centres", type=str, default=None,
                        help="npz/npy holding the frozen VLAD vocabulary; required by --loop-closure-mode vlad")
    parser.add_argument("--loop-closure-use-bow", action="store_true", help="Use ORB+BF and DBoW3 for loop closure")
    parser.add_argument(
        "--dbow3-vocabulary-path",
        type=str,
        default="/tinynav/docs/Vocabulary/ORBvoc.txt",
        help="DBoW3 vocabulary path for bow mode",
    )
    parsed_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    use_bow = parsed_args.loop_closure_use_bow or parsed_args.loop_closure_mode == "bow"
    if use_bow:
        parsed_args.loop_closure_mode = "bow"
        if not os.path.exists(parsed_args.dbow3_vocabulary_path):
            raise FileNotFoundError(
                f"DBoW3 vocabulary file not found: {parsed_args.dbow3_vocabulary_path}"
            )
        extractor = ORBFeatureTRTCompatible()
        matcher = ORBMatcher()
        embedding_extractor = DummyEmbeddingEngine()
    elif parsed_args.loop_closure_mode == "vlad":
        # VLAD shares one descriptor with retrieval, so the DINOv2 embedding has no
        # consumer left. LightGlue is out for a different reason: 2.87 s on the X5
        # against 38.6 ms for brute-force L2 over the same descriptors.
        parsed_args.vlad_centres = parsed_args.vlad_centres or DEFAULT_VLAD_CENTRES
        extractor = make_sp_extractor()
        matcher = SuperPointMatcher()
        embedding_extractor = DummyEmbeddingEngine()
    else:
        extractor = SuperPointTRT()
        matcher = LightGlueTRT()
        embedding_extractor = Dinov2TRT()

    node = MapNode(
        tinynav_db_path=parsed_args.tinynav_db_path,
        tinynav_map_path=parsed_args.tinynav_map_path,
        extractor=extractor,
        matcher=matcher,
        embedding_extractor=embedding_extractor,
        loop_closure_mode=parsed_args.loop_closure_mode,
        vlad_centres_path=parsed_args.vlad_centres,
        loop_closure_use_bow=use_bow,
        dbow3_vocabulary_path=parsed_args.dbow3_vocabulary_path,
        verbose_timer=parsed_args.verbose_timer,
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
