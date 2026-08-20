import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Float32
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tinynav.core.math_utils import matrix_to_quat, msg2np, estimate_pose, tf2np, depth_to_cloud
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import cv2
from codetiming import Timer
import os
import argparse
import sys
import time
from contextlib import contextmanager

from tinynav.tinynav_cpp_bind import pose_graph_solve
from tinynav.core.planning_node import run_raycasting_loopy
import logging
import asyncio
import shelve
import dbm.dumb
import pickle
from tqdm import tqdm
from tf2_msgs.msg import TFMessage
from typing import Callable, Dict, Optional
try:
    from tool.video_db import VideoDB
except ImportError:
    VideoDB = None

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped,Point
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import distance_transform_edt

from rclpy.executors import SingleThreadedExecutor
from rosbag2_py import ConverterOptions, Info, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock
from rclpy.serialization import deserialize_message



logger = logging.getLogger(__name__)

def check_global_frames_ratio(num_frames: int, prev_num_frames: int, frames_ratio: float) -> bool:
    """COLMAP-style gate for expensive global refinement steps."""
    return num_frames >= frames_ratio * prev_num_frames


class StageTimer:
    def __init__(self, verbose_logger: Optional[Callable[[str], None]] = None):
        self._stats = {}
        self.verbose_logger = verbose_logger

    def record(self, name: str, duration_s: float) -> None:
        stats = self._stats.setdefault(
            name,
            {"count": 0, "total_s": 0.0, "min_s": float("inf"), "max_s": 0.0},
        )
        stats["count"] += 1
        stats["total_s"] += duration_s
        stats["min_s"] = min(stats["min_s"], duration_s)
        stats["max_s"] = max(stats["max_s"], duration_s)
        if self.verbose_logger is not None:
            self.verbose_logger(f"[{name}] Elapsed time: {duration_s * 1000:.0f} ms")

    @contextmanager
    def timed(self, name: str):
        start_s = time.perf_counter()
        try:
            yield
        finally:
            self.record(name, time.perf_counter() - start_s)

    def log_summary(self, log_fn: Callable[[str], None]) -> None:
        if not self._stats:
            log_fn("No build-map stage timing data collected.")
            return
        total_s = sum(stats["total_s"] for stats in self._stats.values())
        lines = ["=== Build map stage timing ==="]
        lines.append("stage count total_s mean_ms min_ms max_ms pct")
        for name, stats in sorted(self._stats.items(), key=lambda item: -item[1]["total_s"]):
            count = stats["count"]
            mean_ms = (stats["total_s"] / count) * 1000.0 if count else 0.0
            pct = (100.0 * stats["total_s"] / total_s) if total_s > 0.0 else 0.0
            lines.append(
                f"{name} {count} {stats['total_s']:.3f} {mean_ms:.1f} "
                f"{stats['min_s'] * 1000.0:.1f} {stats['max_s'] * 1000.0:.1f} {pct:.1f}"
            )
        lines.append(f"Grand total: {total_s:.3f} s")
        log_fn("\n".join(lines))


def z_value_to_color(z, z_min, z_max):
    color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
    normalized_z = (z - z_min) / (z_max - z_min)
    if normalized_z < 0.25:
        color.g = normalized_z * 4.0
        color.b = 1.0
    elif normalized_z < 0.5:
        color.g = 1.0
        color.b = 1.0 - (normalized_z - 0.25) * 4.0
    elif normalized_z < 0.75:
        color.r = (normalized_z - 0.5) * 4.0
        color.g = 1.0
    else:
        color.r = 1.0
        color.g = 1.0 - (normalized_z - 0.75) * 4.0
    return color

def merge_local_into_global(global_grid:np.ndarray, global_origin:np.ndarray, local_grid:np.ndarray, local_origin:np.ndarray, resolution:float) -> tuple[np.ndarray, np.ndarray]:
    """
    Merge a local grid into a global grid.
    """
    resolution_half = np.array([resolution / 2.0, resolution / 2.0, resolution / 2.0], dtype=np.float32)
    local_origin_offset = ((local_origin - global_origin + resolution_half) / resolution).astype(np.int32)
    global_grid[local_origin_offset[0]:local_origin_offset[0] + local_grid.shape[0],
                local_origin_offset[1]:local_origin_offset[1] + local_grid.shape[1],
                local_origin_offset[2]:local_origin_offset[2] + local_grid.shape[2]] += local_grid

    return global_grid, global_origin

def solve_pose_graph(pose_graph_used_pose:dict, relative_pose_constraint:list, max_iteration_num:int = 1024) -> dict:
    """
    Solve the bundle adjustment problem.
    """
    if len(relative_pose_constraint) == 0:
        return pose_graph_used_pose
    min_timestamp = min(pose_graph_used_pose.keys())
    constant_pose_index_dict = { min_timestamp : True }

    relative_pose_constraint = [
        (curr_timestamp, prev_timestamp, T_prev_curr, np.array([10.0, 10.0, 10.0]), np.array([30.0, 30.0, 30.0]))
        for curr_timestamp, prev_timestamp, T_prev_curr in relative_pose_constraint]
    optimized_camera_poses = pose_graph_solve(pose_graph_used_pose, relative_pose_constraint, constant_pose_index_dict, max_iteration_num)
    return {t: optimized_camera_poses[t] for t in sorted(optimized_camera_poses.keys())}



# Similarity scales are backend-specific and must not be shared. Calibrated on map_gt
# (63 true loops vs all >5 m pairs) and map_gt<-map_night for relocalization:
#   DINOv2 CLS cosine  true loop ~0.9
#   VLAD cosine        true loop 0.37-0.43, unrelated pairs peak at 0.43 -> 0.35 keeps
#                      100% recall at ~1 false candidate per frame, and PnP's
#                      inliers>=100 gate is what actually accepts a loop.
# Relocalization is cross-session and its two populations overlap heavily (correct pairs
# p5=0.22 vs wrong pairs p95=0.29), so no threshold separates them; rank by top-k and let
# PnP decide, with the threshold only dropping hopeless candidates.
LOOP_CLOSURE_DEFAULTS = {
    #                  loop_thr, loop_top_k, reloc_thr, reloc_top_k
    "embedding": (0.90, 1, 0.85, 3),
    "bow":       (0.90, 1, 0.85, 3),   # DBoW3 scores are not calibrated here
    "vlad":      (0.35, 5, 0.10, 3),
}


def load_vlad_centres(path: str | None) -> np.ndarray | None:
    """Load a frozen VLAD vocabulary from .npy or single-array .npz."""
    if not path:
        return None
    data = np.load(path)
    if hasattr(data, "files"):
        data = data[data.files[0]]
    return np.ascontiguousarray(data, dtype=np.float32)


class LoopClosure:
    """
    Loop-closure candidate search over TinyNavDB keyframes.

    Input query is (kp, desc, embedding). Candidates are ranked by:
      1) embedding cosine similarity (embedding mode),
      2) DBoW3 score (bow mode), or
      3) SuperPoint VLAD cosine similarity (vlad mode).
    """

    def __init__(
        self,
        db: "TinyNavDB",
        timestamps: list[int],
        mode: str = "embedding",
        dbow3_vocabulary_path: str | None = None,
        embedding_similarity_threshold: float = 0.90,
        embedding_top_k: int = 20,
        dbow3_vocabulary=None,
        vlad_centres: np.ndarray | None = None,
    ):
        """
        Args:
            dbow3_vocabulary: an already-loaded pydbow3 Vocabulary to reuse
                instead of re-reading dbow3_vocabulary_path from disk. Pass this
                when several LoopClosure instances share one vocabulary; see
                DBoW3Engine.load_vocabulary().
            vlad_centres: (K, C) frozen VLAD vocabulary for mode='vlad'. Frozen, so a
                descriptor exists from the first keyframe -- unlike a per-map vocabulary,
                which cannot be trained until mapping has already finished.
        """
        self.db = db
        self.timestamps = list(timestamps)
        self.mode = mode
        self.embedding_similarity_threshold = float(embedding_similarity_threshold)
        self.embedding_top_k = int(embedding_top_k)
        self.timestamp_to_idx = {ts: i for i, ts in enumerate(self.timestamps)}
        self.dbow3_engine = None
        self.vlad_centres = None

        if self.mode not in ("embedding", "bow", "vlad"):
            raise ValueError(f"Unsupported LoopClosure mode: {self.mode}")

        if self.mode == "vlad":
            if vlad_centres is None:
                raise ValueError("vlad_centres is required when mode='vlad'")
            self.vlad_centres = np.ascontiguousarray(vlad_centres, dtype=np.float32)
            self.embeddings = np.zeros((0, self.vlad_centres.size), dtype=np.float32)
            rows = [self._vlad_for(ts) for ts in self.timestamps]
            if rows:
                # float32 on purpose: the A55 has no usable float16/int8 matmul, so a
                # narrower index costs 2-7x the search time to save memory.
                self.embeddings = np.ascontiguousarray(np.stack(rows, axis=0), dtype=np.float32)
        elif self.mode == "embedding":
            if len(self.timestamps) == 0:
                self.embeddings = np.zeros((0, 1), dtype=np.float32)
            else:
                self.embeddings = np.stack(
                    [self._normalize_embedding(self.db.get_embedding(ts)) for ts in self.timestamps],
                    axis=0,
                )
        else:
            self.embeddings = np.zeros((0, 1), dtype=np.float32)
            if dbow3_vocabulary_path is None and dbow3_vocabulary is None:
                raise ValueError("dbow3_vocabulary_path is required when mode='bow'")
            from tinynav.core.models_trt import DBoW3Engine
            self.dbow3_engine = DBoW3Engine(dbow3_vocabulary_path, voc=dbow3_vocabulary)
            total = len(self.timestamps)
            for idx, ts in enumerate(self.timestamps):
                # One line per keyframe is ~1100 lines of log for a real map, which
                # is slow enough to show up in the startup profile. Keep the first,
                # the last and every 100th.
                if idx == 0 or idx + 1 == total or (idx + 1) % 100 == 0:
                    logger.info(
                        f"[LoopClosure] loading map keyframe {idx + 1}/{total}, timestamp={int(ts)}"
                    )
                try:
                    cand_features = self.db.get_features(ts)
                except Exception as e:
                    logger.error(
                        f"[LoopClosure] failed loading timestamp={int(ts)}: {e}"
                    )
                    raise
                self.dbow3_engine.add(cand_features)

    def add_timestamp(self, timestamp: int):
        ts = int(timestamp)
        if ts in self.timestamp_to_idx:
            return
        self.timestamp_to_idx[ts] = len(self.timestamps)
        self.timestamps.append(ts)
        if self.mode in ("embedding", "vlad"):
            row = (self._vlad_for(ts) if self.mode == "vlad"
                   else self._normalize_embedding(self.db.get_embedding(ts)))[None, :]
            if self.embeddings.shape[0] == 0:
                self.embeddings = row
            else:
                self.embeddings = np.concatenate([self.embeddings, row], axis=0)
        else:
            cand_features = self.db.get_features(ts)
            self.dbow3_engine.add(cand_features)

    def _vlad_for(self, timestamp: int) -> np.ndarray:
        from tinynav.core.bow_retrieval import valid_superpoint_descriptors
        from tinynav.core.vlad import compute_vlad

        return compute_vlad(valid_superpoint_descriptors(self.db.get_features(int(timestamp))),
                            self.vlad_centres)

    @staticmethod
    def _normalize_embedding(embedding: np.ndarray) -> np.ndarray:
        e = np.asarray(embedding, dtype=np.float32).reshape(-1)
        n = np.linalg.norm(e)
        return e / n if n > 0 else e

    def find_candidate_timestamps(
        self,
        kp: np.ndarray,
        desc: np.ndarray,
        embedding: np.ndarray,
        top_k: int | None = None,
        allowed_timestamps: set[int] | None = None,
    ) -> list[dict]:
        if self.mode in ("embedding", "vlad"):
            if self.embeddings.shape[0] == 0:
                return []
            if self.mode == "vlad":
                from tinynav.core.vlad import compute_vlad
                emb = compute_vlad(np.asarray(desc, dtype=np.float32).reshape(-1, self.vlad_centres.shape[1]),
                                   self.vlad_centres)
            else:
                emb = self._normalize_embedding(embedding)
            similarities = self.embeddings @ emb
            sorted_idx = np.argsort(similarities)[::-1]
            k = self.embedding_top_k if top_k is None else int(top_k)
            candidate_idx = sorted_idx[: max(1, k)]
            scored_candidates = [
                (self.timestamps[int(idx)], float(similarities[idx]))
                for idx in candidate_idx
                if float(similarities[idx]) >= self.embedding_similarity_threshold
            ]
        else:
            if self.dbow3_engine is None:
                return []
            query_features = {
                "kpts": np.asarray(kp)[None, ...] if np.asarray(kp).ndim == 2 else np.asarray(kp),
                "descps": np.asarray(desc)[None, ...] if np.asarray(desc).ndim == 2 else np.asarray(desc),
                "mask": np.ones((1, np.asarray(desc).shape[0], 1), dtype=np.float32)
                if np.asarray(desc).ndim == 2
                else np.ones((1, np.asarray(desc)[0].shape[0], 1), dtype=np.float32),
            }
            k = self.embedding_top_k if top_k is None else int(top_k)
            dbow_results = self.dbow3_engine.query(query_features, max_results=max(1, k))
            scored_candidates = []
            for r in dbow_results:
                idx = int(r["id"])
                if 0 <= idx < len(self.timestamps):
                    scored_candidates.append((self.timestamps[idx], float(r["score"])))

        results = []
        for ts, sim in scored_candidates:
            if allowed_timestamps is not None and int(ts) not in allowed_timestamps:
                continue
            results.append(
                {
                    "timestamp": int(ts),
                    "similarity": sim,
                }
            )

        results.sort(key=lambda x: x["similarity"], reverse=True)
        return results


class DummyEmbeddingEngine:
    async def infer(self, _image: np.ndarray) -> np.ndarray:
        return np.zeros((1, 768), dtype=np.float32)

def generate_occupancy_map(poses, db, K, baseline, resolution = 0.1, step = 100, stage_timer: Optional[StageTimer] = None):
    """
        Generate a occupancy grid map from the depth images.
        The occupancy grid map is a 3D grid with the following values:
            0 : Unknown
            1 : Free
            2 : Occupied
    """
    raycast_shape = (100, 100, 20)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    odom_pose_min_position = np.array([np.inf, np.inf, np.inf], dtype=np.float32)
    odom_pose_max_position = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float32)
    for timestamp, odom_pose in poses.items():
        odom_translation = odom_pose[:3, 3]
        odom_pose_min_position = np.minimum(odom_pose_min_position, odom_translation)
        odom_pose_max_position = np.maximum(odom_pose_max_position, odom_translation)
    odom_pose_min_position = np.floor(odom_pose_min_position / resolution) * resolution
    odom_pose_max_position = np.ceil(odom_pose_max_position / resolution) * resolution
    global_grid_shape = np.ceil(
        (odom_pose_max_position - odom_pose_min_position) / resolution + np.array(raycast_shape)
    ).astype(np.int32)
    print(f"global_grid_shape : {global_grid_shape}")
    global_origin = odom_pose_min_position - 0.5 * np.array(raycast_shape) * resolution
    global_grid = np.zeros(global_grid_shape, dtype=np.float32)

    odom_positions = []

    def _raycast_all_poses():
        nonlocal global_grid, global_origin
        for timestamp, odom_pose in tqdm(poses.items()):
            depth, _, _, _, _ = db.get_depth_embedding_features_images(timestamp)
            odom_translation = odom_pose[:3, 3]
            local_origin = np.floor(odom_translation / resolution) * resolution - 0.5 * np.array(raycast_shape) * resolution
            local_grid = run_raycasting_loopy(depth, odom_pose, raycast_shape, fx, fy, cx, cy, local_origin, step, resolution, filter_ground = True)
            global_grid, global_origin = merge_local_into_global(global_grid, global_origin, local_grid, local_origin, resolution)
            odom_position = odom_pose[:3, 3]
            odom_positions.append(odom_position)

    if stage_timer is not None:
        with stage_timer.timed("occupancy_raycast"):
            _raycast_all_poses()
    else:
        _raycast_all_poses()

    voxels = int(np.prod(global_grid_shape))
    print(
        "[generate_occupancy_map] SDF stage params: "
        f"resolution={resolution}, step={step}, "
        f"num_poses={len(odom_positions)}, global_grid_shape={tuple(global_grid_shape.tolist())}, "
        f"global_origin={global_origin.tolist()}, voxels={voxels}"
    )

    def _compute_sdf():
        if len(odom_positions) == 0:
            return np.full(global_grid_shape, np.inf, dtype=np.float32)
        seed_mask = np.ones(global_grid_shape, dtype=np.uint8)
        odom_positions_np = np.asarray(odom_positions, dtype=np.float32)
        seed_indices = np.rint((odom_positions_np - global_origin) / resolution).astype(np.int32)
        seed_indices = np.clip(seed_indices, 0, global_grid_shape - 1)
        seed_mask[seed_indices[:, 0], seed_indices[:, 1], seed_indices[:, 2]] = 0
        return distance_transform_edt(seed_mask, sampling=(resolution, resolution, resolution)).astype(np.float32)

    # Compute SDF as voxel distance to nearest odom seed using SciPy EDT.
    if stage_timer is not None:
        with stage_timer.timed("occupancy_sdf"):
            sdf_map = _compute_sdf()
    else:
        with Timer(name="sdf_distance_transform_edt", text="[{name}] Elapsed time: {milliseconds:.0f} ms"):
            sdf_map = _compute_sdf()

    # 0 is the unknown.
    grid_type = np.zeros_like(global_grid, dtype=np.uint8)

    grid_type[global_grid > 0] = 2  # Occupied
    grid_type[global_grid < 0] = 1  # Free

    x_y_plane = np.max(grid_type, axis=2)
    x_y_plane_image = np.zeros_like(x_y_plane, dtype=np.float32)
    x_y_plane_image[x_y_plane == 2] = 1.0
    x_y_plane_image[x_y_plane == 1] = 0.5
    x_y_plane_image = (x_y_plane_image * 255).astype(np.uint8)
    return grid_type, global_origin, x_y_plane_image, sdf_map

class IntKeyShelf:
    def __init__(self, filename):
        # Prefer dbm.dumb so new maps stay portable across host/device runtimes (the board's
        # Python has no gdbm/ndbm at all). Maps built by upstream main use shelve's default
        # backend instead, so fall back to it when a non-empty .db is what is on disk --
        # note dumb.open(..., "c") would silently create an empty store next to it.
        db_path = f"{filename}.db"
        if os.path.exists(db_path) and os.path.getsize(db_path) > 0:
            self.db = shelve.open(filename)
        else:
            self._raw_db = dbm.dumb.open(filename, "c")
            self.db = shelve.Shelf(self._raw_db, protocol=pickle.HIGHEST_PROTOCOL)

    def __getitem__(self, key: int):
        return self.db[str(key)]

    def __setitem__(self, key: int, value):
        self.db[str(key)] = value

    def __delitem__(self, key: int):
        del self.db[str(key)]

    def __contains__(self, key: int):
        return str(key) in self.db

    def keys(self):
        return [int(k) for k in self.db.keys()]

    def close(self):
        self.db.close()

    def sync(self):
        self.db.sync()


class OdomPoseRecorder:
    """
    Utility class to record continuous odometry data to disk.
    Saves timestamp-pose pairs for later timestamp-based queries.
    """

    def __init__(self, save_path: str, prefix: str = "poses"):
        self.save_path = save_path
        self.prefix = prefix
        self.file_save_path = os.path.join(save_path, f"{prefix}_continuous_odom.npy")
        self.poses: Dict[int, np.ndarray] = {}  # timestamp_ns -> 4x4 pose matrix

        os.makedirs(save_path, exist_ok=True)

    def record_odometry_msg(self, odom_msg: Odometry) -> None:
        timestamp_ns = int(odom_msg.header.stamp.sec * 1e9) + int(
            odom_msg.header.stamp.nanosec
        )
        pose_matrix = msg2np(odom_msg)
        self.poses[timestamp_ns] = pose_matrix

    def save_to_disk(self) -> None:
        if not self.poses:
            logger.warning(f"No continuous odom poses to save for {self.prefix}")
            return

        logger.info(f"{self.prefix}: Saved {len(self.poses)} continuous odom poses")
        # Create a copy of the dict for saving to avoid any typing issues
        poses_to_save = dict(self.poses)
        np.save(self.file_save_path, poses_to_save, allow_pickle=True)  # type: ignore

        logger.info(f"Saved {len(self.poses)} poses to {self.file_save_path}")

    def load_from_disk(self) -> bool:
        if not os.path.exists(self.file_save_path):
            logger.warning(f"Pose file not found: {self.file_save_path}")
            return False

        try:
            self.poses = np.load(self.file_save_path, allow_pickle=True).item()
            logger.info(
                f"[PoseRecorder] Loaded {len(self.poses)} poses from {self.file_save_path}"
            )
            return True
        except Exception as e:
            logger.error(f"Failed to load poses from {self.file_save_path}: {e}")
            return False

    def clear(self) -> None:
        self.poses.clear()


class TinyNavDB():
    def __init__(self, map_save_path:str, is_scratch:bool = True,
                 save_infra1_video: bool = True, save_rgb_video: bool = True):
        """
        save_*_video only affects *writing*. Reading a map always opens whatever videos
        it has, so a map built without them stays loadable and the consumers below
        already handle absence.

        These two h264 encodes are 95% of the save_image_and_depth stage, measured on
        the board over a 575-keyframe build: rgb 165.8 ms/keyframe = 95.3 s, infra1
        32.1 ms/keyframe = 18.5 s, against 5.9 s for the depth shelve write and
        everything else. That is 113.8 s of the 188.2 s mapping_loop total -- and it had
        been attributed to writing the 801 MB depths.dat, which turns out to be the cheap
        part.

        Nothing on the navigation path reads either video: the rgb_loader/infra1_loader
        closures returned by get_depth_embedding_features_images have no call site
        anywhere, and every caller unpacks them as _. The real consumers are two PC-side
        offline tools -- tool/convert_to_nerf_format.py reads rgb_images_db and
        tool/poi_editor.py reads infra1_images_db. Encoding them on the board costs 95 s
        to serve a tool that does not run there.
        """
        self.map_save_path = map_save_path
        self.is_scratch = is_scratch
        mode = "write" if is_scratch else "read"
        write_infra1 = save_infra1_video or not is_scratch
        write_rgb = save_rgb_video or not is_scratch
        self.infra1_video_db = VideoDB(
            dir_path=f"{map_save_path}/infra1_images_db",
            mode=mode,
            fps=30,
        ) if write_infra1 else None
        self.rgb_video_db = VideoDB(
            dir_path=f"{map_save_path}/rgb_images_db",
            mode=mode,
            fps=30,
        ) if write_rgb else None
        if is_scratch:
            # These used to look for "<name>.db", which IntKeyShelf never writes: it is
            # backed by dbm.dumb, whose files are <name>.dir, <name>.dat and <name>.bak.
            # So all three removals were no-ops, and a scratch build into a directory
            # that already held a map silently *inherited* every previous keyframe --
            # the shelves are opened for append, so depths.dat would simply keep
            # growing. Production only escaped this because node_manager rmtree's
            # map_path first; scripts/run_rosbag_build_map.sh does not.
            for name in ("features", "depths", "embeddings"):
                for suffix in (".dir", ".dat", ".bak"):
                    stale = f"{map_save_path}/{name}{suffix}"
                    if os.path.exists(stale):
                        os.remove(stale)
        self.features = IntKeyShelf(f"{map_save_path}/features")
        self.embeddings = IntKeyShelf(f"{map_save_path}/embeddings")
        self.depths = IntKeyShelf(f"{map_save_path}/depths")

    def set_entry(self, key:int,   depth:np.ndarray = None, embedding:np.ndarray = None, features:dict = None,  infra1_image:np.ndarray = None, rgb_image:np.ndarray = None):
        if infra1_image is not None and self.infra1_video_db is not None:
            self.infra1_video_db.write(key, infra1_image)
        if rgb_image is not None and self.rgb_video_db is not None:
            self.rgb_video_db.write(key, rgb_image)
        if depth is not None:
            self.depths[key] = depth
        if embedding is not None:
            self.embeddings[key] = embedding
        if features is not None:
            self.features[key] = features

    def get_depth_embedding_features_images(self, key:int):
        key_int = int(key)
        def rgb_loader():
            if self.is_scratch or self.rgb_video_db is None:
                return None
            return self.rgb_video_db.read(key_int)

        def infra1_loader():
            if self.is_scratch or self.infra1_video_db is None:
                return None
            return self.infra1_video_db.read(key_int)

        return self.depths[key], self.get_embedding(key), self.features[key], rgb_loader, infra1_loader

    def get_features(self, key:int):
        """Read only the keypoints/descriptors of a keyframe.

        Callers that need nothing else must not go through
        get_depth_embedding_features_images(): that eagerly unshelves the depth
        map too, which is an order of magnitude more bytes than the features
        (1564 MB vs 145 MB across a 1123-keyframe map) and is then discarded.
        """
        return self.features[int(key)]

    def get_embedding(self, key:int):
        key_int = int(key)
        if key_int in self.embeddings:
            return self.embeddings[key_int]
        return np.zeros((1,), dtype=np.float32)

    def close(self):
        self.features.close()
        self.embeddings.close()
        self.depths.close()
        if self.infra1_video_db is not None:
            self.infra1_video_db.close()
        if self.rgb_video_db is not None:
            self.rgb_video_db.close()

    def sync(self):
        self.features.sync()
        self.embeddings.sync()
        self.depths.sync()
        if self.infra1_video_db is not None and hasattr(self.infra1_video_db, "sync"):
            self.infra1_video_db.sync()
        if self.rgb_video_db is not None and hasattr(self.rgb_video_db, "sync"):
            self.rgb_video_db.sync()

class BagPlayer(Node):
    def __init__(self, bag_uri: str, storage_id: str = "sqlite3", serialization_format: str = "cdr",
                 play_rate: float = 0.0, skip_topics: set[str] | None = None,
    ):
        super().__init__("rosbag_player")

        # play_rate 0 means "as fast as the reader goes", which is what this player
        # has always done and what a PC wants: pacing can only ever slow down a
        # build whose consumer already runs faster than real time.
        #
        # It is the wrong default on a slow machine. The consumer sits one process
        # away behind DDS, so the self-throttling of the publish-one-then-spin-once
        # loop below never reaches it, and frames pile up in build_map_node's
        # ApproximateTimeSynchronizer, whose queue holds three full-resolution
        # images per slot. Measured on the X5 inside the Looper camera: an unpaced
        # build of a 71 s bag was OOM-killed after 59 s at 0.5% progress, peak RSS
        # 645 MiB against 1307 MiB of RAM with no swap.
        #
        # So pass a positive rate on any machine that cannot outrun the bag.
        if play_rate < 0.0:
            raise ValueError(f"play_rate must be >= 0 (0 disables pacing), got {play_rate}")
        self.play_rate = float(play_rate)
        self._playback_start_timestamp_ns = None
        self._playback_start_wall_time_s = None

        self._storage_options = StorageOptions(uri=bag_uri, storage_id="sqlite3",)
        self._converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr",)

        self._reader = SequentialReader()
        self._reader.open(self._storage_options, self._converter_options)

        self.start_timestamp_ns = None
        self.end_timestamp_ns = None

        topic_infos = self._reader.get_all_topics_and_types()
        if len(topic_infos) == 0:
            raise ValueError(f"Bag {bag_uri} has no topics")

        self.start_timestamp_ns, self.end_timestamp_ns = self._scan_bag_time_range(
            bag_uri,
            storage_id,
            serialization_format,
        )

        # An exclude list, deliberately, not an allow list. Most of a bag is replayed for
        # nobody -- measured on a 61847-message Looper bag, /camera/camera/imu alone is
        # 69.8% of messages and /camera/camera/infra2/image_rect_raw is 27% of the bytes,
        # and each message costs a deserialize plus three publishes. But the consumers
        # are not all in this process: looper_bridge_node subscribes to the raw camera
        # topics in a separate process and produces the /slam/keyframe_* topics
        # build_map_node actually reads, so a list derived from this node's own
        # subscriptions would starve the bridge and yield an empty map with no error.
        # An exclude list cannot make that mistake: anything not named still plays, so a
        # topic added later keeps working and only what has been proven dead is dropped.
        # Default empty -- the caller names what to skip, because whether a topic is dead
        # depends on which sensor pipeline is running (perception_node needs infra2;
        # looper mode never launches it).
        self._skip_topics = set(skip_topics or ())
        played = [t for t in topic_infos if t.name not in self._skip_topics]
        skipped = sorted(t.name for t in topic_infos if t.name in self._skip_topics)

        # Push the filter into the reader as well as the publisher map, so the skipped
        # rows are never read off eMMC in the first place rather than read and discarded.
        # Best-effort: older rosbag2_py builds lack set_filter, and play_next drops them
        # anyway.
        if skipped:
            try:
                self._reader.set_filter(StorageFilter(topics=[t.name for t in played]))
            except (AttributeError, NameError, TypeError) as e:
                self.get_logger().warn(
                    f"reader-level topic filter unavailable ({e}); skipping at publish time instead"
                )

        # topic -> (publisher, msg_type)
        self._topic_publishers = {}

        for topic_info in played:
            msg_type = get_message(topic_info.type)
            pub = self.create_publisher(msg_type, topic_info.name, 10)
            self._topic_publishers[topic_info.name] = (pub, msg_type)

        self.get_logger().info("Bag topics and message types:")
        for topic_info in sorted(topic_infos, key=lambda t: t.name):
            mark = "  [skipped] " if topic_info.name in self._skip_topics else "  "
            self.get_logger().info(f"{mark}{topic_info.name} -> {topic_info.type}")
        if skipped:
            self.get_logger().info(f"not replaying {len(skipped)} topic(s): {', '.join(skipped)}")

        # /clock publisher (for use_sim_time)
        self._clock_pub = self.create_publisher(Clock, "/clock", 10)
        self._mapping_percent_pub = self.create_publisher(Float32, "/mapping/percent", 10)

        self.get_logger().info(f"BagPlayer opened bag: {bag_uri}")

    def _scan_bag_time_range(self, bag_uri: str, storage_id: str, serialization_format: str) -> tuple[int, int]:
        # metadata.yaml already carries both numbers, and rosbag2_py exposes it -- the
        # previous comment here said no such API had been found, but tool/benchmark/
        # benchmark_mapping.py has been using Info().read_metadata all along. The
        # fallback below is a full extra sequential pass over the whole bag (2.0 GB on
        # this dataset, off the board's eMMC) to recover two integers, so it is worth
        # trying the cheap path first.
        try:
            # read_metadata wants the bag *directory*; callers pass either that or the
            # bag_0.db3 file inside it (node_manager does the latter).
            meta_dir = bag_uri if os.path.isdir(bag_uri) else os.path.dirname(bag_uri)
            metadata = Info().read_metadata(meta_dir, storage_id)
            first_ns = int(metadata.starting_time.nanoseconds)
            last_ns = first_ns + int(metadata.duration.nanoseconds)
            if metadata.message_count > 0 and last_ns > first_ns:
                self.get_logger().info(
                    f"bag time range from metadata: {metadata.message_count} messages, "
                    f"{(last_ns - first_ns) / 1e9:.1f}s (skipped a full scan pass)"
                )
                return first_ns, last_ns
            self.get_logger().warn("bag metadata has no usable time range; scanning instead")
        except Exception as e:
            self.get_logger().warn(f"could not read bag metadata ({e}); scanning instead")

        scan_reader = SequentialReader()
        scan_reader.open(
            StorageOptions(uri=bag_uri, storage_id=storage_id),
            ConverterOptions(
                input_serialization_format=serialization_format,
                output_serialization_format=serialization_format,
            ),
        )

        first_timestamp_ns = None
        last_timestamp_ns = None
        while scan_reader.has_next():
            _, _, timestamp_ns = scan_reader.read_next()
            timestamp_ns = int(timestamp_ns)
            if first_timestamp_ns is None:
                first_timestamp_ns = timestamp_ns
            last_timestamp_ns = timestamp_ns

        if first_timestamp_ns is None or last_timestamp_ns is None:
            raise ValueError(f"Bag {bag_uri} has no messages")

        return first_timestamp_ns, last_timestamp_ns

    _PERCENT_LOG_INTERVAL_S = 2.0  # throttle progress logging

    def _publish_percent(self, percent: float) -> None:
        msg = Float32()
        msg.data = float(percent)
        self._mapping_percent_pub.publish(msg)
        # Emit a throttled log line so the parent process can read progress
        # from stdout without needing a separate bridge node.
        # Always emit 100% (completion signal) regardless of throttle.
        now = self.get_clock().now()
        elapsed = ((now - self._last_percent_log_time).nanoseconds / 1e9
                   if hasattr(self, '_last_percent_log_time') else float('inf'))
        if percent >= 100.0 or elapsed >= self._PERCENT_LOG_INTERVAL_S:
            self.get_logger().info(f"MAPPING_PERCENT:{percent:.1f}")
            self._last_percent_log_time = now

    # Progress is a UI signal at human resolution, but this ran on every bag message --
    # 61847 Float32 publishes and as many get_clock().now() calls on a 108 s bag, to move
    # a percentage that nobody can read faster than a few times a second. The forced 100%
    # at the end of the build does not go through here, so throttling cannot lose it.
    _PERCENT_PUBLISH_EVERY_N = 50

    def _publish_percent_from_timestamp(self, timestamp_ns: int) -> None:
        self._percent_msg_counter = getattr(self, '_percent_msg_counter', 0) + 1
        if self._percent_msg_counter % self._PERCENT_PUBLISH_EVERY_N != 1:
            return
        percent = 100.0 * (timestamp_ns - self.start_timestamp_ns) / (self.end_timestamp_ns - self.start_timestamp_ns)
        self._publish_percent(percent)

    def _pace_to_timestamp(self, timestamp_ns: int) -> None:
        """Sleep so that bag time advances at most ``play_rate`` x wall time.

        Never speeds anything up: if the consumer is already behind, the target
        wall time is in the past and this returns immediately.
        """
        if self.play_rate <= 0.0:
            return
        if self._playback_start_timestamp_ns is None:
            self._playback_start_timestamp_ns = timestamp_ns
            self._playback_start_wall_time_s = time.monotonic()
            return
        elapsed_bag_s = (timestamp_ns - self._playback_start_timestamp_ns) * 1e-9
        target_wall_s = self._playback_start_wall_time_s + elapsed_bag_s / self.play_rate
        sleep_s = target_wall_s - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)

    def play_next(self) -> bool:
        """
        Publish the next message from the bag.
        Returns False when there are no more messages.
        """
        if not self._reader.has_next():
            return False

        topic, serialized_msg, timestamp_ns = self._reader.read_next()

        # Before pacing and before deserializing: a skipped topic should cost nothing
        # beyond the read. This is also the fallback path when the reader-level filter
        # was unavailable, in which case these rows still arrive here.
        if topic in self._skip_topics:
            return True

        self._pace_to_timestamp(int(timestamp_ns))
        self._publish_percent_from_timestamp(int(timestamp_ns))

        # Find publisher + msg type for this topic
        pub_and_type = self._topic_publishers.get(topic)
        if pub_and_type is None:
            # No publisher (should not really happen, but don't crash playback)
            self.get_logger().warn(f"No publisher for topic '{topic}'")
            return True

        pub, msg_type = pub_and_type

        # Deserialize and publish actual message
        msg = deserialize_message(serialized_msg, msg_type)
        pub.publish(msg)

        # Publish /clock with the same timestamp (for use_sim_time)
        if self._clock_pub is not None:
            clock_msg = Clock()
            clock_msg.clock.sec = int(timestamp_ns // 1_000_000_000)
            clock_msg.clock.nanosec = int(timestamp_ns % 1_000_000_000)
            self._clock_pub.publish(clock_msg)

        return True

# Loop closure is disabled on this branch: every call site into it is commented out
# inside keyframe_callback. This constant exists so the disabling is stated in one
# place rather than inferred from three comments, and so the expensive object it
# guards is not built for code that cannot run. Flip it back together with those call
# sites, never on its own.
_LOOP_CLOSURE_ENABLED = os.environ.get('TINYNAV_MAP_LOOP_CLOSURE', '0') == '1'


class BuildMapNode(Node):
    def __init__(
        self,
        map_save_path: str,
        extractor,
        matcher,
        embedding_extractor,
        loop_closure_mode: str = "embedding",
        vlad_centres_path: str | None = None,
        loop_closure_use_bow: bool = False,
        dbow3_vocabulary_path: str | None = None,
        verbose_timer: bool = True,
        global_frames_ratio: float = 1.1,
        sync_queue_size: int = 200,
        publish_visualization: bool = True,
        save_infra1_video: bool = True,
        save_rgb_video: bool = True,
    ):
        super().__init__('build_map_node')
        if global_frames_ratio < 1.0:
            raise ValueError(f"global_frames_ratio must be >= 1.0, got {global_frames_ratio}")
        if sync_queue_size < 1:
            raise ValueError(f"sync_queue_size must be >= 1, got {sync_queue_size}")
        # Everything the local pointcloud and the trajectory Path are for is rviz.
        # Nothing in the saved map depends on either, and on the X5 they were the
        # two most expensive stages of the first keyframe by a wide margin
        # (8538 ms and 4055 ms against 719 ms for all the real work combined), so a
        # headless build should not pay for them.
        self.publish_visualization = publish_visualization
        self.verbose_timer = verbose_timer
        self.logger = logging.getLogger(__name__)
        self.timer_logger = self.logger.info if verbose_timer else self.logger.debug
        self.stage_timer = StageTimer(
            verbose_logger=self.logger.info if verbose_timer else None,
        )
        self.global_frames_ratio = global_frames_ratio
        self._global_prev_num_frames = 0
        self.extractor = extractor
        self.matcher = matcher
        self.embedding_extractor = embedding_extractor
        self.loop_closure_use_bow = bool(loop_closure_use_bow)
        self.loop_closure_mode = "bow" if self.loop_closure_use_bow else loop_closure_mode
        self.vlad_centres = load_vlad_centres(vlad_centres_path)
        self.dbow3_vocabulary_path = dbow3_vocabulary_path

        self.bridge = CvBridge()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/infra2/camera_info', self.info_callback, 10)
        self.depth_sub = Subscriber(self, Image, '/slam/keyframe_depth')
        self.keyframe_image_sub = Subscriber(self, Image, '/slam/keyframe_image')
        self.keyframe_odom_sub = Subscriber(self, Odometry, '/slam/keyframe_odom')
        self.rgb_image_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.continuous_odom_sub = self.create_subscription(Odometry, '/slam/odometry', self.continuous_odom_callback, 100)

        self.marker_pub = self.create_publisher(MarkerArray, '/mapping/pointcloud_markers', 10)
        self.pose_graph_trajectory_pub = self.create_publisher(Path, "/mapping/pose_graph_trajectory", 10)
        # Removed: /mapping/local_map, /mapping/project_3d_to_2d,
        # /mapping/keyframe_matches_images, /mapping/loop_matches_images and
        # /mapping/global_map_marker. Each had zero .publish() calls and zero
        # subscribers anywhere in the repo -- not even in docs/vis.rviz -- so they were
        # five DDS writers' worth of discovery traffic and memory advertising topics that
        # never carried a message.

        # Add stop signal subscription and save finished publisher
        self.mapping_stop_sub = self.create_subscription(Bool, '/benchmark/stop', self.mapping_stop_callback, 10)
        self.mapping_save_finished_pub = self.create_publisher(Bool, '/benchmark/data_saved', 10)
        # Keep the sync queue bounded to reduce memory spikes / OOM risk during map
        # building. This used to say that while holding 200, which bounds nothing:
        # each slot holds a keyframe image, a depth image and an RGB image, so at
        # 544x640 that is roughly 1.4 MB per slot and 280 MB of headroom handed to a
        # producer that, unpaced, will use all of it. On the X5 (1307 MB, no swap)
        # that alone is the difference between a build and an OOM kill.
        self.ts = ApproximateTimeSynchronizer(
            [self.keyframe_image_sub, self.keyframe_odom_sub, self.depth_sub, self.rgb_image_sub],
            sync_queue_size, 0.02,
        )
        self.ts.registerCallback(self.keyframe_callback)

        self.K = None
        self.baseline = None
        self.odom = {}
        self.pose_graph_used_pose = {}
        self.relative_pose_constraint = []
        self.last_keyframe_timestamp = None
        self.processed_keyframes = 0
        self.db_sync_every = max(1, int(os.getenv("TINYNAV_DB_SYNC_EVERY", "1")))
        self.continuous_odom_recorder = OdomPoseRecorder(map_save_path, "mapping")

        os.makedirs(f"{map_save_path}", exist_ok=True)
        self.db = TinyNavDB(
            map_save_path,
            save_infra1_video=save_infra1_video,
            save_rgb_video=save_rgb_video,
        )
        if not save_infra1_video:
            self.get_logger().warn(
                "not writing infra1_images_db -- tool/poi_editor.py cannot show camera "
                "images for this map"
            )
        if not save_rgb_video:
            self.get_logger().warn(
                "not writing rgb_images_db -- tool/convert_to_nerf_format.py cannot "
                "export this map for 3DGS/nerf"
            )

        self.marker_id = 0

        self.loop_similarity_threshold, self.loop_top_k = \
            LOOP_CLOSURE_DEFAULTS[self.loop_closure_mode][:2]
        # Only built when loop closure is actually reachable. Every call site is
        # commented out ("temp disabled" at the add_timestamp and
        # find_loop_and_pose_graph lines below), so on this branch the object had zero
        # callers -- but constructing it in bow mode still ran DBoW3Engine's
        # load_vocabulary + Database.setVocabulary, which the engine's own docstring
        # measures at ~300 MiB resident, held for the entire build. On a 1307 MB board
        # that is 23% of memory reserved for nothing, and it is what forced the
        # --play-rate and --sync-queue-size limits that keep the build off the OOM
        # killer. Re-enabling loop closure means flipping this flag *and* uncommenting
        # those call sites; leaving it None makes a half-done re-enable fail loudly
        # instead of silently skipping loop closure.
        self.loop_closure_enabled = _LOOP_CLOSURE_ENABLED
        self.loop_closure = LoopClosure(
            db=self.db,
            timestamps=[],
            mode=self.loop_closure_mode,
            dbow3_vocabulary_path=self.dbow3_vocabulary_path,
            embedding_similarity_threshold=self.loop_similarity_threshold,
            embedding_top_k=self.loop_top_k,
            vlad_centres=self.vlad_centres,
        ) if self.loop_closure_enabled else None

        self.map_save_path = map_save_path
        self._save_completed = False
        self.tf_sub = Subscriber(self, TFMessage, "/tf")
        self.tf_sub.registerCallback(self.tf_callback)
        self.tf_static_sub = Subscriber(self, TFMessage, "/tf_static")
        self.tf_static_sub.registerCallback(self.tf_callback)
        self.T_rgb_to_infra1 = None
        self.rgb_camera_info_sub = Subscriber(self, CameraInfo, "/camera/camera/color/camera_info")
        self.rgb_camera_info_sub.registerCallback(self.rgb_camera_info_callback)
        self.rgb_camera_K = None

    def tf_callback(self, msg:TFMessage):
        T_infra1_to_link = None
        T_infra1_optical_to_infra1 = None
        T_rgb_to_link = None
        T_rgb_optical_to_rgb = None
        tf_messages: Dict[int, Dict[str, np.ndarray]] = {}
        for t in msg.transforms:
            frame_id, child_frame_id, T = tf2np(t)
            timestamp_ns = int(t.header.stamp.sec * 1e9) + int(t.header.stamp.nanosec)
            if timestamp_ns not in tf_messages:
                tf_messages[timestamp_ns] = {}
            tf_messages[timestamp_ns][f"{frame_id}->{child_frame_id}"] = T
            if frame_id == "camera_link" and child_frame_id == "camera_infra1_frame":
                T_infra1_to_link = T
            if frame_id == "camera_infra1_frame" and child_frame_id == "camera_infra1_optical_frame":
                T_infra1_optical_to_infra1 = T
            if frame_id == "camera_color_frame" and child_frame_id == "camera_color_optical_frame":
                T_rgb_optical_to_rgb = T
            if frame_id == "camera_link" and child_frame_id == "camera_color_frame":
                T_rgb_to_link = T
            # Looper bags use cam_left/cam_rgb directly as camera frames.
            # In this code path, TF matrix is interpreted as child -> frame.
            if frame_id == "cam_left" and child_frame_id == "cam_rgb":
                self.T_rgb_to_infra1 = T

        if T_infra1_optical_to_infra1 is not None and T_rgb_optical_to_rgb is not None and T_infra1_to_link is not None and T_rgb_to_link is not None:
            self.T_rgb_to_infra1 = np.linalg.inv(T_infra1_optical_to_infra1) @ np.linalg.inv(T_infra1_to_link) @ T_rgb_to_link @ T_rgb_optical_to_rgb
        if tf_messages and self.T_rgb_to_infra1 is not None:
            np.save(f"{self.map_save_path}/tf_messages.npy", tf_messages, allow_pickle=True)
            if self.tf_sub is not None:
                self.destroy_subscription(self.tf_sub.sub)
                self.tf_sub = None
            if self.tf_static_sub is not None:
                self.destroy_subscription(self.tf_static_sub.sub)
                self.tf_static_sub = None
            self.get_logger().info("Saved tf_messages.npy and unsubscribed from /tf and /tf_static")

    def rgb_camera_info_callback(self, msg:CameraInfo):
        if self.rgb_camera_K is None:
            self.rgb_camera_K = np.array(msg.k).reshape(3, 3)

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

    def mapping_stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Received benchmark stop signal, starting save process...")
            try:
                self.save_mapping()
                self.get_logger().info("Mapping save completed successfully")

                # Publish save finished signal
                save_finished_msg = Bool()
                save_finished_msg.data = True
                self.mapping_save_finished_pub.publish(save_finished_msg)
                self.get_logger().info("Published data save finished signal")

            except Exception as e:
                self.get_logger().error(f"Error during mapping save: {e}")
                # Still publish completion signal even if there was an error
                save_finished_msg = Bool()
                save_finished_msg.data = False
                self.mapping_save_finished_pub.publish(save_finished_msg)

    def keyframe_callback(self, keyframe_image_msg:Image, keyframe_odom_msg:Odometry, depth_msg:Image, rgb_image_msg:Image):
        with self.stage_timer.timed("mapping_loop"):
            if self.K is None:
                return
            self.process(keyframe_image_msg, keyframe_odom_msg, depth_msg, rgb_image_msg)

    def process(self, keyframe_image_msg:Image, keyframe_odom_msg:Odometry, depth_msg:Image, rgb_image_msg:Image):
        with self.stage_timer.timed("msg_decode"):
            keyframe_image_timestamp = int(keyframe_image_msg.header.stamp.sec * 1e9) + int(keyframe_image_msg.header.stamp.nanosec)
            keyframe_odom_timestamp = int(keyframe_odom_msg.header.stamp.sec * 1e9) + int(keyframe_odom_msg.header.stamp.nanosec)
            keyframe_depth_timestamp = int(depth_msg.header.stamp.sec * 1e9) + int(depth_msg.header.stamp.nanosec)
            if keyframe_image_timestamp != keyframe_odom_timestamp or keyframe_image_timestamp != keyframe_depth_timestamp:
                self.get_logger().error(f"Keyframe timestamp mismatch: {keyframe_image_timestamp} != {keyframe_odom_timestamp} != {keyframe_depth_timestamp}")

            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            odom, _ = msg2np(keyframe_odom_msg)
            infra1_image = self.bridge.imgmsg_to_cv2(keyframe_image_msg, desired_encoding="mono8")
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, desired_encoding="bgr8")

        with self.stage_timer.timed("save_image_and_depth"):
            self.db.set_entry(keyframe_image_timestamp, depth = depth, infra1_image = infra1_image, rgb_image = rgb_image)

        with self.stage_timer.timed("get_embeddings"):
            embedding = self.get_embeddings(infra1_image)
            embedding_norm = np.linalg.norm(embedding)
            if embedding_norm > 0:
                embedding = embedding / embedding_norm
            self.db.set_entry(keyframe_image_timestamp, embedding = embedding)
        with self.stage_timer.timed("feature_extractor"):
            features = asyncio.run(self.extractor.infer(infra1_image))
            self.db.set_entry(keyframe_image_timestamp, features = features)

        with self.stage_timer.timed("loop_and_pose_graph_update"):
            if len(self.odom) == 0 and self.last_keyframe_timestamp is None:
                self.odom[keyframe_image_timestamp] = odom
                self.pose_graph_used_pose[keyframe_image_timestamp] = odom
                # self.loop_closure.add_timestamp(keyframe_image_timestamp)  # temp disabled
            else:
                last_keyframe_odom_pose = self.odom[self.last_keyframe_timestamp]
                T_prev_curr = np.linalg.inv(last_keyframe_odom_pose) @ odom
                self.relative_pose_constraint.append((keyframe_image_timestamp, self.last_keyframe_timestamp, T_prev_curr))
                self.pose_graph_used_pose[keyframe_image_timestamp] = odom
                self.odom[keyframe_image_timestamp] = odom


                def find_loop_and_pose_graph(timestamp):
                    valid_timestamp = [t for t in self.pose_graph_used_pose.keys() if t + 10 * 1e9 < timestamp]
                    if len(valid_timestamp) == 0:
                        return
                    target_embedding = self.db.get_embedding(timestamp)
                    _, _, curr_features, _, _ = self.db.get_depth_embedding_features_images(timestamp)
                    curr_kp = curr_features["kpts"][0] if curr_features["kpts"].ndim == 3 else curr_features["kpts"]
                    curr_desc = curr_features["descps"][0] if curr_features["descps"].ndim == 3 else curr_features["descps"]
                    with self.stage_timer.timed("find_loop"):
                        loop_list = self.loop_closure.find_candidate_timestamps(
                            curr_kp,
                            curr_desc,
                            target_embedding,
                            top_k=self.loop_top_k,
                            allowed_timestamps=set(valid_timestamp),
                        )
                    with self.stage_timer.timed("relative_pose_estimation"):
                        for candidate in loop_list:
                            prev_timestamp = candidate["timestamp"]
                            curr_timestamp = timestamp
                            prev_depth, _, prev_features, _, _ = self.db.get_depth_embedding_features_images(prev_timestamp)
                            curr_depth, _, curr_features, _, _ = self.db.get_depth_embedding_features_images(curr_timestamp)
                            prev_matched_keypoints, curr_matched_keypoints, matches = self.match_keypoints(prev_features, curr_features)
                            success, T_prev_curr, _, _, inliers = estimate_pose(prev_matched_keypoints, curr_matched_keypoints, curr_depth, self.K)
                            if success and len(inliers) >= 100:
                                self.relative_pose_constraint.append((curr_timestamp, prev_timestamp, T_prev_curr))
                                print(f"Added loop relative pose constraint: {curr_timestamp} -> {prev_timestamp}")
                    with self.stage_timer.timed("solve_pose_graph_loop"):
                        self.pose_graph_used_pose = solve_pose_graph(self.pose_graph_used_pose, self.relative_pose_constraint, max_iteration_num = 5)
                # find_loop_and_pose_graph(keyframe_image_timestamp)  # temp disabled
                # self.loop_closure.add_timestamp(keyframe_image_timestamp)  # temp disabled

        self.maybe_run_global_refinement()

        if self.publish_visualization:
            with self.stage_timer.timed("publish_local_pointcloud"):
                cloud = depth_to_cloud(depth, self.K, 30, 3)
                self.publish_local_map(cloud, 'camera_'+str(keyframe_image_timestamp))

            with self.stage_timer.timed("pose_graph_trajectory_publish"):
                self.pose_graph_trajectory_publish(keyframe_image_timestamp)
        self.processed_keyframes += 1
        if self.processed_keyframes % self.db_sync_every == 0:
            self.db.sync()
            self.get_logger().info(
                f"Synced TinyNavDB to disk at keyframe {self.processed_keyframes}"
            )
        self.last_keyframe_timestamp = keyframe_image_timestamp

    def get_embeddings(self, image: np.ndarray) -> np.ndarray:
        # shape: (1, 768)
        return asyncio.run(self.embedding_extractor.infer(image))

    def maybe_run_global_refinement(self) -> None:
        """Batch expensive online pose-graph solve and TF publishing."""
        num_frames = len(self.pose_graph_used_pose)
        if not check_global_frames_ratio(num_frames, self._global_prev_num_frames, self.global_frames_ratio):
            return

        with self.stage_timer.timed("solve_pose_graph_online"):
            self.pose_graph_used_pose = solve_pose_graph(
                self.pose_graph_used_pose,
                self.relative_pose_constraint,
                max_iteration_num=5,
            )
        with self.stage_timer.timed("tf_publish"):
            self.publish_all_transforms()
        self._global_prev_num_frames = num_frames

    def match_keypoints(self, feats0:dict, feats1:dict, image_shape = np.array([848, 480], dtype = np.int64)) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        match_result = asyncio.run(self.matcher.infer(feats0["kpts"], feats1["kpts"], feats0['descps'], feats1['descps'], feats0['mask'], feats1['mask'], image_shape, image_shape))
        match_indices = match_result["match_indices"][0]
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

    def save_mapping(self):
        if self._save_completed:
            self.get_logger().info("Mapping data already saved, skipping duplicate save")
            return

        if self.K is None:
            self.get_logger().info("No camera intrinsics available, skipping save")
            return

        self.get_logger().info("Saving mapping data...")

        # Save continuous poses
        self.continuous_odom_recorder.save_to_disk()

        with self.stage_timer.timed("final_pose_graph"):
            self.pose_graph_used_pose = solve_pose_graph(self.pose_graph_used_pose, self.relative_pose_constraint)

        with self.stage_timer.timed("tf_publish"):
            self.publish_all_transforms()
        self._global_prev_num_frames = len(self.pose_graph_used_pose)

        np.save(f"{self.map_save_path}/poses.npy", self.pose_graph_used_pose, allow_pickle = True)
        np.save(f"{self.map_save_path}/intrinsics.npy", self.K)
        np.save(f"{self.map_save_path}/baseline.npy", self.baseline)
        print(f"T_rgb_to_infra1: {self.T_rgb_to_infra1}")
        np.save(f"{self.map_save_path}/T_rgb_to_infra1.npy", self.T_rgb_to_infra1, allow_pickle = True)
        np.save(f"{self.map_save_path}/rgb_camera_intrinsics.npy", self.rgb_camera_K, allow_pickle = True)

        # Flush and close writable DB first, then reopen DB for occupancy generation.
        self.db.close()
        occupancy_db = TinyNavDB(self.map_save_path, is_scratch=False)

        # Generate occupancy map
        occupancy_resolution = 0.1
        occupancy_step = 10
        occupancy_grid, occupancy_origin, occupancy_2d_image, sdf_map = generate_occupancy_map(
            self.pose_graph_used_pose,
            occupancy_db,
            self.K,
            self.baseline,
            occupancy_resolution,
            occupancy_step,
            stage_timer=self.stage_timer,
        )
        occupancy_db.close()
        with self.stage_timer.timed("occupancy_save_files"):
            occupancy_meta = np.array([occupancy_origin[0], occupancy_origin[1], occupancy_origin[2], occupancy_resolution], dtype=np.float32)
            np.save(f"{self.map_save_path}/occupancy_grid.npy", occupancy_grid)
            np.save(f"{self.map_save_path}/occupancy_meta.npy", occupancy_meta)
            np.save(f"{self.map_save_path}/sdf_map.npy", sdf_map)
            cv2.imwrite(f"{self.map_save_path}/occupancy_2d_image.png", occupancy_2d_image)

        self._save_completed = True
        self.get_logger().info("Full mapping data saved successfully")
        self.stage_timer.log_summary(self.get_logger().info)


    def pointcloud_to_marker_array(self, points, frame_id='camera',colors=None):
        marker_array = MarkerArray()
        
        # Create point cloud Marker
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pointcloud"
        marker.id = self.marker_id
        self.marker_id = self.marker_id + 1

        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set Marker properties
        marker.scale.x = 0.03  # Point width
        marker.scale.y = 0.03  # Point height
        marker.scale.z = 0.0   # For POINTS type, z is not used
        
        # Set orientation (unit quaternion)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Set points
        marker.points = []
        for point in points:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(point[2])
            if (p.y > 0):
                marker.points.append(p)
                c = z_value_to_color(float(point[1]), -3, 1)
                marker.colors.append(c)
        
        # Set lifetime (0 means never expire)
        marker.lifetime.sec = 0
        marker.frame_locked = True
        
        marker_array.markers.append(marker) 
        
        return marker_array

    def publish_local_map(self, point_cloud, frame_id):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        marker_array = self.pointcloud_to_marker_array(point_cloud.tolist(), frame_id)
        self.marker_pub.publish(marker_array)

    def publish_all_transforms(self):
        """Publish all pose TF transforms"""
        if not self.pose_graph_used_pose:
            return
            
        transforms = []        
        for timestamp, pose_in_world in self.pose_graph_used_pose.items():
            transform = TransformStamped()
            
            # Set header
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'world'
            transform.child_frame_id = 'camera_' + str(timestamp)
            
            # Set position
            t = pose_in_world[:3, 3]
            transform.transform.translation.x = t[0]
            transform.transform.translation.y = t[1]
            transform.transform.translation.z = t[2]
            qx,qy,qz,qw =  R.from_matrix(pose_in_world[:3, :3]).as_quat()
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            transforms.append(transform)
        
        # Publish all TF transforms
        self.tf_broadcaster.sendTransform(transforms)
        

    def destroy_node(self):
        try:
            self.save_mapping()
            super().destroy_node()
        except Exception:
            # Ignore errors during destruction as resources may already be freed
            pass

class ImageTransportsNode(Node):
    def __init__(self):
        super().__init__('image_transports_node')
        # Simple compressed → raw image transport for color images.
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_rect_raw/compressed',
            self.image_callback,
            10,
        )
        self.image_pub = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg: CompressedImage):
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_msg.header.stamp = msg.header.stamp
        image_msg.header.frame_id = msg.header.frame_id
        self.image_pub.publish(image_msg)

def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(filename)s:%(lineno)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    rclpy.init(args=args)


    parser = argparse.ArgumentParser()
    parser.add_argument("--bag_file", type=str, default="tinynav_db")
    parser.add_argument("--map_save_path", type=str, default="tinynav_db")
    # Default on, so scripts and PC builds are unchanged. The board's nav build turns
    # them off from node_manager: measured 95.3 s (rgb) + 18.5 s (infra1) of h264
    # encoding per 575-keyframe build, for two videos only PC-side offline tools read.
    parser.add_argument("--no-infra1-video", dest="save_infra1_video", action="store_false",
                        help="Skip the infra1 h264 encode (tool/poi_editor.py reads it)")
    parser.add_argument("--no-rgb-video", dest="save_rgb_video", action="store_false",
                        help="Skip the rgb h264 encode (tool/convert_to_nerf_format.py reads it)")
    parser.add_argument(
        "--skip-topics", type=str, default="",
        help=(
            "Comma-separated bag topics not to replay. Empty (the default) replays "
            "everything, as before. Whether a topic is dead depends on which sensor "
            "pipeline the build runs -- perception_node needs infra2, looper mode never "
            "launches it -- so the caller decides, not this script."
        ),
    )
    # Default off, same reason as map_node: print() per keyframe onto the board's eMMC.
    parser.add_argument("--verbose_timer", action="store_true", default=False, help="Enable verbose timer output")
    parser.add_argument("--no_verbose_timer", dest="verbose_timer", action="store_false", help="Disable verbose timer output")
    parser.add_argument(
        "--global-frames-ratio",
        type=float,
        default=1.1,
        help="Minimum keyframe growth ratio before online pose graph solve and TF republish.",
    )
    parser.add_argument(
        "--play-rate", type=float, default=0.0,
        help="Bag playback speed as a multiple of real time. 0, the default, is "
             "unthrottled -- correct on a machine whose consumer outruns the bag. "
             "Pass a positive value on a slow one: unpaced playback fills the "
             "keyframe sync queue with full-resolution images and OOM-killed a "
             "build on the X5 at 0.5%% progress.",
    )
    parser.add_argument(
        "--sync-queue-size", type=int, default=200,
        help="Keyframe synchroniser queue depth. Each slot holds three "
             "full-resolution images, about 1.4 MB at 544x640, so the default 200 "
             "is 280 MB of headroom. Use ~20 on a memory-constrained board.",
    )
    parser.add_argument(
        "--no-visualization", dest="publish_visualization", action="store_false",
        help="Skip the rviz-only local pointcloud and trajectory publishes. Nothing "
             "in the saved map depends on them and they dominated the per-keyframe "
             "cost on the X5.",
    )
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
        from tinynav.core.models_trt import ORBFeatureTRTCompatible, ORBMatcher
        extractor = ORBFeatureTRTCompatible()
        matcher = ORBMatcher()
        embedding_extractor = DummyEmbeddingEngine()
    else:
        from tinynav.core.models_trt import Dinov2TRT, LightGlueTRT, SuperPointTRT
        extractor = SuperPointTRT()
        matcher = LightGlueTRT()
        embedding_extractor = Dinov2TRT()

    exec_ = SingleThreadedExecutor()
    skip_topics = {t.strip() for t in parsed_args.skip_topics.split(",") if t.strip()}
    player_node = BagPlayer(
        parsed_args.bag_file, play_rate=parsed_args.play_rate, skip_topics=skip_topics
    )
    map_node = BuildMapNode(
        parsed_args.map_save_path,
        extractor=extractor,
        matcher=matcher,
        embedding_extractor=embedding_extractor,
        loop_closure_mode=parsed_args.loop_closure_mode,
        vlad_centres_path=parsed_args.vlad_centres,
        loop_closure_use_bow=use_bow,
        dbow3_vocabulary_path=parsed_args.dbow3_vocabulary_path,
        verbose_timer=parsed_args.verbose_timer,
        global_frames_ratio=parsed_args.global_frames_ratio,
        sync_queue_size=parsed_args.sync_queue_size,
        publish_visualization=parsed_args.publish_visualization,
        save_infra1_video=parsed_args.save_infra1_video,
        save_rgb_video=parsed_args.save_rgb_video,
    )
    image_transports_node = ImageTransportsNode()
    exec_.add_node(player_node)
    exec_.add_node(map_node)
    exec_.add_node(image_transports_node)
    while rclpy.ok() and player_node.play_next():
        exec_.spin_once(timeout_sec=0.001)
    player_node._publish_percent(100.0)
    map_node.save_mapping()

if __name__ == '__main__':
    main()
