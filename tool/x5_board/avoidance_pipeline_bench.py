#!/usr/bin/env python3
"""How long the avoidance pipeline takes, at two resolutions and with numba off. ON THE BOARD.

    python3 tool/x5_board/avoidance_pipeline_bench.py

Times the four per-cycle stages at resolution 0.10 and 0.05, then again through the njit
kernels' .py_func to price numba. Both answers came out against intuition; see
docs/x5/nav_obstacle_tuning.md sections 6 and 7.
"""
import os
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from scipy.ndimage import distance_transform_edt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from tinynav.core.planning_kernels import (
    run_raycasting_loopy, generate_trajectory_library_3d, score_trajectories_by_ESDF)
from tinynav.core.planning_node import build_obstacle_map, ObstacleConfig, normalize_pose_trajectories
from tinynav.core.math_utils import quat_to_matrix
from tinynav.core.robot_config import LEKIWI_CONFIG as R


class Grab(Node):
    """Fourth observer: depth, intrinsics and pose off live topics, nothing published."""

    def __init__(self):
        super().__init__('plan_bench_grab')
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.depth = self.K = self.T = None
        self.create_subscription(Image, '/slam/depth', self._d, q)
        self.create_subscription(CameraInfo, '/slam/camera_info', self._k, q)
        self.create_subscription(Odometry, '/slam/odometry_visual', self._o, q)

    def _d(self, m):
        a = np.frombuffer(m.data, dtype=np.uint16 if m.encoding == 'mono16' else np.float32)
        a = a.reshape(m.height, m.width)
        self.depth = (a.astype(np.float32) / 1000.0) if m.encoding == 'mono16' else a.astype(np.float32)

    def _k(self, m):
        self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)

    def _o(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(np.array([o.x, o.y, o.z, o.w]))
        T[:3, 3] = [p.x, p.y, p.z]
        self.T = T


def tm(fn, n, *a, **kw):
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        r = fn(*a, **kw)
        ts.append((time.perf_counter() - t0) * 1000)
    ts.sort()
    return ts[len(ts) // 2], r


def pipeline(depth, T, K, res, shape, step, label, n=7):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    origin = T[:3, 3] - np.array(shape) * res / 2
    t_ray, occ = tm(run_raycasting_loopy, n, depth, T, shape, fx, fy, cx, cy, origin, step, res)
    np.clip(occ, -0.2, 0.2, out=occ)
    cfg = ObstacleConfig(robot_z_bottom=R.obstacle_z_bottom, robot_z_top=R.obstacle_z_top,
                         dilation_cells=R.dilation_cells)
    t_obs, mask = tm(build_obstacle_map, n, occ, origin, res, T[2, 3], cfg)
    t_esdf, esdf = tm(lambda: distance_transform_edt(~mask).astype(np.float32) * res, n)
    trajs, params = generate_trajectory_library_3d(init_p=T[:3, 3].copy(),
                                                  init_q=np.array([0., 0., 0., 1.]),
                                                  dt=0.1, vx_max=R.max_vx)
    trajs = normalize_pose_trajectories(trajs)
    fl, rl, hw = R.footprint_from_control()
    t_sc, _ = tm(score_trajectories_by_ESDF, n, trajs, esdf, origin, res,
                 R.hard_clearance, R.soft_clearance, fl, rl, hw, R.is_circle)
    tot = t_ray + t_obs + t_esdf + t_sc
    print(f'{label:<28s}{t_ray:9.1f}{t_obs:9.1f}{t_esdf:9.1f}{t_sc:9.1f}{tot:10.1f}   '
          f'cells={int(np.count_nonzero(mask))}')
    return tot


def main():
    rclpy.init()
    g = Grab()
    t0 = time.time()
    while time.time() - t0 < 20 and (g.depth is None or g.K is None or g.T is None):
        rclpy.spin_once(g, timeout_sec=0.1)
    depth, K, T = g.depth, g.K, g.T
    g.destroy_node()
    rclpy.shutdown()
    if depth is None or K is None or T is None:
        print(f'nothing grabbed: depth={depth is not None} K={K is not None} T={T is not None}')
        return 1
    print(f'depth {depth.shape} valid={int(np.count_nonzero(np.isfinite(depth) & (depth > 0)))} '
          f'robot_z={T[2,3]:+.3f} f={K[0,0]:.1f}')

    print(f'\n{"":28s}{"raycast":>9s}{"obst map":>9s}{"ESDF":>9s}{"score":>9s}{"total(ms)":>10s}')
    a = pipeline(depth, T, K, 0.10, (100, 100, 10), 10, 'numba  res=0.10 step=10')
    b = pipeline(depth, T, K, 0.05, (200, 200, 20), 10, 'numba  res=0.05 step=10')
    print(f'\n  ==> res 0.05 / 0.10 = x{b/a:.2f}   (the cycle budget is 200 ms at 5 Hz)')

    # Same kernels, undecorated: njit exposes the original as .py_func, so this prices
    # numba on the very functions that ship rather than on a rewritten copy.
    print(f'\n{"":28s}{"raycast":>9s}{"obst map":>9s}{"ESDF":>9s}{"score":>9s}{"total(ms)":>10s}')
    import tinynav.core.planning_kernels as pk
    pk_ray, pk_sc = pk.run_raycasting_loopy, pk.score_trajectories_by_ESDF
    pk.run_raycasting_loopy = pk_ray.py_func
    pk.score_trajectories_by_ESDF = pk_sc.py_func
    globals()['run_raycasting_loopy'] = pk_ray.py_func
    globals()['score_trajectories_by_ESDF'] = pk_sc.py_func
    c = pipeline(depth, T, K, 0.10, (100, 100, 10), 10, 'pure py res=0.10 step=10', n=2)
    print(f'\n  ==> pure Python / numba = x{c/a:.0f}   (same frame, same kernels, res=0.10)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
