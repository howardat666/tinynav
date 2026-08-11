#!/usr/bin/env python3
"""Side-by-side check of the 2026-08-11 avoidance change, on one live depth frame.

    python3 tool/x5_board/verify_avoidance_change.py [--target-behind]

Runs the whole avoidance path twice on the *same* live frame, once per configuration.
Read-only. The pair is the point: cells should fall, in_collision should stop eating the
in-place turns, and ordinary forward driving should pick the trajectory it picked before.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import replace

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.core.math_utils import quat_to_matrix  # noqa: E402
from tinynav.core.planning_kernels import (  # noqa: E402
    generate_trajectory_library_3d,
    run_raycasting_loopy,
)
from tinynav.core.planning_node import (  # noqa: E402
    ObstacleConfig,
    PlanningNode,
    build_obstacle_map,
    generate_predefined_trajectory_vocabularies,
    normalize_pose_trajectories,
)
from tinynav.core.robot_config import LEKIWI_CONFIG  # noqa: E402

GRID = (100, 100, 10)
RESOLUTION = 0.1
STEP = 10

# `shape='square'` at 0.30 x 0.30 is not an approximation of the old behaviour, it is it:
# the old code took half_size = (radius, radius) for a circle and sampled that box's
# corners, so this reproduces the 0.2121 m reach, 5 lookups, 1e-3 threshold and 4 points.
BEFORE = replace(
    LEKIWI_CONFIG,
    shape='square', length=0.30, width=0.30,
    collision_radius=None,
    obstacle_z_bottom=-0.4,
    obstacle_z_top=0.4,
    dilation_cells=1,
    allow_reverse=False,
)


class Grab(Node):
    """Fourth observer: depth, camera intrinsics and pose, nothing published."""

    def __init__(self):
        super().__init__("verify_avoidance_grab")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.depth = self.K = self.T = None
        # One frame is not enough: a single sweep leaves most columns one layer tall, so
        # the z-span test rejects everything and obstacle_cells comes out 0 regardless.
        self.frames = []
        self.create_subscription(Image, "/slam/depth", self._on_depth, qos)
        self.create_subscription(CameraInfo, "/slam/camera_info", self._on_info, qos)
        self.create_subscription(Odometry, "/slam/odometry_visual", self._on_odom, qos)

    def _on_depth(self, m):
        raw = np.frombuffer(m.data, dtype=np.uint16 if m.encoding == "mono16" else np.float32)
        raw = raw.reshape(m.height, m.width)
        self.depth = raw.astype(np.float32) / 1000.0 if m.encoding == "mono16" else raw.astype(np.float32)
        if self.T is not None:
            self.frames.append((self.depth, self.T.copy()))

    def _on_info(self, m):
        self.K = np.asarray(m.k, dtype=np.float64).reshape(3, 3)

    def _on_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(np.array([o.x, o.y, o.z, o.w]))
        T[:3, 3] = [p.x, p.y, p.z]
        self.T = T


def bare_node(robot, origin):
    """A PlanningNode carrying only the fields the avoidance path reads, so this calls
    the node's own methods and not copies that agree until they quietly diverge."""
    n = object.__new__(PlanningNode)
    n.robot = robot
    n.resolution = RESOLUTION
    n.origin = origin
    n.grid_shape = GRID
    n.rotate_first_min_dist_m = 0.5
    n.min_progress_m = 0.05
    n.force_turn_heading_rad = math.radians(80.0)
    n.escape_min_clearance_m = max(0.4, robot.front_blocked_m + 0.1)
    n.last_param = (0.0, 0.0)
    n.obstacle_config = ObstacleConfig(
        robot_z_bottom=robot.obstacle_z_bottom,
        robot_z_top=robot.obstacle_z_top,
        dilation_cells=robot.dilation_cells,
    )
    return n


def pipeline(label, robot, frames, K, target):
    T = frames[-1][1]
    origin = T[:3, 3] - np.array(GRID) * RESOLUTION / 2.0
    node = bare_node(robot, origin)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    # sync_callback's accumulation, verbatim. Not rolled: the robot is stationary here.
    grid = np.zeros(GRID)
    for depth_i, T_i in frames:
        grid *= 0.99
        grid += run_raycasting_loopy(depth_i, T_i, GRID, fx, fy, cx, cy, origin, STEP, RESOLUTION)
        np.clip(grid, -0.2, 0.2, out=grid)
    occ = grid
    mask = build_obstacle_map(occ, origin, RESOLUTION, T[2, 3], node.obstacle_config)
    esdf = distance_transform_edt(~mask).astype(np.float32) * RESOLUTION

    init_p = node.camera_to_robot_center(T)
    init_q = np.asarray(_quat_of(T), dtype=np.float64)
    trajs, params = generate_trajectory_library_3d(
        init_p=init_p.copy(), init_q=init_q, dt=0.1, vx_max=robot.max_vx)
    trajs = normalize_pose_trajectories(trajs)
    vt, vp = generate_predefined_trajectory_vocabularies(init_p=init_p.copy(), init_q=init_q, dt=0.1)
    vt = normalize_pose_trajectories(vt)
    trajs = np.concatenate([trajs, vt], axis=0)
    params = np.concatenate([params, vp], axis=0)

    scores, _ = node._score_trajectories(trajs, esdf, params)
    scores = np.asarray(scores, dtype=np.float64)

    front_clearance = node._front_obstacle_dist(T, mask)
    front_blocked = front_clearance <= robot.front_blocked_m

    yaw_now = node._yaw_of(init_q)
    to_t = target[:2] - init_p[:2]
    yaw_to_target = math.atan2(float(to_t[1]), float(to_t[0]))
    stand_dist = float(np.linalg.norm(to_t))

    costs = np.array([
        scores[i] * 100000
        + 100 * float(np.linalg.norm(trajs[i][-1, :2] - target[:2]))
        + 40 * abs(node.last_param[0] - params[i][0])
        + 10 * abs(node.last_param[1] - params[i][1])
        + node._motion_gate_penalty(params[i], front_blocked)
        for i in range(len(trajs))
    ])
    admissible = np.flatnonzero(costs < 1e9)
    turns = [int(i) for i in admissible if node._is_turn_in_place(params[i])]
    if len(admissible):
        ends = np.array([trajs[i][-1, :2] for i in admissible])
        gain = stand_dist - float(np.min(np.linalg.norm(ends - target[None, :2], axis=1)))
    else:
        gain = float("nan")
    reason = node._escape_reason(
        front_blocked, stand_dist, abs(node._wrap(yaw_to_target - yaw_now)), gain)

    if len(admissible) == 0 or (front_blocked and not turns):
        pick, how = -1, "holds still"
    elif reason and turns:
        pick, _ = node._pick_escape_turn(init_p, turns, trajs, yaw_to_target, mask)
        how = f"escape={reason}"
    else:
        pick, how = int(np.argsort(costs, kind="stable")[0]), "ranked"

    footprint = node._CIRCLE_SEGMENTS if robot.is_circle else 4
    print(f"\n--- {label} ---")
    print(f"  z_band=[{robot.obstacle_z_bottom:+.2f},{robot.obstacle_z_top:+.2f}] "
          f"dilation={robot.dilation_cells} hull={robot.hull_radius:.2f} "
          f"hard/soft={robot.hard_clearance:.3f}/{robot.soft_clearance:.3f} "
          f"reverse={robot.allow_reverse} footprint_pts={footprint}")
    print(f"  obstacle_cells={int(np.count_nonzero(mask))}  "
          f"esdf_at_robot={node._esdf_at(esdf, init_p):.2f}m  "
          f"front_clearance={node._fmt_clearance(front_clearance)} blocked={front_blocked}")
    print(f"  in_collision={int(np.count_nonzero(np.isinf(scores)))}/{len(scores)}  "
          f"admissible={len(admissible)}  turns_admissible={len(turns)}  "
          f"best_gain={gain:+.2f}m  escape={reason or 'off'}")
    if pick >= 0:
        print(f"  chose vx={params[pick][0]:+.3f} omega={params[pick][1]:+.3f} ({how})")
    else:
        print(f"  {how}")
    return {
        "cells": int(np.count_nonzero(mask)),
        "collision": int(np.count_nonzero(np.isinf(scores))),
        "turns": len(turns),
        "pick": None if pick < 0 else (float(params[pick][0]), float(params[pick][1])),
    }


def _quat_of(T):
    from tinynav.core.math_utils import matrix_to_quat
    return matrix_to_quat(T[:3, :3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0, help="how long to collect frames")
    ap.add_argument("--frames", type=int, default=25, help="depth frames to accumulate")
    ap.add_argument("--target-behind", action="store_true",
                    help="place the synthetic target 2 m behind instead of 2 m ahead")
    args = ap.parse_args()

    rclpy.init()
    g = Grab()
    t0 = time.time()
    while time.time() - t0 < args.seconds and len(g.frames) < args.frames:
        rclpy.spin_once(g, timeout_sec=0.1)
    frames, K = g.frames[:args.frames], g.K
    g.destroy_node()
    rclpy.shutdown()
    if not frames or K is None:
        print(f"no frames: got {len(frames)} K={K is not None} -- is the app running?")
        return 1
    T = frames[-1][1]

    # Along the current heading exercises the ranked path; --target-behind the escape.
    fwd = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    n = math.hypot(fwd[0], fwd[1]) or 1.0
    sign = -1.0 if args.target_behind else 1.0
    target = T[:3, 3] + sign * 2.0 * np.array([fwd[0] / n, fwd[1] / n, 0.0])

    moved = float(np.max([np.linalg.norm(f[1][:3, 3] - T[:3, 3]) for f in frames]))
    print(f"{len(frames)} frames, depth{frames[-1][0].shape} robot_z={T[2, 3]:+.3f} "
          f"f={K[0, 0]:.1f} moved<={moved:.3f}m")
    if moved > 0.1:
        print("  !! the robot moved more than one voxel during collection; the grid is "
              "not rolled here, so these numbers mix poses")
    print(f"target {'BEHIND' if args.target_behind else 'AHEAD'} at "
          f"[{target[0]:.2f},{target[1]:.2f}]")

    before = pipeline("BEFORE (shipped 2026-08-10)", BEFORE, frames, K, target)
    after = pipeline("AFTER  (this change)", LEKIWI_CONFIG, frames, K, target)

    print("\n=== summary ===")
    print(f"  obstacle cells     {before['cells']:5d} -> {after['cells']:5d}"
          f"   (expect a fall, dilation 1 -> 0 measured x2.3, plus the z-band cap)")
    print(f"  trajectories in collision {before['collision']:4d} -> {after['collision']:4d}")
    print(f"  in-place turns admissible {before['turns']:4d} -> {after['turns']:4d}"
          f"   (must be > 0, or the escape hatch fails silently)")
    print(f"  chose              {before['pick']} -> {after['pick']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
