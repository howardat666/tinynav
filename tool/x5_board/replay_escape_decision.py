#!/usr/bin/env python3
"""Replay the 2026-08-10 19:19 failure: 40 s of blind reverse into an obstacle.

Reproduces the real geometry from the run log -- robot at [2.6,-0.75] facing -x with
the POI 1.9 m ahead of it and the front corridor blocked -- and reports what the motion
gate admits and what the escape hatch commits to. The old gate left exactly one
admissible trajectory in a 106-entry library: the hard-coded straight reverse, which
is scored against cells the depth camera never looked at.

Calls the node's own methods (bound to a bare instance, no rclpy) so this tests the
shipped logic rather than a copy of it.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.core.math_utils import quat_to_matrix  # noqa: E402
from tinynav.core.planning_kernels import generate_trajectory_library_3d  # noqa: E402
from tinynav.core.planning_node import (  # noqa: E402
    PlanningNode,
    generate_predefined_trajectory_vocabularies,
    normalize_pose_trajectories,
)
from tinynav.core.robot_config import GO2_CONFIG, LEKIWI_CONFIG  # noqa: E402
from tinynav.platforms.omni3_kinematics import base_pose_to_camera_pose  # noqa: E402

RESOLUTION = 0.1
GRID = (100, 100)


def bare_node(robot):
    """A PlanningNode with only the fields the decision path reads. __init__ needs
    rclpy, and the point here is to run the decision, not a node."""
    n = object.__new__(PlanningNode)
    n.robot = robot
    n.resolution = RESOLUTION
    n.escape_min_clearance_m = 0.4
    n.rotate_first_min_dist_m = 0.5
    n.min_progress_m = 0.05
    n.force_turn_heading_rad = math.radians(80.0)
    n.prefix_margin_m = 0.10
    n.reaction_time_s = 2.0
    n.dt = 0.1
    n.allow_reverse = robot.allow_reverse
    n._escape_scan_step_deg = 15
    n._escape_goal_yaw = None
    n._escape_goal_reached_rad = math.radians(12.0)
    return n


def pose_at(x, y, yaw_deg):
    """4x4 camera-optical pose on the ground plane. Identity is 'facing straight up'
    in this convention, which is how an earlier offline test produced nonsense."""
    p, q = base_pose_to_camera_pose(
        x, y, np.deg2rad(yaw_deg),
        [LEKIWI_CONFIG.camera_x, LEKIWI_CONFIG.camera_y, 0.0],
    )
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(np.asarray(q, dtype=np.float64))
    T[:3, 3] = p
    return T, p, q


def blocked_mask(node, center, fx, fy, at=0.1):
    """Obstacle wall across the corridor `at` metres beyond the footprint edge."""
    mask = np.zeros(GRID, dtype=bool)
    fl, _ = node.robot.probe_geometry()
    lx, ly = -fy, fx
    for w in np.arange(-0.6, 0.61, 0.05):
        for d in (at, at + RESOLUTION):
            wx = center[0] + fx * (fl + d) + lx * w
            wy = center[1] + fy * (fl + d) + ly * w
            i = int((wx - node.origin[0]) / RESOLUTION)
            j = int((wy - node.origin[1]) / RESOLUTION)
            if 0 <= i < GRID[0] and 0 <= j < GRID[1]:
                mask[i, j] = True
    return mask


def build_library(p, q, vx_max):
    trajs, params = generate_trajectory_library_3d(init_p=p.copy(), init_q=q, dt=0.1, vx_max=vx_max)
    trajs = normalize_pose_trajectories(trajs)
    vt, vp = generate_predefined_trajectory_vocabularies(init_p=p.copy(), init_q=q, dt=0.1)
    vt = normalize_pose_trajectories(vt)
    return np.concatenate([trajs, vt], axis=0), np.concatenate([params, vp], axis=0)


def old_gate(param, front_blocked):
    """The shipped-before-this-fix gate, kept verbatim for the comparison."""
    is_backward = param[0] < 0.0
    if front_blocked and not is_backward:
        return 1e9
    if not front_blocked and is_backward:
        return 1e9
    return 0.0


def report(title, node, T, center, p, q, target, mask, front_clearance):
    trajs, params = build_library(p, q, node.robot.max_vx)
    front_blocked = front_clearance <= node.robot.front_blocked_m
    yaw_now = node._yaw_of(q)
    to_t = target[:2] - p[:2]
    yaw_to_target = float(np.arctan2(to_t[1], to_t[0]))

    # The node admits on `costs < 1e9`, which is gate AND collision, so scoring here too.
    # The 23 s wedge was the gate admitting 14 turns and collision killing 12 -- and a
    # gate-only replay of it showed nothing wrong.
    esdf = distance_transform_edt(~mask).astype(np.float32) * RESOLUTION
    scores, _ = node._score_trajectories(trajs, esdf, params)
    scores = np.asarray(scores, dtype=np.float64)

    print(f"\n=== {title} ===")
    print(f"  robot=[{p[0]:.2f},{p[1]:.2f}] yaw={np.degrees(yaw_now):+.0f}deg "
          f"target=[{target[0]:.2f},{target[1]:.2f}] "
          f"heading_err={np.degrees(node._wrap(yaw_to_target - yaw_now)):+.0f}deg")
    print(f"  front_clearance={node._fmt_clearance(front_clearance)} blocked={front_blocked} "
          f"allow_reverse={node.allow_reverse} library={len(params)} "
          f"in_collision={int(np.count_nonzero(np.isinf(scores)))}/{len(scores)}")

    stand_dist = float(np.linalg.norm(target[:2] - p[:2]))
    # 现在的门是按每条轨迹自己那一段判的（_prefix_clearance），不再是一条直线探针，
    # 所以要先算出 forward_ok 再问门 —— 这个工具停在旧的两参数签名上，早就跑不动了。
    prefix_clear = node._prefix_clearance(trajs, esdf)
    forward_ok = prefix_clear >= node.prefix_margin_m
    for name, gate in (("old", lambda pm, fb, i=None: old_gate(pm, fb)),
                       ("new", lambda pm, fb, i=0: node._motion_gate_penalty(
                           pm, i, forward_ok, fb))):
        adm = [i for i in range(len(params))
               if gate(params[i], front_blocked, i) < 1e9 and not np.isinf(scores[i])]
        turns = [i for i in adm if node._is_turn_in_place(params[i])]
        kinds = {"reverse": sum(1 for i in adm if params[i][0] < 0),
                 "turn": len(turns),
                 "forward": sum(1 for i in adm if params[i][0] > 1e-6)}
        if not adm:
            # The node's own "no admissible motion" branch: it holds still rather than
            # moving somewhere the camera has not looked.
            print(f"  {name} gate: admissible=0 {kinds} -- holds still")
            continue
        ends = np.array([trajs[i][-1, :2] for i in adm])
        gain = stand_dist - float(np.min(np.linalg.norm(ends - target[None, :2], axis=1)))
        heading_err = abs(node._wrap(yaw_to_target - yaw_now))
        if name == "old":
            # The gain-only trigger, kept verbatim so the comparison shows what the
            # heading test changes.
            reason = "fires" if front_blocked or (
                stand_dist > node.rotate_first_min_dist_m
                and gain < node.min_progress_m) else ""
        else:
            # The node's own test, not a copy of it. The copy that used to live on this
            # line is how a replay can report an escape the robot never takes.
            reason = node._escape_reason(front_blocked, stand_dist, heading_err, gain)
        fires = bool(reason)
        print(f"  {name} gate: admissible={len(adm)} {kinds} best_gain={gain:+.2f}m "
              f"heading_err={np.degrees(heading_err):+.0f}deg hatch={reason or 'off'}")
        if not fires:
            continue
        if name == "old":
            k = min(adm, key=lambda i: abs(node._wrap(yaw_to_target - node._yaw_of(trajs[i][-1, 3:7]))))
            clear = float('nan')
        else:
            # 现在锁的是朝向：先扫一圈找空的朝向，再挑朝那一侧转得最快的可行原地转。
            yaw_now = node._yaw_of(q)
            goal, clear = node._open_heading(center, mask, yaw_now, yaw_to_target)
            if goal is None:
                side = 1.0 if node._wrap(yaw_to_target - yaw_now) >= 0.0 else -1.0
                goal = node._wrap(yaw_now + side * np.pi / 2.0)
            k = node._pick_turn_toward(turns, params, node._wrap(goal - yaw_now))
            print(f"    goal heading: {np.degrees(goal):+.0f}deg "
                  f"clear={node._fmt_clearance(clear)}")
            if k is None:
                print("    朝该方向的原地转这一拍全撞 -> 保持朝向不动")
                continue
        end_yaw = node._yaw_of(trajs[k][-1, 3:7])
        print(f"    hatch picks: vx={params[k][0]:+.3f} omega={params[k][1]:+.3f} "
              f"end_yaw={np.degrees(end_yaw):+.0f}deg "
              f"clear_along_end_yaw={node._fmt_clearance(clear)} "
              f"end_heading_err={np.degrees(node._wrap(yaw_to_target - end_yaw)):+.0f}deg")


def main():
    node = bare_node(LEKIWI_CONFIG)
    node.origin = np.array(GRID + (30,)) * RESOLUTION / -2.0

    # From the run log at 1786360819.8: measured=[2.69,-0.72] yaw=-171deg,
    # target=[0.73,-0.32], front_clearance=0.10m, and the planner chose vx=-0.200.
    T, p, q = pose_at(2.69, -0.72, -171.0)
    center = node.camera_to_robot_center(T)
    fwd = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    n = np.hypot(fwd[0], fwd[1])
    mask = blocked_mask(node, center, fwd[0] / n, fwd[1] / n, at=0.0)
    fc = node._front_obstacle_dist(T, mask)
    report("blocked front, target ahead (the 19:19 failure)", node, T, center,
           p, q, np.array([0.73, -0.32, 0.16]), mask, fc)

    # Same pose, nothing in front: the 18:12 standstill, target 2 m behind.
    clearmask = np.zeros(GRID, dtype=bool)
    T2, p2, q2 = pose_at(2.69, -0.72, 0.0)
    c2 = node.camera_to_robot_center(T2)
    report("clear front, target behind (the 18:12 standstill)", node, T2, c2,
           p2, q2, np.array([0.69, -0.72, 0.16]), clearmask,
           node._front_obstacle_dist(T2, clearmask))

    # Ordinary driving must be untouched: same gate verdicts as before the fix.
    T3, p3, q3 = pose_at(0.0, 0.0, 0.0)
    c3 = node.camera_to_robot_center(T3)
    report("clear front, target ahead (ordinary driving)", node, T3, c3,
           p3, q3, np.array([2.0, 0.0, 0.16]), clearmask,
           node._front_obstacle_dist(T3, clearmask))

    # A quadruped keeps its reverse option; only the blocked branch gains turns.
    go2 = bare_node(GO2_CONFIG)
    go2.origin = node.origin
    T4, p4, q4 = pose_at(2.69, -0.72, -171.0)
    c4 = go2.camera_to_robot_center(T4)
    f4 = T4[:3, :3] @ np.array([0.0, 0.0, 1.0])
    n4 = np.hypot(f4[0], f4[1])
    m4 = blocked_mask(go2, c4, f4[0] / n4, f4[1] / n4, at=0.0)
    report("go2, blocked front (allow_reverse stays on)", go2, T4, c4,
           p4, q4, np.array([0.73, -0.32, 0.16]), m4, go2._front_obstacle_dist(T4, m4))

    # The 13:53:55 standstill, replayed from the logged numbers rather than a synthetic
    # pose. It is here because no synthetic pose reproduced it: the gain the geometry
    # happens to produce is what decided the run, and it landed on the threshold
    # exactly. Kept as a regression so a future tweak to min_progress_m cannot quietly
    # reopen a 69 s deadlock.
    print("\n=== regression: the logged 13:53:55 values ===")
    for label, n in (("old (gain only)", None), ("new (heading first)", node)):
        stand_dist, heading_err, gain = 1.913, math.radians(172.0), 0.05
        if n is None:
            fired = stand_dist > 0.5 and gain < 0.05
            print(f"  {label:20s} -> {'fires' if fired else 'off  '}  "
                  f"(0.05 < 0.05 is False, which is why the robot stood still)")
        else:
            print(f"  {label:20s} -> {n._escape_reason(False, stand_dist, heading_err, gain) or 'off'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
