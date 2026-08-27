#!/usr/bin/env python3
"""脱困转向必须朝一个方向转到底，不能左右摆头。

场景抄 2026-08-27 板上那次：车头前 0.2 m 一堵墙，其余方向 0.5 m 内全空。
旧实现（锁转向符号）每 0.9 s 变一次向、累计只转 13°；新实现锁的是朝向。
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tinynav.core.planning_node import PlanningNode          # noqa: E402
from tinynav.core.robot_config import DIFFCAR_CONFIG         # noqa: E402

RES = 0.1
GRID = (100, 100)
ORIGIN = np.array([-5.0, -5.0, -0.3])


def bare_node():
    n = object.__new__(PlanningNode)
    n.robot = DIFFCAR_CONFIG
    n.resolution = RES
    n.origin = ORIGIN
    n.escape_min_clearance_m = 0.4
    n._escape_scan_step_deg = 15
    return n


def wall_mask(wall_x):
    """x >= wall_x 全是障碍，其余空。"""
    m = np.zeros(GRID, dtype=bool)
    xs = ORIGIN[0] + (np.arange(GRID[0]) + 0.5) * RES
    m[xs >= wall_x, :] = True
    return m


def run(node, mask, centre, yaw0, omegas, dt=0.1, steps=30, cycles=40):
    """把决策循环跑起来：每周期挑朝向、挑转速、按该转速积分一个周期的转角。"""
    yaw = yaw0
    signs = []
    for _ in range(cycles):
        goal, _ = node._open_heading(centre, mask, yaw, yaw_to_target=yaw0)
        if goal is None:
            goal = node._wrap(yaw + math.pi / 2.0)
        if node._escape_goal_yaw is None or abs(node._wrap(node._escape_goal_yaw - yaw)) < node._escape_goal_reached_rad:
            node._escape_goal_yaw = goal
        err = node._wrap(node._escape_goal_yaw - yaw)
        turns = list(range(len(omegas)))
        params = np.array([[0.0, w] for w in omegas])
        k = node._pick_turn_toward(turns, params, err)
        if k is None:
            continue
        w = omegas[k]
        signs.append(math.copysign(1, w))
        yaw = node._wrap(yaw + w * dt * steps)
        # 走廊空了就算脱困成功
        if node._clearance_along(centre, math.cos(yaw), math.sin(yaw), mask) >= node.escape_min_clearance_m:
            return yaw, signs, True
    return yaw, signs, False


fail = 0
# 车在 x=0，墙在 x=0.3（前缘 0.10 -> 墙离前缘 0.2 m），车头 +x
for name, omegas in [("只有最小转速可用", [0.15, -0.15]),
                     ("有大转速可用",     [0.15, -0.15, 0.6, -0.6, 1.05, -1.05])]:
    node = bare_node()
    node._escape_goal_yaw = None
    node._escape_goal_reached_rad = math.radians(12.0)
    yaw, signs, ok = run(node, wall_mask(0.3), np.array([0.0, 0.0, 0.0]), 0.0, omegas)
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    good = ok and flips == 0
    fail += not good
    print("  %-16s 周期 %2d 次  变向 %d 次  最终朝向 %+.0f°  脱困=%s  %s"
          % (name, len(signs), flips, math.degrees(yaw), ok, "OK" if good else "**FAIL**"))

# 一圈都堵住时不能返回一个假的"空朝向"
node = bare_node()
g, c = node._open_heading(np.array([0.0, 0.0, 0.0]), np.ones(GRID, dtype=bool), 0.0, 0.0)
ok = g is None
fail += not ok
print("  %-16s 返回 %s  %s" % ("四周全堵", g, "OK" if ok else "**FAIL**"))

# 平手时偏向目标那一侧
node = bare_node()
g_left, _ = node._open_heading(np.array([0.0, 0.0, 0.0]), wall_mask(0.3), 0.0, math.radians(90))
g_right, _ = node._open_heading(np.array([0.0, 0.0, 0.0]), wall_mask(0.3), 0.0, math.radians(-90))
ok = g_left is not None and g_right is not None and g_left > 0 > g_right
fail += not ok
print("  %-16s 目标在左 -> %+.0f°，在右 -> %+.0f°  %s"
      % ("偏向目标一侧", math.degrees(g_left or 0), math.degrees(g_right or 0),
         "OK" if ok else "**FAIL**"))

print("全部通过" if not fail else "%d 个用例失败" % fail)
sys.exit(1 if fail else 0)
