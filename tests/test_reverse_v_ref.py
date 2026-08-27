#!/usr/bin/env python3
"""_rebuild_path 必须给倒车路径一个负的 v_ref。

符号错的后果是车朝障碍开而不是退开，且日志里规划侧看起来完全正常 ——
2026-08-27 实测 17/17 条 RETREAT 被执行成前进，所以这个符号必须有测试。
"""
import math
import sys
import os

import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tinynav.platforms.cmd_vel_control import CmdVelControlNode  # noqa: E402
from tinynav.core.math_utils import matrix_to_quat  # noqa: E402


def body_to_world(yaw):
    """相机光学约定：车体 z 是车头、车体 y 朝下。世界系 z 竖直向上。

    planning_node 造轨迹时绕**车体 y**（也就是朝下轴）转 yaw，位移沿车体 z，
    而 _rebuild_path 取 R @ [0,0,1] 的 xy 分量当朝向 —— 所以测试必须用同一个约定，
    否则 yaw 会退化成 0/π，用例看着过了其实什么都没测。
    """
    c, s_ = math.cos(yaw), math.sin(yaw)
    return np.array([[s_, 0.0, c],
                     [-c, 0.0, s_],
                     [0.0, -1.0, 0.0]], dtype=np.float64)


def make_path(speed, omega, yaw0=0.0, n=31, dt=0.1):
    """按 planning_node 的口径造一条路径：位姿朝向是车的真实朝向，位移沿车头方向。"""
    p = np.zeros(2)
    yaw = yaw0
    path = Path()
    for i in range(n):
        yaw += omega * dt
        p = p + np.array([math.cos(yaw), math.sin(yaw)]) * speed * dt
        q = matrix_to_quat(body_to_world(yaw))
        ps = PoseStamped()
        ps.header.stamp.sec = int(i * dt)
        ps.header.stamp.nanosec = int((i * dt % 1.0) * 1e9)
        ps.pose.position.x, ps.pose.position.y = float(p[0]), float(p[1])
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])
        path.poses.append(ps)
    return path


CASES = [
    ("直退 0.06",        -0.06, 0.00, 0.0, -1),
    ("带转向倒车 +0.5",  -0.06, 0.50, 0.0, -1),
    ("带转向倒车 -0.5",  -0.06, -0.50, 0.0, -1),
    ("直退，车头朝 -x",  -0.06, 0.00, math.pi, -1),
    ("前进 0.25",        +0.25, 0.00, 0.0, +1),
    ("前进带转向",       +0.25, 0.30, 0.0, +1),
    ("前进，车头朝 +y",  +0.25, 0.00, math.pi / 2, +1),
]

fail = 0
for name, speed, omega, yaw0, want in CASES:
    ref = CmdVelControlNode._rebuild_path(None, make_path(speed, omega, yaw0))
    v = ref[:-1, 3]
    ok = bool(np.all(np.sign(v) == want)) and abs(abs(v).mean() - abs(speed)) < 0.01
    fail += not ok
    print("  %-18s v_ref 均值 %+.3f (期望符号 %+d, |v|≈%.2f)  %s"
          % (name, v.mean(), want, abs(speed), "OK" if ok else "**FAIL**"))

# 原地转：位移≈0，符号无意义，只要求不炸且量级接近 0
ref = CmdVelControlNode._rebuild_path(None, make_path(0.0, 0.5))
ok = abs(ref[:-1, 3]).max() < 1e-6
fail += not ok
print("  %-18s |v_ref| 最大 %.2e  %s" % ("原地转", abs(ref[:-1, 3]).max(), "OK" if ok else "**FAIL**"))

print("全部通过" if not fail else "%d 个用例失败" % fail)
sys.exit(1 if fail else 0)
