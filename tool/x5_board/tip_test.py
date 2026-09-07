#!/usr/bin/env python3
"""用 VIO 俯仰角量倾倒门限。不需要秤。

只装前脚轮时，**前进加速**才是风险相位：惯性力向后 → 绕驱动轴抬头 → 前脚轮卸载 →
后方没有支撑。刹车和倒车加速是把脚轮压紧，安全方向。

判据不是"翻了没翻"，而是俯仰角的峰值随加速度怎么长：
  线性、峰值小、立刻回落  -> 只是悬挂/轮胎的弹性变形，脚轮还压着地
  某一档突然跳一个台阶     -> 脚轮离地了，那一档就是门限
  抬起来不回落            -> 已经过门限，立刻人工扶住

位移极小（每次阶跃约 0.1 m，正反成对相消），1x1 m 空地够用。
⚠️ VIO 的**平移**在快速原地转时会飞（实测 1 m），但**姿态**逐段与轮速里程计一致到 1°，
所以这里只读姿态。
"""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)


def pitch_of(q) -> float:
    """相机光学约定（+z 前、+x 右、+y 下）在 z 朝上世界系里的俯仰角，抬头为正。

    前向向量 f = R @ [0,0,1]；它的世界 z 分量就是 sin(俯仰)。这样取和 yaw 无关，
    车转到哪个方向都能直接比。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    fz = 1.0 - 2.0 * (x * x + y * y)
    return math.asin(max(-1.0, min(1.0, fz)))


class TipTest(Node):
    def __init__(self, args):
        super().__init__("tip_test")
        self.pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.rows: list[tuple[float, float, float, float]] = []   # t, pitch, v_cmd, v_meas
        self._pitch = float("nan")
        self._vmeas = 0.0
        self._cmd = 0.0
        self.create_subscription(PoseStamped, args.vio_topic,
                                 lambda m: setattr(self, "_pitch", pitch_of(m.pose.orientation)),
                                 _QOS)
        self.create_subscription(Odometry, args.odom_topic,
                                 lambda m: setattr(self, "_vmeas", m.twist.twist.linear.x),
                                 _QOS)
        self.t0 = time.monotonic()
        self.create_timer(0.05, self._tick)
        self.plan = build_plan(args.vmax)
        self.marks: list[tuple[float, float, float]] = []          # 起点, 终点, 目标速度

    def _t(self) -> float:
        return time.monotonic() - self.t0

    def _tick(self) -> None:
        t, acc = self._t(), 0.0
        v = 0.0
        for dur, vv in self.plan:
            if t < acc + dur:
                v = vv
                break
            acc += dur
        self._cmd = v
        m = Twist()
        m.linear.x = v
        self.pub.publish(m)
        self.rows.append((t, self._pitch, v, self._vmeas))

    def duration(self) -> float:
        return sum(d for d, _ in self.plan)


def build_plan(vmax: float) -> list[tuple[float, float]]:
    """(时长, vx)。每一档前进都配一个等量的后退把位移收回来。
    档位递增是为了给人留反应时间 —— 固件的加速度上限是恒定的，但阶跃越大、
    维持加速的时间越长，抬头能转过的角度也越大。"""
    plan = [(3.0, 0.0)]
    for v in (0.08, 0.12, 0.18, vmax):
        plan += [(0.6, v), (2.5, 0.0), (0.6, -v), (2.5, 0.0)]
    return plan


def report(rows, vmax) -> None:
    a = np.array([(t, p, c, m) for t, p, c, m in rows if not math.isnan(p)])
    if len(a) < 50:
        print("VIO 姿态样本太少，没法判")
        return
    t, pitch, cmd, meas = a[:, 0], np.rad2deg(a[:, 1]), a[:, 2], a[:, 3]
    base = np.median(pitch[cmd == 0.0])
    print(f"静止基线俯仰 {base:+.2f}°（抬头为正）\n")
    print("  阶跃  |  实测加速度  | 俯仰峰值(相对基线) | 阶跃结束 1s 后是否回落")
    # 找每个非零指令段
    nz = cmd != 0.0
    edges = np.flatnonzero(np.diff(nz.astype(int)) == 1) + 1
    for i in edges:
        j = i
        while j < len(cmd) and cmd[j] == cmd[i]:
            j += 1
        seg = slice(i, min(j + 20, len(cmd)))            # 阶跃 + 之后 1 s
        dv = np.diff(meas[i:j])
        dtv = np.diff(t[i:j])
        acc = float(np.max(dv / np.maximum(dtv, 1e-3))) if len(dv) else float("nan")
        peak = pitch[seg] - base
        k = int(np.argmax(np.abs(peak)))
        after = slice(min(j + 10, len(cmd) - 1), min(j + 30, len(cmd)))
        settled = abs(np.median(pitch[after]) - base) < 0.5 if after.stop > after.start else None
        print(f"  {cmd[i]:+.2f}  |  {acc:8.2f}    |     {peak[k]:+6.2f}°        | "
              f"{'✅ 回落' if settled else ('⚠️ 没回落' if settled is False else '—')}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vmax", type=float, default=0.25)
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--vio-topic", default="/camera/camera/vio_image")
    ap.add_argument("--odom-topic", default="/wheel/odometry")
    args = ap.parse_args()
    rclpy.init()
    node = TipTest(args)
    node.get_logger().info(f"tip test: {node.duration():.0f} s, 最大档 {args.vmax} m/s")
    try:
        while rclpy.ok() and node._t() < node.duration() + 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(6):
            node.pub.publish(Twist())
            time.sleep(0.02)
        rows = list(node.rows)
        node.destroy_node()
        rclpy.shutdown()
    report(rows, args.vmax)


if __name__ == "__main__":
    main()
