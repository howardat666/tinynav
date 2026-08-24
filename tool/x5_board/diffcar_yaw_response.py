#!/usr/bin/env python3
"""原地扫转向速率，量地毯/瓷砖上的实际偏航率和起转延迟。走 ROS 话题，不碰串口。

  python3 diffcar_yaw_response.py [保持秒数]

为什么要它：2026-08-24 在地毯上发现「0.30 rad/s x 1.0 s 只兑现了不到一半」。
前馈 kff/截距 是在瓷砖上拟合的，地毯阻力大得多，闭环得靠积分器(ki=400)慢慢卷起来
才破得动静摩擦 —— 而规划器大约每 1.2 s 重发一次轨迹，短指令等于白发。

偏航率用 VIO 的 z 轴。注意 vio_100hz 的姿态里，原地偏航出现在 **z** 上而不是光学系的
y 上(2026-08-24 实测 [+0.24 +0.25 +52.59] 度)，且 z 正 = 逆时针 = 左。
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

RATES = (0.15, 0.20, 0.30, 0.40, 0.60, 0.80)
SETTLE = 1.0          # 每段之间的停顿
MOVE_THRESH = 0.05    # rad/s，判"起转了"的门限


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def rotvec_z(q0, q1):
    rel = qmul(q1, (-q0[0], -q0[1], -q0[2], q0[3]))
    v = np.array(rel[:3], float)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return 0.0
    return float(v[2] / n * 2.0 * np.arctan2(n, rel[3]))


class Yaw(Node):
    def __init__(self):
        super().__init__("yaw_response")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_q = self.vio_q = None
        self.batt = None
        self.create_subscription(Odometry, "/wheel/odometry", self._odom, 10)
        self.create_subscription(PoseStamped, "/camera/camera/vio_100hz", self._vio, 20)
        self.create_subscription(Float32, "/battery_voltage",
                                 lambda m: setattr(self, "batt", m.data), 5)

    def _odom(self, m):
        o = m.pose.pose.orientation
        self.odom_q = (o.x, o.y, o.z, o.w)

    def _vio(self, m):
        o = m.pose.orientation
        self.vio_q = (o.x, o.y, o.z, o.w)


def odom_yaw(q):
    return 2.0 * np.arctan2(q[2], q[3])


def hold(node, w, secs):
    """保持一段，返回(稳态偏航率 VIO, 稳态偏航率 轮子, 起转延迟, 总VIO角, 总轮子角, 最低电压)."""
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.02)
    q_v0, y_od0 = node.vio_q, odom_yaw(node.odom_q)
    sv, so = [], []
    vmin = 99.0
    t0 = last_cmd = time.time()
    t_move = None
    while True:
        now = time.time()
        if now - t0 >= secs:
            break
        rclpy.spin_once(node, timeout_sec=0.005)
        if now - last_cmd > 0.05:
            m = Twist()
            m.angular.z = w
            node.cmd.publish(m)
            last_cmd = now
        if node.batt:
            vmin = min(vmin, node.batt)
        sv.append((now - t0, rotvec_z(q_v0, node.vio_q)))
        so.append((now - t0, odom_yaw(node.odom_q) - y_od0))
        if t_move is None and len(sv) > 6:
            # 用最近 0.3s 的斜率判起转，别被单帧噪声骗
            recent = [p for p in sv if p[0] > sv[-1][0] - 0.3]
            if len(recent) > 3:
                r = np.polyfit([p[0] for p in recent], [p[1] for p in recent], 1)[0]
                if abs(r) > MOVE_THRESH:
                    t_move = sv[-1][0] - 0.15
    m = Twist()
    t1 = time.time()
    while time.time() - t1 < SETTLE:
        node.cmd.publish(m)
        rclpy.spin_once(node, timeout_sec=0.01)

    def rate(s):
        late = [p for p in s if p[0] > secs - 1.5]
        if len(late) < 8:
            return 0.0
        return float(np.polyfit([p[0] for p in late], [p[1] for p in late], 1)[0])
    return rate(sv), rate(so), t_move, sv[-1][1], so[-1][1], vmin


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    node = Yaw()
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom_q and node.vio_q and node.batt:
            break
    if not (node.odom_q and node.vio_q):
        sys.exit("拿不到 /wheel/odometry 或 /camera/camera/vio_100hz")
    print("电池 %.2f V，每档保持 %.1f s，正反各一次(净转角约零)\n" % (node.batt or 0, secs))
    print("  指令     VIO实测   轮子实测  兑现率   起转延迟   VIO总角  轮子总角  最低电压")
    rows = []
    for w in RATES:
        for sign in (+1, -1):
            rv, ro, tm, tv, to, vmin = hold(node, sign * w, secs)
            ratio = abs(rv) / w if w else 0.0
            rows.append((w, sign, abs(rv), abs(ro), ratio, tm))
            print("  %+.2f    %6.3f    %6.3f   %5.1f%%   %8s   %+7.1f  %+7.1f   %5.2f"
                  % (sign * w, rv, ro, 100 * ratio,
                     ("%.2f s" % tm) if tm is not None else "没起转",
                     np.degrees(tv), np.degrees(to), vmin))
            if vmin < 9.9:
                print("  电压掉到 %.2f V，停止" % vmin)
                node.destroy_node(); rclpy.shutdown(); return

    print("\n" + "=" * 60)
    print("判读")
    print("=" * 60)
    for w in RATES:
        g = [r for r in rows if r[0] == w]
        if not g:
            continue
        print("  指令 %.2f rad/s: 平均兑现 %5.1f%%   平均起转延迟 %s"
              % (w, 100 * np.mean([r[4] for r in g]),
                 ("%.2f s" % np.mean([r[5] for r in g if r[5] is not None]))
                 if any(r[5] is not None for r in g) else "没起转"))
    ok = [r for r in rows if r[4] > 0.85]
    print("\n  兑现率 >85%% 的最低指令 = %s"
          % ("%.2f rad/s" % min(r[0] for r in ok) if ok else "没有(全都不达标)"))
    # 轮子/VIO 的转角比就是有效轮距的标定量：>1 说明里程计高报转角，轮距该调大
    good = [(r[3] / r[2]) for r in rows if r[2] > 0.05]
    if good:
        print("  轮子/VIO 偏航率比 = %.4f +- %.4f (n=%d)  -> 有效轮距该乘这个数"
              % (np.mean(good), np.std(good), len(good)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
