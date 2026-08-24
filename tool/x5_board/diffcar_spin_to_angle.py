#!/usr/bin/env python3
"""原地转到"轮速里程计认为的"某个角度就停，好让人用地砖缝当量角器读残差。走 ROS 话题。

  python3 diffcar_spin_to_angle.py [目标角度 默认360] [rad/s 默认0.5]

为什么转整圈而不是 90 度:转完一圈车该回到出厂朝向，**残差不用除以任何东西**，1.4% 的标度
误差直接显示成 5 度，肉眼对着地砖缝能读到约 ±1 度 = 0.3%。转 90 度的话同样的标度误差只有
1.3 度，和肉眼精度同量级，白测。

⚠️ 跑之前先记下车现在的朝向（对齐地砖缝，或在地上贴条胶带对着车身某条边）。
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


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
    return 0.0 if n < 1e-12 else float(v[2] / n * 2.0 * np.arctan2(n, rel[3]))


class Spin(Node):
    def __init__(self):
        super().__init__("spin_to_angle")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.oq = self.vq = None
        self.batt = None
        self.create_subscription(Odometry, "/wheel/odometry",
                                 lambda m: setattr(self, "oq", (m.pose.pose.orientation.z,
                                                                m.pose.pose.orientation.w)), 10)
        self.create_subscription(PoseStamped, "/camera/camera/vio_100hz",
                                 lambda m: setattr(self, "vq", (m.pose.orientation.x,
                                                                m.pose.orientation.y,
                                                                m.pose.orientation.z,
                                                                m.pose.orientation.w)), 20)
        self.create_subscription(Float32, "/battery_voltage",
                                 lambda m: setattr(self, "batt", float(m.data)), 5)


def main():
    target = np.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 360.0)
    w = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    rclpy.init()
    node = Spin()
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.oq and node.vq:
            break
    if not (node.oq and node.vq):
        raise SystemExit("拿不到 /wheel/odometry 或 /camera/camera/vio_100hz")

    oy = lambda: 2.0 * np.arctan2(node.oq[0], node.oq[1])
    prev, acc = oy(), 0.0
    vprev, vacc = node.vq, 0.0      # VIO 也逐帧累加:rotvec 只到 360 度，整圈会绕回 0
    print("电池 %.2f V，目标 %.1f 度 @ %.2f rad/s（约 %.1f s）"
          % (node.batt or 0, np.degrees(target), w, abs(target) / w))
    t0 = last = time.time()
    vmin = 99.0
    while abs(acc) < abs(target) and time.time() - t0 < abs(target) / w + 20:
        rclpy.spin_once(node, timeout_sec=0.004)
        cur = oy()
        d = cur - prev
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        acc += d
        prev = cur
        vacc += rotvec_z(vprev, node.vq)
        vprev = node.vq
        if node.batt:
            vmin = min(vmin, node.batt)
        if time.time() - last > 0.05:
            m = Twist()
            m.angular.z = w if target > 0 else -w
            node.cmd.publish(m)
            last = time.time()
    m = Twist()
    t1 = time.time()
    while time.time() - t1 < 1.5:
        node.cmd.publish(m)
        rclpy.spin_once(node, timeout_sec=0.01)
    # 停下之后还会滑一点，再收一次
    cur = oy()
    d = cur - prev
    while d > np.pi:
        d -= 2 * np.pi
    while d < -np.pi:
        d += 2 * np.pi
    acc += d
    vacc += rotvec_z(vprev, node.vq)
    vio = np.degrees(vacc)

    print("\n轮速里程计 转过 %+.2f 度（指令目标 %+.1f）" % (np.degrees(acc), np.degrees(target)))
    print("VIO        转过 %+.2f 度" % vio)
    print("最低电压   %.2f V" % vmin)
    tot = abs(np.degrees(acc))
    turns = round(tot / 360.0) * 360.0
    print("\n=== 拿地砖缝量：车比出发朝向少转了几度？（转过头就是负数）===")
    print("  参照是**实际**转过的 %.2f 度，不是指令的 %.0f —— 停车滑行会多走几度"
          % (tot, np.degrees(abs(target))))
    print("  少转 X 度 -> 里程计角度标度 k = %.2f/(%.0f-X)，有效轮距应乘 k" % (tot, turns))
    for x in (-2, 0, 2, 3, 5, 8):
        k = tot / (turns - x)
        print("    X=%-3d -> k=%.4f  轮距 0.3234 -> %.4f m" % (x, k, 0.3234 * k))
    if abs(vio) > 1:
        print("  VIO 转过 %.2f 度，VIO/里程计 = %.4f" % (vio, abs(vio) / max(1e-6, tot)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
