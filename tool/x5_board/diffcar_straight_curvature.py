#!/usr/bin/env python3
"""直线前进/后退，比较 VIO 和轮式里程计各自算出的每米曲率。走 ROS 话题，不碰串口。

  python3 diffcar_straight_curvature.py [单程米] [趟数] [m/s]

一个测试同时回答两个问题（kappa = 每米航向变化，带符号，除以带符号的行程）：

  前进的 kappa 和后退的 kappa 同号  -> 左右轮有效半径不一样(反向时 dtheta 和 dx 一起变号)
  前进的 kappa 和后退的 kappa 变号  -> 外部侧向力(路面横坡 / 脚轮拖拽)

  VIO 的 kappa 和 里程计的 kappa 反号 -> 确认是半径差：闭环抹平编码器速度，半径小的那侧
                                        计数反而多，固件用同一个周长换算就把符号搞反了

kappa 用回归拿，不用首尾相减：后退时后脚轮变成前置脚轮会甩一下，头 0.15 m 必须扔掉，
而回归还能顺带给出 R2 —— 半径差是匀速累积(R2 高)，甩头是瞬态(R2 低)。
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

FX = 309.49
CX, CY = 272.22, 318.80
SKIP_M = 0.15          # 每趟头这么多米不参与回归：脚轮要甩、闭环要起步
BRAKE_M = 0.40         # 深度刹车线（车体前缘在相机前 0.05 m）
HALF_W = 0.20


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


class Drive(Node):
    def __init__(self):
        super().__init__("straight_curvature")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_q = self.vio_q = None
        self.pos = None
        self.batt = None
        self.clear = None
        self._plane = None
        self._uu = self._vv = None
        self.create_subscription(Odometry, "/wheel/odometry", self._odom, 10)
        self.create_subscription(PoseStamped, "/camera/camera/vio_100hz", self._vio, 20)
        self.create_subscription(Float32, "/battery_voltage",
                                 lambda m: setattr(self, "batt", m.data), 5)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self._depth, 1)

    def _odom(self, m):
        o = m.pose.pose.orientation
        self.odom_q = (o.x, o.y, o.z, o.w)
        self.pos = np.array([m.pose.pose.position.x, m.pose.pose.position.y])

    def _vio(self, m):
        o = m.pose.orientation
        self.vio_q = (o.x, o.y, o.z, o.w)

    def _fit_floor(self, X, Y, Z):
        sel = np.isfinite(Z) & (self._vv > 420) & (self._vv < 580) & (np.abs(self._uu - CX) < 120)
        if sel.sum() < 3000:
            return None
        P = np.stack([X[sel], Y[sel], Z[sel]], 1)
        for _ in range(6):
            c = P.mean(0)
            n = np.linalg.svd(P - c, full_matrices=False)[2][-1]
            d = (P - c) @ n
            keep = np.abs(d) < max(0.006, 2.0 * d.std())
            if keep.sum() < 800:
                break
            P = P[keep]
        c = P.mean(0)
        n = np.linalg.svd(P - c, full_matrices=False)[2][-1]
        if n[1] < 0:
            n = -n
        return n, float(c @ n)

    def _depth(self, m):
        Z = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
        Z[(Z <= 0.15) | (Z > 10.0)] = np.nan
        if self._uu is None:
            self._uu, self._vv = np.meshgrid(np.arange(m.width), np.arange(m.height))
        X = (self._uu - CX) * Z / FX
        Y = (self._vv - CY) * Z / FX
        if self._plane is None:
            self._plane = self._fit_floor(X, Y, Z)
            if self._plane is None:
                return
            n, d = self._plane
            print("  地面平面: 垂距 %.4f m, 俯仰 %+.1f 度"
                  % (d, np.degrees(np.arctan2(n[2], n[1]))), flush=True)
        n, d = self._plane
        h = d - (X * n[0] + Y * n[1] + Z * n[2])
        ob = np.isfinite(Z) & (h > 0.05) & (h < 0.35) & (np.abs(X) < HALF_W) & (Z > 0.20)
        self.clear = float(np.percentile(Z[ob], 1)) if ob.sum() > 40 else 99.0


def odom_yaw(q):
    return 2.0 * np.arctan2(q[2], q[3])


def leg(node, v, dist):
    """跑一趟，返回 (里程计 kappa, VIO kappa, 实际行程, 里程计R2, VIO R2, 刹车原因)."""
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.02)
    q_v0 = node.vio_q
    y0 = odom_yaw(node.odom_q)
    p_prev = node.pos.copy()
    s = 0.0
    rec = []
    brake = None
    t0 = last_cmd = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.004)
        now = time.time()
        if node.pos is not None:
            s += float(np.linalg.norm(node.pos - p_prev))
            p_prev = node.pos.copy()
        dy = odom_yaw(node.odom_q) - y0
        while dy > np.pi:
            dy -= 2 * np.pi
        while dy < -np.pi:
            dy += 2 * np.pi
        rec.append((s, dy, rotvec_z(q_v0, node.vio_q)))
        if s >= dist:
            break
        if v > 0 and node.clear is not None and node.clear < BRAKE_M:
            brake = "深度刹车，前方只剩 %.2f m" % node.clear
            break
        if now - t0 > dist / abs(v) + 8.0:
            brake = "超时"
            break
        if now - last_cmd > 0.05:
            m = Twist()
            m.linear.x = v
            node.cmd.publish(m)
            last_cmd = now
    m = Twist()
    t1 = time.time()
    while time.time() - t1 < 1.2:
        node.cmd.publish(m)
        rclpy.spin_once(node, timeout_sec=0.01)

    use = [r for r in rec if r[0] > SKIP_M]
    if len(use) < 20:
        return None
    x = np.array([r[0] for r in use])
    sgn = 1.0 if v > 0 else -1.0

    def fit(y):
        a, b = np.polyfit(x, y, 1)
        pred = a * x + b
        ss = 1 - ((y - pred) ** 2).sum() / max(1e-12, ((y - y.mean()) ** 2).sum())
        return a * sgn, ss          # 除以带符号行程 -> kappa 带符号
    ko, r2o = fit(np.array([r[1] for r in use]))
    kv, r2v = fit(np.array([r[2] for r in use]))
    return ko, kv, s * sgn, r2o, r2v, brake


def main():
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 0.60
    cycles = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    v = float(sys.argv[3]) if len(sys.argv) > 3 else 0.20
    rclpy.init()
    node = Drive()
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom_q is not None and node.vio_q and node.pos is not None and node.clear is not None:
            break
    if node.clear is None or node.odom_q is None:
        sys.exit("数据不全")
    room = node.clear - BRAKE_M
    if room < dist:
        print("前方余量 %.2f m，刹车线 %.2f -> 单程压到 %.2f m" % (node.clear, BRAKE_M, max(0.30, room)))
        dist = max(0.30, room)
    if dist < 0.30:
        sys.exit("前方只有 %.2f m，摆不开" % node.clear)
    print("电池 %.2f V，单程 %.2f m，%d 趟往返，%.2f m/s\n" % (node.batt or 0, dist, cycles, v))

    rows = []
    print("  趟次         里程计 kappa      VIO kappa       行程      里程计R2  VIO R2")
    for i in range(cycles):
        for sgn, tag in ((+1, "前进"), (-1, "后退")):
            r = leg(node, sgn * v, dist)
            if r is None:
                print("  %d %s: 样本不足" % (i + 1, tag))
                continue
            ko, kv, s, r2o, r2v, brake = r
            rows.append((sgn, ko, kv, s))
            print("  %d %s   %+8.4f 度/m   %+8.4f 度/m   %+6.3f m    %.3f    %.3f%s"
                  % (i + 1, tag, np.degrees(ko), np.degrees(kv), s, r2o, r2v,
                     "   !! " + brake if brake else ""))
            if node.batt and node.batt < 9.9:
                print("  电压 %.2f V，停止" % node.batt)
                node.destroy_node(); rclpy.shutdown(); return

    print("\n" + "=" * 66)
    print("判读")
    print("=" * 66)
    for sgn, tag in ((+1, "前进"), (-1, "后退")):
        g = [r for r in rows if r[0] == sgn]
        if not g:
            continue
        print("  %s (n=%d): 里程计 kappa = %+.4f +- %.4f 度/m   VIO kappa = %+.4f +- %.4f 度/m"
              % (tag, len(g),
                 np.degrees(np.mean([r[1] for r in g])), np.degrees(np.std([r[1] for r in g])),
                 np.degrees(np.mean([r[2] for r in g])), np.degrees(np.std([r[2] for r in g]))))
    kf = [r for r in rows if r[0] > 0]
    kb = [r for r in rows if r[0] < 0]
    if kf and kb:
        mf, mb = np.mean([r[2] for r in kf]), np.mean([r[2] for r in kb])
        print("\n  1) 前进/后退的 VIO kappa: %+.4f vs %+.4f 度/m -> %s"
              % (np.degrees(mf), np.degrees(mb),
                 "同号，左右轮有效半径不一样" if mf * mb > 0 else "变号，是外部侧向力(横坡/脚轮拖拽)"))
    allo, allv = np.mean([r[1] for r in rows]), np.mean([r[2] for r in rows])
    print("  2) 里程计 kappa %+.4f  vs  VIO kappa %+.4f 度/m -> %s"
          % (np.degrees(allo), np.degrees(allv),
             "反号，确认是半径差(里程计把航向符号搞反了)" if allo * allv < 0 else "同号，里程计航向符号是对的"))
    if allo * allv < 0 and abs(allv) > 1e-5:
        # 真实曲率 kv 需要左右地面速度差 kv*L；里程计报了 ko，两者之差就是周长比失配
        base = 0.3234
        mism = float((abs(allv) + abs(allo)) * base)      # 每米的地面行程差
        print("\n  左右轮有效周长失配 = %.3f%%  (70mm 轮上直径差 %.2f mm)"
              % (100 * mism, 1000 * mism * 70.0))
        side = "左" if allv > 0 else "右"
        print("  %s轮偏小。修法(不用重烧固件): 把%s轮 ppr 调大 %.3f%%，再敲 w 存起来"
              % (side, side, 100 * mism))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
