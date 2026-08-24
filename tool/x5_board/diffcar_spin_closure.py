#!/usr/bin/env python3
"""转整整一圈，用图像闭合绝对标定有效轮距。走 ROS 话题，不碰串口。

  python3 diffcar_spin_closure.py [rad/s]

原理：原地自转 360 度后，相机回到**完全相同的位姿**，所以那一刻图像视差恰好为零 ——
这是唯一一个不受近景视差污染的角度基准。做法是转过 360 度一点，沿途记下每帧的水平
轮廓和当时的里程计读数，事后找"相对起始帧位移为零"的那一刻，读出里程计当时报的角度。
里程计报 360+e 度就说明它高报 e/360，有效轮距要乘 (360+e)/360。

不用 0.15~0.30 rad/s：地毯上那一段静摩擦主导，兑现率只有 57~81%(2026-08-24 实测)。
也不用 0.80：那一档 VIO 的瞬时速率和自己的总角度对不上，明显丢帧/跳变。
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
BAND = (120, 420)
MAXS = 60
OVERSHOOT_DEG = 25.0     # 多转这么多，好让闭合点落在记录区间里
EVAL_FROM_DEG = 320.0    # 窗口要能覆盖里程计标度 0.89~1.07，窄了闭合点会掉到区间外


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


def profile(img):
    band = img[BAND[0]:BAND[1], :].astype(np.float32)
    return np.abs(np.diff(band, axis=1)).sum(axis=0)


def best_shift(a, b):
    """返回 s 使 b[u] ~= a[u-s]（内容右移为正），以及峰值相关系数。"""
    n = len(a)
    best, bs, cs = -9e9, 0, {}
    for s in range(-MAXS, MAXS + 1):
        x, y = (b[s:], a[:n - s]) if s >= 0 else (b[:n + s], a[-s:])
        x = x - x.mean()
        y = y - y.mean()
        d = float(np.sqrt((x * x).sum() * (y * y).sum()))
        c = float((x * y).sum() / d) if d > 1e-9 else -9e9
        cs[s] = c
        if c > best:
            best, bs = c, s
    if -MAXS < bs < MAXS:
        y0, y1, y2 = cs[bs - 1], cs[bs], cs[bs + 1]
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-12:
            bs = bs + 0.5 * (y0 - y2) / den
    return float(bs), best


class Spin(Node):
    def __init__(self):
        super().__init__("spin_closure")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_q = self.vio_q = None
        self.batt = None
        self.img = None
        self.img_seq = 0
        self.create_subscription(Odometry, "/wheel/odometry", self._odom, 10)
        self.create_subscription(PoseStamped, "/camera/camera/vio_100hz", self._vio, 20)
        self.create_subscription(Float32, "/battery_voltage",
                                 lambda m: setattr(self, "batt", m.data), 5)
        self.create_subscription(Image, "/camera/camera/infra1/image_rect_raw", self._infra, 1)

    def _odom(self, m):
        o = m.pose.pose.orientation
        self.odom_q = (o.x, o.y, o.z, o.w)

    def _vio(self, m):
        o = m.pose.orientation
        self.vio_q = (o.x, o.y, o.z, o.w)

    def _infra(self, m):
        self.img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        self.img_seq += 1


def unwrapped(prev, cur, acc):
    """里程计 yaw 是 (-pi,pi]，转一圈必然跨越。逐帧解缠。"""
    d = cur - prev
    while d > np.pi:
        d -= 2 * np.pi
    while d < -np.pi:
        d += 2 * np.pi
    return acc + d, cur


def one_revolution(node, w):
    sign = 1.0 if w > 0 else -1.0
    target = np.radians(360.0 + OVERSHOOT_DEG)
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.02)
    p0 = profile(node.img)
    q_v0 = node.vio_q
    prev = 2.0 * np.arctan2(node.odom_q[2], node.odom_q[3])
    acc = 0.0
    seen = node.img_seq
    rec = []                       # (里程计角 rad, VIO角 rad, 轮廓)
    t0 = last_cmd = time.time()
    vmin = 99.0
    while abs(acc) < target and time.time() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.004)
        now = time.time()
        if now - last_cmd > 0.05:
            m = Twist()
            m.angular.z = w
            node.cmd.publish(m)
            last_cmd = now
        acc, prev = unwrapped(prev, 2.0 * np.arctan2(node.odom_q[2], node.odom_q[3]), acc)
        if node.batt:
            vmin = min(vmin, node.batt)
        if node.img_seq != seen:
            seen = node.img_seq
            if abs(np.degrees(acc)) > EVAL_FROM_DEG:
                rec.append((acc, rotvec_z(q_v0, node.vio_q), profile(node.img)))
    m = Twist()
    t1 = time.time()
    while time.time() - t1 < 1.5:
        node.cmd.publish(m)
        rclpy.spin_once(node, timeout_sec=0.01)

    print("  转完: 里程计累计 %+.2f 度, VIO %+.2f 度, 闭合区记录 %d 帧, 最低电压 %.2f V"
          % (np.degrees(acc), np.degrees(rotvec_z(q_v0, node.vio_q)), len(rec), vmin))
    if len(rec) < 6:
        print("  !! 闭合区帧数不够，没法定闭合点")
        return None
    ev = [(np.degrees(a), np.degrees(v)) + best_shift(p0, p) for a, v, p in rec]
    good = [e for e in ev if e[3] > 0.60]
    print("    里程计角   VIO角   相对起始帧位移   相关系数")
    for e in ev:
        print("     %+8.2f  %+8.2f      %+8.2f px       %.3f%s"
              % (e[0], e[1], e[2], e[3], "" if e[3] > 0.60 else "   <- 丢弃"))
    if len(good) < 4:
        print("  !! 可用帧不够")
        return None
    # 位移过零点就是真正的 360 度。线性插值。
    xs = np.array([e[2] for e in good])          # px
    od = np.array([e[0] for e in good])
    vi = np.array([e[1] for e in good])
    order = np.argsort(xs)
    if xs[order][0] > 0 or xs[order][-1] < 0:
        print("  !! 位移没跨过零(范围 %+.1f ~ %+.1f px)，闭合点在记录区间之外" % (xs.min(), xs.max()))
        return None
    od0 = float(np.interp(0.0, xs[order], od[order]))
    vi0 = float(np.interp(0.0, xs[order], vi[order]))
    print("  -> 图像闭合(真 %+.0f 度)那一刻: 里程计报 %+.2f 度, VIO 报 %+.2f 度"
          % (360.0 * sign, od0, vi0))
    return od0, vi0, sign


def main():
    w = float(sys.argv[1]) if len(sys.argv) > 1 else 0.50
    rclpy.init()
    node = Spin()
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom_q and node.vio_q and node.img is not None and node.batt:
            break
    if node.img is None or not node.odom_q:
        sys.exit("数据不全")
    print("电池 %.2f V，速率 %.2f rad/s，一圈约 %.1f s\n" % (node.batt, w, 2 * np.pi / w))

    out = []
    for sign, tag in ((+1, "逆时针(左)"), (-1, "顺时针(右)")):
        print(">>> %s 转一圈" % tag)
        r = one_revolution(node, sign * abs(w))
        if r:
            out.append((tag, r))
        print()

    print("=" * 64)
    print("有效轮距标定")
    print("=" * 64)
    ks = []
    for tag, (od0, vi0, sign) in out:
        k = abs(od0) / 360.0
        ks.append(k)
        print("  %s: 里程计 %+.2f 度 / 真 360 度 = %.4f   (VIO %.4f)"
              % (tag, od0, k, abs(vi0) / 360.0))
    if ks:
        k = float(np.mean(ks))
        print("\n  里程计角度标度 = %.4f  (>1 高报，<1 低报)" % k)
        print("  有效轮距应为 0.3234 x %.4f = %.4f m" % (k, 0.3234 * k))
        print("  固件命令:  B 不用动;  轮距在固件常量里，要改得重烧 (WHEEL_BASE)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
