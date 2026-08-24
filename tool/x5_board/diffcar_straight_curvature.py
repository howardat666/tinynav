#!/usr/bin/env python3
"""直线前进/后退，比较 VIO 和轮式里程计各自算出的每米曲率。走 ROS 话题，不碰串口。

  python3 diffcar_straight_curvature.py [--cycles 1] [--max-leg 1.5] [--speed 0.20]

一个测试同时回答三个问题（kappa = 每米航向变化，带符号，除以带符号的行程）：

  前进的 kappa 和后退的 kappa 同号  -> 左右轮有效半径不一样(反向时 dtheta 和 dx 一起变号)
  前进的 kappa 和后退的 kappa 变号  -> 外部侧向力(路面横坡 / 脚轮拖拽)
  VIO 的 kappa 和 里程计的 kappa 反号 -> 确认是半径差：闭环抹平编码器速度，半径小的那侧
                                        计数反而多，固件用同一个周长换算就把符号搞反了

🔑 **一对"前进+后退"做完之后离原点的残差，就是脚轮换向那一甩的量。** 纯半径差给出的是一条
固定曲率的圆弧，前进走一段、后退走同一段是同一条弧，会原路退回、不累积；能累积的只有脚轮
换向的滞回。所以残差既是安全阀(超过 MAX_RESID 就停，免得逐趟偏出过道)，也是一个独立测量。

单程宁长勿多：信号按行程线性增长(约 0.43 度/米)，而每次换向都要付一次脚轮的甩头。
1 趟 1.3 m 的信噪比比 3 趟 0.6 m 好，而且只换向一次。要更多样本就让人把车摆回去再跑一次。

kappa 用回归拿不用首尾相减，头 SKIP_M 米扔掉(脚轮要甩、闭环要起步)，而且**被扔掉的那一段
单独报出来** —— 那就是甩头的瞬态本身。回归顺带给出 R2:半径差是匀速累积(R2 高)，甩头是瞬态(R2 低)。
"""
import argparse
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
SKIP_M = 0.15          # 每趟头这么多米不参与回归，但会单独报出来
BRAKE_M = 0.40         # 深度刹车线（车体前缘在相机前 0.05 m）
HALF_W = 0.20
MAX_RESID = 0.12       # 一对往返后离原点超过这个就停：再跑下去会偏出过道
V_ABORT = 9.9


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


def wrap(a):
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


class Drive(Node):
    def __init__(self):
        super().__init__("straight_curvature")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = None            # (x, y, theta)
        self.vio = None             # (xyz, quat)
        self.batt = None
        self.clear = None
        self._plane = None
        self._uu = self._vv = None
        self.create_subscription(Odometry, "/wheel/odometry", self._on_odom, 10)
        self.create_subscription(PoseStamped, "/camera/camera/vio_100hz", self._on_vio, 20)
        self.create_subscription(Float32, "/battery_voltage",
                                 lambda m: setattr(self, "batt", float(m.data)), 5)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self._depth, 1)

    def _on_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self.odom = (p.x, p.y, 2.0 * np.arctan2(o.z, o.w))

    def _on_vio(self, m):
        p, o = m.pose.position, m.pose.orientation
        self.vio = (np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w))

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


def leg(node, v, dist):
    """跑一趟。返回 dict:两侧 kappa/R2、实际行程、被扔掉那段的甩头瞬态、刹车原因。"""
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.02)
    x0, y0, t0 = node.odom
    q_v0 = node.vio[1]
    p_prev = np.array([x0, y0])
    s = 0.0
    rec = []                    # (行程, 轮侧横偏, 轮侧转角, VIO转角)
    brake = None
    tstart = last_cmd = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.004)
        now = time.time()
        p = np.array(node.odom[:2])
        s += float(np.linalg.norm(p - p_prev))
        p_prev = p
        dx, dy = node.odom[0] - x0, node.odom[1] - y0
        rec.append((s,
                    -dx * np.sin(t0) + dy * np.cos(t0),
                    wrap(node.odom[2] - t0),
                    rotvec_z(q_v0, node.vio[1])))
        if s >= dist:
            break
        if v > 0 and node.clear is not None and node.clear < BRAKE_M:
            brake = "深度刹车，前方只剩 %.2f m" % node.clear
            break
        if now - tstart > dist / abs(v) + 8.0:
            brake = "超时"
            break
        if node.batt and node.batt < V_ABORT:
            brake = "电压 %.2f V" % node.batt
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

    sgn = 1.0 if v > 0 else -1.0
    use = [r for r in rec if r[0] > SKIP_M]
    if len(use) < 20:
        return None
    x = np.array([r[0] for r in use])

    def fit(col):
        y = np.array([r[col] for r in use])
        a, b = np.polyfit(x, y, 1)
        pred = a * x + b
        ss = 1 - ((y - pred) ** 2).sum() / max(1e-12, ((y - y.mean()) ** 2).sum())
        return a * sgn, ss

    ko, r2o = fit(2)
    kv, r2v = fit(3)
    # 被扔掉那一段就是换向后的甩头瞬态，单独报
    head = [r for r in rec if r[0] <= SKIP_M]
    trans = (head[-1][1], head[-1][2], head[-1][3]) if head else (0.0, 0.0, 0.0)
    return dict(ko=ko, kv=kv, s=s * sgn, r2o=r2o, r2v=r2v, brake=brake, trans=trans)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=1, help="往返几对。默认 1：宁长勿多")
    ap.add_argument("--max-leg", type=float, default=1.5)
    ap.add_argument("--speed", type=float, default=0.20)
    args = ap.parse_args()

    rclpy.init()
    node = Drive()
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom and node.vio is not None and node.clear is not None and node.batt:
            break
    if node.odom is None or node.vio is None or node.clear is None:
        raise SystemExit("数据不全：/wheel/odometry(app 起了吗) /vio_100hz /depth")

    dist = min(args.max_leg, node.clear - BRAKE_M)
    if dist < 0.35:
        raise SystemExit("前方只有 %.2f m，摆不开(刹车线 %.2f)" % (node.clear, BRAKE_M))
    print("电池 %.2f V，前方余量 %.2f m -> 单程 %.2f m，%d 对往返，%.2f m/s"
          % (node.batt, node.clear, dist, args.cycles, args.speed))
    print("预计信号 约 %.2f 度/趟（按 0.43 度/米）\n" % (0.43 * dist))
    origin = node.vio[0].copy()

    rows = []
    for i in range(args.cycles):
        for sgn, tag in ((+1, "前进"), (-1, "后退")):
            r = leg(node, sgn * args.speed, dist)
            if r is None:
                print("  第%d对 %s: 样本不足" % (i + 1, tag))
                continue
            rows.append((sgn, r))
            print("  第%d对 %s  里程计 kappa %+8.4f 度/m (R2 %.3f)   VIO kappa %+8.4f 度/m (R2 %.3f)"
                  % (i + 1, tag, np.degrees(r["ko"]), r["r2o"], np.degrees(r["kv"]), r["r2v"]))
            print("           行程 %+6.3f m   起步 %.2f m 内的甩头: 横偏 %+.4f m 转角 %+.2f 度%s"
                  % (r["s"], SKIP_M, r["trans"][0], np.degrees(r["trans"][1]),
                     "   !! " + r["brake"] if r["brake"] else ""))
        d = node.vio[0] - origin
        resid = float(np.hypot(d[0], d[1]))
        print("  -> 这一对做完，离原点 %.4f m（纯半径差应当原路退回，所以这就是脚轮滞回的量）"
              % resid)
        if resid > MAX_RESID:
            print("  !! 残差超过 %.2f m，停止 —— 再跑下去会偏出过道" % MAX_RESID)
            break

    print("\n" + "=" * 68)
    print("判读")
    print("=" * 68)
    kf = [r for sgn, r in rows if sgn > 0]
    kb = [r for sgn, r in rows if sgn < 0]
    for tag, g in (("前进", kf), ("后退", kb)):
        if g:
            print("  %s (n=%d): 里程计 kappa %+.4f 度/m   VIO kappa %+.4f 度/m"
                  % (tag, len(g), np.degrees(np.mean([r["ko"] for r in g])),
                     np.degrees(np.mean([r["kv"] for r in g]))))
    if kf and kb:
        mf = np.mean([r["kv"] for r in kf])
        mb = np.mean([r["kv"] for r in kb])
        print("\n  1) VIO 的 kappa 前进 %+.4f vs 后退 %+.4f 度/m -> %s"
              % (np.degrees(mf), np.degrees(mb),
                 "同号，左右轮有效半径不一样" if mf * mb > 0
                 else "变号，是外部侧向力(横坡/脚轮拖拽)"))
    allr = [r for _, r in rows]
    if allr:
        allo = float(np.mean([r["ko"] for r in allr]))
        allv = float(np.mean([r["kv"] for r in allr]))
        print("  2) 里程计 kappa %+.4f  vs  VIO kappa %+.4f 度/m -> %s"
              % (np.degrees(allo), np.degrees(allv),
                 "反号，确认是半径差(里程计把航向符号搞反了)" if allo * allv < 0
                 else "同号，里程计航向符号是对的"))
        if allo * allv < 0 and abs(allv) > 1e-5:
            base = 0.3234
            mism = float((abs(allv) + abs(allo)) * base)
            side = "左" if allv > 0 else "右"
            print("\n  左右轮有效周长失配 %.3f%%（70mm 轮上直径差 %.2f mm），%s轮偏小"
                  % (100 * mism, 1000 * mism * 70.0, side))
            print("  修法(不用重烧固件): 把%s轮 ppr 调大 %.3f%%，再敲 w 存起来"
                  % (side, 100 * mism))
        tr = [abs(r["trans"][0]) for r in allr]
        print("  3) 每趟起步 %.2f m 内的甩头横偏 中位 %.4f m 最大 %.4f m -> 每次换向的代价"
              % (SKIP_M, float(np.median(tr)), max(tr)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
