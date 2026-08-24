#!/usr/bin/env python3
"""原地自转，用三个独立参照定死转向符号。在板上跑，全程走 ROS 话题不碰串口。

  python3 diffcar_spin_sign_check.py [rad/s] [秒]

要回答的问题：固件报 theta 为负(顺时针)，人眼看到的是左偏 —— 到底哪个对？
符号错的后果是「固件的左右轮槽位可能还是反的」，那样路径跟踪会整体镜像。

图像位移是方向上的铁证：相机向左转(逆时针)，画面内容必然向右移(+u)。这一条和
轮距、周长、VIO 尺度全都无关，只用到 fx。
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
BAND = (120, 420)      # 取中间这几行做水平梯度：躲开地面(v>420)和天花板
MAXS = 45              # 相邻帧最大搜索位移，px。0.5rad/s @10Hz 约 15px
SPIN_CLEAR_M = 0.30    # 原地自转的后角扫掠半径实测 0.251m，留余量
HALF_W = 0.20
CAM_H = 0.18


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def rotvec(q0, q1):
    """从 q0 转到 q1 的旋转向量(轴 x 角度, rad)。"""
    rel = qmul(q1, (-q0[0], -q0[1], -q0[2], q0[3]))
    v = np.array(rel[:3], float)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3)
    return v / n * 2.0 * np.arctan2(n, rel[3])


def profile(img):
    """一帧压成 1D 水平轮廓：竖直边缘的强度沿列求和。"""
    band = img[BAND[0]:BAND[1], :].astype(np.float32)
    return np.abs(np.diff(band, axis=1)).sum(axis=0)


def best_shift(a, b):
    """返回 s 使 b[u] ~= a[u-s]，即内容移动了 +s(向右)。附带峰值相关系数。"""
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
    # 抛物线细化到亚像素
    if -MAXS < bs < MAXS:
        y0, y1, y2 = cs[bs - 1], cs[bs], cs[bs + 1]
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-12:
            bs = bs + 0.5 * (y0 - y2) / den
    return float(bs), best


class Spin(Node):
    def __init__(self):
        super().__init__("spin_sign_check")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_q = None
        self.vio_q = None
        self.batt = None
        self.img = None
        self.img_n = 0
        self.clear = None
        self._plane = None
        self._uu = self._vv = None
        self.create_subscription(Odometry, "/wheel/odometry", self._odom, 10)
        self.create_subscription(PoseStamped, "/camera/camera/vio_100hz", self._vio, 20)
        self.create_subscription(Float32, "/battery_voltage", lambda m: setattr(self, "batt", m.data), 5)
        self.create_subscription(Image, "/camera/camera/infra1/image_rect_raw", self._infra, 1)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self._depth, 1)

    def _odom(self, m):
        o = m.pose.pose.orientation
        self.odom_q = (o.x, o.y, o.z, o.w)

    def _vio(self, m):
        o = m.pose.orientation
        self.vio_q = (o.x, o.y, o.z, o.w)

    def _infra(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        self.img = a
        self.img_n += 1

    def _fit_floor(self, X, Y, Z):
        sel = np.isfinite(Z) & (self._vv > 420) & (self._vv < 580) & (np.abs(self._uu - CX) < 120)
        if sel.sum() < 3000:
            return None
        P = np.stack([X[sel], Y[sel], Z[sel]], 1)
        for _ in range(6):
            c = P.mean(0)
            n = np.linalg.svd(P - c, full_matrices=False)[2][-1]
            keep = np.abs((P - c) @ n) < max(0.006, 2.0 * ((P - c) @ n).std())
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
            print("  地面平面: 垂距 %.4f m (尺量 %.2f), 俯仰 %+.1f 度"
                  % (d, CAM_H, np.degrees(np.arctan2(n[2], n[1]))), flush=True)
        n, d = self._plane
        h = d - (X * n[0] + Y * n[1] + Z * n[2])
        ob = np.isfinite(Z) & (h > 0.05) & (h < 0.35) & (np.abs(X) < HALF_W) & (Z > 0.20)
        self.clear = float(np.percentile(Z[ob], 1)) if ob.sum() > 40 else 99.0


def spin_once(node, w, secs, tag):
    """转一段，回报三个参照。返回 (轮子度, VIO旋转向量, 图像度, 相关系数中位数)。"""
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.02)
    q_od0, q_vio0 = node.odom_q, node.vio_q
    prof_prev = profile(node.img) if node.img is not None else None
    px = 0.0
    corrs = []
    t0 = last_cmd = time.time()
    t_prof = 0.0
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.005)
        now = time.time()
        if now - last_cmd > 0.05:
            m = Twist()
            m.angular.z = w
            node.cmd.publish(m)
            last_cmd = now
        if node.img is not None and now - t_prof > 0.10:
            t_prof = now
            p = profile(node.img)
            if prof_prev is not None:
                s, c = best_shift(prof_prev, p)
                if c > 0.55:                 # 纹理太差的帧直接丢，别把噪声累进去
                    px += s
                    corrs.append(c)
            prof_prev = p
    m = Twist()
    node.cmd.publish(m)
    t1 = time.time()
    while time.time() - t1 < 1.5:
        rclpy.spin_once(node, timeout_sec=0.01)
        node.cmd.publish(m)

    d_od = np.degrees(2.0 * np.arctan2(node.odom_q[2], node.odom_q[3])
                      - 2.0 * np.arctan2(q_od0[2], q_od0[3]))
    rv = np.degrees(rotvec(q_vio0, node.vio_q)) if (q_vio0 and node.vio_q) else np.zeros(3)
    img_deg = np.degrees(px / FX)
    med = float(np.median(corrs)) if corrs else 0.0
    print("\n--- %s (指令 angular.z = %+.2f rad/s x %.1f s = %+.1f 度) ---" % (tag, w, secs, np.degrees(w * secs)))
    print("  轮子里程计 dyaw   = %+7.2f 度" % d_od)
    print("  VIO 旋转向量      = [%+7.2f %+7.2f %+7.2f] 度 (相机光学系 x右 y下 z前)" % tuple(rv))
    print("  图像累计水平位移  = %+7.1f px  ->  %+7.2f 度   (相关系数中位数 %.3f, %d 帧)"
          % (px, img_deg, med, len(corrs)))
    print("  电池 %.2f V   前方余量 %.2f m" % (node.batt or 0.0, node.clear if node.clear else -1))
    return d_od, rv, img_deg, med


def main():
    w = float(sys.argv[1]) if len(sys.argv) > 1 else 0.40
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5
    rclpy.init()
    node = Spin()
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom_q and node.vio_q and node.img is not None and node.clear is not None:
            break
    miss = [n for n, v in (("wheel/odometry", node.odom_q), ("vio_100hz", node.vio_q),
                           ("infra1", node.img), ("depth", node.clear)) if v is None]
    if miss:
        sys.exit("拿不到: %s" % ", ".join(miss))
    print("三路数据都到齐。电池 %.2f V，前方余量 %.2f m，infra1 %dx%d"
          % (node.batt or 0.0, node.clear, node.img.shape[1], node.img.shape[0]))
    if node.clear < SPIN_CLEAR_M:
        sys.exit("前方只有 %.2f m，小于自转需要的 %.2f m —— 先挪车" % (node.clear, SPIN_CLEAR_M))

    print("\n>>> 先做一次很轻的探动，确认地毯上转得动、不触发堵转")
    spin_once(node, 0.30, 1.0, "探动 左(逆时针)")
    spin_once(node, -0.30, 1.0, "探动 右(顺时针) 回位")

    print("\n>>> 正式测量")
    a = spin_once(node, +abs(w), secs, "左转(逆时针, angular.z 正)")
    b = spin_once(node, -abs(w), secs, "右转(顺时针, angular.z 负) 回位")

    print("\n" + "=" * 62)
    print("判读")
    print("=" * 62)
    for tag, (d_od, rv, img_deg, med) in (("左转指令", a), ("右转指令", b)):
        phys = "左(逆时针)" if img_deg > 0 else "右(顺时针)"
        print("  %s: 图像说车实际转了 %s %.1f 度; 轮子里程计说 %+.1f 度; VIO y轴 %+.1f 度"
              % (tag, phys, abs(img_deg), d_od, rv[1]))
    if a[2] > 0 and b[2] < 0:
        print("  -> 指令方向和实际方向一致: 固件左右轮槽位正确")
    elif a[2] < 0 and b[2] > 0:
        print("  -> !! 指令方向和实际相反: 固件左右轮槽位是反的，转向和路径跟踪会镜像")
    else:
        print("  -> 两次图像位移同号，说明没转起来或纹理不够，别下结论")
    if a[2] and a[0]:
        print("  轮子/图像 幅度比 左 %.4f  右 %.4f  (>1 = 里程计高报转角 = 有效轮距该调大)"
              % (a[0] / a[2], b[0] / b[2]))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
