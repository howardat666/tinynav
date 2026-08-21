#!/usr/bin/env python3
"""按里程计开一段固定距离，全程用深度图实时刹车。在板上跑。

  python3 diffcar_drive_measured.py [米] [速度]

用途:轮周长的绝对定标 —— 只有卷尺量出来的真实位移能定死它,VIO 有未知的几个百分点偏置。
在车体后缘贴一条胶带,跑完量"胶带 -> 车体后缘",拿实测米数和这里报的里程计读数比。

⚠️ 地面必须用**真实拟合的平面**来剔除,不能假设相机水平。实测俯仰 1~3 度,在 3 m 处就是
0.10 m 的高度误差,足以把远处地面误判成障碍物 —— 试过一次,把 3.5 m 的余量报成 1.79 m。
"""
import re
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, "/root/car")
import carlib  # noqa: E402

FX = FY = 309.49
CX, CY = 272.22, 318.80
STOP_M = 0.60          # 车体前缘在相机前 0.05 m,所以这是车头离障碍还有 0.55 m
HALF_W = 0.20          # 车半宽 0.175 + 余量
CAM_H = 0.18
_POSE_RE = re.compile(r"x=(-?[\d.]+) y=(-?[\d.]+) theta=(-?[\d.]+)")


class Runner(Node):
    def __init__(self):
        super().__init__("drive_measured")
        self.clear = None
        self.vio = []
        self.frames = 0
        self._plane = None
        self._uu = self._vv = None
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self._depth, 1)
        self.create_subscription(
            PoseStamped, "/camera/camera/vio_100hz",
            lambda m: self.vio.append(
                (time.time(), np.array([m.pose.position.x, m.pose.position.y,
                                        m.pose.position.z]))), 50)

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

    def _depth(self, msg):
        Z = np.frombuffer(msg.data, np.uint16).reshape(
            msg.height, msg.width).astype(np.float32) / 1000.0
        Z[(Z <= 0.15) | (Z > 10.0)] = np.nan
        if self._uu is None:
            self._uu, self._vv = np.meshgrid(np.arange(msg.width), np.arange(msg.height))
        X = (self._uu - CX) * Z / FX
        Y = (self._vv - CY) * Z / FY
        if self._plane is None:
            self._plane = self._fit_floor(X, Y, Z)
            if self._plane is None:
                self.frames += 1
                return
            n, d = self._plane
            print("  地面平面: 垂距 %.4f m (尺量 %.2f), 俯仰 %+.1f 度"
                  % (d, CAM_H, np.degrees(np.arctan2(n[2], n[1]))), flush=True)
        n, d = self._plane
        h = d - (X * n[0] + Y * n[1] + Z * n[2])
        ob = np.isfinite(Z) & (h > 0.05) & (h < 0.35) & (np.abs(X) < HALF_W) & (Z > 0.25)
        self.clear = float(np.percentile(Z[ob], 1)) if ob.sum() > 40 else 99.0
        self.frames += 1


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.20
    rclpy.init()
    node = Runner()
    car = carlib.Car()
    time.sleep(0.8)
    car.send("s")
    time.sleep(0.4)
    car.drain()

    t0 = time.time()
    while node.frames < 4 and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.clear is None:
        sys.exit("拿不到深度")
    print("起始前向余量 %.2f m (刹车线 %.2f, 需要 %.2f)" % (node.clear, STOP_M, target + STOP_M))
    if node.clear < target + STOP_M:
        car.kill()
        sys.exit("余量不够,先清场或减小距离")

    car.send("q")
    car.send("z")
    time.sleep(0.5)
    car.drain()
    i0 = len(node.vio)
    t0 = last_cmd = last_pose = time.time()
    x = 0.0
    brake = None
    while True:
        rclpy.spin_once(node, timeout_sec=0.002)
        now = time.time()
        if node.clear is not None and node.clear < STOP_M:
            brake = "深度刹车: 前方只剩 %.2f m" % node.clear
            break
        if now - t0 > target / speed + 10.0:
            brake = "超时"
            break
        # 🔴 不能用 car.pose():它内部 ask() 会 sleep 0.5 s，那 0.5 s 里既不 spin_once
        # (深度刹车全瞎，0.2 m/s 下等于盲走 10 cm)也不发指令。改成自己发 p、从 drain 里挑。
        if now - last_pose > 0.10:
            last_pose = now
            car.send("p")
        for line in car.drain():
            m = _POSE_RE.search(line)
            if m:
                x = float(m.group(1))
        if x >= target - 0.01:
            break
        if now - last_cmd > 0.05:
            car.send("u %.3f 0.000" % speed)
            last_cmd = now
    car.send("u 0 0")
    t1 = time.time()
    while time.time() - t1 < 1.5:
        rclpy.spin_once(node, timeout_sec=0.01)
    car.send("s")
    time.sleep(0.5)

    pose, counts = car.pose(), car.counts()
    P = np.array([v[1] for v in node.vio[i0:]])
    T = np.array([v[0] for v in node.vio[i0:]])
    chord = float(np.linalg.norm(P[-1] - P[0])) if len(P) > 2 else 0.0
    # VIO 静止后会在 TRACKING_STATIC 里滞留数秒、吞掉开头的运动，所以另算一个"起步 6 s 之后"
    # 的窗口，那一段才是它真的在跟踪的。
    late = T - T[0] > 6.0
    sub_v = float(np.linalg.norm(P[-1] - P[late][0])) if late.sum() > 2 else 0.0
    print()
    print("固件里程计 x = %.4f m   (计数 左%d 右%d, theta=%+.2f 度)"
          % (pose[0], counts[0], counts[1], pose[2]))
    print("VIO 全程弦长  = %.4f m   (起步 6 s 之后那段 = %.4f m)" % (chord, sub_v))
    print("剩余前向余量  = %.2f m" % (node.clear if node.clear is not None else -1))
    if brake:
        print("!! " + brake)
    print()
    print("=== 拿卷尺量「胶带 -> 车体后缘」,把厘米数告诉我 ===")
    print("    固件认为它走了 %.4f m" % pose[0])
    car.kill()


if __name__ == "__main__":
    main()
