#!/usr/bin/env python3
"""车体正前方走廊里，按离地高度分层数点。只读。"""
import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image

FX = FY = 309.49
CX, CY = 272.22, 318.80
CAM_H = 0.18
HALF_W = 0.20


class P(Node):
    def __init__(self):
        super().__init__("depth_layers")
        self.z = None
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.cb, 1)

    def cb(self, m):
        self.z = np.frombuffer(m.data, np.uint16).reshape(
            m.height, m.width).astype(np.float32) / 1000.0


rclpy.init()
n = P()
t0 = time.time()
while n.z is None and time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.z is None:
    print("拿不到深度")
    raise SystemExit(1)

Z = n.z.copy()
Z[(Z <= 0.15) | (Z > 10)] = np.nan
uu, vv = np.meshgrid(np.arange(Z.shape[1]), np.arange(Z.shape[0]))
X = (uu - CX) * Z / FX
Y = (vv - CY) * Z / FY
H = CAM_H - Y                      # 离地高度：Y 向下为正

print("有效像素 %d/%d   深度 %.2f~%.2f m" % (
    np.isfinite(Z).sum(), Z.size, np.nanmin(Z), np.nanmax(Z)))
band = (np.abs(X) < HALF_W) & np.isfinite(Z) & (Z > 0.25)
print("正前方 %.2f m 宽走廊内有效点 %d" % (2 * HALF_W, band.sum()))
print()
print("按离地高度分层（相机在 %.2f m）:" % CAM_H)
for lo, hi in [(-0.15, 0.0), (0.0, 0.05), (0.05, 0.10), (0.10, 0.15),
               (0.15, 0.20), (0.20, 0.30), (0.30, 0.50)]:
    sel = band & (H >= lo) & (H < hi)
    nearest = np.nanmin(Z[sel]) if sel.sum() else float("nan")
    print("  离地 [%+.2f,%+.2f] m: %7d 点   最近 %.2f m" % (lo, hi, sel.sum(), nearest))
print()
# 障碍物判据要求竖直跨度 >= 0.2 m，10 cm 体素 -> 至少 3 层
occ_layers = 0
for lo in np.arange(-0.2, 0.5, 0.1):
    sel = band & (H >= lo) & (H < lo + 0.1)
    if sel.sum() > 30:
        occ_layers += 1
print("占据的 0.1 m 层数 = %d   (z_span 判据要 >= 3 层才算障碍)" % occ_layers)
n.destroy_node()
rclpy.shutdown()
