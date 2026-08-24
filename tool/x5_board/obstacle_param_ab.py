#!/usr/bin/env python3
"""累积多帧成占据栅格，然后在同一份栅格上对照不同的障碍参数。只读，不动车。

单帧不行：run_raycasting_loopy 在 kernel 内把每帧截断到 ±0.1，而判据是 > 0.1 严格大于，
所以一个体素要被 >= 2 帧打到才算占据。单帧工具永远报 0 个格子。
"""
import sys
import time
from dataclasses import replace

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import Image

sys.path.insert(0, "/userdata/x5/tinynav")
from tinynav.core.math_utils import quat_to_matrix              # noqa: E402
from tinynav.core.planning_kernels import run_raycasting_loopy  # noqa: E402
from tinynav.core.planning_node import build_obstacle_map       # noqa: E402
from tinynav.core.robot_config import DIFFCAR_CONFIG as R       # noqa: E402

GRID = (100, 100, 10)
RES = 0.1
STEP = 10
FX = FY = 309.49
CX, CY = 272.22, 318.80
NFRAMES = 12


class S(Node):
    def __init__(self):
        super().__init__("obst_ab")
        self.z = None
        self.T = None
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.d, 1)
        self.create_subscription(PoseStamped, "/camera/camera/vio_image", self.p, 10)

    def d(self, m):
        self.z = np.frombuffer(m.data, np.uint16).reshape(
            m.height, m.width).astype(np.float32) / 1000.0

    def p(self, m):
        T = np.eye(4)
        q = m.pose.orientation
        T[:3, :3] = quat_to_matrix(np.array([q.x, q.y, q.z, q.w]))
        T[:3, 3] = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
        self.T = T


rclpy.init()
n = S()
t0 = time.time()
while (n.z is None or n.T is None) and time.time() - t0 < 25:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.z is None or n.T is None:
    print("拿不全深度或位姿")
    raise SystemExit(1)

origin = n.T[:3, 3] - np.array(GRID) * RES / 2.0
robot_z = n.T[2, 3]
grid = np.zeros(GRID)
got = 0
seen = None
t0 = time.time()
while got < NFRAMES and time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.2)
    if n.z is None or n.T is None:
        continue
    key = float(np.nansum(n.z[::40, ::40]))
    if key == seen:
        continue
    seen = key
    grid += run_raycasting_loopy(n.z.copy(), n.T.copy(), GRID, FX, FY, CX, CY,
                                origin, STEP, RES)
    np.clip(grid, -0.2, 0.2, out=grid)
    got += 1

print("累积 %d 帧   robot_z=%+.3f   占据体素 %d" % (
    got, robot_z, (grid > R.obstacle.occ_threshold).sum()))
print()
print("同一份栅格，不同参数：")
print("%-28s %8s %8s %10s" % ("配置", "格子数", "相对", "ESDF最小(m)"))
base = None
for span in (0.2, 0.1):
    for dil in (0, 1, 2):
        cfg = replace(R.obstacle, min_wall_span_m=span, dilation_cells=dil)
        mask = build_obstacle_map(grid, origin, RES, robot_z, cfg)
        c = int(mask.sum())
        if base is None:
            base = c
        esdf = distance_transform_edt(~mask).astype(np.float32) * RES if c else None
        tag = "span=%.1f dil=%d" % (span, dil)
        if span == R.obstacle.min_wall_span_m and dil == R.obstacle.dilation_cells:
            tag += "  <= 现在"
        print("%-28s %8d %8s %10s" % (
            tag, c,
            "x%.2f" % (c / base) if base else "-",
            "%.2f" % esdf.min() if esdf is not None else "-"))
n.destroy_node()
rclpy.shutdown()
