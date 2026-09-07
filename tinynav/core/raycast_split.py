"""把"标命中"和"刻空闲"两件事的取样密度分开的射线投射。

单独一个文件而不是塞进 planning_kernels.py：numba 的磁盘缓存以文件的 mtime/size 为
键，动那个文件会让板上下次启动多付一次 70~90 s 的重编译，而这个 kernel 现在还没被
planning_node 用上。

为什么要拆：原版一条射线要写约 30~40 个体素，其中只有 1 个是"命中"，其余全是"刻空"
—— 97% 的算力在刻空上，而两件事需要的取样密度完全不同。命中稀了会漏矮东西和远处
地面；刻空则相邻射线刻的几乎是同一批体素，纯重复劳动。原版用同一个 step 逼着两者
同进同退。

hit_step == carve_step 时与 run_raycasting_loopy 数学等价（实测最大差 3e-14，纯浮点
求和顺序；判据是 > 0.1，差 13 个数量级）。
"""

import numpy as np
from numba import njit


@njit(cache=True)
def run_raycasting_split(depth_image, T_cam_to_world, grid_shape, fx, fy, cx, cy,
                         origin, hit_step, carve_step, resolution):
    occupancy_grid = np.zeros(grid_shape)
    H, W = depth_image.shape
    nx, ny, nz = grid_shape
    ox, oy, oz = origin
    sx = int(np.floor((T_cam_to_world[0, 3] - ox) / resolution))
    sy = int(np.floor((T_cam_to_world[1, 3] - oy) / resolution))
    sz = int(np.floor((T_cam_to_world[2, 3] - oz) / resolution))

    # 第 1 遍：刻空闲，用【疏】取样。语义和原版逐字一致 —— 整条线含终点体素都 -0.05。
    for v in range(0, H, carve_step):
        for u in range(0, W, carve_step):
            d = depth_image[v, u]
            if (not np.isfinite(d)) or d <= 0:
                continue
            px = (u - cx) * d / fx
            py = (v - cy) * d / fy
            ex = int(np.floor((T_cam_to_world[0, 0] * px + T_cam_to_world[0, 1] * py
                               + T_cam_to_world[0, 2] * d + T_cam_to_world[0, 3] - ox) / resolution))
            ey = int(np.floor((T_cam_to_world[1, 0] * px + T_cam_to_world[1, 1] * py
                               + T_cam_to_world[1, 2] * d + T_cam_to_world[1, 3] - oy) / resolution))
            ez = int(np.floor((T_cam_to_world[2, 0] * px + T_cam_to_world[2, 1] * py
                               + T_cam_to_world[2, 2] * d + T_cam_to_world[2, 3] - oz) / resolution))
            dx = ex - sx
            dy = ey - sy
            dz = ez - sz
            steps = max(abs(dx), max(abs(dy), abs(dz)))
            if steps == 0:
                continue
            for i in range(steps + 1):
                t = i / steps
                a = int(round(sx + t * dx))
                b = int(round(sy + t * dy))
                c = int(round(sz + t * dz))
                if 0 <= a < nx and 0 <= b < ny and 0 <= c < nz:
                    occupancy_grid[a, b, c] -= 0.05

    # 第 2 遍：标命中，用【密】取样。每条射线只写 1 个体素，所以密取样几乎不要钱。
    for v in range(0, H, hit_step):
        for u in range(0, W, hit_step):
            d = depth_image[v, u]
            if (not np.isfinite(d)) or d <= 0:
                continue
            px = (u - cx) * d / fx
            py = (v - cy) * d / fy
            ex = int(np.floor((T_cam_to_world[0, 0] * px + T_cam_to_world[0, 1] * py
                               + T_cam_to_world[0, 2] * d + T_cam_to_world[0, 3] - ox) / resolution))
            ey = int(np.floor((T_cam_to_world[1, 0] * px + T_cam_to_world[1, 1] * py
                               + T_cam_to_world[1, 2] * d + T_cam_to_world[1, 3] - oy) / resolution))
            ez = int(np.floor((T_cam_to_world[2, 0] * px + T_cam_to_world[2, 1] * py
                               + T_cam_to_world[2, 2] * d + T_cam_to_world[2, 3] - oz) / resolution))
            if 0 <= ex < nx and 0 <= ey < ny and 0 <= ez < nz:
                occupancy_grid[ex, ey, ez] += 0.2

    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                if occupancy_grid[i, j, k] < -0.1:
                    occupancy_grid[i, j, k] = -0.1
                elif occupancy_grid[i, j, k] > 0.1:
                    occupancy_grid[i, j, k] = 0.1
    return occupancy_grid
