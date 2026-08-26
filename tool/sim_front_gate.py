"""前方视距门限的闭环仿真 —— 用真的感知内核，不需要板子。

合成深度图 -> run_raycasting_loopy -> build_obstacle_map -> _clearance_along 的口径 ->
按分档门限选速度 -> 前进，看车体面最近逼到多少。相机动、物体在世界里静止：反过来做的话
每帧物体换一批体素，而内核把 new_occ 夹在 +-0.1、阈值是 > 0.1，需要连续两帧才算障碍，
于是"墙也测不到"这种假结论就出来了（2026-08-26 踩过）。

跑法（本机没有 numba，用镜像）：
  docker run --rm --entrypoint bash -v $PWD:/proj tinynav-x5:sp \
    -lc 'cd /proj && python3 tool/sim_front_gate.py'
"""
import os, sys, math, types, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# 镜像里没有 ROS 消息包，而我们只用到纯数值内核 —— 打桩让 import 链走通
for name in ("geometry_msgs", "geometry_msgs.msg", "std_msgs", "std_msgs.msg",
             "nav_msgs", "nav_msgs.msg", "sensor_msgs", "sensor_msgs.msg"):
    if name not in sys.modules:
        m = types.ModuleType(name)
        m.__path__ = []
        sys.modules[name] = m
for cls in ("PoseStamped", "TransformStamped", "Pose", "Point", "Quaternion",
            "Twist", "Header", "Path", "Odometry", "OccupancyGrid", "Image",
            "CameraInfo", "PointCloud", "PointCloud2", "PointField"):
    for mod in ("geometry_msgs.msg", "std_msgs.msg", "nav_msgs.msg", "sensor_msgs.msg"):
        setattr(sys.modules[mod], cls, type(cls, (), {}))
from tinynav.core.planning_kernels import run_raycasting_loopy

# build_obstacle_map 在 planning_node 里，那个模块要 rclpy；只把函数抠出来跑
src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tinynav/core/planning_node.py')).read()
a = src.index("def build_obstacle_map"); b = src.index("def generate_predefined_trajectory_vocabularies")
from scipy.ndimage import binary_dilation
from dataclasses import dataclass
@dataclass
class OC:
    robot_z_bottom: float = -0.3
    robot_z_top: float = 0.4
    occ_threshold: float = 0.1
    min_wall_span_m: float = 0.2
    dilation_cells: int = 1
ns = {'np': np, 'binary_dilation': binary_dilation, 'ObstacleConfig': OC}
exec(src[a:b], ns)
build_obstacle_map = ns['build_obstacle_map']

# --- 板上的真实内参与几何 ---
W, H = 544, 640                      # depth 是 (H, W)
FX, FY, CX, CY = 330.0, 330.2, 271.1, 319.8
CAM_H = 0.18                         # 相机光心离地
RES, GRID = 0.1, (100, 100, 9)
OFFSET = np.array([0.0, 0.0, 0.15])
# 相机光学系: x 右 y 下 z 前；世界: x 前 y 左 z 上
T = np.array([[0., 0., 1., 0.],
              [-1., 0., 0., 0.],
              [0., -1., 0., CAM_H],
              [0., 0., 0., 1.]])

u = (np.arange(W) - CX) / FX          # 每列的 x/z
v = (np.arange(H) - CY) / FY          # 每行的 y/z

def render(obj_rel_x, obj_w, obj_h, cam_z=CAM_H, floor=True, max_range=6.0):
    """深度图 (H,W)。obj_rel_x = 物体前表面相对相机的距离（光学 z）。"""
    dirs_x = np.broadcast_to(u[None, :], (H, W))
    dirs_y = np.broadcast_to(v[:, None], (H, W))
    d = np.full((H, W), np.inf)
    z = obj_rel_x
    wy = -dirs_x * z
    wz = cam_z - dirs_y * z
    hit = (np.abs(wy) <= obj_w / 2) & (wz >= 0.0) & (wz <= obj_h)
    d = np.where(hit, z, d)
    if floor:
        with np.errstate(divide="ignore", invalid="ignore"):
            zf = np.where(dirs_y > 1e-6, cam_z / dirs_y, np.inf)
        d = np.minimum(d, np.where(np.isfinite(zf) & (zf <= max_range), zf, np.inf))
    return np.where(np.isfinite(d), d, 0.0).astype(np.float64)


def approach(obj_w, obj_h, step, span_m, dil, start=3.0, stop=0.15, dz=0.07,
             floor=True, occ_th=0.1, verbose=False):
    """相机以 dz/帧 前进，物体在世界里静止。返回它第一次进入障碍图时的距离。

    关键：物体必须在世界坐标里固定 —— 每帧 new_occ 被内核夹在 +-0.1，而阈值是 > 0.1，
    所以一个格子要连续两帧拿到正证据才算障碍。让物体动会让它永远攒不起来。"""
    cfg = OC(min_wall_span_m=span_m, dilation_cells=dil, occ_threshold=occ_th)
    OBJ_X = 4.0                                  # 物体在世界里的固定位置
    cam_x0 = OBJ_X - start
    # 栅格固定（等价于每帧 roll 到相机，只要不出界）
    origin = np.array([cam_x0 + 5.0 - 5.0, 0.0, CAM_H]) - np.array(GRID) * RES / 2 + OFFSET
    origin[0] = cam_x0 - GRID[0] * RES / 2 + 2.0   # 往前多留 2 m
    occ = np.zeros(GRID)
    dist = start
    while dist >= stop:
        cam_x = OBJ_X - dist
        Tc = T.copy(); Tc[0, 3] = cam_x
        depth = render(dist, obj_w, obj_h, floor=floor)
        occ *= 0.99
        occ += run_raycasting_loopy(depth, Tc, GRID, FX, FY, CX, CY, origin, step, RES)
        np.clip(occ, -0.2, 0.2, out=occ)
        mask = build_obstacle_map(occ, origin, RES, robot_z=CAM_H, config=cfg)
        ii, jj = np.nonzero(mask)
        if len(ii):
            wx = origin[0] + (ii + 0.5) * RES
            wy = origin[1] + (jj + 0.5) * RES
            sel = (np.abs(wy) <= 0.225) & (wx > cam_x + 0.10)
            if np.any(sel):
                return dist, float(np.min(wx[sel]) - cam_x)
        if verbose:
            print(f"    dist={dist:.2f} cells={int(mask.sum())}")
        dist -= dz
    return None


HW, FL = 0.175, 0.10          # probe_geometry()
def clearance_along(center_x, mask, origin, res, max_dist):
    """按 _clearance_along 的口径：走廊 |lateral|<=hw+0.5res, along>=fl，从车体面算起。"""
    ii, jj = np.nonzero(mask)
    if not len(ii): return max_dist + 1.0
    dx = origin[0] + (ii + 0.5) * res - center_x
    dy = origin[1] + (jj + 0.5) * res - 0.0
    inc = (np.abs(dy) <= HW + 0.5 * res) & (dx >= FL)
    if not np.any(inc): return max_dist + 1.0
    d = float(np.min(dx[inc])) - FL
    return d if d <= max_dist else max_dist + 1.0

def run(max_vx, reaction_s, probe_max, obj_w, obj_h, dt=1.0/4.3, start=3.0,
        step=10, span=0.2, dil=1, ctrl_lag_s=0.37):
    """闭环：每帧按门限选最快可用速度，前进，看最近逼到多少。"""
    cfg = OC(min_wall_span_m=span, dilation_cells=dil)
    speeds = np.linspace(0.0, max_vx, 7)
    OBJ_X = 6.0
    cam_x = OBJ_X - start
    origin = np.array([cam_x - GRID[0]*RES/2 + 2.0, -GRID[1]*RES/2, CAM_H - GRID[2]*RES/2 + OFFSET[2]])
    occ = np.zeros(GRID); v = 0.0; hist=[]
    lag_frames = max(1, int(round(ctrl_lag_s / dt)))
    pending = []
    for k in range(200):
        dist = OBJ_X - cam_x
        depth = render(dist, obj_w, obj_h)
        occ *= 0.99
        Tc = T.copy(); Tc[0,3] = cam_x
        occ += run_raycasting_loopy(depth, Tc, GRID, FX, FY, CX, CY, origin, step, RES)
        np.clip(occ, -0.2, 0.2, out=occ)
        mask = build_obstacle_map(occ, origin, RES, robot_z=CAM_H, config=cfg)
        # 控制中心在相机后 0.09-0.04=0.05 m（camera_x=0.09, control_x=0.04）
        clr = clearance_along(cam_x - 0.05, mask, origin, RES, probe_max)
        allowed = [s for s in speeds[1:] if clr >= s * reaction_s]
        cmd = max(allowed) if allowed else 0.0
        pending.append(cmd)                      # 指令滞后
        v = pending.pop(0) if len(pending) > lag_frames else 0.0
        gap = dist - FL                          # 车体面到物体
        hist.append((round(dist,2), round(clr,2) if clr<=probe_max else float('inf'), round(cmd,3), round(gap,2)))
        if gap <= 0.0: return "撞上", hist
        if v == 0.0 and cmd == 0.0 and k > 5: return f"停住，车体面离物体 {gap:.2f} m", hist
        cam_x += v * dt
    return "未收敛", hist



if __name__ == "__main__":
    print("=== 扫 reaction_time_s（0.5 m/s 撞墙）===")
    for rs in (0.6, 1.0, 1.2, 1.6, 2.0):
        res, _ = run(0.5, rs, 1.2, 3.0, 1.5)
        print(f"  reaction={rs:.1f}s -> {res}")
    print("=== 控制滞后 0.8 s（板子排队时）===")
    for rs in (1.6, 2.0):
        res, _ = run(0.5, rs, 1.2, 3.0, 1.5, ctrl_lag_s=0.8)
        print(f"  reaction={rs:.1f}s -> {res}")
