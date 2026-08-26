"""三套配置的闭环对照：main / 我们之前 / 我们现在。

用真的 PlanningNode 方法做决策 —— 打桩 ROS 后 __new__ 出一个实例、只填决策要用的字段，
所以门限、脱困、后退这些逻辑是逐字的，不是重写的。感知也是真内核
(run_raycasting_loopy -> build_obstacle_map -> distance_transform_edt)。

跑法（本机无 numba，用镜像）：
  docker run --rm --entrypoint bash -v $PWD:/proj tinynav-x5:sp \
    -lc 'cd /proj && python3 tool/sim_config_compare.py'
"""
import os, sys, math, types, itertools
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

# ---------------------------------------------------------------- ROS 打桩
def _stub(name, attrs=()):
    m = sys.modules.get(name)
    if m is None:
        m = types.ModuleType(name); m.__path__ = []; sys.modules[name] = m
    for a in attrs:
        if not hasattr(m, a):
            setattr(m, a, type(a, (), {"__init__": lambda self, *a, **k: None}))
    return m

MSG = ("Image", "CameraInfo", "PointField", "PointCloud2", "PointCloud", "Path",
       "Odometry", "OccupancyGrid", "PoseStamped", "Point32", "Bool", "Header",
       "String", "Twist", "Pose", "Point", "Quaternion", "TransformStamped")
for pkg in ("sensor_msgs", "nav_msgs", "geometry_msgs", "std_msgs"):
    _stub(pkg); _stub(pkg + ".msg", MSG)
_stub("sensor_msgs_py"); _stub("sensor_msgs_py.point_cloud2")
_stub("cv_bridge", ("CvBridge",))
_stub("message_filters", ("Subscriber", "ApproximateTimeSynchronizer", "TimeSynchronizer"))

rclpy = _stub("rclpy")
rclpy.init = lambda *a, **k: None
rclpy.shutdown = lambda *a, **k: None
rclpy.spin = lambda *a, **k: None
_stub("rclpy.executors", ("ExternalShutdownException",))
_stub("rclpy.qos", ("DurabilityPolicy", "QoSProfile", "ReliabilityPolicy", "HistoryPolicy"))
class _Node:
    def __init__(self, *a, **k): pass
sys.modules.setdefault("rclpy.node", types.ModuleType("rclpy.node"))
sys.modules["rclpy.node"].Node = _Node
class _Time:
    def __init__(self, *a, **k): pass
class _Duration:
    def __init__(self, seconds=0.0, **k): self.s = seconds
    def to_msg(self): return self
_stub("rclpy.time"); sys.modules["rclpy.time"].Time = _Time
_stub("rclpy.duration"); sys.modules["rclpy.duration"].Duration = _Duration

import tinynav.core.planning_node as PN
from tinynav.core.robot_config import DIFFCAR_CONFIG, ObstacleConfig
from scipy.ndimage import distance_transform_edt
from dataclasses import replace

# ---------------------------------------------------------------- 世界与相机
W, H = 544, 640
FX, FY, CX, CY = 330.0, 330.2, 271.1, 319.8
CAM_H = 0.18
RES = 0.1
GRID = (100, 100, 9)
OFFSET = np.array([0.0, 0.0, 0.15])
DT = 1.0 / 4.3                      # depth 实际约 4.3 Hz
u = (np.arange(W) - CX) / FX
v = (np.arange(H) - CY) / FY


def cam_T(x, y, yaw, h=CAM_H):
    """相机光学系 -> 世界。光学: x 右 y 下 z 前；世界: x 前 y 左 z 上。"""
    c, s = math.cos(yaw), math.sin(yaw)
    R_bw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    R_cb = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    T = np.eye(4)
    T[:3, :3] = R_bw @ R_cb
    T[:3, 3] = (x, y, h)
    return T


def render(boxes, T, max_range=8.0):
    """世界里一组竖直长方体 (x0,x1,y0,y1,z_top) 的深度图 (H,W)，含地面。"""
    R = T[:3, :3]
    o = T[:3, 3]
    dirs = np.stack([np.broadcast_to(u[None, :], (H, W)),
                     np.broadcast_to(v[:, None], (H, W)),
                     np.ones((H, W))], axis=-1)          # 光学系方向（未归一）
    dw = dirs @ R.T                                       # 世界方向
    d = np.full((H, W), np.inf)
    # 地面 z=0
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(dw[..., 2] < -1e-6, -o[2] / dw[..., 2], np.inf)
    d = np.minimum(d, np.where(np.isfinite(t) & (t > 0) & (t <= max_range), t, np.inf))
    # slab 求交
    for (x0, x1, y0, y1, zt) in boxes:
        lo = np.zeros((H, W)); hi = np.full((H, W), max_range)
        for k, (a, b) in enumerate(((x0, x1), (y0, y1), (0.0, zt))):
            dk = dw[..., k]; ok = np.abs(dk) > 1e-9
            with np.errstate(divide="ignore", invalid="ignore"):
                t0 = np.where(ok, (a - o[k]) / dk, -np.inf)
                t1 = np.where(ok, (b - o[k]) / dk, np.inf)
            tmin = np.minimum(t0, t1); tmax = np.maximum(t0, t1)
            inside = (o[k] >= a) & (o[k] <= b)
            tmin = np.where(ok, tmin, np.where(inside, -np.inf, np.inf))
            tmax = np.where(ok, tmax, np.where(inside, np.inf, -np.inf))
            lo = np.maximum(lo, tmin); hi = np.minimum(hi, tmax)
        hitm = (lo <= hi) & (lo > 0)
        d = np.where(hitm, np.minimum(d, lo), d)
    # depth 是沿光轴的 z，不是斜距
    zc = d / np.sqrt(dirs[..., 0] ** 2 + dirs[..., 1] ** 2 + 1.0)
    return np.where(np.isfinite(zc), zc, 0.0).astype(np.float64)


# ---------------------------------------------------------------- 三套配置
CFGS = {
    # main：二值门限 0.30 m + 「堵住只准倒车」硬互斥，没有原地转脱困、没有后退凭据
    "main": dict(max_vx=0.5, front_blocked_m=0.30, probe_max=0.5, reaction=None,
                 dilation=2, z_bottom=-0.4, z_top=0.4, reverse_speed=0.2,
                 vx_cont_w=10.0, escape=False, retreat=False, omega_max=math.pi/3,
                 grid=(100,100,10), goffset=(0.0,0.0,0.0)),
    # 我们之前：二值门限 0.30 m，但堵住时准原地转；有脱困和后退（退 0.6 m/次，上限 2 s）
    "prev": dict(max_vx=0.3, front_blocked_m=0.30, probe_max=0.5, reaction=None,
                 dilation=1, z_bottom=-0.3, z_top=0.4, reverse_speed=0.2,
                 vx_cont_w=40.0, escape=True, retreat=True, retreat_max_s=2.0, omega_max=1.05,
                 grid=(100,100,9), goffset=(0.0,0.0,0.15)),
    # 现在：分档门限（每档速度按 v*reaction 要视距），探针 1.2 m，退 0.18 m/次
    "now":  dict(max_vx=0.5, front_blocked_m=0.20, probe_max=1.2, reaction=2.0,
                 dilation=1, z_bottom=-0.3, z_top=0.4, reverse_speed=0.06,
                 vx_cont_w=40.0, escape=True, retreat=True, retreat_max_s=4.0, omega_max=1.05,
                 grid=(100,100,9), goffset=(0.0,0.0,0.15)),
    # new = 2026-08-26 落地的真代码：门限走 PlanningNode._prefix_clearance，
    # 倒车词汇表 5 条(直退 + 正负 43/86 度)，「堵住」= 一条前进轨迹都不可行。
    "new":  dict(max_vx=0.5, front_blocked_m=0.20, probe_max=1.2, reaction=2.0,
                 dilation=1, z_bottom=-0.3, z_top=0.4, reverse_speed=0.06,
                 vx_cont_w=40.0, escape=True, retreat=True, retreat_max_s=4.0,
                 omega_max=1.05, grid=(100,100,9), goffset=(0.0,0.0,0.15),
                 real_gate=True, multi_reverse=True),
    # 提案：门限按**每条轨迹自己要走的那一段**来判，而不是沿当前朝向的一条直线。
    # 前进可行 <=> 它将要执行的那 reaction 秒里，车体五点的 ESDF 都还有 prefix_margin。
    "fix":  dict(max_vx=0.5, front_blocked_m=0.20, probe_max=1.2, reaction=2.0,
                 dilation=1, z_bottom=-0.3, z_top=0.4, reverse_speed=0.06,
                 vx_cont_w=40.0, escape=True, retreat=True, retreat_max_s=4.0,
                 omega_max=1.05, grid=(100,100,9), goffset=(0.0,0.0,0.15),
                 per_traj=True, prefix_margin=0.10),
    # 提案2 = 提案1 + 把"离障碍多远"从一个近距离硬爆炸项换成有界的远距离偏好：
    # 现在的软代价只在 0.1 m 以内才出现，且 score*1e5 一出现就压倒 100*dist，
    # 于是规划器在 0.2 m 以外对障碍完全无感，等到有感时已经绕不开了。
    "fix2": dict(max_vx=0.5, front_blocked_m=0.20, probe_max=1.2, reaction=2.0,
                 dilation=1, z_bottom=-0.3, z_top=0.4, reverse_speed=0.06,
                 vx_cont_w=40.0, escape=True, retreat=True, retreat_max_s=4.0,
                 omega_max=1.05, grid=(100,100,9), goffset=(0.0,0.0,0.15),
                 per_traj=True, prefix_margin=0.10, clear_ref=0.6, clear_w=400.0),
    # 提案3 = 提案2 + 把"朝向"放进代价函数，并把原地转脱困缩回"真的没有前进可选"时。
    # 现在代价只看轨迹终点的位置，所以朝向只能靠 escape hatch 这条旁路来管，而那条旁路
    # 一触发就整个接管排序、且带 4 s 方向锁 —— 仿真里车因此原地转了将近两整圈。
    "fix3": dict(max_vx=0.5, front_blocked_m=0.20, probe_max=1.2, reaction=2.0,
                 dilation=1, z_bottom=-0.3, z_top=0.4, reverse_speed=0.06,
                 vx_cont_w=40.0, escape=True, retreat=True, retreat_max_s=4.0,
                 omega_max=1.05, grid=(100,100,9), goffset=(0.0,0.0,0.15),
                 per_traj=True, prefix_margin=0.10, clear_ref=0.6, clear_w=400.0,
                 head_w=150.0, escape_only_blocked=True),
    # 提案4 = 提案3 + 把代价里的"到目标的直线距离"换成"在本地障碍图上绕过去的距离"
    # （对本地栅格做一次 Dijkstra 波前）。前三套都在障碍正前方有一个局部极小：绕行的
    # 轨迹终点直线距离更远，于是"站着不动"永远最优 —— 换成测地距离这个极小就没了。
    "fix4": dict(max_vx=0.5, front_blocked_m=0.20, probe_max=1.2, reaction=2.0,
                 dilation=1, z_bottom=-0.3, z_top=0.4, reverse_speed=0.06,
                 vx_cont_w=40.0, escape=True, retreat=True, retreat_max_s=4.0,
                 omega_max=1.05, grid=(100,100,9), goffset=(0.0,0.0,0.15),
                 per_traj=True, prefix_margin=0.10, clear_ref=0.6, clear_w=150.0,
                 head_w=150.0, escape_only_blocked=True, wavefront=True,
                 pass_clearance=0.20),
}


def make_node(cfg):
    n = PN.PlanningNode.__new__(PN.PlanningNode)
    obst = ObstacleConfig(robot_z_bottom=cfg["z_bottom"], robot_z_top=cfg["z_top"],
                          dilation_cells=cfg["dilation"])
    n.robot = replace(DIFFCAR_CONFIG, max_vx=cfg["max_vx"],
                      front_blocked_m=cfg["front_blocked_m"], obstacle=obst)
    n.resolution = RES
    n.grid_shape = tuple(cfg["grid"])
    n.grid_offset = np.array(cfg["goffset"])
    n.origin = np.array(n.grid_shape) * RES / -2.0 + n.grid_offset
    n.dt = 0.1
    n.front_probe_max_m = cfg["probe_max"]
    n.prefix_margin_m = cfg.get("prefix_margin", 0.10)
    n.reaction_time_s = cfg["reaction"] if cfg["reaction"] else 0.0
    n.rotate_first_min_dist_m = 0.3
    n.force_turn_heading_rad = math.radians(35.0)
    n.min_progress_m = 0.05
    n.escape_min_clearance_m = max(0.4, n.robot.front_blocked_m + 0.1)
    n._escape_turn_sign = None
    n._escape_turn_ns = 0
    n._escape_lock_max_s = 4.0
    from collections import deque
    n._centre_history = deque()
    n._retreat_history_m = 2.0
    n._retreat_history_max_s = 60.0
    n._retreat_min_step_m = 0.02
    n._retreat_match_m = 0.20
    n._retreat_max_s = cfg.get("retreat_max_s", 4.0)
    n._retreat_start_ns = 0
    n._escape_before_retreat_s = 3.0
    n._escape_episode_ns = 0
    n._last_param = np.zeros(2)
    return n


def traj_lib(cfg, n, p, q):
    """与 planning_node 同一条构造链：格点库 + 词汇表（原地转来自格点库 vx=0 那一档）。"""
    trajs, params = PN.generate_trajectory_library_3d(
        init_p=p.copy(), init_q=q.copy(), dt=n.dt,
        vx_max=cfg["max_vx"], omega_max=cfg["omega_max"])
    trajs = PN.normalize_pose_trajectories(trajs)
    vt, vp = PN.generate_predefined_trajectory_vocabularies(
        init_p=p.copy(), init_q=q.copy(), dt=n.dt)
    vt = PN.normalize_pose_trajectories(vt)
    if not cfg.get("multi_reverse"):          # 历史配置只有直退那一条
        keep = [i for i in range(len(vp)) if abs(vp[i][1]) < 1e-9]
        vt, vp = vt[keep], vp[keep]
    rs = cfg["reverse_speed"]
    if abs(rs - 0.06) > 1e-9:                     # 源码写死 0.06，按配置线性换
        k = rs / 0.06
        for i in range(len(vp)):
            if vp[i][0] < 0.0:
                vt[i][:, :3] = p[None, :] + (vt[i][:, :3] - p[None, :]) * k
                vp[i][0] *= k
    return np.concatenate([trajs, vt], axis=0), np.concatenate([params, vp], axis=0)


def decide(n, cfg, T, obstacle_mask, esdf, target_xy, now_ns, trajs, params, scores,
           last_param, min_d=None, ctg=None):
    """镜像 planning_node 的决策段，三套配置只在门限/脱困/后退这三处分叉。"""
    init_p = T[:3, 3]
    centre = n.camera_to_robot_center(T)
    n._record_centre(centre, now_ns)
    front_clearance = n._front_obstacle_dist(T, obstacle_mask, max_dist=cfg["probe_max"])
    gate_m = n._front_gate_m() if cfg["reaction"] else n.robot.front_blocked_m
    front_blocked = front_clearance <= gate_m
    prefix_ok = None
    forward_ok = None
    if cfg.get("real_gate"):
        forward_ok = n._prefix_clearance(trajs, esdf) >= n.prefix_margin_m
        front_blocked = not any(
            forward_ok[i] for i in range(len(params))
            if params[i][0] > 0.0 and not n._is_turn_in_place(params[i]))
    if cfg.get("per_traj"):
        prefix_ok = _prefix_clearance_ok(n, trajs, params, esdf, cfg["reaction"],
                                         cfg["prefix_margin"])
        # 「前方堵住」= 一条前进轨迹都不可行，不再是一条直线探针跨过某个数字。
        fwd = [i for i in range(len(params))
               if params[i][0] > 0.0 and not n._is_turn_in_place(params[i])]
        front_blocked = not any(prefix_ok[i] for i in fwd)

    def gate(param, i=None):
        if cfg.get("real_gate") and i is not None:            # new：调真方法
            return n._motion_gate_penalty(param, i, forward_ok, front_blocked)
        if cfg.get("per_traj") and i is not None:             # fix：按轨迹自己那一段
            if param[0] < 0.0:
                return 0.0 if (front_blocked and n.robot.allow_reverse) else 1e9
            if n._is_turn_in_place(param):
                return 0.0
            return 0.0 if prefix_ok[i] else 1e9
        if cfg["reaction"]:                                   # now：旧的分档门限
            if param[0] < 0.0:
                return 0.0 if (front_blocked and n.robot.allow_reverse) else 1e9
            if n._is_turn_in_place(param):
                return 0.0
            return 0.0 if front_clearance >= param[0] * cfg["reaction"] else 1e9
        if cfg["escape"]:                                     # prev：二值，堵住准原地转
            if front_blocked and not n._is_turn_in_place(param):
                return 1e9
            return 0.0
        # main：硬互斥，堵住只准倒车
        is_back = param[0] < 0.0
        if front_blocked != is_back:
            return 1e9
        return 0.0

    tgt = np.array(target_xy)
    stand_dist = float(np.linalg.norm(init_p[:2] - tgt))
    if ctg is not None:
        ci = int((init_p[0] - n.origin[0]) / n.resolution)
        cj = int((init_p[1] - n.origin[1]) / n.resolution)
        if 0 <= ci < ctg.shape[0] and 0 <= cj < ctg.shape[1] and np.isfinite(ctg[ci, cj]):
            stand_dist = float(ctg[ci, cj])
    yaw_now = n._yaw_of(_quat_of(T))
    yaw_to_target = math.atan2(*(tgt - init_p[:2])[::-1])

    costs = np.empty(len(params))
    for i in range(len(params)):
        end = trajs[i][-1, :3]
        if ctg is not None:
            ei = int((end[0] - n.origin[0]) / n.resolution)
            ej = int((end[1] - n.origin[1]) / n.resolution)
            dist = (float(ctg[ei, ej]) if (0 <= ei < ctg.shape[0] and 0 <= ej < ctg.shape[1]
                                           and np.isfinite(ctg[ei, ej]))
                    else float(np.linalg.norm(end[:2] - tgt)) + 5.0)
        else:
            dist = float(np.linalg.norm(end[:2] - tgt))
        soft = 0.0
        if cfg.get("clear_ref"):
            soft = cfg["clear_w"] * max(0.0, cfg["clear_ref"] - float(min_d[i])) / cfg["clear_ref"]
        costs[i] = ((0.0 if cfg.get("clear_ref") else scores[i] * 100000)
                    + (1e9 if (cfg.get("clear_ref") and scores[i] == float("inf")) else 0.0)
                    + soft + 100 * dist
                    + (cfg["head_w"] * abs(n._wrap(yaw_to_target - n._yaw_of(trajs[i][-1, 3:7])))
                       if cfg.get("head_w") else 0.0)
                    + cfg["vx_cont_w"] * abs(last_param[0] - params[i][0])
                    + 10 * abs(last_param[1] - params[i][1]) + gate(params[i], i))
    admissible = np.flatnonzero(costs < 1e9)
    turns = [int(i) for i in admissible if n._is_turn_in_place(params[i])]
    tag = ""

    if len(admissible) == 0:
        if cfg["retreat"]:
            ri = n._retreat_index(params, trajs, now_ns)
            if ri is not None:
                return ri, "RETREAT", front_clearance, front_blocked
        return None, "STUCK", front_clearance, front_blocked
    if cfg["escape"] and front_blocked and not turns:
        if cfg["retreat"]:
            ri = n._retreat_index(params, trajs, now_ns)
            if ri is not None:
                return ri, "RETREAT", front_clearance, front_blocked
        return None, "STUCK", front_clearance, front_blocked

    best = int(admissible[int(np.argmin(costs[admissible]))])
    if not cfg["escape"]:
        return best, "", front_clearance, front_blocked

    ends = np.array([trajs[i][-1, :2] for i in admissible])
    best_gain = stand_dist - float(np.min(np.linalg.norm(ends - tgt[None, :], axis=1)))
    if cfg.get("escape_only_blocked"):
        reason = "blocked" if front_blocked else ""
    else:
        reason = n._escape_reason(front_blocked, stand_dist,
                                  abs(n._wrap(yaw_to_target - yaw_now)), best_gain)
    if reason and turns:
        if not n._escape_episode_ns:
            n._escape_episode_ns = now_ns
        age = (now_ns - n._escape_episode_ns) / 1e9
        pick, esc_clear = n._pick_escape_turn(centre, turns, trajs, yaw_to_target,
                                              obstacle_mask, params, now_ns)
        if cfg["retreat"] and (esc_clear < n.escape_min_clearance_m
                               or age > n._escape_before_retreat_s):
            ri = n._retreat_index(params, trajs, now_ns)
            if ri is not None:
                n._escape_turn_sign = None
                return ri, "RETREAT", front_clearance, front_blocked
        return pick, "TURN", front_clearance, front_blocked
    n._escape_turn_sign = None
    n._retreat_start_ns = 0
    n._escape_episode_ns = 0
    return best, "", front_clearance, front_blocked


def cost_to_go(esdf, origin, res, target_xy, pass_clear):
    """本地障碍图上到目标的测地距离场（Dijkstra，8 邻接）。

    目标常在栅格外，那就投影到边界上离它最近的可通行格子 —— 局部规划器要的是
    "往那边绕"的方向，不是精确的全局距离。"""
    import heapq
    h, w = esdf.shape
    free = esdf >= pass_clear
    # float64 不是随手写的：存成 float32 会把 nd 舍成略小的数，弹出时 d0 > g[a,b] 恒真，
    # 整个洪泛在第一圈就停了（实测只扩了 5 个格子）。
    g = np.full((h, w), np.inf, dtype=np.float64)
    # 目标常在本地栅格外。沿"机器人->目标"这条射线把它夹进栅格（内缩 5 格），
    # 再取附近最近的可通行格 —— 直接取"离目标索引最近的自由格"会挑到栅格边角上，
    # 车会朝那个角跑（实测跑出 16 m）。
    tx = float(np.clip(target_xy[0], origin[0] + 5 * res, origin[0] + (h - 5) * res))
    ty = float(np.clip(target_xy[1], origin[1] + 5 * res, origin[1] + (w - 5) * res))
    ti = int((tx - origin[0]) / res)
    tj = int((ty - origin[1]) / res)
    if free[ti, tj]:
        seeds = [(ti, tj)]
    else:
        ii, jj = np.nonzero(free)
        if not len(ii):
            return g
        k = int(np.argmin((ii - ti) ** 2 + (jj - tj) ** 2))
        seeds = [(int(ii[k]), int(jj[k]))]
    pq = []
    for (a, b) in seeds:
        g[a, b] = 0.0
        heapq.heappush(pq, (0.0, a, b))
    nbr = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
           (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    while pq:
        d0, a, b = heapq.heappop(pq)
        if d0 > g[a, b]:
            continue
        for da, db, c in nbr:
            na, nb = a + da, b + db
            if 0 <= na < h and 0 <= nb < w and free[na, nb]:
                nd = d0 + c * res
                if nd < g[na, nb]:
                    g[na, nb] = nd
                    heapq.heappush(pq, (nd, na, nb))
    return g


def traj_min_esdf(n, trajs, esdf, n_steps=None):
    """每条轨迹上车体五点的最小 ESDF。n_steps=None 表示整条。"""
    P = trajs if n_steps is None else trajs[:, :n_steps, :]
    fl, rl, hw = n.robot.footprint_from_control()
    qx, qy, qz, qw = P[..., 3], P[..., 4], P[..., 5], P[..., 6]
    fx = 2.0 * (qx * qz + qw * qy); fy = 2.0 * (qy * qz - qw * qx)
    nrm = np.hypot(fx, fy); nrm[nrm < 1e-6] = 1.0
    fx = fx / nrm; fy = fy / nrm
    lx, ly = -fy, fx
    xs = np.stack([P[..., 0],
                   P[..., 0] + fx * fl + lx * hw, P[..., 0] + fx * fl - lx * hw,
                   P[..., 0] - fx * rl + lx * hw, P[..., 0] - fx * rl - lx * hw], axis=-1)
    ys = np.stack([P[..., 1],
                   P[..., 1] + fy * fl + ly * hw, P[..., 1] + fy * fl - ly * hw,
                   P[..., 1] - fy * rl + ly * hw, P[..., 1] - fy * rl - ly * hw], axis=-1)
    ii = ((xs - n.origin[0]) / n.resolution).astype(np.int32)
    jj = ((ys - n.origin[1]) / n.resolution).astype(np.int32)
    inb = (ii >= 0) & (ii < esdf.shape[0]) & (jj >= 0) & (jj < esdf.shape[1])
    d = np.where(inb, esdf[np.clip(ii, 0, esdf.shape[0] - 1),
                           np.clip(jj, 0, esdf.shape[1] - 1)], np.inf)
    return d.min(axis=(1, 2))


def _prefix_clearance_ok(n, trajs, params, esdf, reaction_s, margin):
    """每条轨迹在它将要执行的那 reaction 秒里，车体五点的 ESDF 是否都 >= margin。

    与直线探针的区别就是这一句：转开的弧线在自己那段路上是空的，直线探针却因为
    正前方有东西而把它一起禁掉 —— 那正是「明明有路却只会原地摆头」的来源。"""
    fl, rl, hw = n.robot.footprint_from_control()
    n_pref = max(2, min(trajs.shape[1], int(round(reaction_s / n.dt)) + 1))
    P = trajs[:, :n_pref, :]
    qx, qy, qz, qw = P[..., 3], P[..., 4], P[..., 5], P[..., 6]
    fx = 2.0 * (qx * qz + qw * qy); fy = 2.0 * (qy * qz - qw * qx)
    nrm = np.hypot(fx, fy); nrm[nrm < 1e-6] = 1.0
    fx = fx / nrm; fy = fy / nrm
    lx, ly = -fy, fx
    xs = np.stack([P[..., 0],
                   P[..., 0] + fx * fl + lx * hw, P[..., 0] + fx * fl - lx * hw,
                   P[..., 0] - fx * rl + lx * hw, P[..., 0] - fx * rl - lx * hw], axis=-1)
    ys = np.stack([P[..., 1],
                   P[..., 1] + fy * fl + ly * hw, P[..., 1] + fy * fl - ly * hw,
                   P[..., 1] - fy * rl + ly * hw, P[..., 1] - fy * rl - ly * hw], axis=-1)
    ii = ((xs - n.origin[0]) / n.resolution).astype(np.int32)
    jj = ((ys - n.origin[1]) / n.resolution).astype(np.int32)
    ok_idx = (ii >= 0) & (ii < esdf.shape[0]) & (jj >= 0) & (jj < esdf.shape[1])
    d = np.where(ok_idx, esdf[np.clip(ii, 0, esdf.shape[0] - 1),
                              np.clip(jj, 0, esdf.shape[1] - 1)], np.inf)
    return d.min(axis=(1, 2)) >= margin


def _quat_of(T):
    from tinynav.core.math_utils import matrix_to_quat
    return matrix_to_quat(T[:3, :3])


# ---------------------------------------------------------------- 闭环
def simulate(name, boxes, target_xy, start=(0.0, 0.0, 0.0), tmax=45.0,
             ctrl_lag_s=0.37, verbose=False):
    cfg = CFGS[name]
    n = make_node(cfg)
    x, y, yaw = start
    occ = np.zeros(n.grid_shape)
    last_param = np.zeros(2)
    pending = []
    lag = max(1, int(round(ctrl_lag_s / DT)))
    stats = dict(t=0.0, turn=0, retreat=0, stuck=0, fwd=0, sign_flips=0,
                 min_gap=9.9, reached=False, dist=None, path=0.0)
    prev_omega_sign = 0
    steps = int(tmax / DT)
    for k in range(steps):
        T = cam_T(x, y, yaw)
        # 感知。栅格跟随相机重定心，用真的 roll —— 不 roll 而直接挪 origin，累积的
        # occ 每帧都换一批格子，谁也攒不到"连续两帧"，于是墙永远测不到（踩过）。
        depth = render(boxes, T)
        gs = n.grid_shape
        centre_now = n.origin + np.array(gs) * RES / 2 - n.grid_offset
        if np.linalg.norm(T[:3, 3] - centre_now) > 0.1:
            new_origin = T[:3, 3] - np.array(gs) * RES / 2 + n.grid_offset
            occ, n.origin = PN.roll_occupancy_grid(occ, n.origin, new_origin, RES)
        occ *= 0.99
        occ += PN.run_raycasting_loopy(depth, T, gs, FX, FY, CX, CY, n.origin, 10, RES)
        np.clip(occ, -0.2, 0.2, out=occ)
        mask = PN.build_obstacle_map(occ, n.origin, RES, robot_z=float(T[2, 3]),
                                     config=n.robot.obstacle)
        esdf = distance_transform_edt(~mask).astype(np.float32) * RES
        # 决策
        p = np.array([x, y, CAM_H]); q = _quat_of(T)
        trajs, params = traj_lib(cfg, n, p, q)
        scores, _ = n._score_trajectories(trajs, esdf, params)
        now_ns = int(k * DT * 1e9)
        min_d = traj_min_esdf(n, trajs, esdf) if cfg.get("clear_ref") else None
        ctg = (cost_to_go(esdf, n.origin, RES, target_xy, cfg["pass_clearance"])
               if cfg.get("wavefront") else None)
        idx, tag, clr, blocked = decide(n, cfg, T, mask, esdf, target_xy, now_ns,
                                        trajs, params, scores, last_param, min_d, ctg)
        if idx is None:
            vx, om = 0.0, 0.0
            stats["stuck"] += 1
        else:
            vx, om = float(params[idx][0]), float(params[idx][1])
            last_param = params[idx]
            if tag == "RETREAT": stats["retreat"] += 1
            elif tag == "TURN":  stats["turn"] += 1
            elif vx > 0:         stats["fwd"] += 1
        s = 0 if abs(om) < 1e-6 else (1 if om > 0 else -1)
        if s and prev_omega_sign and s != prev_omega_sign:
            stats["sign_flips"] += 1
        if s: prev_omega_sign = s
        # 执行（带滞后）
        pending.append((vx, om))
        cvx, com = pending.pop(0) if len(pending) > lag else (0.0, 0.0)
        x += cvx * math.cos(yaw) * DT
        y += cvx * math.sin(yaw) * DT
        yaw += com * DT
        stats["path"] += abs(cvx) * DT
        stats["t"] = (k + 1) * DT
        # 车体面到最近障碍（沿车头方向，粗算：车体矩形到 box 的距离）
        stats["min_gap"] = min(stats["min_gap"], _body_gap(x, y, yaw, n.robot, boxes))
        d = math.hypot(target_xy[0] - x, target_xy[1] - y)
        stats["dist"] = d
        if verbose and k % 4 == 0:
            print(f"    t={stats['t']:5.1f} x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+6.1f} "
                  f"clr={clr:.2f} blk={int(blocked)} {tag or 'fwd':7s} vx={vx:+.2f} om={om:+.2f} d={d:.2f}")
        if stats["min_gap"] <= 0.0:
            stats["reached"] = False
            return stats | {"verdict": "撞上"}
        if d < 0.3:
            stats["reached"] = True
            return stats | {"verdict": f"到达 {stats['t']:.1f}s"}
    return stats | {"verdict": "超时"}


def _body_gap(x, y, yaw, robot, boxes):
    """车体矩形四角+边中点到各 box 的最短外部距离（负=进入）。"""
    fl, rl, hw = robot.footprint_from_control()
    cx = x + robot.control_x * math.cos(yaw)      # 控制中心在世界
    cy = y + robot.control_x * math.sin(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(a, b) for a in (fl, 0.0, -rl) for b in (-hw, 0.0, hw)]
    g = 9.9
    for a, b in pts:
        wx = cx + a * c - b * s
        wy = cy + a * s + b * c
        for (x0, x1, y0, y1, _zt) in boxes:
            dx = max(x0 - wx, 0.0, wx - x1)
            dy = max(y0 - wy, 0.0, wy - y1)
            g = min(g, math.hypot(dx, dy) if (dx or dy) else -0.01)
    return g


# ---------------------------------------------------------------- 场景
def wall(xc, halfw=1.5, th=0.2, y0=None, zt=1.2):
    y0 = -halfw if y0 is None else y0
    return (xc, xc + th, y0, y0 + 2 * halfw, zt)

SCEN = {
    "S0 空旷直行":    dict(boxes=[(20.0, 20.2, -3.0, 3.0, 1.2)], target=(5.0, 0.0)),
    "S1 正撞墙":      dict(boxes=[wall(4.0)], target=(8.0, 0.0)),
    "S2 侧让 0.9m 缝": dict(boxes=[(3.0, 3.2, -3.0, -0.45, 1.2),
                                  (3.0, 3.2, 0.45, 3.0, 1.2)], target=(6.0, 0.0)),
    "S3 偏心椅子":    dict(boxes=[(3.0, 3.4, -0.25, 0.15, 0.5)], target=(6.0, 0.0)),
    "S4 死胡同":      dict(boxes=[wall(3.0, halfw=0.8),
                                  (1.0, 3.2, 0.7, 0.9, 1.2),
                                  (1.0, 3.2, -0.9, -0.7, 1.2)], target=(6.0, 0.0)),
    # 斜着开进死胡同：车头已经偏 17 度，直退会离来路越退越远 —— 这正是带转向的
    # 倒车要解决的那一种，也是 2026-08-26 那趟实车楔住的形状。
    "S5 斜进死胡同":  dict(boxes=[wall(3.0, halfw=0.8),
                                  (1.0, 3.2, 0.7, 0.9, 1.2),
                                  (1.0, 3.2, -0.9, -0.7, 1.2)], target=(6.0, 0.0),
                           start=(0.0, -0.3, 0.30)),
}


if __name__ == "__main__":
    lag = float(os.environ.get("LAG", "0.37"))
    for k_env, k_cfg in (("CW", "clear_w"), ("CR", "clear_ref"), ("HW", "head_w")):
        if os.environ.get(k_env):
            for _c in ("fix2", "fix3", "fix4"):
                CFGS[_c][k_cfg] = float(os.environ[k_env])
    only = os.environ.get("ONLY", "")
    names = os.environ.get("CFG", "main,prev,now,new").split(",")
    rows = []
    for sname, sc in SCEN.items():
        if only and only not in sname: continue
        for cname in names:
            r = simulate(cname, sc["boxes"], sc["target"],
                         start=sc.get("start", (0.0, 0.0, 0.0)), ctrl_lag_s=lag,
                         verbose=bool(os.environ.get("V")))
            rows.append((sname, cname, r))
            print(f"{sname:14s} {cname:5s} {r['verdict']:12s} "
                  f"gap={r['min_gap']:+.2f} 前进{r['fwd']:3d} 转{r['turn']:3d} "
                  f"退{r['retreat']:3d} 卡{r['stuck']:3d} 变向{r['sign_flips']:3d} "
                  f"剩余={r['dist']:.2f}m 走了={r['path']:.2f}m")
        print()
