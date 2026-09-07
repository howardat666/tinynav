"""前视点选择的回放回归：用 2026-09-03 16:24 那趟的真实几何。

🔴 那趟的失效：车沿【缓存】路径走过 lookahead(2.0 m) 之后，路径自己的起点满足
radial >= 2.0，而选择逻辑取的是整条路径上【最早】的合格点 → chosen_index 塌到 0，
目标跳回出发点、落在车后方 2.2~3.0 m，车原地蹭了 40 秒。
判据（不是"index 严格单调" —— 车自己会小幅来回，前视点跟着抖 1~2 格是纯追踪的正常行为）：
  ① 目标必须【比车更靠近 POI】—— 这一条在旧代码上会响亮地失败（目标 5.8 m 而车 3.4 m）
  ② 不许出现大幅回退（>5 格），塌回路径头就是 -50 格量级"""
import sys, re, numpy as np
from unittest.mock import MagicMock
from builtin_interfaces.msg import Time as _Time
sys.path.insert(0, "/userdata/x5/tinynav")
import tinynav.core.map_node as m
import ast

src = open("/userdata/x5/tinynav/tinynav/core/map_node.py", encoding="utf-8").read()
init = src[src.index("    def __init__(\n"):src.index("\n    def _start_nav_path_search_warmup")]
defaults = {}
for mm in re.finditer(r"^\s*self\.([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+?)$", init, re.M):
    name, expr = mm.group(1), mm.group(2).strip()
    try: defaults[name] = ast.literal_eval(expr)
    except Exception:
        d = re.search(r"'(-?[\d.]+)'\)", expr)
        if d and expr.startswith(("float(os.environ", "int(os.environ")):
            defaults[name] = float(d.group(1)) if "float" in expr else int(d.group(1))

n = m.MapNode.__new__(m.MapNode)
for k, v in defaults.items(): setattr(n, k, v)
for a in ["current_pose_in_map_pub","poi_pub","poi_change_pub","target_pose_pub",
          "global_plan_pub","global_plan_odom_pub","nav_done_pub","nav_progress_pub",
          "poi_status_pub","tf_broadcaster"]:
    setattr(n, a, MagicMock())
n.get_logger = MagicMock(); n.get_clock = MagicMock()
n.get_clock.return_value.now.return_value.to_msg.return_value = _Time(sec=1788424000, nanosec=0)
n.T_from_map_to_odom = np.eye(4)
POI = np.array([-4.241587, 0.534606, 0.0])
n.pois = [POI]; n.poi_index = 0
n.odom = {}; n.pose_graph_used_pose = {}; n.timer_logger = None
n._arrival_prev = None; n._last_nav_path_sig = None
n._leg_initial_length = None; n._leg_start_time = None; n._speed_estimate = None
n.cached_nav_path_in_map = None; n.cached_nav_path_poi_index = -1
n._pursuit_index = 0; n._pursuit_path_sig = None
n._publish_poi_status = MagicMock()

# 那趟真实路径：头 (1.5,-0.2) -> 尾 (-4.3,0.5)，71 点
path = np.array([[1.5 + (-4.3-1.5)*t, -0.2 + (0.5+0.2)*t, 0.4] for t in np.linspace(0,1,71)])
n.generate_nav_path_in_map = lambda pose_in_map, target_poi: path

# 日志里 robot_map 的真实序列
traj = [(1.60,-0.14),(1.61,-0.14),(0.91,0.12),(0.40,0.26),(-0.26,0.33),(-0.67,0.34),
        (-0.90,0.34),(-0.85,0.33),(-0.92,0.06),(-0.87,-0.00),(-0.82,-0.00),(-0.81,0.03),
        (-1.21,0.70),(-1.32,0.23),(-1.41,0.23),(-2.20,0.30),(-3.10,0.40),(-4.00,0.50)]
idxs, dists, robot_d = [], [], []
print(" 车位置            target_map        index  目标离POI  车离POI")
for k,(x,y) in enumerate(traj):
    T = np.eye(4); T[:3,3] = [x,y,0.4]
    n.try_publish_nav_path(1788424000_000000000 + k*500_000_000, odom_pose=T)
    ci = n._pursuit_index
    beyond = np.flatnonzero(np.linalg.norm(path[:,:2]-np.array([x,y]),axis=1)[ci:] >= n.nav_lookahead_m)
    chosen = int(ci+beyond[0]) if len(beyond) else len(path)-1
    tp = path[chosen]
    d = float(np.linalg.norm(tp[:2]-POI[:2]))
    rd = float(np.linalg.norm(np.array([x,y])-POI[:2]))
    idxs.append(chosen); dists.append(d); robot_d.append(rd)
    flag = "" if d < rd else "  🔴 目标比车还远"
    print("[%+.2f,%+.2f]   [%+.2f,%+.2f]      %3d   %6.2f   %6.2f%s"%(x,y,tp[0],tp[1],chosen,d,rd,flag))

ok = True
bad = [(d,rd) for d,rd in zip(dists,robot_d) if d >= rd]
if bad: print("🔴 有 %d 拍目标比车离 POI 更远（旧代码就是这样）"%len(bad)); ok=False
for a,b in zip(idxs, idxs[1:]):
    if b < a - 5: print("🔴 index 大幅回退: %d -> %d"%(a,b)); ok=False
mx = max(idxs)-min(idxs)
print("\n%s  ① 目标始终比车更靠近 POI  ② 无大幅回退（最大回退 %d 格）"%(
    "✅ 通过:" if ok else "❌ 失败:", max([a-b for a,b in zip(idxs,idxs[1:])]+[0])))

# ---- 反向验证：这个测试必须能抓住【旧逻辑】，否则它没有意义 ----
print("\n=== 旧逻辑对照（beyond 从路径头扫，取 beyond[0]）===")
old_bad = 0
for x, y in traj:
    radial = np.linalg.norm(path[:, :2] - np.array([x, y]), axis=1)
    beyond = np.flatnonzero(radial >= n.nav_lookahead_m)
    ci = int(beyond[0]) if len(beyond) else len(path) - 1
    d = float(np.linalg.norm(path[ci][:2] - POI[:2]))
    rd = float(np.linalg.norm(np.array([x, y]) - POI[:2]))
    if d >= rd:
        old_bad += 1
        if old_bad <= 3:
            print("  车[%+.2f,%+.2f] -> index=%d 目标[%+.2f,%+.2f] 离POI %.2f 而车 %.2f 🔴"
                  % (x, y, ci, path[ci][0], path[ci][1], d, rd))
print("  旧逻辑有 %d/%d 拍目标比车更远 —— %s"
      % (old_bad, len(traj), "测试确实能抓住 ✅" if old_bad else "测试抓不住，判据无效 ❌"))
