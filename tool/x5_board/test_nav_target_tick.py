"""nav 定时器路径的离线回归。在板上跑：

    . /userdata/x5/env.sh && python3 /userdata/x5/tinynav/tool/x5_board/test_nav_target_tick.py

🔴 为什么存在：2026-09-03 把全局路线从 keyframe_callback 搬到定时器（上游 #178）之后
连续两次在真机上才暴露问题，都是「只在关键帧路径存在的东西」——
  ① 位姿源订了 /slam/odometry，而它 Publisher count = 0 → latest_odom_pose 永远 None
     → try_publish_nav_path 一次都没跑 → 箭头不出现、POI 不走，零日志
  ② self.odom[timestamp] 只有关键帧的键 → KeyError 把整个节点打死

这两个都能在这里离线抓到。**改 try_publish_nav_path 或它的调用方之后先跑这个再推板。**

⚠️ 必须【直接调】try_publish_nav_path，不能只调 nav_target_timer_callback ——
后者有 try/except 护栏，会把异常吞成一条 error 日志，测试看不见（我踩过）。
Probe.__getattr__ 只是用来一遍列出缺的桩属性，不是被测代码的一部分。"""
import sys, numpy as np
from unittest.mock import MagicMock
from builtin_interfaces.msg import Time as _Time
sys.path.insert(0, "/userdata/x5/tinynav")
import tinynav.core.map_node as m
import re, ast

# 从真实 __init__ 里抓出所有 self.X = <字面量/简单表达式> 的默认值
src = open("/userdata/x5/tinynav/tinynav/core/map_node.py", encoding="utf-8").read()
init = src[src.index("    def __init__(\n"):src.index("\n    def _start_nav_path_search_warmup")]
defaults = {}
for mm in re.finditer(r"^\s*self\.([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+?)$", init, re.M):
    name, expr = mm.group(1), mm.group(2).strip()
    try:
        defaults[name] = ast.literal_eval(expr)
    except Exception:
        if expr.startswith("float(os.environ"):
            d = re.search(r"'([\d.]+)'\)", expr)
            if d: defaults[name] = float(d.group(1))
        elif expr.startswith("int(os.environ"):
            d = re.search(r"'(-?\d+)'\)", expr)
            if d: defaults[name] = int(d.group(1))

missing = []
class Probe(m.MapNode):
    def __getattr__(self, name):          # 只在正常查找失败时调用
        missing.append(name)
        return 0.0

n = Probe.__new__(Probe)
for k, v in defaults.items():
    object.__setattr__(n, k, v)
for a in ["current_pose_in_map_pub","poi_pub","poi_change_pub","target_pose_pub",
          "global_plan_pub","global_plan_odom_pub","nav_done_pub","nav_progress_pub",
          "poi_status_pub","tf_broadcaster"]:
    setattr(n, a, MagicMock())
n.get_logger = MagicMock()
n.get_clock = MagicMock()
n.get_clock.return_value.now.return_value.to_msg.return_value = _Time(sec=1788423246, nanosec=0)
n.T_from_map_to_odom = np.eye(4)
n.pois = [np.array([2.0, 0.0, 0.0])]
n.poi_index = 0
n.odom = {}                      # 空：定时器的时间戳不在里面
n.pose_graph_used_pose = {}
n.timer_logger = None
n.cached_nav_path_in_map = None
n.cached_nav_path_poi_index = -1
n._arrival_prev = None
n._last_nav_path_sig = None
path = np.array([[x, 0.0, 0.0] for x in np.linspace(0, 2.0, 21)])
n.generate_nav_path_in_map = lambda pose_in_map, target_poi: path
n._publish_poi_status = MagicMock()

try:
    n.try_publish_nav_path(1788423246275760450, odom_pose=np.eye(4))
    print("走完了，没抛异常")
except Exception as e:
    print("抛了:", type(e).__name__, e)
print("\n=== 定时器路径上取不到的属性（去重，按出现顺序）===")
seen=[]
for x in missing:
    if x not in seen and not x.startswith("__"): seen.append(x)
for x in seen: print("  ", x)
print("\n发布计数: current_pose_in_map=%d  poi=%d  target_pose=%d  global_plan=%d" % (
    n.current_pose_in_map_pub.publish.call_count, n.poi_pub.publish.call_count,
    n.target_pose_pub.publish.call_count, n.global_plan_pub.publish.call_count))
errs = n.get_logger.return_value.error.call_args_list
print("logger.error 次数 =", len(errs))

print("\n=== 追加用例 ===")
# ② 第二拍应该走缓存
calls=[]
n.generate_nav_path_in_map = lambda pose_in_map, target_poi: (calls.append(1), path)[1]
n.try_publish_nav_path(1788423246775760450, odom_pose=np.eye(4))
print("② 第二拍重搜次数 =", len(calls), "(期望 0)")
# ③ 偏离缓存路线 -> 应该重搜
far = np.eye(4); far[:3,3] = [0.0, 3.0, 0.0]
n.try_publish_nav_path(1788423247275760450, odom_pose=far)
print("③ 偏离 3m 后重搜次数 =", len(calls), "(期望 1)")
# ④ stall 打开 + 车不动 -> 超时后重搜
import time as _t
n.nav_stall_timeout_s = 0.5
n._nav_stall_best_remaining = None; n._nav_stall_since = None
n.try_publish_nav_path(1788423247775760450, odom_pose=np.eye(4)); c0=len(calls)
_t.sleep(0.8)
n.try_publish_nav_path(1788423248275760450, odom_pose=np.eye(4))
print("④ stall 超时后重搜次数增量 =", len(calls)-c0, "(期望 1)")
# ⑤ 定时器护栏：位姿源为空 + 无关键帧 -> 安静返回，不许抛
n.latest_odom_pose=None; n.pose_graph_used_pose={}
n.nav_target_timer_callback()
print("⑤ 无位姿源时定时器安静返回 ✅  logger.error =", len(n.get_logger.return_value.error.call_args_list))
# ⑥ 关键帧兜底
n.pose_graph_used_pose={999: np.eye(4)}
n.nav_target_timer_callback()
print("⑥ 关键帧兜底后 target_pose 发布次数 =", n.target_pose_pub.publish.call_count)
print("\n全部用例结束")
