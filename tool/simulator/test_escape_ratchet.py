"""直接测 12° 棘轮本身。仿真套跑不出这一幕，所以复现板上 2026-09-09 220.1s 那 13.1 秒。

只调**真的** PlanningNode._open_heading。旧行为 = 同一个函数传
(min_deg=12, avoid=(), prefer_side=None) —— 新加的 avoid 检查在 avoid=() 时恒为假、
prefer_side=None 时走原来那行 prefer 计算，所以这两组参数就是改动前后的两个版本，
不用另抄一份实现（抄一份就是代码里警告过的"复本迟早分叉"）。
"""
import math, os, sys
sys.path.insert(0, '/tinynav' if os.path.isdir('/tinynav') else
                   '/home/dm/looper/tinynav-x5')
import tinynav.core.planning_node as P

# 板上现场：前方大开（front_clearance >1.20m），右前方 62°~120° 一片堵。
# 边界取 62° 的依据：日志里 yaw=51 时 yaw+12=63 判堵、yaw=49 时 yaw+12=61 判空。
BLOCKED = (math.radians(62.0), math.radians(120.0))

class Stub:
    _escape_scan_step_deg = 15
    escape_min_clearance_m = 0.50
    _wrap = staticmethod(P.PlanningNode._wrap)   # 原本是 staticmethod
    _open_heading = P.PlanningNode._open_heading
    def _clearance_along(self, centre, cx, cy, mask):
        y = math.atan2(cy, cx)
        return 0.10 if BLOCKED[0] <= y <= BLOCKED[1] else 0.90

YAW0, TO_TARGET = math.radians(65.0), math.radians(116.0)
REACHED, RATE, DT = math.radians(12.0), math.radians(8.2), 0.2

def sim(label, use_fix, ticks=70):
    s = Stub(); yaw = YAW0
    goal = None; tried = []; side = None
    resel = flips = 0; last_dir = 0; gross = 0.0; errs = []
    for _ in range(ticks):
        err = s._wrap(goal - yaw) if goal is not None else 0.0
        if goal is None or abs(err) < REACHED:
            if use_fix:
                if side is None:
                    side = 1.0 if s._wrap(TO_TARGET - yaw) >= 0 else -1.0
                g, _ = s._open_heading(None, None, yaw, TO_TARGET, min_deg=15,
                                       avoid=tried, avoid_sep=math.radians(30.0),
                                       prefer_side=side)
            else:
                g, _ = s._open_heading(None, None, yaw, TO_TARGET,
                                       min_deg=12 if goal is not None else 0)
            if g is None:
                if side is None:
                    side = 1.0 if s._wrap(TO_TARGET - yaw) >= 0 else -1.0
                g = s._wrap(yaw + side * math.pi / 2.0)
            goal = g; tried.append(g); resel += 1
            err = s._wrap(goal - yaw)
        errs.append(math.degrees(abs(err)))
        d = 1 if err > 0 else -1
        if last_dir and d != last_dir: flips += 1
        last_dir = d
        step = math.copysign(min(RATE * DT, abs(err)), err)
        yaw = s._wrap(yaw + step); gross += abs(step)
    print(f"{label:8s} 重选 {resel:3d} 次 | 方向翻转 {flips:3d} 次 | 累计转 "
          f"{math.degrees(gross):6.1f}° | 净转 {abs(math.degrees(s._wrap(yaw-YAW0))):5.1f}° "
          f"| |err| 中位 {sorted(errs)[len(errs)//2]:4.1f}° 最大 {max(errs):5.1f}°")

print("70 拍 x 0.2 s = 14.0 s，对应板上那 13.1 秒\n")
sim("改动前", False)
sim("改动后", True)
