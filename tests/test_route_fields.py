#!/usr/bin/env python3
"""build_route_fields 的两张查表图，以及打分内核的两个路线项。

移植自上游 PR #226 的单测，改成 x5 的内核签名（多了车体铺满取样和 is_circle）。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tinynav.core.planning_node import build_route_fields          # noqa: E402
from tinynav.core.planning_kernels import score_trajectories_by_ESDF  # noqa: E402

RES = 0.1
ORIGIN = np.array([0.0, 0.0, -0.3])
SHAPE = (50, 50)
fail = 0


def cell(x, y):
    return int((x - ORIGIN[0]) / RES), int((y - ORIGIN[1]) / RES)


def check(name, cond, detail=""):
    global fail
    fail += not cond
    print("  %-38s %s %s" % (name, "OK" if cond else "**FAIL**", detail))


# 1. 没有路线
pd, rm, has = build_route_fields([], SHAPE, ORIGIN, RES)
check("空路线 -> has_route=False", has is False)
check("空路线 -> 两张图都是 1e3", pd.min() == 1e3 and rm.min() == 1e3)

# 2. 一条从 (0.5,2.5) 到 (4.5,2.5) 的直线，总长 4.0 m
route = [(0.5, 2.5), (4.5, 2.5)]
pd, rm, has = build_route_fields(route, SHAPE, ORIGIN, RES)
r_on, c_on = cell(2.5, 2.5)
r_off, c_off = cell(2.5, 0.5)
check("直线路线 -> has_route=True", has is True)
check("路线上的格子 path_dist≈0", pd[r_on, c_on] < RES, "=%.3f" % pd[r_on, c_on])
check("偏离 2 m 的格子 path_dist≈2", abs(pd[r_off, c_off] - 2.0) < 0.2, "=%.3f" % pd[r_off, c_off])
check("路线中点 remaining≈2.0", abs(rm[r_on, c_on] - 2.0) < 0.2, "=%.3f" % rm[r_on, c_on])
check("偏离格继承最近路线格的 remaining",
      abs(rm[r_off, c_off] - rm[r_on, c_on]) < 0.2)

# 3. 打分内核：贴着路线的轨迹 vs 偏离 2 m 的轨迹
esdf = np.full(SHAPE, 10.0, dtype=np.float32)


def traj_along(y):
    """沿 +x 走 1 m 的一条轨迹，朝向 +x（相机光学约定：车头是车体 z）。"""
    n = 11
    t = np.zeros((n, 7))
    t[:, 0] = np.linspace(2.0, 3.0, n)
    t[:, 1] = y
    # 车头 +x：绕车体 y（朝下轴）转，使 R@[0,0,1] = [1,0,0]
    t[:, 3], t[:, 4], t[:, 5], t[:, 6] = 0.5, -0.5, 0.5, 0.5
    return t


on = np.ascontiguousarray(np.stack([traj_along(2.5)]))
off = np.ascontiguousarray(np.stack([traj_along(0.5)]))
_, _, pc_on, er_on = score_trajectories_by_ESDF(on, esdf, pd, rm, ORIGIN, RES,
                                                0.0, 0.1, 0.10, 0.18, 0.175, False)
_, _, pc_off, er_off = score_trajectories_by_ESDF(off, esdf, pd, rm, ORIGIN, RES,
                                                  0.0, 0.1, 0.10, 0.18, 0.175, False)
check("贴着路线 path_cost≈0", pc_on[0] < 0.15, "=%.3f" % pc_on[0])
check("偏离 2 m path_cost≈2", abs(pc_off[0] - 2.0) < 0.2, "=%.3f" % pc_off[0])
# 终点在 x=3.0，路线到 x=4.5，所以剩余是 1.5 而不是"走过的 1.0"
start_rem_on = float(rm[cell(2.0, 2.5)])
check("贴着路线走 1 m -> remaining 恰好减 1",
      abs((start_rem_on - er_on[0]) - 1.0) < 0.15,
      "起点 %.2f -> 终点 %.2f" % (start_rem_on, er_on[0]))
check("偏离的那条不因为终点更靠后就白捡进展", er_off[0] >= er_on[0] - 1e-6,
      "on=%.2f off=%.2f" % (er_on[0], er_off[0]))

# 4. 折回来的路线（发卡弯）：横切过去不能白捡进展
hair = [(0.5, 2.5), (3.0, 2.5), (3.0, 2.7), (0.5, 2.7)]
pd2, rm2, _ = build_route_fields(hair, SHAPE, ORIGIN, RES)
cut = np.zeros((11, 7))
cut[:, 0] = 2.0
cut[:, 1] = np.linspace(2.5, 2.7, 11)      # 直接跨过 0.2 m 到回程那条上
cut[:, 3], cut[:, 4], cut[:, 5], cut[:, 6] = 0.5, -0.5, 0.5, 0.5
_, _, _, er_cut = score_trajectories_by_ESDF(np.ascontiguousarray(np.stack([cut])),
                                             esdf, pd2, rm2, ORIGIN, RES,
                                             0.0, 0.1, 0.10, 0.18, 0.175, False)
start_rem = float(rm2[cell(2.0, 2.5)])
check("发卡弯横切拿不到超过自身弧长的进展",
      er_cut[0] >= start_rem - 0.25, "剩余 %.2f，起点 %.2f，走了 0.20" % (er_cut[0], start_rem))

print("全部通过" if not fail else "%d 个用例失败" % fail)
sys.exit(1 if fail else 0)
