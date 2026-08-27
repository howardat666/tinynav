#!/usr/bin/env python3
"""验证梯度脱困的符号：障碍在前必须倒车，障碍在后必须前进。
用解析 ESDF（到一面墙的距离），不依赖 scipy。
算术逐字抄自 planning_node 的 _esdf_gradient / _worst_footprint_point。
"""
import math

import numpy as np

RES = 0.1
ORIGIN = np.array([-5.0, -5.0, -0.3])
SHAPE = (100, 100)
FL, RL, HW = 0.10, 0.18, 0.175
GRAD_MIN = 0.30
ESC = 0.08


def build_esdf(x_wall):
    """墙在 x=x_wall，ESDF = |x - x_wall|。索引 [i,j] -> x=i, y=j（与 planning_node 一致）。"""
    xs = ORIGIN[0] + (np.arange(SHAPE[0]) + 0.5) * RES
    return np.abs(xs - x_wall)[:, None] * np.ones((1, SHAPE[1]))


def esdf_at(esdf, p):
    i = int((p[0] - ORIGIN[0]) / RES)
    j = int((p[1] - ORIGIN[1]) / RES)
    if 0 <= i < esdf.shape[0] and 0 <= j < esdf.shape[1]:
        return float(esdf[i, j])
    return float('nan')


def worst_point(esdf, init_p, fwd):
    left = (-fwd[1], fwd[0])
    n_a = max(2, int(math.ceil((FL + RL) / RES)) + 1)
    n_c = max(2, int(math.ceil(2.0 * HW / RES)) + 1)
    best, best_d = None, float('inf')
    for ia in range(n_a):
        oa = -RL + (FL + RL) * ia / (n_a - 1)
        for ic in range(n_c):
            oc = -HW + 2.0 * HW * ic / (n_c - 1)
            px = init_p[0] + fwd[0] * oa + left[0] * oc
            py = init_p[1] + fwd[1] * oa + left[1] * oc
            d = esdf_at(esdf, (px, py))
            if not math.isnan(d) and d < best_d:
                best_d, best = d, (px, py)
    return best, best_d


def grad(esdf, p):
    i = int((p[0] - ORIGIN[0]) / RES)
    j = int((p[1] - ORIGIN[1]) / RES)
    if not (1 <= i < esdf.shape[0] - 1 and 1 <= j < esdf.shape[1] - 1):
        return None, None
    gx = (float(esdf[i + 1, j]) - float(esdf[i - 1, j])) / (2.0 * RES)
    gy = (float(esdf[i, j + 1]) - float(esdf[i, j - 1])) / (2.0 * RES)
    n = math.hypot(gx, gy)
    return (gx / n, gy / n) if n >= 1e-6 else (None, None)


def run(name, x_wall, fwd, expect):
    esdf = build_esdf(x_wall)
    p = np.array([0.0, 0.0, 0.0])
    pt, d = worst_point(esdf, p, fwd)
    gx, gy = grad(esdf, pt)
    if gx is None:
        print("  %-28s 梯度取不到 -> 返回 None（保持静止）" % name)
        return expect == "none"
    s = gx * fwd[0] + gy * fwd[1]
    v = ESC if s > GRAD_MIN else -ESC
    got = "forward" if v > 0 else "reverse"
    ok = got == expect
    print("  %-28s 最深点=(%+.2f,%+.2f) esdf=%.2f  上坡=(%+.2f,%+.2f) s=%+.2f -> %s  %s"
          % (name, pt[0], pt[1], d, gx, gy, s, got, "OK" if ok else "❌ 期望 " + expect))
    return ok


print("=== 车头朝 +x ===")
allok = True
allok &= run("墙在前方 x=+0.25", 0.25, (1.0, 0.0), "reverse")
allok &= run("墙在后方 x=-0.35", -0.35, (1.0, 0.0), "forward")
print("=== 车头朝 -x（镜像，防止把 fwd 写死）===")
allok &= run("墙在前方 x=-0.25", -0.25, (-1.0, 0.0), "reverse")
allok &= run("墙在后方 x=+0.35", 0.35, (-1.0, 0.0), "forward")
print("=== 车头朝 +y（防止 x/y 索引写反）===")
allok &= run("墙在侧方 x=+0.25", 0.25, (0.0, 1.0), "reverse")
print()
print("全部通过" if allok else "🔴 有失败项")
