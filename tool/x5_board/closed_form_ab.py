#!/usr/bin/env python3
"""Ceres vs 闭式解的配对比较。零回放、零近似：板上每次拟合本来就把两者的残差
并排打进日志，而且是在【同一批约束】上算的，所以这是天然的配对实验。

判据说明：resid = 解把每条观测投回去的偏差中位数。两个求解器最小化的目标不同
（Ceres 的平移残差被当前旋转估计旋转过，且平移米和旋转弧度共用同一个权重），
所以残差小不自动等于"更对"，但在同一批约束上更贴近观测中心是硬事实。
"""
import re, sys, statistics as st

RE = re.compile(r'map->odom fit: solver resid t_p50=([\d.]+)m r_p50=([\d.]+)deg \| '
                r'recent(\d+) resid t=([\d.]+)m r=([\d.]+)deg \| '
                r'closed-form resid t_p50=([\d.]+)m r_p50=([\d.]+)deg \| gap=([\d.]+)m')


def q(v, f):
    v = sorted(v)
    return v[min(int(len(v) * f), len(v) - 1)] if v else float('nan')


tot = {k: [] for k in ("st", "sr", "ct", "cr", "gap")}
print(f"{'日志':30s} {'配对':>5} {'Ceres t':>8} {'闭式 t':>8} {'Ceres r':>8} {'闭式 r':>8} "
      f"{'闭式更优':>9} {'gap p90':>8}")
for path in sys.argv[1:]:
    rows = [m.groups() for m in (RE.search(l) for l in open(path, errors='replace')) if m]
    # 头几次拟合只有 1~2 条约束，两个解必然相同，会稀释掉真实差异
    rows = [r for r in rows if int(r[2]) >= 5]
    if len(rows) < 20:
        print(f"{path.split('/')[-1]:30s} 样本太少({len(rows)})")
        continue
    stt = [float(r[0]) for r in rows]; srr = [float(r[1]) for r in rows]
    ctt = [float(r[5]) for r in rows]; crr = [float(r[6]) for r in rows]
    gap = [float(r[7]) for r in rows]
    win = sum(1 for a, b in zip(stt, ctt) if b < a - 1e-9)
    tie = sum(1 for a, b in zip(stt, ctt) if abs(b - a) <= 1e-9)
    for k, v in (("st", stt), ("sr", srr), ("ct", ctt), ("cr", crr), ("gap", gap)):
        tot[k] += v
    print(f"{path.split('/')[-1]:30s} {len(rows):5d} {st.median(stt):7.3f}m {st.median(ctt):7.3f}m "
          f"{st.median(srr):7.2f}° {st.median(crr):7.2f}° "
          f"{100*win/len(rows):8.1f}% {q(gap,.9):7.3f}m   (打平 {100*tie/len(rows):.0f}%)")

n = len(tot["st"])
if n:
    win = sum(1 for a, b in zip(tot["st"], tot["ct"]) if b < a - 1e-9)
    d = [b - a for a, b in zip(tot["st"], tot["ct"])]
    print(f"\n合计 {n} 次配对拟合")
    print(f"  平移残差中位: Ceres {st.median(tot['st']):.3f}m -> 闭式 {st.median(tot['ct']):.3f}m")
    print(f"  旋转残差中位: Ceres {st.median(tot['sr']):.2f}° -> 闭式 {st.median(tot['cr']):.2f}°")
    print(f"  闭式更优 {100*win/n:.1f}% 的次数；逐次差值中位 {st.median(d):+.4f}m "
          f"(p10={q(d,.1):+.3f} p90={q(d,.9):+.3f})")
    print(f"  两解相距 gap: p50={q(tot['gap'],.5):.3f}m p90={q(tot['gap'],.9):.3f}m "
          f"max={max(tot['gap']):.3f}m")
