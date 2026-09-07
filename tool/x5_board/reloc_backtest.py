import re, sys, math, statistics as st
M = sys.argv[1]
# 把 reloc pose 和紧跟其后的 reloc odom 配成对，得到"这一个解观测到的 map->odom"
sols, pend = [], None
for ln in open(M, errors='replace'):
    m = re.search(r'reloc pose: t=(\d+) cam_map=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] yaw_map=([-+\d.]+)deg .*cand_spread=([\d.]+)m cand_span=([\d.]+)s ratio=([\d.]+)', ln)
    if m:
        pend = dict(ts=int(m.group(1)), mx=float(m.group(2)), my=float(m.group(3)),
                    myaw=float(m.group(5)), spread=float(m.group(6)), ratio=float(m.group(8)))
        continue
    m = re.search(r'reloc odom: t=(\d+) cam_odom=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] yaw_odom=([-+\d.]+)deg', ln)
    if m and pend and int(m.group(1)) == pend['ts']:
        pend.update(ox=float(m.group(2)), oy=float(m.group(3)), oyaw=float(m.group(5)))
        # 观测到的 map->odom：yaw 之差，平移 t = p_odom - R(dyaw)*p_map
        d = math.radians(pend['oyaw'] - pend['myaw'])
        c, s_ = math.cos(d), math.sin(d)
        pend['tyaw'] = (pend['oyaw'] - pend['myaw'] + 180) % 360 - 180
        pend['tx'] = pend['ox'] - (c*pend['mx'] - s_*pend['my'])
        pend['ty'] = pend['oy'] - (s_*pend['mx'] + c*pend['my'])
        sols.append(pend); pend = None
print("配成对的解 %d 个（每个都带它自己观测到的 map->odom）\n" % len(sols))

def q(v, f):
    v = sorted(v); return v[min(int(len(v)*f), len(v)-1)]
def wrapdiff(a, b): return (a-b+180) % 360 - 180

# 用全体解的中位变换当"真值"参照（大多数解是对的，中位数抗离群）
ref_yaw = st.median(s_['tyaw'] for s_ in sols)
ref_x = st.median(s_['tx'] for s_ in sols)
ref_y = st.median(s_['ty'] for s_ in sols)
print("参照（全体中位）map->odom: yaw=%+.1f°  t=(%+.2f,%+.2f)\n" % (ref_yaw, ref_x, ref_y))
for s_ in sols:
    s_['eyaw'] = abs(wrapdiff(s_['tyaw'], ref_yaw))
    s_['et'] = math.hypot(s_['tx']-ref_x, s_['ty']-ref_y)
    s_['bad'] = s_['eyaw'] > 15.0 or s_['et'] > 1.0     # 明显偏离共识 = 坏解

bad = [x for x in sols if x['bad']]; good = [x for x in sols if not x['bad']]
print("按【偏离共识】重新标注：坏解 %d，好解 %d\n" % (len(bad), len(good)))
print("=== 完整分位（不只 p50）===")
for name, key in (("候选散布 m", 'spread'), ("内点率   ", 'ratio')):
    for lbl, grp in (("坏解", bad), ("好解", good)):
        v = [x[key] for x in grp]
        print("  %s %s  p10=%.2f p20=%.2f p50=%.2f p80=%.2f p90=%.2f" % (
            name, lbl, q(v,.1), q(v,.2), q(v,.5), q(v,.8), q(v,.9)))
    print()

span = (sols[-1]['ts'] - sols[0]['ts']) / 1e9
print("=== 回测：过滤之后剩下的解，其 map->odom 有多一致 ===")
print("  说明：yaw离散 = 幸存解的变换朝向的四分位距，越小越好；坏解占比 = 幸存里还有多少坏的")
print("  %-26s %5s %6s %8s %9s %8s" % ("过滤条件", "幸存", "Hz", "yaw离散", "坏解占比", "最大偏差"))
def report(label, keep):
    if len(keep) < 5:
        print("  %-26s %5d  样本太少" % (label, len(keep))); return
    ys = sorted(abs(wrapdiff(k['tyaw'], ref_yaw)) for k in keep)
    iqr = q([k['tyaw'] for k in keep], .75) - q([k['tyaw'] for k in keep], .25)
    nb = sum(1 for k in keep if k['bad'])
    print("  %-26s %5d %6.2f %7.1f° %8.1f%% %7.1f°" % (
        label, len(keep), len(keep)/span, iqr, 100*nb/len(keep), ys[-1]))
report("不过滤（现状）", sols)
for thr in (0.30, 0.32, 0.35, 0.40, 0.45, 0.50):
    report("内点率 >= %.2f" % thr, [x for x in sols if x['ratio'] >= thr])
for thr in (3.0, 2.0, 1.0, 0.5):
    report("散布 <= %.1f m" % thr, [x for x in sols if x['spread'] <= thr])
for r, sp in ((0.35, 2.0), (0.40, 2.0), (0.40, 1.0), (0.45, 1.0)):
    report("内点率>=%.2f 且 散布<=%.1f" % (r, sp),
           [x for x in sols if x['ratio'] >= r and x['spread'] <= sp])
