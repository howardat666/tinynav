import re, sys, statistics as st
M = sys.argv[1]
# 纯日志、零重建：把每次拟合的 |d_yaw|（结果）和它前面那个解的质量（原因）配对
ev, pend = [], None
for ln in open(M, errors='replace'):
    m = re.search(r'reloc pose: .*cand_spread=([\d.]+)m cand_span=([\d.]+)s ratio=([\d.]+)', ln)
    if m:
        pend = (float(m.group(1)), float(m.group(3))); continue
    m = re.search(r'map->odom: .*d_t=([\d.]+)m d_yaw=([-+\d.]+)deg constraints=(\d+) clean=(\d+)/', ln)
    if m and pend:
        ev.append(dict(spread=pend[0], ratio=pend[1], dt=float(m.group(1)),
                       dyaw=abs(float(m.group(2))), n=int(m.group(3)), clean=int(m.group(4))))
        pend = None
print("拟合事件 %d 个（每个带触发它的那条约束的质量）\n" % len(ev))
def q(v,f):
    v=sorted(v); return v[min(int(len(v)*f), len(v)-1)]
def show(label, sub, base=None):
    if len(sub) < 8:
        print("  %-24s n=%-4d 样本太少" % (label, len(sub))); return None
    d = [e['dyaw'] for e in sub]
    m50, m80, m90, mx = q(d,.5), q(d,.8), q(d,.9), max(d)
    tag = ""
    if base: tag = "   相对不过滤 p80 %+.0f%%" % (100*(m80-base)/base)
    print("  %-24s n=%-4d |d_yaw| p50=%4.1f° p80=%5.1f° p90=%5.1f° max=%6.1f°%s" % (
        label, len(sub), m50, m80, m90, mx, tag))
    return m80
print("=== 结果指标：这次拟合的朝向抖了多少（越小越好）===")
base = show("不过滤（全部）", ev)
print()
print("--- 只保留内点率 >= X 的那些约束触发的拟合 ---")
for thr in (0.30, 0.35, 0.40, 0.45, 0.50, 0.60):
    show("内点率 >= %.2f" % thr, [e for e in ev if e['ratio'] >= thr], base)
print()
print("--- 只保留散布 <= X 的 ---")
for thr in (3.0, 2.0, 1.0, 0.5):
    show("散布 <= %.1f m" % thr, [e for e in ev if e['spread'] <= thr], base)
print()
print("--- clean（散布<=0.5 且 内点率>=0.70，代码里已有的定义）---")
show("clean 计数 >= 1", [e for e in ev if e['clean'] >= 1], base)
show("clean 计数 >= 5", [e for e in ev if e['clean'] >= 5], base)
print()
print("--- 对照：约束数（我已加的引导门限走的就是这条）---")
for lo in (10, 20, 40):
    show("约束数 >= %d" % lo, [e for e in ev if e['n'] >= lo], base)
print()
print("=== 反向看：抖得最厉害的那些拟合，触发它们的约束长什么样 ===")
ev.sort(key=lambda e: -e['dyaw'])
worst, rest = ev[:20], ev[20:]
for nm, g in (("最抖的 20 次", worst), ("其余", rest)):
    print("  %-12s 内点率 p50=%.2f p80=%.2f | 散布 p50=%.2f p80=%.2f | 约束数 p50=%d" % (
        nm, q([e['ratio'] for e in g],.5), q([e['ratio'] for e in g],.8),
        q([e['spread'] for e in g],.5), q([e['spread'] for e in g],.8),
        q([e['n'] for e in g],.5)))
