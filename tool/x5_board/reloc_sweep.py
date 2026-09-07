import re, sys
M = sys.argv[1]
P = []
for ln in open(M, errors='replace'):
    m = re.search(r'reloc pose: t=(\d+) cam_map=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\].*cand_spread=([\d.]+)m cand_span=([\d.]+)s ratio=([\d.]+)', ln)
    if m:
        P.append(dict(x=float(m.group(2)), y=float(m.group(3)),
                      spread=float(m.group(5)), ratio=float(m.group(7))))
# 标签：与上一个解的位置跳变 >1.2 m 就算"坏"（和现有里程计门限同一判据）
for i, p in enumerate(P):
    p['bad'] = (i > 0 and ((p['x']-P[i-1]['x'])**2 + (p['y']-P[i-1]['y'])**2) ** 0.5 > 1.2)
bad = [p for p in P if p['bad']]
good = [p for p in P if not p['bad']]
print("解 %d 个：跳变(坏) %d，正常 %d\n" % (len(P), len(bad), len(good)))

def dist(v, name):
    v = sorted(v)
    q = lambda f: v[min(int(len(v)*f), len(v)-1)]
    return "%s p10=%.2f p50=%.2f p90=%.2f max=%.2f" % (name, q(.1), q(.5), q(.9), v[-1])
print("  坏解 " + dist([p['spread'] for p in bad], '候选散布'))
print("  好解 " + dist([p['spread'] for p in good], '候选散布'))
print("  坏解 " + dist([p['ratio'] for p in bad], '内点率  '))
print("  好解 " + dist([p['ratio'] for p in good], '内点率  '))

print("\n=== 候选散布门限扫描（散布 > 门限就拒）===")
print("  门限     拦下坏解        误伤好解")
for thr in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0):
    kb = sum(1 for p in bad if p['spread'] > thr)
    kg = sum(1 for p in good if p['spread'] > thr)
    print("  %4.1f m   %3d/%-3d (%3.0f%%)   %3d/%-3d (%3.0f%%)" % (
        thr, kb, len(bad), 100*kb/max(len(bad),1), kg, len(good), 100*kg/max(len(good),1)))

print("\n=== 内点率门限扫描（低于门限就拒）===")
print("  门限     拦下坏解        误伤好解")
for thr in (0.25, 0.28, 0.30, 0.32, 0.35, 0.40, 0.45):
    kb = sum(1 for p in bad if p['ratio'] < thr)
    kg = sum(1 for p in good if p['ratio'] < thr)
    print("  %.2f    %3d/%-3d (%3.0f%%)   %3d/%-3d (%3.0f%%)" % (
        thr, kb, len(bad), 100*kb/max(len(bad),1), kg, len(good), 100*kg/max(len(good),1)))

print("\n=== 两个一起（散布>A 或 内点率<B 就拒）===")
print("   散布A  内点率B   拦下坏解        误伤好解       剩余可用解")
for a in (2.0, 3.0, 4.0):
    for b in (0.25, 0.30, 0.35):
        kb = sum(1 for p in bad if p['spread'] > a or p['ratio'] < b)
        kg = sum(1 for p in good if p['spread'] > a or p['ratio'] < b)
        print("   %4.1f   %.2f     %3d/%-3d (%3.0f%%)   %3d/%-3d (%3.0f%%)   %d" % (
            a, b, kb, len(bad), 100*kb/max(len(bad),1), kg, len(good),
            100*kg/max(len(good),1), len(good)-kg))
