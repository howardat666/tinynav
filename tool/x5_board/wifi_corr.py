import re, sys, collections
L = sys.argv[1]
rows = []
for ln in open(L, errors='replace'):
    m = re.match(r'(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d) up=(\d+) load=([\d.]+).*?link=(\d).*?fa=(\d+)', ln)
    if m:
        rows.append(dict(t=m.group(1), up=int(m.group(2)), load=float(m.group(3)),
                         link=int(m.group(4)), fa=int(m.group(5))))
print("board_health 采样 %d 条" % len(rows))
# link 1->0 的下降沿
edges = [i for i in range(1, len(rows)) if rows[i-1]['link'] == 1 and rows[i]['link'] == 0]
print("link 从 1 掉到 0 的次数: %d\n" % len(edges))
loads_at_drop, fa_before, fa_at = [], [], []
print("  时刻                 断前load  断前fa  断时fa   fa倍数")
for i in edges:
    pre = rows[max(0, i-6):i]
    fb = sorted(r['fa'] for r in pre)
    med = fb[len(fb)//2] if fb else 0
    loads_at_drop.append(rows[i-1]['load'])
    fa_before.append(med); fa_at.append(rows[i]['fa'])
    print("  %s  %7.2f  %6d  %6d   %5.1fx" % (
        rows[i]['t'], rows[i-1]['load'], med, rows[i]['fa'],
        rows[i]['fa']/max(med, 1)))
if loads_at_drop:
    ls = sorted(loads_at_drop)
    print("\n  掉线时 load: 中位 %.2f  最小 %.2f  最大 %.2f" % (
        ls[len(ls)//2], ls[0], ls[-1]))
al = sorted(r['load'] for r in rows)
print("  全体样本 load: 中位 %.2f  p90 %.2f  最大 %.2f" % (
    al[len(al)//2], al[int(len(al)*0.9)], al[-1]))
lo = [r for r in rows if r['load'] < 3.0]
hi = [r for r in rows if r['load'] > 10.0]
def rate(sub):
    if len(sub) < 2: return float('nan'), 0
    e = sum(1 for i in range(1, len(sub)) if sub[i-1]['link']==1 and sub[i]['link']==0)
    return e, len(sub)
print("\n  load<3 的样本里断链下降沿: %d / %d" % rate(lo))
print("  load>10 的样本里断链下降沿: %d / %d" % rate(hi))
fs = sorted(r['fa'] for r in rows if r['link'] == 1)
print("\n  link 正常时 fa: 中位 %d  p90 %d  p99 %d" % (
    fs[len(fs)//2], fs[int(len(fs)*0.9)], fs[int(len(fs)*0.99)]))
