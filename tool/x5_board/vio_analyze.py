import re, sys, math
P = sys.argv[1]
rows = []
for ln in open(P, errors="replace"):
    m = re.search(r"\[(\d+\.\d+)\].*navigating: .*robot=\[([-\d.]+),([-\d.]+),([-\d.]+)\]", ln)
    if m:
        rows.append(tuple(float(m.group(i)) for i in (1,2,3,4)))
print("VIO 位姿采样 %d 条，时长 %.0f s" % (len(rows), rows[-1][0]-rows[0][0]))
t0 = rows[0][0]
MAXV = 0.25
print("\n=== 位移超过物理极限的跳变（v > 2x max_vx = 0.5 m/s）===")
n = 0
for i in range(1, len(rows)):
    t, x, y, z = rows[i]; tp, xp, yp, zp = rows[i-1]
    dt = t - tp
    d = math.dist((x, y), (xp, yp))
    if dt > 0 and d/dt > 2*MAXV:
        n += 1
        near0 = "  ← 落到原点附近" if math.hypot(x, y) < 0.3 else ""
        print("  t+%6.1fs  dt=%.2fs  跳 %.2f m (%.2f m/s)  (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)%s"
              % (t-t0, dt, d, d/dt, xp, yp, zp, x, y, z, near0))
print("共 %d 次" % n)
# z 归零 = 强烈的重置信号
zz = [r for r in rows if abs(r[3]) < 0.02]
print("\nz 掉到 |z|<0.02 的采样: %d 条（正常 z 约 0.12~0.13）" % len(zz))
if zz:
    print("  时刻: " + "  ".join("t+%.1f" % (r[0]-t0) for r in zz[:12]))
# 采样间隔（一卡一卡）
gaps = [(rows[i][0]-rows[i-1][0], rows[i][0]-t0) for i in range(1, len(rows))]
gaps_s = sorted(g[0] for g in gaps)
print("\n=== navigating 行间隔（规划节点的心跳，正常约 1 s）===")
print("  p50=%.2f  p90=%.2f  max=%.2f s" % (
    gaps_s[len(gaps_s)//2], gaps_s[int(len(gaps_s)*0.9)], gaps_s[-1]))
big = [g for g in gaps if g[0] > 3.0]
print("  超过 3 s 的空窗: %d 次" % len(big))
for g in big[:10]:
    print("     t+%6.1fs  空了 %.1f s" % (g[1], g[0]))
