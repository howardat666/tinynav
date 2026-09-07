import re, sys
M = sys.argv[1]
rows = []
for ln in open(M, errors="replace"):
    m = re.search(r"\[(\d+\.\d+)\].*poi (\d+)/(\d+): dist_xy=([\d.]+)m .*robot_map=\[([-\d.]+),([-\d.]+)\] poi_map=\[([-\d.]+),([-\d.]+)\]", ln)
    if m:
        rows.append(dict(t=float(m.group(1)), idx=int(m.group(2)), tot=int(m.group(3)),
                         d=float(m.group(4)), rx=float(m.group(5)), ry=float(m.group(6)),
                         px=float(m.group(7)), py=float(m.group(8))))
print("poi dist_xy 采样 %d 条" % len(rows))
if not rows:
    sys.exit(0)
t0 = rows[0]["t"]
# 按 poi 目标分段（poi_map 变了就是换目标了）
segs, cur = [], [rows[0]]
for r in rows[1:]:
    if (abs(r["px"]-cur[-1]["px"]) > 1e-6 or abs(r["py"]-cur[-1]["py"]) > 1e-6):
        segs.append(cur); cur = [r]
    else:
        cur.append(r)
segs.append(cur)
print("共 %d 段（每段一个 POI 目标）\n" % len(segs))
for k, s in enumerate(segs):
    print("--- 第 %d 段  目标 poi_map=[%.2f,%.2f]  时长 %.0f s  采样 %d 条" % (
        k+1, s[0]["px"], s[0]["py"], s[-1]["t"]-s[0]["t"], len(s)))
    print("    dist_xy 起 %.3f  最小 %.3f  末 %.3f   （到达门限 0.40）" % (
        s[0]["d"], min(x["d"] for x in s), s[-1]["d"]))
    # 一拍内 dist_xy 的跳变 vs 车在地图里的位移
    jumps = []
    for i in range(1, len(s)):
        dd = s[i]["d"] - s[i-1]["d"]
        move = ((s[i]["rx"]-s[i-1]["rx"])**2 + (s[i]["ry"]-s[i-1]["ry"])**2) ** 0.5
        dt = s[i]["t"] - s[i-1]["t"]
        if move > 0.25:      # 一拍内地图位姿挪了超过 0.25 m
            jumps.append((s[i]["t"]-t0, dt, move, s[i-1]["d"], s[i]["d"]))
    if jumps:
        print("    🔴 地图位姿一拍内跳 >0.25 m：%d 次" % len(jumps))
        for j in jumps[:6]:
            print("       t+%6.1fs  dt=%.2fs  跳 %.2f m   dist_xy %.3f -> %.3f" % j)
    # 最后 6 条，看它是怎么"到"的
    print("    最后 6 条: " + "  ".join("%.3f" % x["d"] for x in s[-6:]))
