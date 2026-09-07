import re, sys
M, P = sys.argv[1], sys.argv[2]
rows = []
for ln in open(M, errors="replace"):
    m = re.search(r"\[(\d+\.\d+)\].*poi (\d+)/(\d+): dist_xy=([\d.]+)m .*robot_map=\[([-\d.]+),([-\d.]+)\] poi_map=\[([-\d.]+),([-\d.]+)\]", ln)
    if m:
        rows.append((float(m.group(1)), float(m.group(4)), float(m.group(5)), float(m.group(6)),
                     float(m.group(7)), float(m.group(8))))
# 找一拍内 dist_xy 掉超过 1.0 m 的
print("=== dist_xy 一拍内掉超过 1.0 m（物理上不可能）===")
hits = []
for i in range(1, len(rows)):
    t, d, rx, ry, px, py = rows[i]
    t0, d0, rx0, ry0, px0, py0 = rows[i-1]
    if abs(px-px0) > 1e-6 or abs(py-py0) > 1e-6:
        continue                     # 换目标了，不算
    if d0 - d > 1.0:
        mv = ((rx-rx0)**2 + (ry-ry0)**2) ** 0.5
        hits.append(t)
        print("  t=%.2f  dt=%.2fs   dist_xy %.3f -> %.3f" % (t, t-t0, d0, d))
        print("     robot_map (%.2f,%.2f) -> (%.2f,%.2f)  地图位姿位移 %.2f m  等效速度 %.2f m/s"
              % (rx0, ry0, rx, ry, mv, mv/max(t-t0, 1e-6)))
if not hits:
    print("  无")
# 同一时刻规划节点看到的 VIO 位姿（原始里程计，不经过重定位）
vio = []
for ln in open(P, errors="replace"):
    m = re.search(r"\[(\d+\.\d+)\].*navigating: .*robot=\[([-\d.]+),([-\d.]+)", ln)
    if m:
        vio.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
if not vio:
    for ln in open(P, errors="replace"):
        m = re.search(r"\[(\d+\.\d+)\].*navigating: ([^\n]*)", ln)
        if m:
            print("\n(planning 的 navigating 行样例) " + m.group(2)[:200]); break
else:
    print("\n=== 同期原始 VIO 位姿（不经重定位）===")
    for t in hits:
        near = [v for v in vio if abs(v[0]-t) < 2.2]
        if len(near) >= 2:
            mv = ((near[-1][1]-near[0][1])**2 + (near[-1][2]-near[0][2])**2) ** 0.5
            print("  t=%.2f 附近 %.1fs 内 VIO 位移 %.3f m（地图却跳了几米）"
                  % (t, near[-1][0]-near[0][0], mv))
