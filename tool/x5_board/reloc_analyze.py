import re, sys
M = sys.argv[1]
pnp, poses, fits, rejects = [], [], [], []
for ln in open(M, errors='replace'):
    t = re.search(r'\[(\d+\.\d+)\]', ln)
    t = float(t.group(1)) if t else None
    m = re.search(r'PnP quality: inliers=(\d+)/(\d+) \((\d+)%\).*reproj px p50=([\d.]+) p90=([\d.]+) max=([\d.]+)', ln)
    if m:
        pnp.append((t, int(m.group(1)), int(m.group(2)), int(m.group(3)),
                    float(m.group(4)), float(m.group(5)), float(m.group(6))))
    m = re.search(r'reloc pose: t=(\d+) cam_map=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] yaw_map=([-+\d.]+)deg ref_map=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] cand_spread=([\d.]+)m cand_span=([\d.]+)s ratio=([\d.]+)', ln)
    if m:
        poses.append(dict(t=t, x=float(m.group(2)), y=float(m.group(3)), z=float(m.group(4)),
                          yaw=float(m.group(5)), rx=float(m.group(6)), ry=float(m.group(7)),
                          spread=float(m.group(9)), span=float(m.group(10)), ratio=float(m.group(11))))
    m = re.search(r'map->odom: t=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] yaw=([-+\d.]+)deg .*d_t=([\d.]+)m d_yaw=([-+\d.]+)deg constraints=(\d+) clean=(\d+)/(\d+) spread_p50=([\d.]+)m ratio_p50=([\d.]+)', ln)
    if m:
        fits.append(dict(t=t, x=float(m.group(1)), y=float(m.group(2)), yaw=float(m.group(4)),
                         dt=float(m.group(5)), dyaw=float(m.group(6)), n=int(m.group(7)),
                         clean=int(m.group(8)), tot=int(m.group(9)),
                         sp50=float(m.group(10)), r50=float(m.group(11))))
    if 'reloc rejected' in ln or 'odometry gate' in ln or '里程计一致性' in ln:
        rejects.append(ln.strip()[:180])

print("PnP 记录 %d 条，接受的重定位解 %d 个，map->odom 拟合 %d 次，门限拒绝日志 %d 条\n" % (
    len(pnp), len(poses), len(fits), len(rejects)))
if pnp:
    t0 = pnp[0][0]
    print("=== 前 14 次 PnP（门限：内点>=12 且 内点率>=25%）===")
    print("  t+     内点/总数   率   重投影p50  判定")
    for r in pnp[:14]:
        ok = r[1] >= 12 and r[3] >= 25
        print("  %5.1fs  %3d/%-4d  %3d%%  %6.2f px  %s" % (
            r[0]-t0, r[1], r[2], r[3], r[4], '通过' if ok else '🔴 拒'))
    ins = sorted(r[1] for r in pnp); rat = sorted(r[3] for r in pnp)
    print("\n  全程 内点数 p10=%d p50=%d p90=%d   内点率 p10=%d%% p50=%d%% p90=%d%%" % (
        ins[len(ins)//10], ins[len(ins)//2], ins[int(len(ins)*.9)],
        rat[len(rat)//10], rat[len(rat)//2], rat[int(len(rat)*.9)]))
    print("  被拒次数: 内点<12 → %d ；内点率<25%% → %d ；两者都不满足 → %d" % (
        sum(1 for r in pnp if r[1] < 12), sum(1 for r in pnp if r[3] < 25),
        sum(1 for r in pnp if r[1] < 12 and r[3] < 25)))

if poses:
    t0 = poses[0]['t']
    print("\n=== 接受的解：前 12 个（解与它拿到的候选差多少 = 解是不是被候选牵着走）===")
    print("  t+      解 cam_map        候选 ref_map      解-候选  候选散布  时间跨度  内点率")
    for p in poses[:12]:
        d = ((p['x']-p['rx'])**2 + (p['y']-p['ry'])**2) ** 0.5
        print("  %6.1fs (%+.2f,%+.2f)  (%+.2f,%+.2f)   %.2fm   %.2fm   %5.1fs   %.2f" % (
            p['t']-t0, p['x'], p['y'], p['rx'], p['ry'], d, p['spread'], p['span'], p['ratio']))
    print("\n  === 相邻两个解之间的跳变 ===")
    big = 0
    for i in range(1, len(poses)):
        a, b = poses[i-1], poses[i]
        d = ((b['x']-a['x'])**2 + (b['y']-a['y'])**2) ** 0.5
        dt = b['t'] - a['t']
        if d > 1.2:
            big += 1
            if big <= 8:
                print("  🔴 t+%6.1fs  跳 %.2fm / %.1fs  (%+.2f,%+.2f)->(%+.2f,%+.2f)  内点率 %.2f→%.2f  散布 %.2f→%.2f" % (
                    b['t']-t0, d, dt, a['x'], a['y'], b['x'], b['y'], a['ratio'], b['ratio'], a['spread'], b['spread']))
    print("  超过里程计门限 1.2m 的相邻跳变: %d / %d" % (big, len(poses)-1))
    sp = sorted(p['spread'] for p in poses); rr = sorted(p['ratio'] for p in poses)
    print("\n  接受解的 候选散布 p50=%.2fm p90=%.2fm max=%.2fm" % (
        sp[len(sp)//2], sp[int(len(sp)*.9)], sp[-1]))
    print("  接受解的 内点率   p10=%.2f p50=%.2f p90=%.2f" % (
        rr[len(rr)//10], rr[len(rr)//2], rr[int(len(rr)*.9)]))

if fits:
    t0 = fits[0]['t']
    print("\n=== map->odom 的朝向（错的重定位会让它阶跃）===")
    print("  前 10 次: " + "  ".join("%.0f°" % f['yaw'] for f in fits[:10]))
    jumps = [(fits[i]['t']-t0, fits[i-1]['yaw'], fits[i]['yaw'])
             for i in range(1, len(fits))
             if abs((fits[i]['yaw']-fits[i-1]['yaw']+180) % 360 - 180) > 15]
    print("  朝向一拍变化 >15° 的次数: %d" % len(jumps))
    for j in jumps[:8]:
        print("     t+%6.1fs  %.0f° -> %.0f°" % j)
    dts = sorted(f['dt'] for f in fits)
    print("  拟合残差 d_t p50=%.3fm p90=%.3fm" % (dts[len(dts)//2], dts[int(len(dts)*.9)]))
if rejects:
    print("\n=== 门限拒绝（前 6 条）===")
    for r in rejects[:6]:
        print("  " + r)
