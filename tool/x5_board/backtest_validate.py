import re, sys, math, statistics as st
M = sys.argv[1]
# 把「我重建的单解变换」和「日志里真实拟合出的 map->odom」按时间对齐，检验重建是否可信
sols, fits, pend = [], [], None
for ln in open(M, errors='replace'):
    t = re.search(r'\[(\d+\.\d+)\]', ln); t = float(t.group(1)) if t else None
    m = re.search(r'reloc pose: t=(\d+) cam_map=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] yaw_map=([-+\d.]+)deg', ln)
    if m:
        pend = dict(t=t, ts=int(m.group(1)), mx=float(m.group(2)), my=float(m.group(3)), myaw=float(m.group(5)))
        continue
    m = re.search(r'reloc odom: t=(\d+) cam_odom=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] yaw_odom=([-+\d.]+)deg', ln)
    if m and pend and int(m.group(1)) == pend['ts']:
        pend['tyaw'] = (float(m.group(5)) - pend['myaw'] + 180) % 360 - 180
        sols.append(pend); pend = None
        continue
    m = re.search(r'map->odom: t=\[([-+\d.]+),([-+\d.]+),([-+\d.]+)\] yaw=([-+\d.]+)deg', ln)
    if m:
        fits.append((t, float(m.group(4))))
print("单解重建 %d 个，日志拟合 %d 次" % (len(sols), len(fits)))
# 对每次拟合，取它之前最近 10 个单解的中位，和拟合值比
def wd(a,b): return (a-b+180)%360-180
err = []
for ft, fy in fits:
    win = [s['tyaw'] for s in sols if s['t'] is not None and s['t'] <= ft][-10:]
    if len(win) >= 3:
        err.append(abs(wd(st.median(win), fy)))
if err:
    err.sort()
    print("\n『我重建的单解中位』 vs 『日志里真实的 map->odom yaw』:")
    print("  差值 p50=%.1f°  p90=%.1f°  max=%.1f°  (n=%d)" % (
        err[len(err)//2], err[int(len(err)*.9)], err[-1], len(err)))
    print("  → 差值小说明重建可信；差值大说明我的二维重建不能代表真实的三维变换")
# 单解变换 yaw 随时间的走势（看是不是真在漂）
if sols:
    t0 = sols[0]['t']
    seg = [(s['t']-t0, s['tyaw']) for s in sols]
    print("\n单解变换 yaw 的时间走势（每 20 个取一个）:")
    print("  " + "  ".join("t+%.0fs:%+.0f°" % seg[i] for i in range(0, len(seg), 20)))
if fits:
    t0 = fits[0][0]
    print("\n日志里真实 map->odom yaw 的走势（每 20 次取一个）:")
    print("  " + "  ".join("t+%.0fs:%+.0f°" % (fits[i][0]-t0, fits[i][1]) for i in range(0, len(fits), 20)))
