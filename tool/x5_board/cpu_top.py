import os, time, re
def snap():
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                p = f.read().rsplit(')', 1)[1].split()
            ut, st = int(p[11]), int(p[12])
            with open(f'/proc/{pid}/cmdline','rb') as f:
                cl = f.read().replace(b'\0', b' ').decode('utf-8','replace').strip()
            with open(f'/proc/{pid}/statm') as f:
                rss = int(f.read().split()[1]) * 4096 // 1024 // 1024
            out[pid] = (ut+st, cl or f'[{pid}]', rss)
        except Exception:
            pass
    return out
HZ = os.sysconf('SC_CLK_TCK')
a = snap(); time.sleep(5.0); b = snap()
rows = []
for pid, (t1, cl, rss) in b.items():
    if pid in a:
        d = (t1 - a[pid][0]) / HZ / 5.0 * 100
        if d > 1.0:
            rows.append((d, rss, cl))
rows.sort(reverse=True)
ncpu = os.cpu_count()
print(f"{ncpu} 核。5 秒采样的瞬时 CPU（%，100=一个核满）:")
tot = 0
for d, rss, cl in rows[:16]:
    tot += d
    name = re.sub(r'^.*?(python3?|/)', r'\1', cl)
    print(f"  {d:6.1f}%  {rss:5d} MB  {cl[:88]}")
print(f"  ---- 合计 {tot:.0f}% / {ncpu*100}%  ({tot/ncpu:.0f}% 平均核占用)")
print(open('/proc/loadavg').read().strip())
