#!/usr/bin/env python3
"""按【线程】拆一个进程的 CPU，看哪一块最贵。用法: thread_cpu.py <进程名> [秒数]"""
import os, sys, time, subprocess
NAME = sys.argv[1]
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
HZ = os.sysconf('SC_CLK_TCK')
pid = subprocess.run(['pgrep', '-x', NAME], capture_output=True, text=True).stdout.split()
if not pid:
    pid = subprocess.run(['pgrep', '-f', NAME], capture_output=True, text=True).stdout.split()
pid = pid[0]
def snap():
    o = {}
    for t in os.listdir(f'/proc/{pid}/task'):
        try:
            with open(f'/proc/{pid}/task/{t}/stat') as f:
                p = f.read().rsplit(')', 1)[1].split()
            with open(f'/proc/{pid}/task/{t}/comm') as f:
                nm = f.read().strip()
            rt = int(p[37]) if len(p) > 37 else 0        # rt_priority
            o[t] = (int(p[11]) + int(p[12]), nm, rt)
        except Exception:
            pass
    return o
a = snap(); time.sleep(DUR); b = snap()
rows = []
for t, (c, nm, rt) in b.items():
    if t in a:
        d = (c - a[t][0]) / HZ / DUR * 100
        if d > 0.5:
            rows.append((d, t, nm, rt))
rows.sort(reverse=True)
print(f"{NAME} pid={pid}  线程 {len(b)} 个，采样 {DUR:.0f}s")
print(f"  {'CPU%':>7s} {'tid':>7s} {'实时优先级':>10s}  线程名")
tot = 0.0
for d, t, nm, rt in rows[:22]:
    tot += d
    print(f"  {d:7.1f} {t:>7s} {(str(rt) if rt else '-'):>10s}  {nm}")
print(f"  {'合计':>7s} {tot:6.1f}%   （只列 >0.5% 的）")
