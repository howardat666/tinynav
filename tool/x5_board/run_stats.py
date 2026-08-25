#!/usr/bin/env python3
"""Per-process CPU/RSS plus load, memory, temperature and BPU, one TSV row a second.

    python3 run_stats.py [period_s]

WHY NOT run_stats.sh
  The shell version resolved each node name with its own pass over /proc, forking a
  `tr` per process per name: 197 processes x 8 names measured 5.5 s per sample on this
  board, so asking for 1 Hz got 0.18 Hz -- and it burned that CPU inside the very
  measurement it was taking. This does one pass, no forks.

WHY NOT ps pcpu
  ps averages over the process lifetime, so a spike two minutes ago is diluted into an
  hour of history. This differences utime+stime across the period.
"""
import os
import sys
import time
from pathlib import Path

# (列名, 在 cmdline 里找的串). 必须带 .py 而不是裸名字：'map_node' 是 'build_map_node'
# 的子串，裸匹配会把同一个建图进程同时算成两个节点 —— 实测两列报出完全相同的 105.9%/379MB。
NODE_MATCH = (
    ('diffcar_control', 'diffcar_control.py'),
    ('wheel_odometry_node', 'wheel_odometry_node.py'),
    ('looper_bridge_node', 'looper_bridge_node.py'),
    ('planning_node', 'planning_node.py'),
    ('map_node', '/map_node.py'),
    ('build_map_node', 'build_map_node.py'),
    ('cmd_vel_control', 'cmd_vel_control.py'),
    ('uvicorn', '-m uvicorn'),
    ('insight_full', 'insight_full'),
)
NODES = tuple(n for n, _ in NODE_MATCH)
LOG_DIR = Path(os.environ.get('TINYNAV_APP_LOG_DIR', '/userdata/x5/logs'))
TICK = os.sysconf('SC_CLK_TCK')
BPU = ('/sys/devices/system/bpu/ratio', '/sys/devices/system/bpu/bpu0/ratio')
# 2026-08-24: 七个进程的 CPU 加起来 597%，而 load 是 14.7 —— 约 200% 不在任何进程名下。
# /proc/stat 的 sys/softirq 能直接说清那是不是内核，而 lo 的字节数说清是不是回环流量
# （实测 17.3 MB/s，深度图 680 kB > 536 kB 的共享内存段，只能走 UDP）。cpu_mhz 是为了
# 分辨"变慢"是负载还是降频：温度中位 92.4 °C，降频线 95。
_STAT = '/proc/stat'
_NETDEV = '/proc/net/dev'
_FREQ = '/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq'


def _read(path, default=''):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def _cpu_jiffies():
    """(user, nice, sys, idle, iowait, irq, softirq) from the aggregate cpu line."""
    line = _read(_STAT).split('\n', 1)[0]
    parts = line.split()
    if parts[:1] != ['cpu'] or len(parts) < 8:
        return None
    return tuple(int(v) for v in parts[1:8])


def _lo_bytes():
    for line in _read(_NETDEV).splitlines():
        if line.strip().startswith('lo:'):
            f = line.split(':', 1)[1].split()
            return int(f[0]), int(f[8])
    return None


def _scan():
    """{node: (pid, jiffies, rss_mb)} from a single pass over /proc."""
    found = {}
    for entry in os.scandir('/proc'):
        if not entry.name.isdigit():
            continue
        cmd = _read(f'/proc/{entry.name}/cmdline').replace('\0', ' ')
        if not cmd:
            continue
        # insight_full is not python; everything else must be, so this sampler's own
        # command line -- which contains every name in NODES -- cannot match itself.
        is_py = cmd.startswith('python3') or cmd.startswith('/usr/bin/python3')
        for n, needle in NODE_MATCH:
            if n in found or needle not in cmd:
                continue
            if (n == 'insight_full') == is_py:
                continue
            stat = _read(f'/proc/{entry.name}/stat').rsplit(') ', 1)
            if len(stat) != 2:
                continue
            f = stat[1].split()
            # utime and stime are fields 14 and 15 of /proc/pid/stat, which are
            # index 11 and 12 once the comm field and its parenthesis are split off.
            jif = int(f[11]) + int(f[12])
            rss = 0
            for line in _read(f'/proc/{entry.name}/status').splitlines():
                if line.startswith('VmRSS:'):
                    rss = int(line.split()[1]) // 1024
                    break
            found[n] = (entry.name, jif, rss)
    return found


def main():
    period = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    out = LOG_DIR / f"run_stats_{time.strftime('%Y%m%d_%H%M%S')}.tsv"
    cols = ['time', 'temp_c', 'load', 'avail_mb', 'bpu_pct', 'cpu_mhz',
            'sys_pct', 'softirq_pct', 'iowait_pct', 'user_pct', 'lo_mbs']
    for n in NODES:
        cols += [f'{n}_cpu', f'{n}_rss']
    with out.open('w') as f:
        f.write('\t'.join(cols) + '\n')
    print(f'recording every {period}s -> {out}', flush=True)

    prev, prev_t = {}, None
    prev_cpu, prev_lo = _cpu_jiffies(), _lo_bytes()
    while True:
        now = time.time()
        temp = int(_read('/sys/class/thermal/thermal_zone0/temp', '0') or 0) / 1000.0
        load = _read('/proc/loadavg', '0').split()[0]
        avail = 0
        for line in _read('/proc/meminfo').splitlines():
            if line.startswith('MemAvailable:'):
                avail = int(line.split()[1]) // 1024
                break
        bpu = next((v.strip() for v in (_read(p).strip() for p in BPU) if v), '-')

        cur_cpu, cur_lo = _cpu_jiffies(), _lo_bytes()
        shares = ['-'] * 4
        if cur_cpu and prev_cpu:
            d = [c - p for c, p in zip(cur_cpu, prev_cpu)]
            tot = sum(d)
            if tot > 0:
                # 全机百分比（100% = 全部 8 核），和进程列的口径不同：那些是单核百分比。
                shares = [f'{100.0 * d[i] / tot:.1f}' for i in (2, 6, 4, 0)]
        lo_mbs = '-'
        if cur_lo and prev_lo and prev_t and now > prev_t:
            lo_mbs = f'{(cur_lo[0] - prev_lo[0]) / (now - prev_t) / 1048576:.2f}'
        mhz = _read(_FREQ).strip()
        mhz = str(int(mhz) // 1000) if mhz.isdigit() else '-'
        prev_cpu, prev_lo = cur_cpu, cur_lo

        cur = _scan()
        row = [time.strftime('%H:%M:%S'), f'{temp:.1f}', load, str(avail), bpu, mhz] + shares + [lo_mbs]
        for n in NODES:
            if n not in cur:
                row += ['-', '-']
                continue
            pid, jif, rss = cur[n]
            was = prev.get(n)
            # Same pid required: a restarted node's counter starts over, and
            # differencing across that shows up as a huge negative CPU.
            if was and was[0] == pid and prev_t and now > prev_t:
                row.append(f'{100.0 * (jif - was[1]) / TICK / (now - prev_t):.1f}')
            else:
                row.append('-')
            row.append(str(rss))
        with out.open('a') as f:
            f.write('\t'.join(row) + '\n')
        prev, prev_t = cur, now
        time.sleep(max(0.0, period - (time.time() - now)))


if __name__ == '__main__':
    main()
