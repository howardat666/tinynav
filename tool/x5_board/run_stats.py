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


def _read(path, default=''):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


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
    cols = ['time', 'temp_c', 'load', 'avail_mb', 'bpu_pct']
    for n in NODES:
        cols += [f'{n}_cpu', f'{n}_rss']
    with out.open('w') as f:
        f.write('\t'.join(cols) + '\n')
    print(f'recording every {period}s -> {out}', flush=True)

    prev, prev_t = {}, None
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

        cur = _scan()
        row = [time.strftime('%H:%M:%S'), f'{temp:.1f}', load, str(avail), bpu]
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
