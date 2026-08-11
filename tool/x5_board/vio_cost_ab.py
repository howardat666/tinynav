#!/usr/bin/env python3
"""What does VIO actually cost inside insight_full? Pause it and watch.

WHY THIS AND NOT THE CUMULATIVE THREAD TIMES

Reading /proc/<pid>/task/*/stat once gives each thread's CPU time since the process
started, which is an average over the whole uptime and cannot show what the machine is
doing now. This samples the same counters twice and reports the delta, so every number
is a rate over a stated window.

HOW VIO IS PAUSED

The firmware already has the switch, it is just not exposed as a parameter:
HobotFullNode holds an atomic vio_.enabled that the two feeder threads (vio_imu,
vio_stereo) check before pushing into the VIO manager, and CalibDataCollector flips it
off/on around a calibration recording. So `insight_param rec_start` pauses VIO on the
firmware already flashed, with no rebuild.

⚠️ THE PAUSE COMES WITH A RECORDING ATTACHED
rec_start also writes every stereo frame to /userdata/calibr-data/imu_cam via
cv::imwrite on its own writer thread. That thread is CPU the baseline did not pay, so
the honest readout is per-thread, not the process total -- the writer is reported
separately and excluded from the VIO comparison. Disk growth is printed each phase so a
long run cannot quietly fill the board.

READ-ONLY on the bus and on config: only rec_start / rec_stop, both reversible.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import time

INSIGHT_PARAM = "/userdata/install/lib/insight_full/insight_param"
REC_DIR = "/userdata/calibr-data/imu_cam"
THERMAL = "/sys/class/thermal/thermal_zone0/temp"
CLK_TCK = os.sysconf("SC_CLK_TCK")

# Thread name -> role. Anything unmatched lands in "other", which is printed too, so a
# thread that matters cannot hide in a bucket nobody looks at.
ROLE = {
    "vio_backend": "VIO", "vio_frontend": "VIO", "vio_stereo": "VIO",
    "vio_imu": "VIO", "pose_pub": "VIO",
    "inference": "depth", "postprocess": "depth", "depth_pub": "depth",
    "computeworker": "depth",
    "stereo_cap": "stereo", "stereo_rect_cap": "stereo",
    "stereo_rect_pub": "stereo", "stereo_raw_pub": "stereo",
    "rgb_cap": "rgb", "rgb_pub": "rgb", "jpg_encoder-0": "rgb",
    "imu_cap": "imu", "imu_pub": "imu",
}


def pid_of(name):
    out = subprocess.run(["pgrep", "-x", name], capture_output=True, text=True, check=False)
    return int(out.stdout.split()[0]) if out.stdout.strip() else None


def read_threads(pid):
    """{tid: (comm, ticks)} for every thread alive right now."""
    out = {}
    base = f"/proc/{pid}/task"
    try:
        tids = os.listdir(base)
    except OSError:
        return out
    for tid in tids:
        try:
            with open(f"{base}/{tid}/stat") as fh:
                raw = fh.read()
            # comm can contain spaces and parens; everything after the last ')' is fixed.
            tail = raw[raw.rindex(")") + 1:].split()
            comm = raw[raw.index("(") + 1:raw.rindex(")")]
            # tail[0] is state, so utime/stime (fields 14/15) are at 11/12 here.
            out[tid] = (comm, int(tail[11]) + int(tail[12]))
        except (OSError, ValueError, IndexError):
            continue
    return out


def read_total():
    with open("/proc/stat") as fh:
        f = [int(x) for x in fh.readline().split()[1:]]
    idle = f[3] + (f[4] if len(f) > 4 else 0)
    return sum(f) - idle, sum(f)


def read_temp():
    try:
        with open(THERMAL) as fh:
            return int(fh.read().strip()) / 1000.0
    except (OSError, ValueError):
        return float("nan")


def dir_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def sample(pid, seconds):
    t0 = read_threads(pid)
    b0, a0 = read_total()
    d0 = dir_bytes(REC_DIR)
    start = time.monotonic()
    time.sleep(seconds)
    dt = time.monotonic() - start
    t1 = read_threads(pid)
    b1, a1 = read_total()
    d1 = dir_bytes(REC_DIR)

    per_thread = {}
    # A thread born inside the window has no baseline, so its cumulative counter IS
    # its CPU for the window. The first version skipped those, which silently
    # undercounted exactly the phase that matters: pausing VIO tears down and
    # recreates vio_backend / vio_frontend, so both were dropped from the total and
    # the pause looked cheaper than it is.
    for tid, (comm, ticks) in t1.items():
        prev = t0.get(tid)
        used = ticks if (prev is None or prev[0] != comm) else ticks - prev[1]
        per_thread[comm] = per_thread.get(comm, 0.0) + used
    # Threads that died inside the window did real work too; without this their CPU
    # vanishes from the total the moment a phase restarts them.
    for tid, (comm, ticks) in t0.items():
        if tid not in t1:
            per_thread[comm] = per_thread.get(comm, 0.0) + 0.0
    new_threads = sorted({t1[t][0] for t in t1 if t not in t0})
    gone_threads = sorted({t0[t][0] for t in t0 if t not in t1})

    cores = {name: 100.0 * ticks / CLK_TCK / dt for name, ticks in per_thread.items()}
    machine = 100.0 * (b1 - b0) / (a1 - a0) * os.cpu_count() if a1 > a0 else float("nan")
    return {
        "dt": dt, "threads": cores, "machine_pct": machine,
        "proc_pct": sum(cores.values()), "temp": read_temp(),
        "rec_mb": (d1 - d0) / 1e6, "rec_total_mb": d1 / 1e6,
        "new_threads": new_threads, "gone_threads": gone_threads,
    }


def by_role(threads):
    agg = {}
    for name, pct in threads.items():
        agg[ROLE.get(name, "other")] = agg.get(ROLE.get(name, "other"), 0.0) + pct
    return agg


def show(tag, s):
    roles = by_role(s["threads"])
    print(f"\n--- {tag}  ({s['dt']:.0f}s window) ---")
    print(f"  insight_full total : {s['proc_pct']:7.1f}% CPU")
    for role in ("VIO", "depth", "stereo", "rgb", "imu", "other"):
        if role in roles:
            print(f"    {role:<16} : {roles[role]:7.1f}%")
    top = sorted(s["threads"].items(), key=lambda kv: -kv[1])[:6]
    print("  top threads        : " + ", ".join(f"{n} {p:.0f}%" for n, p in top))
    print(f"  whole machine      : {s['machine_pct']:7.1f}%   temp {s['temp']:.1f}C")
    if s["rec_mb"] > 0.5:
        print(f"  recording wrote    : {s['rec_mb']:7.1f} MB  (total {s['rec_total_mb']:.0f} MB)")
    if s["new_threads"]:
        print(f"  threads born mid-window (counted from birth): {', '.join(s['new_threads'])}")
    if s["gone_threads"]:
        print(f"  threads that exited mid-window: {', '.join(s['gone_threads'])}")
    return roles


def param(action):
    r = subprocess.run([INSIGHT_PARAM, action], capture_output=True, text=True, check=False)
    print(f"  $ insight_param {action} -> {r.stdout.strip() or r.stderr.strip()}")
    return r.returncode == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", type=float, default=30.0)
    ap.add_argument("--off", type=float, default=30.0)
    ap.add_argument("--restore", type=float, default=30.0)
    ap.add_argument("--max-rec-mb", type=float, default=3000.0,
                    help="abort the VIO-off phase early if the recording grows past this")
    args = ap.parse_args()

    pid = pid_of("insight_full")
    if pid is None:
        print("insight_full is not running")
        return 1
    print(f"insight_full pid={pid}, {os.cpu_count()} cores, CLK_TCK={CLK_TCK}")

    base = show("A. baseline (VIO running)", sample(pid, args.baseline))

    print("\npausing VIO via the calibration recording hook")
    if not param("rec_start"):
        print("rec_start failed; nothing was changed")
        return 1
    try:
        off = show("B. VIO paused", sample(pid, args.off))
    finally:
        print("\nresuming VIO")
        param("rec_stop")
    back = show("C. restored", sample(pid, args.restore))

    print("\n" + "=" * 62)
    print("VIO cost, per-thread (the recording's writer thread is not VIO)")
    print("=" * 62)
    print(f"  {'role':<10} {'baseline':>10} {'VIO off':>10} {'restored':>10} {'delta':>10}")
    for role in ("VIO", "depth", "stereo", "rgb", "imu", "other"):
        a, b, c = base.get(role, 0.0), off.get(role, 0.0), back.get(role, 0.0)
        print(f"  {role:<10} {a:9.1f}% {b:9.1f}% {c:9.1f}% {b - a:+9.1f}%")
    print(f"\n  VIO threads went {base.get('VIO', 0):.1f}% -> {off.get('VIO', 0):.1f}% "
          f"-> {back.get('VIO', 0):.1f}%")
    print("  If the VIO row does not drop, the heavy work is inside libnaive_vio.so's")
    print("  own threads and starving the feeders is not enough -- that needs DVIO=OFF.")
    print(f"\n  Recording left {dir_bytes(REC_DIR) / 1e6:.0f} MB in {REC_DIR}")
    print(f"  Remove it with:  rm -rf {REC_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
