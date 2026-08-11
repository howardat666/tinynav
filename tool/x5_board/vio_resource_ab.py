#!/usr/bin/env python3
"""How much CPU, BPU and memory does turning VIO off actually give back?

Three resources, three different answers, and only one of them is a simple delta.

CPU     Measured. Per-thread deltas from /proc/<pid>/task/*/stat, so the number is a
        rate over a stated window rather than an average since boot.

BPU     Read from /sys/devices/system/bpu/bpu0/ratio, but the source already settles
        it: BpuGate is constructed for DepthEngine and HandEngine only, and neither
        insight_full_node_vio.cpp nor vio_manager.hpp references the BPU or any DNN
        API. VIO is MSCKF on CPU. So the expected answer is zero, and the reading
        during the pause CANNOT confirm it -- the pause arrives via a calibration
        recording that also stops depth, so BPU falls for depth's reasons.

memory  Measured, and the one where "pause" and "compile out" genuinely differ.
        vio_.enabled only stops feeding the manager; the VioManager object and its
        allocations stay resident. Anything given back is thread stacks from the VIO
        threads that get torn down, not the working set. -DDVIO=OFF is what removes
        the allocations, and this tool cannot measure that without a rebuild.

Also inventories the VIO threads in each phase, because "1% CPU" means one thing if
the threads are gone and another if they are spinning.

READ-ONLY on config: rec_start / rec_stop, both reversible.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import time

INSIGHT_PARAM = "/userdata/install/lib/insight_full/insight_param"
REC_DIR = "/userdata/calibr-data/imu_cam"
BPU_RATIO = "/sys/devices/system/bpu/bpu0/ratio"
THERMAL = "/sys/class/thermal/thermal_zone0/temp"
CLK_TCK = os.sysconf("SC_CLK_TCK")
VIO_THREADS = ("vio_backend", "vio_frontend", "vio_stereo", "vio_imu", "pose_pub")


def sh(cmd, **kw):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, check=False, **kw)


def pid_of(name):
    r = sh(f"pgrep -x {name}")
    return int(r.stdout.split()[0]) if r.stdout.strip() else None


def read_threads(pid):
    out = {}
    try:
        tids = os.listdir(f"/proc/{pid}/task")
    except OSError:
        return out
    for tid in tids:
        try:
            with open(f"/proc/{pid}/task/{tid}/stat") as fh:
                raw = fh.read()
            comm = raw[raw.index("(") + 1:raw.rindex(")")]
            tail = raw[raw.rindex(")") + 1:].split()
            out[tid] = (comm, int(tail[11]) + int(tail[12]))
        except (OSError, ValueError, IndexError):
            continue
    return out


def read_mem(pid):
    """VmRSS split into anonymous and file-backed. Anonymous is the part a heap
    allocation shows up in, which is what -DDVIO=OFF would actually remove."""
    out = {"rss": 0, "anon": 0, "file": 0, "stack": 0}
    try:
        with open(f"/proc/{pid}/status") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    out["rss"] = int(line.split()[1])
    except OSError:
        pass
    try:
        with open(f"/proc/{pid}/smaps_rollup") as fh:
            for line in fh:
                k, _, v = line.partition(":")
                if not v.strip().endswith("kB"):
                    continue
                n = int(v.split()[0])
                if k == "Anonymous":
                    out["anon"] = n
                elif k in ("Private_Clean", "Shared_Clean"):
                    out["file"] += n
    except OSError:
        pass
    return out


def read_int(path):
    try:
        with open(path) as fh:
            return int(fh.read().strip())
    except (OSError, ValueError):
        return -1


def machine():
    with open("/proc/stat") as fh:
        f = [int(x) for x in fh.readline().split()[1:]]
    idle = f[3] + (f[4] if len(f) > 4 else 0)
    return sum(f) - idle, sum(f)


def dir_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def sample(pid, seconds, bpu_hz=5.0):
    t0, (b0, a0) = read_threads(pid), machine()
    m0, d0 = read_mem(pid), dir_bytes(REC_DIR)
    bpu, temps = [], []
    start = time.monotonic()
    # BPU ratio is an instantaneous gauge, not a counter, so it has to be sampled
    # through the window rather than read once at each end.
    while time.monotonic() - start < seconds:
        bpu.append(read_int(BPU_RATIO))
        temps.append(read_int(THERMAL) / 1000.0)
        time.sleep(1.0 / bpu_hz)
    dt = time.monotonic() - start
    t1, (b1, a1) = read_threads(pid), machine()
    m1, d1 = read_mem(pid), dir_bytes(REC_DIR)

    per = {}
    for tid, (comm, ticks) in t1.items():
        prev = t0.get(tid)
        per[comm] = per.get(comm, 0.0) + (ticks if prev is None or prev[0] != comm
                                          else ticks - prev[1])
    cores = {n: 100.0 * v / CLK_TCK / dt for n, v in per.items()}
    vio_now = sorted(c for _, (c, _) in t1.items() if c in VIO_THREADS)
    bpu = [x for x in bpu if x >= 0]
    return {
        "dt": dt, "cores": cores,
        "vio_cpu": sum(v for n, v in cores.items() if n in VIO_THREADS),
        "proc_cpu": sum(cores.values()),
        "machine": 100.0 * (b1 - b0) / (a1 - a0) * os.cpu_count() if a1 > a0 else float("nan"),
        "bpu_mean": sum(bpu) / len(bpu) if bpu else float("nan"),
        "bpu_max": max(bpu) if bpu else float("nan"),
        "temp": sum(temps) / len(temps) if temps else float("nan"),
        "rss0": m0["rss"], "rss1": m1["rss"],
        "anon0": m0["anon"], "anon1": m1["anon"],
        "vio_threads": vio_now, "n_threads": len(t1),
        "rec_mb": (d1 - d0) / 1e6,
    }


def show(tag, s):
    print(f"\n--- {tag}  ({s['dt']:.0f}s) ---")
    print(f"  CPU  insight_full {s['proc_cpu']:6.1f}%   of which VIO threads {s['vio_cpu']:6.1f}%"
          f"   machine {s['machine']:.0f}%")
    print(f"  BPU  mean {s['bpu_mean']:5.1f}%  max {s['bpu_max']:.0f}%      temp {s['temp']:.1f}C")
    print(f"  MEM  RSS {s['rss0'] / 1024:.1f} -> {s['rss1'] / 1024:.1f} MB"
          f"   anonymous {s['anon0'] / 1024:.1f} -> {s['anon1'] / 1024:.1f} MB")
    print(f"  VIO threads alive ({len(s['vio_threads'])}): "
          f"{', '.join(s['vio_threads']) or 'NONE'}   [{s['n_threads']} threads total]")
    if s["rec_mb"] > 0.5:
        print(f"  (calibration recording wrote {s['rec_mb']:.0f} MB this phase)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=45.0)
    args = ap.parse_args()

    pid = pid_of("insight_full")
    if pid is None:
        print("insight_full is not running")
        return 1
    print(f"insight_full pid={pid}, {os.cpu_count()} cores, bpu cores="
          f"{read_int('/sys/devices/system/bpu/core_num')}")

    a = sample(pid, args.seconds)
    show("A. baseline, VIO running", a)

    print("\npausing VIO")
    sh(f"{INSIGHT_PARAM} rec_start")
    try:
        b = sample(pid, args.seconds)
        show("B. VIO paused  (⚠️ depth is also stopped by the recording)", b)
    finally:
        print("\nresuming VIO")
        sh(f"{INSIGHT_PARAM} rec_stop")
    c = sample(pid, args.seconds)
    show("C. restored", c)

    print("\n" + "=" * 70)
    print("WHAT TURNING VIO OFF GIVES BACK")
    print("=" * 70)
    base = (a["vio_cpu"] + c["vio_cpu"]) / 2
    print(f"  CPU     {base:.1f}% -> {b['vio_cpu']:.1f}%   = {base - b['vio_cpu']:.1f} points "
          f"({(base - b['vio_cpu']) / 100:.2f} of {os.cpu_count()} cores)")
    print(f"          residual {b['vio_cpu']:.1f}% is the feeder threads polling the flag")
    print(f"  BPU     {a['bpu_mean']:.1f}% -> {b['bpu_mean']:.1f}%  NOT ATTRIBUTABLE: the "
          f"recording stops depth too,")
    print("          and depth is the only BPU consumer (BpuGate is built for "
          "DepthEngine/HandEngine")
    print("          only; the VIO sources reference no BPU or DNN API). "
          "Expect VIO to free 0 BPU.")
    rss_a = (a["rss1"] + c["rss1"]) / 2 / 1024
    print(f"  MEM     RSS {rss_a:.1f} MB -> {b['rss1'] / 1024:.1f} MB "
          f"= {rss_a - b['rss1'] / 1024:+.1f} MB")
    print("          A pause cannot free the VioManager's allocations -- only its torn-down")
    print("          thread stacks. -DDVIO=OFF is what removes the working set, and that")
    print("          needs a rebuild to measure.")
    print(f"\n  Recording left {dir_bytes(REC_DIR) / 1e6:.0f} MB; rm -rf {REC_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
