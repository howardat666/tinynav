#!/usr/bin/env python3
"""Is the servo bus still losing packets, and does the app's data flow still modulate it?

WHY ABA AND NOT A THEN B

Cross-time comparison on this bus has been wrong before: the identical command measured
25.2% and 13.4% forty minutes apart at constant voltage and temperature. So the app-off
condition is measured twice, before and after the app-on block. If the two A blocks
agree, the B block means something; if they do not, the run measured drift and says so
instead of producing a confident wrong number.

WHAT "APP ON" MEANS HERE

wheel_odometry_node owns /dev/ttyS3, and pyserial cannot share it -- a second opener
gets "device reports readiness to read but returned no data" and every number is
garbage. So the app is started for its DATA FLOW (camera pipeline, DDS, DDR traffic),
which is what the earlier work identified as the modulator, and then wheel_odometry
alone is stopped so this tool can hold the port. Everything else keeps running.

READ-ONLY on the bus: sync_read of Present_Position only, plus Present_Voltage between
blocks. Nothing is written, nothing can move.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError  # noqa: E402

APP = "/userdata/x5/tinynav/tool/x5_board/app_start.sh"
THERMAL = "/sys/class/thermal/thermal_zone0/temp"


def sh(cmd, timeout=120):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                          timeout=timeout, check=False)


def tty_holders():
    r = sh("ls -l /proc/*/fd 2>/dev/null | grep -c ttyS3")
    try:
        return int(r.stdout.strip())
    except ValueError:
        return -1


def temp():
    try:
        with open(THERMAL) as fh:
            return int(fh.read().strip()) / 1000.0
    except (OSError, ValueError):
        return float("nan")


def machine_busy(seconds=1.0):
    def snap():
        with open("/proc/stat") as fh:
            f = [int(x) for x in fh.readline().split()[1:]]
        idle = f[3] + (f[4] if len(f) > 4 else 0)
        return sum(f) - idle, sum(f)
    b0, a0 = snap()
    time.sleep(seconds)
    b1, a1 = snap()
    return 100.0 * (b1 - b0) / (a1 - a0) * os.cpu_count() if a1 > a0 else float("nan")


def block(bus, ids, cycles, rate_hz):
    """One measurement block. num_retry=0 so this is the SINGLE-ATTEMPT rate, which is
    always larger than what navigation sees after its 3 retries -- do not compare the
    two numbers directly."""
    period = 1.0 / rate_hz
    fails = 0
    miss = {i: 0 for i in ids}
    lat = []
    t_start = time.monotonic()
    for _ in range(cycles):
        t0 = time.monotonic()
        try:
            r = bus.sync_read("Present_Position", ids, num_retry=0, allow_partial=True)
            if r.missing_ids:
                fails += 1
                for i in r.missing_ids:
                    miss[i] += 1
            else:
                lat.append((r.t_reply - r.t_request) * 1e3)
        except FeetechBusError:
            # Nothing came back at all: charge every id, since the query itself is the
            # most likely casualty and no id can be exonerated.
            fails += 1
            for i in ids:
                miss[i] += 1
        rest = period - (time.monotonic() - t0)
        if rest > 0:
            time.sleep(rest)
    dur = time.monotonic() - t_start
    lat.sort()
    p = lambda q: lat[min(int(q * len(lat)), len(lat) - 1)] if lat else float("nan")  # noqa: E731
    try:
        volts = bus.read("Present_Voltage", ids[0], num_retry=8) / 10.0
    except FeetechBusError:
        volts = float("nan")
    return {
        "cycles": cycles, "fails": fails, "pct": 100.0 * fails / cycles,
        "miss": miss, "dur": dur, "temp": temp(), "volts": volts,
        "lat_mean": sum(lat) / len(lat) if lat else float("nan"),
        "lat_p50": p(0.50), "lat_p95": p(0.95), "lat_max": lat[-1] if lat else float("nan"),
    }


def show(tag, r, ids, busy):
    print(f"\n--- {tag} ---")
    print(f"  {r['fails']}/{r['cycles']} single attempts failed = {r['pct']:.2f}%"
          f"   ({r['dur']:.0f}s)")
    print("  per-id absence : " + "  ".join(
        f"id{i} {100.0 * r['miss'][i] / r['cycles']:.2f}%" for i in ids))
    print(f"  latency ms     : mean {r['lat_mean']:.2f}  p50 {r['lat_p50']:.2f}  "
          f"p95 {r['lat_p95']:.2f}  max {r['lat_max']:.2f}")
    print(f"  conditions     : {r['volts']:.1f}V  {r['temp']:.1f}C  machine {busy:.0f}% CPU")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS3")
    ap.add_argument("--baudrate", type=int, default=1_000_000)
    ap.add_argument("--ids", default="7,8,9")
    ap.add_argument("--cycles", type=int, default=3000)
    ap.add_argument("--rate-hz", type=float, default=50.0)
    ap.add_argument("--scheme", default="b")
    args = ap.parse_args()
    ids = [int(x) for x in args.ids.split(",")]

    if tty_holders() != 0:
        print(f"/dev/ttyS3 has {tty_holders()} holder(s); stop the app first:\n"
              f"  bash {APP} stop")
        return 1

    bus = FeetechBus(port=args.port, baudrate=args.baudrate, timeout=0.02)
    bus.connect()
    results = {}
    try:
        busy = machine_busy()
        results["A1"] = block(bus, ids, args.cycles, args.rate_hz)
        show("A1. app OFF (insight_full only)", results["A1"], ids, busy)

        print(f"\nstarting the app (scheme {args.scheme}) for its data flow, then "
              f"stopping wheel_odometry so this tool can keep the port")
        bus.disconnect()
        sh(f"bash {APP} start {args.scheme}")
        # The app needs to reach steady state -- the bridge and planning both pay a
        # numba warmup, and measuring during it would measure the warmup.
        time.sleep(45)
        sh("pkill -f '[w]heel_odometry_node.py'")
        for _ in range(30):
            if tty_holders() == 0:
                break
            time.sleep(1)
        if tty_holders() != 0:
            print(f"  wheel_odometry still holds the port ({tty_holders()}); aborting B")
        else:
            bus.connect()
            busy = machine_busy()
            results["B"] = block(bus, ids, args.cycles, args.rate_hz)
            show("B. app ON (camera pipeline + DDS + planning running)", results["B"], ids, busy)
            bus.disconnect()

        print("\nstopping the app again")
        sh(f"bash {APP} stop")
        for _ in range(30):
            if tty_holders() == 0:
                break
            time.sleep(1)
        time.sleep(5)
        bus.connect()
        busy = machine_busy()
        results["A2"] = block(bus, ids, args.cycles, args.rate_hz)
        show("A2. app OFF again (repeatability check)", results["A2"], ids, busy)
    finally:
        try:
            bus.disconnect()
        except (FeetechBusError, OSError):
            pass

    print("\n" + "=" * 66)
    a1, a2 = results.get("A1"), results.get("A2")
    b = results.get("B")
    if a1 and a2:
        drift = abs(a1["pct"] - a2["pct"])
        print(f"  A1 {a1['pct']:.2f}%   A2 {a2['pct']:.2f}%   drift between them "
              f"{drift:.2f} points")
        if b:
            base = (a1["pct"] + a2["pct"]) / 2
            print(f"  B  {b['pct']:.2f}%   app effect {b['pct'] - base:+.2f} points")
            if drift >= abs(b["pct"] - base):
                print("  ⚠️  the drift between the two A blocks is at least as large as the")
                print("      app effect, so this run cannot attribute anything to the app.")
    print("  Single-attempt rate. Navigation retries 3x, so what it sees is far lower;")
    print("  and the wheels were parked -- driving current is not in any of these numbers.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
