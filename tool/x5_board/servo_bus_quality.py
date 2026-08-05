#!/usr/bin/env python3
"""Measure servo-bus packet loss as a number. READ-ONLY: never writes a register.

    # from the camera, UART-SERVO jumper (the path navigation actually uses)
    python3 tool/x5_board/servo_bus_quality.py --ports /dev/ttyS3

    # from a laptop, USB-SERVO jumper (the reference path)
    sudo python3 tool/x5_board/servo_bus_quality.py --ports /dev/ttyACM0

    python3 tool/x5_board/servo_bus_quality.py --cycles 500 --json out.json

WHY THIS EXISTS
---------------
servo_scan.py answers "is anything there". That is the wrong question for an
intermittent bus, where everything is there most of the time. Counting the ``None``
fields it prints is not a measurement either: the sample size is 30 reads, which
cannot separate 2% loss from 10%.

The real question is which of two physical paths loses packets, and the only way to
answer it is the same number on both, taken minutes apart at the same battery
voltage. So this does a fixed number of identical sync-read cycles and reports loss
per motor, retries, latency percentiles and the voltage it saw.

WHAT COUNTS AS A LOSS
---------------------
One cycle is one ``sync_read`` of ``Present_Position`` across all requested ids --
exactly what wheel_odometry_node's 50 Hz loop does, so the numbers transfer
directly. A cycle is a failure if the read raises after its retries; a *partial* is
a cycle that needed a retry to succeed. Partials matter as much as failures: each
retry costs a full timeout, and that is what drags the effective pose rate down.
Measured on this robot, 2% of cycles failing pulled /wheel/camera_pose from a
configured 50 Hz to 28 Hz in a recording, because the retries stole the time.

READING THE RESULT
------------------
Loss under ~0.5% is a healthy bus. Anything above a few percent needs a cause, and
the causes are physical, in this order of likelihood: no common signal ground
between the two boards (a shared battery negative is not a signal ground),
switching noise from a charger on the same rail, an unshielded or untwisted cable
at 1 Mbaud, and low pack voltage. Compare the two paths before touching any of
them -- if one path is clean and the other is not, the fault is in the dirty path's
wiring and nowhere else.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError

# What wheel_odometry_node reads every cycle. Matching it exactly is the point --
# a bus that passes a lighter probe and fails the real one teaches nothing.
CYCLE_REGISTER = "Present_Position"


def parse_ids(spec: str) -> list[int]:
    ids: list[int] = []
    for chunk in spec.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "-" in chunk:
            lo, hi = chunk.split("-", 1)
            ids.extend(range(int(lo), int(hi) + 1))
        else:
            ids.append(int(chunk))
    return ids


def pct(values: list[float], q: float) -> float:
    """Percentile by nearest rank. No numpy: this has to run on the board."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, round(q / 100.0 * (len(ordered) - 1))))
    return ordered[idx]


def measure(port: str, baudrate: int, ids: list[int], cycles: int, rate_hz: float,
            retries: int) -> dict:
    bus = FeetechBus(port=port, baudrate=baudrate)
    bus.connect()

    period = 1.0 / rate_hz if rate_hz > 0 else 0.0
    latencies: list[float] = []
    failures = 0
    partials = 0
    # Per-id absence is what localizes a daisy-chain break: loss concentrated on the
    # last id is a different fault from loss spread evenly across all three.
    missing_per_id = {i: 0 for i in ids}
    voltages: list[float] = []
    errors: list[str] = []

    next_t = time.monotonic()
    try:
        for n in range(cycles):
            if period:
                next_t += period
            try:
                # allow_partial=True so a single silent motor yields missing_ids
                # instead of an exception -- per-id detail is what localizes a
                # daisy-chain break. The strict behaviour is reconstructed below:
                # wheel_odometry_node runs with allow_partial=False, so any missing
                # id is a failed cycle for it, and that is what gets reported.
                result = bus.sync_read(
                    CYCLE_REGISTER, ids, num_retry=retries, allow_partial=True
                )
                latencies.append((result.t_reply - result.t_request) * 1e3)
                if result.retries:
                    partials += 1
                for i in result.missing_ids:
                    missing_per_id[i] += 1
                if result.missing_ids:
                    failures += 1
            except FeetechBusError as exc:
                failures += 1
                for i in ids:
                    missing_per_id[i] += 1
                if len(errors) < 5:
                    errors.append(f"{type(exc).__name__}: {exc}")

            # Voltage occasionally rather than every cycle: it is a slow signal and
            # each extra read is another chance to disturb the thing being measured.
            if n % 50 == 0:
                try:
                    raw = bus.read("Present_Voltage", ids[0], num_retry=1)
                    if raw is not None:
                        voltages.append(raw / 10.0)
                except FeetechBusError:
                    pass

            if period:
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
    finally:
        bus.disconnect()

    return {
        "port": port,
        "baudrate": baudrate,
        "ids": ids,
        "cycles": cycles,
        "target_rate_hz": rate_hz,
        "failed_cycles": failures,
        "retried_cycles": partials,
        "clean_cycles": cycles - failures - partials,
        "loss_pct": 100.0 * failures / cycles if cycles else float("nan"),
        "retry_pct": 100.0 * partials / cycles if cycles else float("nan"),
        "missing_per_id_pct": {
            i: 100.0 * c / cycles for i, c in missing_per_id.items()
        },
        "latency_ms": {
            "mean": statistics.fmean(latencies) if latencies else float("nan"),
            "p50": pct(latencies, 50),
            "p95": pct(latencies, 95),
            "max": max(latencies) if latencies else float("nan"),
        },
        "voltage_v": {
            "min": min(voltages) if voltages else None,
            "max": max(voltages) if voltages else None,
            "samples": len(voltages),
        },
        "first_errors": errors,
    }


def report(r: dict) -> int:
    print("=" * 78)
    print(f"bus quality: {r['port']} @ {r['baudrate']} baud, ids {r['ids']}")
    print("=" * 78)
    print(f"cycles          : {r['cycles']} at {r['target_rate_hz']:.0f} Hz target")
    print(f"clean            : {r['clean_cycles']}")
    print(f"needed a retry   : {r['retried_cycles']}  ({r['retry_pct']:.2f}%)")
    print(f"failed outright  : {r['failed_cycles']}  ({r['loss_pct']:.2f}%)")
    print()
    print("per-id absence:")
    for i, p in sorted(r["missing_per_id_pct"].items()):
        print(f"  id {i:>3}: {p:6.2f}%")
    lat = r["latency_ms"]
    print()
    print(f"latency ms      : mean {lat['mean']:.2f}  p50 {lat['p50']:.2f}  "
          f"p95 {lat['p95']:.2f}  max {lat['max']:.2f}")
    v = r["voltage_v"]
    if v["samples"]:
        print(f"bus voltage     : {v['min']:.1f} - {v['max']:.1f} V ({v['samples']} samples)")
    else:
        print("bus voltage     : never read successfully -- itself a finding")
    for e in r["first_errors"]:
        print(f"  error: {e}")

    print()
    total_bad = r["loss_pct"] + r["retry_pct"]
    if total_bad < 0.5:
        print("VERDICT: clean. This path is not the problem.")
        return 0
    print(f"VERDICT: {total_bad:.2f}% of cycles were not clean -- this path is degraded.")
    print()
    print("Run the identical command on the other path within a few minutes, at the")
    print("same pack voltage, and compare. If the other path is clean, the fault is")
    print("physical and in this path's wiring. Check, in this order:")
    print("  1. Common signal ground between the two boards. A shared battery")
    print("     negative is not a signal ground. This is the most common cause and")
    print("     its signature is exactly this: random subsets of ids not replying.")
    print("  2. Charger connected to the same rail -- switching noise. Unplug, re-run.")
    print("  3. Cable at 1 Mbaud: length, no twisted pair, no shield, connector seating.")
    print("  4. Pack voltage. Below ~11 V on a 3S pack the servos are marginal.")
    print("Only if all four are clean is dropping the baud rate worth considering, and")
    print("that means writing servo EEPROM, so it is the last resort rather than a fix.")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--ports", default="/dev/ttyS3",
                        help="single port to measure (default the camera's UART)")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", default="7-9", help="LeKiwi base wheels are 7,8,9")
    parser.add_argument("--cycles", type=int, default=500,
                        help="number of sync-read cycles (default 500 = 10 s at 50 Hz)")
    parser.add_argument("--rate-hz", type=float, default=50.0,
                        help="cycle rate; 50 matches wheel_odometry_node. 0 = flat out")
    parser.add_argument("--retries", type=int, default=3,
                        help="retries before a cycle counts as failed")
    parser.add_argument("--json", default=None, help="also write the raw result here")
    args = parser.parse_args()

    port = args.ports.split(",")[0].strip()
    if not os.path.exists(port):
        print(f"{port} does not exist", file=sys.stderr)
        return 2

    ids = parse_ids(args.ids)
    print(f"measuring {port}, {args.cycles} cycles at {args.rate_hz:.0f} Hz "
          f"-- READ-ONLY, nothing can move")
    try:
        r = measure(port, args.baudrate, ids, args.cycles, args.rate_hz, args.retries)
    except (OSError, FeetechBusError) as exc:
        print(f"cannot open {port}: {type(exc).__name__}: {exc}", file=sys.stderr)
        print("in use by wheel_odometry_node, or no permission (needs sudo on a laptop)",
              file=sys.stderr)
        return 2

    rc = report(r)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(r, f, indent=2)
        print(f"\nraw result -> {args.json}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
