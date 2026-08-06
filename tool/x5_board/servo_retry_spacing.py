#!/usr/bin/env python3
"""Test whether spacing the retries apart breaks the correlation between them.

THE OBSERVATION THAT MOTIVATES THIS
-----------------------------------
A single sync_read attempt on ttyS3 returns all three replies roughly 50-75% of the
time. If the four attempts sync_read makes were independent, the chance of all four
failing would be about 0.35^4, i.e. under 2%. The measured end-to-end failure rate is
25%. Off by more than a factor of ten, which means the attempts are not independent:
a retry lands in the same bad state the first attempt did.

feetech_bus documents this deliberately -- "Retries are immediate (no sleep) so the
steady-state cost is unchanged" -- which is the right call for a bus whose failures
are random, and the wrong one for a bus whose failures persist for a while.

If the adapter's derived TxEn is a one-shot that needs time to settle, then simply
waiting between attempts should decorrelate them, and the end-to-end failure rate
should collapse towards the independent-trials prediction. That is a one-line change
in the driver, so it is worth measuring before touching any wiring.

WHAT IS MEASURED
----------------
For each candidate gap: a fixed number of 50 Hz cycles, each cycle allowed the same
number of attempts as the driver uses today, with the gap inserted between attempts.
Reported per gap: end-to-end failure rate, how many attempts a successful cycle
needed, and latency percentiles -- because a gap that fixes the loss but blows the
20 ms budget is not usable.

READ-ONLY: sync_read only, no register is written.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError

REGISTER = "Present_Position"


def pct(values: list[float], q: int) -> float:
    if not values:
        return float("nan")
    if len(values) < 2:
        return values[0]
    return statistics.quantiles(values, n=100)[q - 1]


def run_gap(bus: FeetechBus, ids: list[int], cycles: int, rate_hz: float,
            attempts: int, gap_s: float):
    period = 1.0 / rate_hz
    failures = 0
    attempt_hist: list[int] = []
    latencies: list[float] = []

    for _ in range(cycles):
        t0 = time.monotonic()
        ok = False
        for attempt in range(attempts):
            if attempt and gap_s > 0:
                time.sleep(gap_s)
            try:
                # num_retry=0: this loop owns the retry policy, so the driver must
                # not add its own immediate ones on top.
                result = bus.sync_read(REGISTER, ids, num_retry=0)
            except FeetechBusError:
                continue
            if result.complete:
                ok = True
                attempt_hist.append(attempt + 1)
                latencies.append((time.monotonic() - t0) * 1000.0)
                break
        if not ok:
            failures += 1
        remaining = period - (time.monotonic() - t0)
        if remaining > 0:
            time.sleep(remaining)

    return failures, attempt_hist, latencies


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", default="7,8,9")
    parser.add_argument("--cycles", type=int, default=500)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--attempts", type=int, default=4,
                        help="attempts per cycle; 4 matches the driver's num_retry=3")
    parser.add_argument("--gaps-ms", default="0,0.5,2,5",
                        help="inter-attempt gaps to compare, in milliseconds")
    args = parser.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    gaps = [float(g) for g in args.gaps_ms.split(",") if g.strip()]

    print("=" * 78)
    print(f"retry spacing on {args.port}, ids {ids}")
    print(f"{args.cycles} cycles at {args.rate_hz} Hz, {args.attempts} attempts per cycle")
    print("=" * 78)
    print(f"{'gap':>8}  {'failed':>8}  {'mean tries':>11}  {'p50 ms':>8}  {'p95 ms':>8}")
    print("-" * 78)

    bus = FeetechBus(port=args.port, baudrate=args.baudrate, timeout=0.02)
    bus.connect()
    try:
        for gap_ms in gaps:
            failures, tries, latencies = run_gap(
                bus, ids, args.cycles, args.rate_hz, args.attempts, gap_ms / 1000.0
            )
            rate = 100.0 * failures / args.cycles
            mean_tries = statistics.fmean(tries) if tries else float("nan")
            print(f"{gap_ms:>6.1f}ms  {rate:>7.2f}%  {mean_tries:>11.2f}  "
                  f"{pct(latencies, 50):>8.2f}  {pct(latencies, 95):>8.2f}")
    finally:
        bus.disconnect()

    print("-" * 78)
    print("Reading it: if the failure rate falls sharply as the gap grows, the attempts")
    print("were correlated and spacing them is a real one-line fix. If it stays flat, the")
    print("failures are not a settling-time effect and this lever is dead too.")
    print("A gap is only usable if p95 still fits the 20 ms odometry tick.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
