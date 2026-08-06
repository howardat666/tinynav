#!/usr/bin/env python3
"""Decide whether bus failures are CLUSTERED or UNIFORM. That choice picks the fix.

WHY THIS IS THE RIGHT QUESTION NOW
----------------------------------
Everything measurable about the mechanism has been falsified: not RX overrun (oe is
zero), not TX echo (0 of 60 trials), not baud error (measured divisor 12.532, so DLF
fractional division is active), not a second process on the port, not CPU preemption
(SCHED_FIFO changed nothing), not Return_Delay_Time, not a TX preamble.

What is left is a distributional question, and it separates the two remaining families
cleanly:

  clustered   failures arrive in bursts, with long clean stretches between them. That
              is what an intermittent physical contact looks like -- a marginal crimp,
              a connector under slight tension, a cold joint. Software cannot fix it;
              retries land inside the same burst, which is exactly the correlation
              already observed (four attempts only get the failure rate from ~35% down
              to ~25%, where independence would predict ~1.5%).
  uniform     failures are independent coin flips at some fixed probability. That is a
              systematic timing or protocol effect, and retries and sequencing DO help
              in proportion.

Reported: the run-length distribution of consecutive failures against the geometric
distribution independence would produce, plus the rate per time bucket so drift within
a single run is visible rather than smeared into the average.

Cross-time comparisons on this bus are untrustworthy -- the identical command measured
25.2% and 13.4% forty minutes apart at constant voltage and temperature. So this takes
its evidence from the STRUCTURE of one continuous run, which no drift can fake.

READ-ONLY: sync_read only, no register is written.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from collections import Counter

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError

REGISTER = "Present_Position"


def read_cpu() -> tuple[int, int]:
    """(busy_jiffies, total_jiffies) from the aggregate cpu line of /proc/stat."""
    with open("/proc/stat") as fh:
        fields = [int(x) for x in fh.readline().split()[1:]]
    total = sum(fields)
    # Fields 3 and 4 are idle and iowait; everything else is the CPU doing something.
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    return total - idle, total


def busy_fraction(prev: tuple[int, int], now: tuple[int, int]) -> float:
    d_busy, d_total = now[0] - prev[0], now[1] - prev[1]
    return 100.0 * d_busy / d_total if d_total > 0 else float("nan")


def pearson(xs: list[float], ys: list[float]) -> float:
    n = min(len(xs), len(ys))
    if n < 3:
        return float("nan")
    xs, ys = xs[:n], ys[:n]
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 0 or syy <= 0:
        return float("nan")
    return sxy / (sxx * syy) ** 0.5


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", default="7,8,9")
    parser.add_argument("--cycles", type=int, default=3000)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--buckets", type=int, default=10)
    args = parser.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    period = 1.0 / args.rate_hz
    bucket_size = max(1, args.cycles // args.buckets)

    bus = FeetechBus(port=args.port, baudrate=args.baudrate, timeout=0.02)
    bus.connect()
    # One attempt per cycle. Retries would smear the very structure being measured.
    outcomes: list[bool] = []
    # CPU utilisation sampled at bucket boundaries. The GH1.25 cable carries the
    # camera's 12 V supply alongside the UART signals, so the X5's own current draw
    # shares a return path with the signal ground reference. If that is the noise
    # source, bucket failure rate should track bucket CPU load; if the fault is a
    # marginal contact, the two are unrelated.
    cpu_samples: list[float] = []
    try:
        prev = read_cpu()
        for i in range(args.cycles):
            if i and i % bucket_size == 0:
                now = read_cpu()
                cpu_samples.append(busy_fraction(prev, now))
                prev = now
            t0 = time.monotonic()
            try:
                result = bus.sync_read(REGISTER, ids, num_retry=0)
                outcomes.append(bool(result.complete))
            except FeetechBusError:
                outcomes.append(False)
            remaining = period - (time.monotonic() - t0)
            if remaining > 0:
                time.sleep(remaining)
        cpu_samples.append(busy_fraction(prev, read_cpu()))
    finally:
        bus.disconnect()

    n = len(outcomes)
    fails = sum(1 for ok in outcomes if not ok)
    p = fails / n if n else 0.0

    print("=" * 78)
    print(f"failure pattern on {args.port}, ids {ids}")
    print(f"{n} single-attempt cycles at {args.rate_hz} Hz")
    print("=" * 78)
    print(f"  single-attempt failure rate : {100.0 * p:.2f}%  ({fails}/{n})")

    # Run lengths of consecutive failures.
    runs: list[int] = []
    current = 0
    for ok in outcomes:
        if ok:
            if current:
                runs.append(current)
            current = 0
        else:
            current += 1
    if current:
        runs.append(current)

    hist = Counter(runs)
    print("-" * 78)
    print("  consecutive-failure run lengths (independence -> geometric decay by p each step)")
    print(f"    {'len':>5}  {'observed':>9}  {'expected if independent':>24}")
    total_runs = len(runs)
    for length in sorted(hist):
        # Under independence a failure run has length L with probability (1-p)*p^(L-1).
        expected = total_runs * (1.0 - p) * (p ** (length - 1))
        print(f"    {length:>5}  {hist[length]:>9}  {expected:>24.1f}")
    if runs:
        print(f"    mean run length: observed {statistics.fmean(runs):.2f}, "
              f"independent would be {1.0 / (1.0 - p) if p < 1 else float('inf'):.2f}")
        print(f"    longest run    : {max(runs)}")

    # Drift inside this single run.
    print("-" * 78)
    print(f"  per-bucket failure rate vs CPU load ({args.buckets} slices of this run)")
    print(f"    {'cycles':>13}  {'fail%':>7}  {'cpu%':>6}")
    size = bucket_size
    bucket_rates: list[float] = []
    for b in range(args.buckets):
        chunk = outcomes[b * size:(b + 1) * size]
        if not chunk:
            continue
        rate = 100.0 * sum(1 for ok in chunk if not ok) / len(chunk)
        bucket_rates.append(rate)
        cpu = cpu_samples[b] if b < len(cpu_samples) else float("nan")
        bar = "#" * int(rate / 3)
        print(f"    {b * size:>5}-{b * size + len(chunk):<7} {rate:>6.2f}%  {cpu:>5.1f}%  {bar}")

    r = pearson(bucket_rates, cpu_samples)
    print(f"    correlation(fail%, cpu%) = {r:+.3f}")
    if r == r:  # not NaN
        if r > 0.5:
            print("      -> STRONG positive: failures track CPU load. Consistent with the")
            print("         camera's own current draw injecting noise on the shared GH1.25")
            print("         return path. That is electrical, and reseating cannot fix it.")
        elif r < -0.5:
            print("      -> STRONG negative: unexpected; failures fall as load rises.")
        else:
            print("      -> WEAK: failures do not track CPU load, so the camera's current")
            print("         draw is not the modulator. Points back at an intermittent")
            print("         connection inside the cable or at a solder joint.")

    print("-" * 78)
    print("Reading it: a mean run length well above the independent prediction, or a few")
    print("long runs where independence predicts none, means CLUSTERED -- an intermittent")
    print("physical contact, and no retry policy can fix it. Run lengths matching the")
    print("geometric column mean UNIFORM -- independent failures, which sequencing and")
    print("retries reduce in proportion.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
