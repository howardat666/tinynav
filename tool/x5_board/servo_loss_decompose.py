#!/usr/bin/env python3
"""Split the bus loss into a QUERY rate and a REPLY rate, read-only, and test whether
the losses are independent.

THE IDEA
--------
One SYNC READ transaction is one query outbound and N replies inbound. Vary N and the
outcome probabilities move in a way that separates the two directions. Writing q_q for
the chance the query is lost and q_r for the chance one reply is lost, independence
would give

    P(all N answered) = (1 - q_q) * (1 - q_r)^N

so each adjacent ratio P(N+1)/P(N) equals (1 - q_r), and P(1) then yields q_q. Two
independent estimates of the same quantity fall out of three conditions -- and whether
they AGREE is itself the test of independence. If the ratios differ, replies within one
burst are correlated and the two-parameter model is simply wrong, which is worth knowing
before anyone builds a retry policy on it.

THE CONFOUND, AND THE CONTROL FOR IT
------------------------------------
A SYNC READ query grows one byte per ID, so a 3-ID query is not the same packet as a
1-ID query, and q_q may not be constant across the conditions. Hence a fourth condition:
IDs 7,20,21, where 20 and 21 do not exist. Its query is exactly as long as the 3-ID one
but only one reply comes back, so comparing it against the plain 1-ID condition isolates
the effect of query length alone.

WHY THE CONDITIONS ARE INTERLEAVED
----------------------------------
This bus is non-stationary: within a single 60 s run the failure rate wanders between
4% and 56%, and the identical command measured 25.2% and 13.4% forty minutes apart. Any
comparison built from blocks measured one after another is measuring the drift. So each
cycle round-robins through the conditions, which makes them share the same drift and
cancels it out of every ratio.

READ-ONLY: sync_read only, no register is written. Absent IDs simply stay silent.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError

REGISTER = "Present_Position"

# (label, ids queried, ids that can actually answer)
CONDITIONS = [
    ("N=1  [7]",         [7],        [7]),
    ("N=2  [7,8]",       [7, 8],     [7, 8]),
    ("N=3  [7,8,9]",     [7, 8, 9],  [7, 8, 9]),
    ("ctl  [7,20,21]",   [7, 20, 21], [7]),
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--cycles", type=int, default=600,
                        help="cycles PER CONDITION (they are interleaved)")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    args = parser.parse_args()

    period = 1.0 / args.rate_hz
    full = [0] * len(CONDITIONS)      # every expected reply arrived
    empty = [0] * len(CONDITIONS)     # not one reply arrived
    answered = [0] * len(CONDITIONS)  # total replies received
    # For N=3 only: how many of the three answered, per cycle.
    hist3 = [0, 0, 0, 0]

    bus = FeetechBus(port=args.port, baudrate=args.baudrate, timeout=0.02)
    bus.connect()
    try:
        for cycle in range(args.cycles):
            for idx, (_, ids, real) in enumerate(CONDITIONS):
                t0 = time.monotonic()
                got: list[int] = []
                try:
                    # num_retry=0 and allow_partial: one shot, and keep whatever came
                    # back. Retries would fold the two directions back together.
                    result = bus.sync_read(REGISTER, ids, num_retry=0, allow_partial=True)
                    got = [m for m in real if m in result.values]
                except FeetechBusError:
                    got = []
                answered[idx] += len(got)
                if len(got) == len(real):
                    full[idx] += 1
                if not got:
                    empty[idx] += 1
                if idx == 2:
                    hist3[len(got)] += 1
                remaining = period - (time.monotonic() - t0)
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        bus.disconnect()

    n = args.cycles
    print("=" * 78)
    print(f"loss decomposition on {args.port}")
    print(f"{n} interleaved cycles per condition at {args.rate_hz} Hz, single attempt")
    print("=" * 78)
    print(f"  {'condition':<16} {'P(all answered)':>16} {'P(none answered)':>17} {'per-reply':>10}")
    for idx, (label, ids, real) in enumerate(CONDITIONS):
        p_full = full[idx] / n
        p_none = empty[idx] / n
        per_reply = answered[idx] / (n * len(real))
        print(f"  {label:<16} {100 * p_full:>15.2f}% {100 * p_none:>16.2f}% {100 * per_reply:>9.2f}%")

    p1, p2, p3, pc = (full[i] / n for i in range(4))

    print("-" * 78)
    print("  independence test: adjacent ratios both estimate (1 - q_reply)")
    r21 = p2 / p1 if p1 else float("nan")
    r32 = p3 / p2 if p2 else float("nan")
    print(f"    P(N=2)/P(N=1) = {r21:.4f}")
    print(f"    P(N=3)/P(N=2) = {r32:.4f}")
    if r21 == r21 and r32 == r32:
        spread = abs(r32 - r21) / max(r21, r32)
        print(f"    disagreement  = {100 * spread:.1f}%")
        if spread < 0.05:
            print("      -> the two agree: replies are lost INDEPENDENTLY of each other,")
            print("         and the two-parameter model holds.")
        else:
            print("      -> they DISAGREE: replies within one burst are CORRELATED, so")
            print("         'each reply has a fixed independent loss rate' is wrong and")
            print("         no retry policy derived from it will predict correctly.")

    print("-" * 78)
    print("  direction split (from the model; only meaningful if it passed above)")
    q_r = 1.0 - r32 if r32 == r32 else float("nan")
    q_q = 1.0 - (p1 / (1.0 - q_r)) if q_r == q_r and q_r < 1 else float("nan")
    print(f"    q_reply (one reply lost)   = {100 * q_r:.2f}%")
    print(f"    q_query (query lost)       = {100 * q_q:.2f}%")

    print("-" * 78)
    print("  query-length control: same 3-ID query, only one real reply")
    print(f"    P(id 7 answers | 1-ID query) = {100 * p1:.2f}%")
    print(f"    P(id 7 answers | 3-ID query) = {100 * pc:.2f}%")
    if p1:
        print(f"    ratio = {pc / p1:.4f}  ({'a longer query IS lost more often' if pc / p1 < 0.95 else 'query length does not matter'})")

    print("-" * 78)
    print("  N=3: how many of the three answered, vs independence")
    from math import comb
    for k in range(4):
        exp = n * (1 - q_q) * comb(3, k) * ((1 - q_r) ** k) * (q_r ** (3 - k)) if q_r == q_r else float("nan")
        if k == 0:
            exp = n * (q_q + (1 - q_q) * q_r ** 3) if q_r == q_r else float("nan")
        print(f"    {k} answered: observed {hist3[k]:>4}   independence predicts {exp:>7.1f}")
    print("    A poor fit here is the same finding as a failed ratio test, seen directly.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
