#!/usr/bin/env python3
"""Separate the two mechanisms that the id-count sweep cannot tell apart.

Loss on the X5's ttyS3 path climbs monotonically with the id count -- 0.2% at one
id, 4.2% at two, 27.8% at three -- while the UART reports no framing error, no
break and no overrun across thousands of cycles. So the bytes never reach the RX
pin at all, and two different mechanisms both fit:

  (1) TX side: a SYNC READ query packet grows with the id count. If the X5's UART
      leaves inter-byte gaps while sending a longer packet, the adapter's automatic
      direction control can read the gap as "transmission over", release the bus and
      re-drive it mid-packet. The query arrives mangled, no servo understands it, and
      all ids go silent at once -- with nothing for the RX side to flag.

  (2) RX side: replies come back-to-back, separated by tens of microseconds. If the
      adapter is still driving the bus when the first reply starts, or turns around
      too slowly between replies, the replies are suppressed electrically.

Both scale with the id count, which is why the sweep conflates them. This does not:

  A  ids 7,8,9    long query, 3 replies   <- the real navigation case
  B  ids 7,20,21  long query, 1 reply     <- absent ids never answer
  C  ids 7        short query, 1 reply    <- the clean baseline

Case B has A's query packet and C's reply burst. Whichever it resembles is the side
the fault is on.

Every case scores the same thing: did id 7 answer? It is first in the query of all
three cases, so it is first to reply in all three -- the comparison is apples to
apples. ``num_retry=0`` deliberately measures the raw first-shot success; retries
would hide exactly the effect being measured.

READ-ONLY: sync_read only, never a write. Absent ids simply stay silent, which is
what makes case B safe.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError

PROBE_ID = 7

CASES = [
    ("A  long query, 3 replies", [7, 8, 9]),
    ("B  long query, 1 reply  ", [7, 20, 21]),
    ("C  short query, 1 reply ", [7]),
]


def run_case(bus: FeetechBus, ids: list[int], cycles: int, rate_hz: float):
    period = 1.0 / rate_hz
    answered = 0
    latencies: list[float] = []
    for _ in range(cycles):
        t0 = time.monotonic()
        try:
            result = bus.sync_read(
                "Present_Position", ids, num_retry=0, allow_partial=True
            )
            if PROBE_ID in result.values:
                answered += 1
                latencies.append((time.monotonic() - t0) * 1000.0)
        except FeetechBusError:
            pass
        remaining = period - (time.monotonic() - t0)
        if remaining > 0:
            time.sleep(remaining)
    return answered, latencies


def pct(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    return statistics.quantiles(values, n=100)[int(q) - 1]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--cycles", type=int, default=500)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    args = parser.parse_args()

    print("=" * 78)
    print(f"query-vs-reply discriminator on {args.port} @ {args.baudrate} baud")
    print(f"{args.cycles} cycles per case at {args.rate_hz} Hz -- READ-ONLY, nothing can move")
    print("=" * 78)

    bus = FeetechBus(port=args.port, baudrate=args.baudrate, timeout=0.02)
    bus.connect()
    try:
        rows = []
        for label, ids in CASES:
            answered, latencies = run_case(bus, ids, args.cycles, args.rate_hz)
            rate = 100.0 * answered / args.cycles
            rows.append((label, ids, rate, latencies))
            print(
                f"{label}  ids={str(ids):<12} "
                f"id{PROBE_ID} answered {rate:6.2f}%   "
                f"p50 {pct(latencies, 50):5.2f} ms  p95 {pct(latencies, 95):6.2f} ms"
            )
    finally:
        bus.disconnect()

    print("-" * 78)
    a, b, c = (row[2] for row in rows)
    # Which baseline B sits closer to is the whole answer.
    if abs(b - a) < abs(b - c):
        print("VERDICT: B tracks A -- the long QUERY packet is the problem (TX side).")
        print("  Fewer ids per query, or a TX path that sends a packet without")
        print("  inter-byte gaps, is the direction to look. Sequential single reads")
        print("  would also fix it, since each query is then short.")
    else:
        print("VERDICT: B tracks C -- the back-to-back REPLIES are the problem (RX side).")
        print("  The query gets out fine; the adapter cannot turn the bus around fast")
        print("  enough between replies. Sequential single reads fix it by construction,")
        print("  since only one reply is ever in flight.")
    print(f"  A={a:.2f}%  B={b:.2f}%  C={c:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
