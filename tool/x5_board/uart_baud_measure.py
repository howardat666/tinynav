#!/usr/bin/env python3
"""Measure a UART's ACTUAL transmit baud rate instead of computing it.

Why this cannot be computed: ttyS3 reports baud_base 12500000, so 1 Mbaud needs a
divisor of 12.5. An integer-only divider must round to 12 (+4.167%) or 13 (-3.846%),
either of which is outside the ~2% window a UART link needs. But a Synopsys
DesignWare 8250 has a DLF fractional-divisor register, and if the driver programs it
the real rate is exact and the whole theory collapses. baud_base does not say which.

WHY A SINGLE TIMED WRITE IS NOT ENOUGH
--------------------------------------
The obvious measurement -- time one write plus tcdrain, divide bits by seconds --
does not work on a loaded board. tcdrain blocks until the shifter empties, but the
process then has to be *scheduled* to observe that, and with load above 6 that wakeup
lands 20-30 ms late. Measured that way this port reported -25% at 1 Mbaud and -2.6%
at 115200: the error shrinking with the rate is the signature of a fixed overhead,
not of a bad divisor, because slower transfers dilute it.

So measure the SLOPE, not the ratio. Timing several sizes at the same rate gives

    elapsed(N) = N * 10 / baud + overhead

and a least-squares fit over N recovers 1/baud from the slope while the unknown
overhead falls out as the intercept. Each size takes its minimum across repeats:
preemption can only ever make a transfer look slower, never faster, so the fastest
observation is the honest one.

WHY NOT ttyS3: random bytes on the servo bus could, by chance, form a well-formed
WRITE packet and change a register -- position, ID, torque limit. A port with nothing
attached has no such failure mode. ttyS5 on this board shows tx:0 rx:0 in
/proc/tty/driver/serial, i.e. it has never been used.

READ-ONLY with respect to the robot: nothing is attached to the port being measured.
"""

from __future__ import annotations

import argparse
import time

import serial

# 8N1: one start bit, eight data bits, one stop bit.
BITS_PER_BYTE = 10

# 12500000 = the baud_base ttyS3 reports (200 MHz core clock / 16).
BAUD_BASE = 12_500_000


def time_size(ser: serial.Serial, nbytes: int, repeats: int) -> float:
    """Fastest observed wall time to push nbytes out of the shifter."""
    payload = bytes(nbytes)
    best = float("inf")
    for _ in range(repeats):
        t0 = time.monotonic()
        ser.write(payload)
        ser.flush()
        elapsed = time.monotonic() - t0
        best = min(best, elapsed)
    return best


def fit_baud(points: list[tuple[int, float]]) -> tuple[float, float]:
    """Least-squares fit of elapsed = bits/baud + overhead. Returns (baud, overhead)."""
    xs = [n * BITS_PER_BYTE for n, _ in points]
    ys = [t for _, t in points]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx  # seconds per bit
    intercept = my - slope * mx
    return (1.0 / slope if slope > 0 else float("nan")), intercept


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyS5",
                        help="a port with NOTHING attached (default ttyS5, tx:0 rx:0)")
    parser.add_argument("--rates", default="1000000,500000")
    parser.add_argument("--sizes", default="8192,32768,131072",
                        help="byte counts to time; the fit needs at least three")
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()

    wanted = [int(r) for r in args.rates.split(",") if r.strip()]
    sizes = [int(s) for s in args.sizes.split(",") if s.strip()]

    print("=" * 78)
    print(f"actual transmit baud rate on {args.port}  (slope method)")
    print(f"sizes {sizes}, {args.repeats} repeats each, 8N1, fixed overhead fitted out")
    print("=" * 78)

    for want in wanted:
        try:
            ser = serial.Serial(args.port, want, timeout=0.0)
        except (serial.SerialException, OSError) as exc:
            print(f"{want:>10}: cannot open: {exc}")
            continue
        try:
            # One untimed pass so any lazy clock/divisor setup is already done.
            ser.write(bytes(sizes[0]))
            ser.flush()
            points = [(n, time_size(ser, n, args.repeats)) for n in sizes]
        finally:
            ser.close()

        baud, overhead = fit_baud(points)
        err = 100.0 * (baud - want) / want
        divisor = BAUD_BASE / baud if baud > 0 else float("nan")
        print(f"\n  requested {want} baud")
        for n, t in points:
            print(f"    {n:>7} B -> {t * 1000:9.2f} ms")
        print(f"    fitted baud      : {baud:.0f}   ({err:+.3f}%)")
        print(f"    fitted overhead  : {overhead * 1000:.2f} ms  (the scheduling latency)")
        print(f"    implied divisor  : {divisor:.3f}   [{BAUD_BASE} / baud]")

    print("\n" + "-" * 78)
    print("Reading it: divisor landing on a clean integer means integer-only division,")
    print("so 1 Mbaud (divisor 12.5) is unreachable and both ends should move to a rate")
    print("12500000 divides exactly -- 500000 (divisor 25) or 250000 (divisor 50).")
    print("A divisor near 12.5 with ~0% error at 1 Mbaud means DLF fractional division")
    print("is active and the baud-rate theory is dead.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
