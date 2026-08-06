#!/usr/bin/env python3
"""Set Return_Delay_Time on the LeKiwi base wheels, reversibly.

WHY
---
The Waveshare adapter's half-duplex direction control is driven by a TxEn signal. On
the USB path the CH343 supplies TxEn from its dedicated TNOW pin, which is exact --
measured 0 losses in 2500 cycles. The GH1.25 UART header carries only TX/RX/GND/12V,
no TxEn, so the adapter has to derive it from TX activity, and that derived signal
releases the bus at an approximate moment. Measured on this robot: 25% of queries get
no reply at all and another 25% get only the first of three, with the UART reporting
no framing error, no break and no overrun -- i.e. the replies are not corrupted, they
are electrically overridden by the adapter still driving the bus.

Return_Delay_Time is how long a servo waits after receiving a command before it
answers. All three wheels ship with 0, meaning they answer at the worst possible
instant -- while the derived TxEn may still be asserted. Making them wait gives the
adapter time to let go.

Units are ~2 us per count on the STS series. At 1 Mbaud one byte is 10 us, so 20
counts is roughly 40 us: four byte-times of margin, and about 0.3 ms added to a read
cycle that has 20 ms to spend.

SAFETY
------
Return_Delay_Time only delays replies. It cannot move a motor, change a limit, or
change an ID. Fully reversible: run again with --value 0.

Address 7 is below FIRST_SRAM_ADDRESS, so it lives in EEPROM and a write is silently
discarded unless Lock is 0 first -- acknowledged, no error byte, no effect. So this
unlocks, writes, re-locks, and then reads the value back to prove it took. Lock is
restored in a finally block: a servo left unlocked would let a later stray write land
in EEPROM.

Reads and writes both use generous retries on purpose: this bus loses ~20% of first
attempts, and that is the fault being worked around, not a reason to give up. Writing
the same value twice is idempotent, so a retried write is harmless.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError

REGISTER = "Return_Delay_Time"
RETRIES = 10


def apply_to(bus: FeetechBus, motor_id: int, value: int) -> str:
    try:
        before = bus.read(REGISTER, motor_id, num_retry=RETRIES)
    except FeetechBusError as exc:
        return f"id {motor_id}: cannot read current value, skipped ({exc})"

    if before == value:
        return f"id {motor_id}: already {value}, nothing to do"

    try:
        bus.write("Lock", motor_id, 0, num_retry=RETRIES)
        try:
            bus.write(REGISTER, motor_id, value, num_retry=RETRIES)
        finally:
            # Re-lock no matter what: an unlocked servo would accept a later stray
            # write straight into EEPROM.
            bus.write("Lock", motor_id, 1, num_retry=RETRIES)
    except FeetechBusError as exc:
        return f"id {motor_id}: write failed ({exc})"

    try:
        after = bus.read(REGISTER, motor_id, num_retry=RETRIES)
    except FeetechBusError as exc:
        return f"id {motor_id}: wrote {value} but cannot verify ({exc})"

    verdict = "OK" if after == value else "DID NOT TAKE"
    return f"id {motor_id}: {before} -> {after}  (wanted {value})  {verdict}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", default="7,8,9")
    parser.add_argument("--value", type=int, required=True,
                        help="Return_Delay_Time counts (~2 us each); 0 restores the default")
    args = parser.parse_args()

    if not 0 <= args.value <= 254:
        parser.error("value must be 0..254")

    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    print("=" * 74)
    print(f"{REGISTER} := {args.value}  (~{args.value * 2} us)  on {args.port} ids {ids}")
    print("=" * 74)

    bus = FeetechBus(port=args.port, baudrate=args.baudrate, timeout=0.02)
    bus.connect()
    try:
        for motor_id in ids:
            print("  " + apply_to(bus, motor_id, args.value))
    finally:
        bus.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
