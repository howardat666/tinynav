#!/usr/bin/env python3
"""Report the things about a UART that decide whether 1 Mbaud is even achievable.

Two of the remaining hypotheses for the ttyS3 servo-bus loss live entirely in
properties Linux does not expose through /proc, and neither has an error counter:

  baud rate error   The 16550A divides baud_base by an integer. If baud_base is not
                    a multiple of the requested rate, the achievable rate is off by
                    whatever the rounding costs, and the receiver's sampling drifts
                    across each frame. Past roughly 2% the far end starts mis-framing
                    -- intermittently at first, which is exactly what an intermittent
                    fault looks like.
  TX FIFO depth     A packet shorter than the FIFO goes out in one gapless burst
                    regardless of CPU load. A packet longer than it needs the CPU to
                    refill mid-transmission, so under load the transmission develops
                    inter-byte gaps -- and a half-duplex adapter whose direction
                    control is retriggered by TX activity can read a gap as
                    "transmission finished" and turn the bus around mid-packet.
                    There is no counter for a TX FIFO running dry; `oe` counts
                    receive overruns only. This is the blind spot.

Also reports ASYNC_LOW_LATENCY, which controls whether the driver defers pushing
received bytes up on a timer.

READ-ONLY: opens the port and issues TIOCGSERIAL. Sends nothing, writes no register.
"""

from __future__ import annotations

import argparse
import fcntl
import struct
import sys

import serial

TIOCGSERIAL = 0x541E

# struct serial_struct, first eight fields -- all that is needed here.
_FMT = "iiIiiiii"
_FIELDS = (
    "type", "line", "port", "irq", "flags",
    "xmit_fifo_size", "custom_divisor", "baud_base",
)

# include/uapi/linux/serial.h
_FLAG_BITS = {
    "ASYNC_HUP_NOTIFY": 0x0001,
    "ASYNC_FOURPORT": 0x0002,
    "ASYNC_SAK": 0x0004,
    "ASYNC_SPLIT_TERMIOS": 0x0008,
    "ASYNC_SPD_HI": 0x0010,
    "ASYNC_SPD_VHI": 0x0020,
    "ASYNC_SKIP_TEST": 0x0040,
    "ASYNC_AUTO_IRQ": 0x0080,
    "ASYNC_SESSION_LOCKOUT": 0x0100,
    "ASYNC_PGRP_LOCKOUT": 0x0200,
    "ASYNC_CALLOUT_NOHUP": 0x0400,
    "ASYNC_LOW_LATENCY": 0x2000,
}

UART_TYPES = {0: "none", 1: "8250", 2: "16450", 3: "16550", 4: "16550A", 5: "Cirrus",
              6: "16650", 7: "16650V2", 8: "16750", 9: "Startech", 10: "16C950",
              11: "16654", 12: "16850", 13: "RSA"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--query-bytes", type=int, default=11,
                        help="length of the SYNC READ query, to compare against the TX FIFO")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baudrate, timeout=0.0)
    try:
        buf = bytearray(128)
        fcntl.ioctl(ser.fileno(), TIOCGSERIAL, buf, True)
        vals = dict(zip(_FIELDS, struct.unpack_from(_FMT, buf)))
    finally:
        ser.close()

    print("=" * 74)
    print(f"UART facts for {args.port}")
    print("=" * 74)
    print(f"  type            : {UART_TYPES.get(vals['type'], vals['type'])}")
    print(f"  baud_base       : {vals['baud_base']}")
    print(f"  xmit_fifo_size  : {vals['xmit_fifo_size']}")
    print(f"  custom_divisor  : {vals['custom_divisor']}")
    set_flags = [name for name, bit in _FLAG_BITS.items() if vals["flags"] & bit]
    print(f"  flags           : 0x{vals['flags']:08x}  {' '.join(set_flags) or '(none)'}")

    print("-" * 74)
    base, want = vals["baud_base"], args.baudrate
    if base > 0:
        divisor = round(base / want) or 1
        actual = base / divisor
        err = 100.0 * (actual - want) / want
        print(f"  requested {want} baud -> divisor {divisor} -> actual {actual:.1f} baud")
        print(f"  baud rate error : {err:+.3f}%")
        if abs(err) < 0.5:
            print("    -> exact enough; baud rate error is NOT the fault")
        elif abs(err) < 2.0:
            print("    -> marginal: tolerable alone, but stacks with the servo's own error")
        else:
            print("    -> TOO LARGE: this alone explains intermittent mis-framing")
    else:
        print("  baud_base is 0 -- the driver does not report a divisor for this port")

    print("-" * 74)
    fifo = vals["xmit_fifo_size"]
    if fifo <= 0:
        print(f"  TX FIFO size not reported; cannot rule out mid-packet refill")
    elif args.query_bytes <= fifo:
        print(f"  {args.query_bytes} byte query fits in the {fifo} byte TX FIFO")
        print("    -> one write() goes out gaplessly even under load; TX underrun is NOT")
        print("       the fault for a packet this short")
    else:
        print(f"  {args.query_bytes} byte query EXCEEDS the {fifo} byte TX FIFO")
        print("    -> the CPU must refill mid-transmission, so load can inject inter-byte")
        print("       gaps. No counter exists for this. Prime suspect.")
    if "ASYNC_LOW_LATENCY" not in set_flags:
        print("  ASYNC_LOW_LATENCY is off: received bytes are pushed up on a timer, which")
        print("  adds latency but cannot lose bytes. Worth setting, not a root cause.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
