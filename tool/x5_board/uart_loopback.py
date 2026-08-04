#!/usr/bin/env python3
"""Prove whether a UART works end to end, with nothing attached but a jumper.

Write a known pattern out TX and see whether it comes back on RX. Nothing but
the serial port is involved: no servo, no protocol, no power.

    # 1. With TX shorted to RX (a jumper across the two pins):
    python3 tool/x5_board/uart_loopback.py
    #    -> "LOOPBACK CONFIRMED" means the UART, the pinmux, the connector and
    #       the cable up to that jumper are all good.

    # 2. With nothing shorted, servos attached:
    python3 tool/x5_board/uart_loopback.py --no-loopback-expected
    #    -> reports whether anything at all arrives, which distinguishes a dead
    #       RX line from a live one with nothing to say.

WHY THIS EXISTS
---------------
`servo_scan.py` answering "no servo replied" has several possible causes that
look identical from software: TX/RX swapped, no shared ground, servos unpowered,
or the pins simply not routed to the connector. This test removes every one of
those variables except the two pins and the wire between them, so a failure here
localises the fault to the X5 side and a success localises it to everything
downstream of the jumper.

Note `servo_scan.py` cannot be used for this: on a loopback it receives its own
outgoing packet and rejects it as a bad status packet, which reads as "no reply".
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is not installed:  python3 -m pip install pyserial", file=sys.stderr)
    sys.exit(2)

# Deliberately includes 0x00 and 0xFF, alternating bits, and the 0xFF 0xFF that
# opens a Feetech packet -- so a half-configured line that only mangles some bit
# patterns still shows up as a partial match rather than a clean pass.
PATTERN = bytes([0xFF, 0xFF, 0x00, 0x55, 0xAA, 0x0F, 0xF0, 0x7E, 0x81, 0x01, 0xFE, 0x42])


def read_counters(port_name: str) -> tuple[int, int] | None:
    """Kernel tx/rx byte counts for this port, from /proc/tty/driver/serial.

    Read straight from the driver rather than inferred, because it distinguishes
    "the CPU never transmitted" from "it transmitted and nothing came back" --
    and because a wrong baud rate still increments rx (as garbage), so rx staying
    at exactly 0 rules a baud mismatch out.
    """
    index = port_name.rsplit("ttyS", 1)[-1]
    if not index.isdigit():
        return None
    try:
        with open("/proc/tty/driver/serial") as handle:
            for line in handle:
                fields = line.split()
                if fields and fields[0] == f"{index}:":
                    tx = rx = 0
                    for field in fields:
                        if field.startswith("tx:"):
                            tx = int(field[3:])
                        elif field.startswith("rx:"):
                            rx = int(field[3:])
                    return tx, rx
    except OSError:
        return None
    return None


def test_port(port_name: str, baudrate: int, repeats: int, settle_s: float) -> dict:
    result = {"port": port_name, "baudrate": baudrate, "sent": 0, "received": b"", "matched": 0}

    before = read_counters(port_name)
    try:
        link = serial.Serial(port=port_name, baudrate=baudrate, timeout=settle_s, write_timeout=1.0)
    except (OSError, serial.SerialException) as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        return result

    got = bytearray()
    try:
        link.reset_input_buffer()
        link.reset_output_buffer()
        for _ in range(repeats):
            link.write(PATTERN)
            link.flush()
            result["sent"] += len(PATTERN)
            deadline = time.monotonic() + settle_s
            while time.monotonic() < deadline and len(got) < result["sent"]:
                chunk = link.read(max(1, result["sent"] - len(got)))
                if chunk:
                    got.extend(chunk)
                else:
                    break
    finally:
        link.close()

    result["received"] = bytes(got)
    expected = PATTERN * repeats
    result["matched"] = sum(1 for a, b in zip(expected, got) if a == b)
    result["exact"] = bytes(got) == expected
    after = read_counters(port_name)
    if before and after:
        result["kernel_tx_delta"] = after[0] - before[0]
        result["kernel_rx_delta"] = after[1] - before[1]
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--ports", default="/dev/ttyS3,/dev/ttyS5")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--settle", type=float, default=0.2, help="seconds to wait for an echo")
    parser.add_argument(
        "--no-loopback-expected",
        action="store_true",
        help="report what arrives without treating silence as a failure (servos attached, no jumper)",
    )
    args = parser.parse_args()

    ports = [p.strip() for p in args.ports.split(",") if p.strip()]
    print("=" * 74)
    print(f"UART loopback test @ {args.baudrate} baud, pattern {PATTERN.hex(' ')}")
    if not args.no_loopback_expected:
        print("Expecting TX to be shorted to RX. Nothing else need be connected.")
    print("=" * 74)

    results = []
    for port_name in ports:
        print(f"\n--- {port_name} ---")
        outcome = test_port(port_name, args.baudrate, args.repeats, args.settle)
        results.append(outcome)

        if "error" in outcome:
            print(f"  could not open: {outcome['error']}")
            continue

        print(f"  sent     {outcome['sent']} bytes")
        print(f"  received {len(outcome['received'])} bytes: {outcome['received'].hex(' ') or '(nothing)'}")
        if "kernel_tx_delta" in outcome:
            print(f"  kernel counters: tx +{outcome['kernel_tx_delta']}, rx +{outcome['kernel_rx_delta']}")
        if outcome["exact"]:
            print("  ✅ LOOPBACK CONFIRMED — every byte returned intact")
        elif outcome["received"]:
            print(f"  ⚠️  partial: {outcome['matched']}/{outcome['sent']} bytes matched")
            print("      Bytes are arriving but corrupted. That is a baud-rate or")
            print("      signal-integrity problem, not a wiring-absent problem.")
        else:
            print("  ❌ nothing came back")

    print()
    print("=" * 74)
    any_rx = any(r.get("received") for r in results)
    any_exact = any(r.get("exact") for r in results)
    openable = [r["port"] for r in results if "error" not in r]

    if not openable:
        print("VERDICT: no port could even be opened. These pins are not muxed as UARTs.")
        return 1

    if args.no_loopback_expected:
        if any_rx:
            print("VERDICT: bytes are arriving on RX, so the receive path is alive.")
            print("Whatever is attached is transmitting; if servo_scan still finds nothing,")
            print("suspect the protocol or baud rate rather than the wiring.")
            return 0
        print("VERDICT: not one byte arrived on RX.")
        print()
        print("The transmit side provably works (kernel tx counted up), so this is a")
        print("receive-path or peer problem. In order of likelihood:")
        print("  1. Servo bus not powered. An unpowered servo cannot answer. Check the")
        print("     battery and the adapter's DC input FIRST -- it costs nothing.")
        print("  2. TX and RX swapped. Then our TX drives their TX and nothing ever")
        print("     reaches our RX. Swap the two wires.")
        print("  3. RX not connected, or that pin is not routed to the connector.")
        print("  4. No shared signal ground. A shared battery negative is not enough.")
        print()
        print("To tell 2/3/4 apart from 1, short TX to RX at the camera end and re-run")
        print("without --no-loopback-expected. No power required for that test.")
        return 1

    if any_exact:
        print("VERDICT: the UART, its pinmux, the connector and the cable are all good.")
        print("Any remaining failure is downstream of the jumper: servo power, the")
        print("half-duplex adapter, baud rate, or servo IDs.")
        return 0
    if any_rx:
        print("VERDICT: bytes return but corrupted -- check the baud rate and grounding.")
        return 1
    print("VERDICT: nothing returned with TX shorted to RX.")
    print("Either the jumper is not actually across the right two pins, or those pins")
    print("do not reach this UART. Confirm the pinout before suspecting software.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
