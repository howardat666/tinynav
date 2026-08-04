#!/usr/bin/env python3
"""Identify which physical wire is a UART's TX or RX, with a multimeter.

For when the wires are soldered, undocumented, or both -- and swapping them to
find out is not an option.

    # Mode 1: make TX detectable. Probe each wire with a multimeter (DC volts).
    python3 uart_wire_id.py beacon --port /dev/ttyS3

    # Mode 2: report anything that arrives, so grounding a wire identifies RX.
    python3 uart_wire_id.py listen --port /dev/ttyS3

WHY A MULTIMETER CAN SEE THIS
-----------------------------
An idle UART line sits high (3.3 V). `beacon` transmits 0x00 back to back, which
holds the line low for 9 of every 10 bit times, so a cheap DC multimeter -- which
averages -- reads a clearly different voltage instead of a steady 3.3 V. Run it
at a low baud rate so even a slow meter settles.

Expected readings, probing each candidate wire against ground:

    ~3.3 V steady      an idle UART line, or an RX input's pull-up
    ~0.3-1.0 V         THIS IS TX while the beacon runs (mostly-low waveform)
    ~0 V               ground, or a wire connected to nothing
    fluctuating        a data line with real traffic on it

Stop the beacon and TX returns to ~3.3 V. That change is the confirmation: a wire
whose voltage tracks the beacon starting and stopping is the TX wire.

FINDING RX
----------
RX cannot be identified by probing, because it is an input and just sits at its
pull-up voltage. Drive it instead: run `listen`, then briefly connect each
candidate wire to ground. Grounding a UART's RX looks exactly like a start bit
followed by zeros, so the listener prints received bytes (usually 0x00) the
moment you touch the right wire.

    !! Use a ~1 kOhm resistor in series rather than a bare jumper. If the wire
    !! you touch turns out to be an output driving high, a bare short to ground
    !! puts the full pad current through it. 1 kOhm still pulls a 3.3 V input
    !! well below its logic threshold while limiting the current to ~3 mA.
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


def read_rx_count(port_name: str) -> int | None:
    """Kernel rx byte count, so a glitch storm can be told from real bytes."""
    index = port_name.rsplit("ttyS", 1)[-1]
    if not index.isdigit():
        return None
    try:
        with open("/proc/tty/driver/serial") as handle:
            for line in handle:
                fields = line.split()
                if fields and fields[0] == f"{index}:":
                    for field in fields:
                        if field.startswith("rx:"):
                            return int(field[3:])
    except OSError:
        return None
    return None


def cmd_beacon(args) -> int:
    link = serial.Serial(args.port, args.baudrate, timeout=0)
    # 0x00 is deliberate: one start bit plus eight zero data bits holds the line
    # low for 9 of every 10 bit times, which is the lowest average voltage a UART
    # can produce. 0x55 would average near half and be harder to distinguish
    # from a floating pin.
    block = bytes(256)
    print(f"BEACON on {args.port} at {args.baudrate} baud, sending 0x00 continuously.")
    print("Probe each candidate wire with a multimeter on DC volts, against ground.")
    print()
    print("  ~3.3 V steady   idle UART line, or an RX input pull-up")
    print("  ~0.3-1.0 V      <-- THIS IS TX")
    print("  ~0 V            ground, or a wire connected to nothing")
    print()
    print(f"Runs for {args.duration:.0f} s. Watch a candidate wire drop when this starts")
    print("and return to 3.3 V when it stops -- that change is the proof.")
    print()
    deadline = time.monotonic() + args.duration
    written = 0
    try:
        while time.monotonic() < deadline:
            written += link.write(block)
            link.flush()
            remaining = deadline - time.monotonic()
            print(f"  transmitting... {remaining:5.1f} s left, {written} bytes sent", end="\r", flush=True)
    except KeyboardInterrupt:
        print()
        print("interrupted")
    finally:
        link.close()
    print()
    print(f"beacon stopped after {written} bytes; the line is back to idle high")
    return 0


def cmd_listen(args) -> int:
    link = serial.Serial(args.port, args.baudrate, timeout=0.1)
    link.reset_input_buffer()
    rx0 = read_rx_count(args.port)
    print(f"LISTENING on {args.port} at {args.baudrate} baud. Transmitting nothing.")
    print()
    print("Touch each candidate wire to ground THROUGH A ~1 kOhm RESISTOR.")
    print("The wire that makes bytes appear below is RX.")
    print()
    print(f"Runs for {args.duration:.0f} s.")
    print()
    deadline = time.monotonic() + args.duration
    total = bytearray()
    events = 0
    try:
        while time.monotonic() < deadline:
            chunk = link.read(256)
            if chunk:
                events += 1
                total.extend(chunk)
                stamp = args.duration - (deadline - time.monotonic())
                print(f"  [{stamp:6.2f}s] {len(chunk):4d} bytes: {chunk[:16].hex(' ')}")
    except KeyboardInterrupt:
        print()
        print("interrupted")
    finally:
        link.close()

    rx1 = read_rx_count(args.port)
    print()
    print(f"total {len(total)} bytes in {events} bursts")
    if rx0 is not None and rx1 is not None:
        print(f"kernel rx counter: {rx0} -> {rx1}  (+{rx1 - rx0})")
    if not total:
        print()
        print("Nothing arrived. Either no candidate wire is this UART's RX, or the")
        print("resistor/contact did not actually pull it low. Note that grounding a")
        print("*floating* pin also produces bytes, so silence here is a stronger")
        print("result than noise: it means the pin is not reachable from that wire.")
        return 1
    zeros = sum(1 for b in total if b == 0x00)
    print(f"of which exactly 0x00: {zeros}/{len(total)}")
    print()
    if zeros > len(total) // 2:
        print("Mostly 0x00, which is what pulling RX low looks like: that wire is RX.")
    else:
        print("Bytes arrived but few are 0x00. Treat with suspicion -- fast edges on a")
        print("neighbouring line couple into a floating pin and decode as mostly-ones")
        print("bytes (0xff, 0xfe, 0xf7...). Genuine grounding gives 0x00.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("mode", choices=["beacon", "listen"])
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=9600,
        help="default 9600: slow enough for a DC multimeter to settle, and slow "
        "enough that crosstalk glitches cannot fake a frame",
    )
    parser.add_argument("--duration", type=float, default=60.0, help="seconds to run")
    args = parser.parse_args()

    return cmd_beacon(args) if args.mode == "beacon" else cmd_listen(args)


if __name__ == "__main__":
    sys.exit(main())
