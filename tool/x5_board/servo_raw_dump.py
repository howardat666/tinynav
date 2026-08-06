#!/usr/bin/env python3
"""Dump the raw bytes a SYNC READ actually brings back, and classify them.

Roughly 20% of transactions on the X5's ttyS3 path vanish while the UART reports no
framing error, no break and no overrun -- so the bytes are not being corrupted in
transit, something else is. Every remaining hypothesis makes a different prediction
about what is physically in the receive buffer, so stop inferring and look:

  echo        the buffer opens with a copy of the query we just sent. Feetech status
              packets and query packets share the 0xFF 0xFF header, so an echoed
              query parses as a valid packet from id 0xFE, the reader discards it as
              "not one of my ids", and every subsequent read is then off by the
              echo's length -- which fails on checksum, with nothing for the UART
              layer to flag. This is what a half-duplex adapter does when its
              direction control does not mute RX during TX.
  short       fewer reply bytes than asked for. Replies really are being suppressed
              or truncated on the bus.
  nothing     zero bytes back. The query never reached the servos at all.
  clean       exactly the expected reply bytes, correctly framed.

The mix across many trials is the answer. An intermittent fault needs the *rate* of
each class, which is why this repeats rather than dumping once.

READ-ONLY: only INST_SYNC_READ is ever sent, which cannot change a register.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections import Counter

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import serial

from tinynav.platforms.feetech_bus import (
    BROADCAST_ID,
    INST_SYNC_READ,
    FeetechBus,
    checksum,
)

# Present_Position: 2 bytes at address 56 on the STS/SMS series. Taken from the bus
# class itself rather than hardcoded, so a table change cannot silently desync this.
REGISTER = "Present_Position"


def build_query(addr: int, length: int, ids: list[int]) -> bytes:
    params = [addr, length, *(i & 0xFF for i in ids)]
    body = [BROADCAST_ID & 0xFF, len(params) + 2, INST_SYNC_READ, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def parse_reply_ids(got: bytes) -> tuple[int, ...]:
    """IDs of the well-formed status packets in the buffer, in arrival order.

    Arrival order is the whole point: SYNC READ makes each servo answer in the slot
    its ID occupies in the query, so a reply set of (7,) or (7,8) is the burst being
    cut short, while (8,9) or (7,9) would mean individual replies are going missing
    independently of position. Those two say different things about the fault, and
    counting per-ID absence cannot tell them apart.
    """
    ids: list[int] = []
    i, n = 0, len(got)
    while i + 4 <= n:
        if got[i] != 0xFF or got[i + 1] != 0xFF:
            i += 1
            continue
        length = got[i + 3]
        total = 4 + length  # FF FF id len <err..data..chk>, len counts err..chk
        if i + total > n:
            break
        body = got[i + 2:i + total - 1]
        if checksum(body) == got[i + total - 1]:
            ids.append(got[i + 2])
            i += total
        else:
            i += 1
    return tuple(ids)


def classify(query: bytes, got: bytes, expected_len: int) -> str:
    if not got:
        return "nothing"
    if got.startswith(query):
        return "echo"
    # A partial echo still counts as an echo: the diagnosis is the same and the
    # reader desynchronises either way.
    if len(query) >= 4 and got.startswith(query[:4]):
        return "echo-partial"
    if len(got) < expected_len:
        return "short"
    if len(got) > expected_len:
        return "extra"
    return "clean"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", default="7,8,9")
    parser.add_argument("--trials", type=int, default=60)
    parser.add_argument("--settle-ms", type=float, default=30.0,
                        help="how long to keep collecting after the query is sent")
    parser.add_argument("--show", type=int, default=8, help="hexdump the first N trials")
    parser.add_argument("--preamble", type=int, default=0,
                        help="leading 0x00 bytes to send before the query, to assert the "
                             "adapter's derived TxEn before the real packet starts")
    args = parser.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    # Borrow the bus only to resolve the register address, then talk to the port
    # directly -- the whole point is to bypass _receive_status's framing.
    probe = FeetechBus(port=args.port, baudrate=args.baudrate, timeout=0.02)
    addr, length = probe._address_of(REGISTER)

    query = build_query(addr, length, ids)
    # A 0x00 byte at 8N1 is a start bit plus eight zero bits: nearly a full byte-time
    # of line activity, which is the strongest possible trigger for a TxEn one-shot.
    # Servos scan for the 0xFF 0xFF header, so leading zeros are skipped, and 0x00
    # cannot begin a packet -- there is no way for a preamble to be misread as one.
    sent = bytes(args.preamble) + query
    # FF FF id len err <data...> checksum
    expected_len = (6 + length) * len(ids)

    print("=" * 78)
    print(f"raw dump on {args.port} @ {args.baudrate} baud, ids={ids}")
    print(f"query  ({len(query)} B): {query.hex(' ')}")
    print(f"preamble             : {args.preamble} x 0x00 -> {len(sent)} B on the wire")
    print(f"expect ({expected_len} B): {len(ids)} x {6 + length} byte status packets")
    print(f"{args.trials} trials, {args.settle_ms:.0f} ms collection window -- READ-ONLY")
    print("=" * 78)

    ser = serial.Serial(args.port, args.baudrate, timeout=0.0)
    counts: Counter[str] = Counter()
    id_patterns: Counter[tuple[int, ...]] = Counter()
    completion_ms: list[float] = []
    try:
        for trial in range(args.trials):
            ser.reset_input_buffer()
            ser.write(sent)
            ser.flush()
            t_send = time.monotonic()
            buf = bytearray()
            t_last_byte = None
            deadline = t_send + args.settle_ms / 1000.0
            while time.monotonic() < deadline:
                n = ser.in_waiting
                if n:
                    buf.extend(ser.read(n))
                    t_last_byte = time.monotonic()
                else:
                    time.sleep(0.0005)
            got = bytes(buf)
            # Echo is checked against what actually went on the wire, preamble included.
            kind = classify(sent, got, expected_len)
            counts[kind] += 1
            id_patterns[parse_reply_ids(got)] += 1
            # When the last byte landed, relative to the send. lerobot issue #526
            # reports a case where the fix was simply a longer timeout -- the bytes
            # were arriving, just after the driver had given up. Our driver waits
            # 20 ms, so any completion beyond that is a loss the driver invents.
            if t_last_byte is not None:
                completion_ms.append((t_last_byte - t_send) * 1000.0)
            if trial < args.show:
                print(f"  trial {trial:>3}  {kind:<13} {len(got):>3} B  {got[:40].hex(' ')}")
            time.sleep(0.02)
    finally:
        ser.close()

    print("-" * 78)
    total = sum(counts.values())
    for kind, n in counts.most_common():
        print(f"  {kind:<13} {n:>4}  ({100.0 * n / total:5.1f}%)")

    print("-" * 78)
    print("  which IDs answered, in arrival order:")
    queried = tuple(ids)
    prefix_only = True
    for pattern, n in id_patterns.most_common():
        note = ""
        if pattern and pattern != queried[:len(pattern)]:
            note = "  <-- NOT a prefix of the query"
            prefix_only = False
        label = ",".join(str(i) for i in pattern) if pattern else "(none)"
        print(f"    {label:<14} {n:>4}  ({100.0 * n / total:5.1f}%){note}")
    if prefix_only:
        print("    every pattern is a PREFIX of the query order: replies are cut off")
        print("    partway through the burst, never dropped out of the middle. So the")
        print("    burst is being truncated, not individual servos going silent -- and")
        print("    the last servo in the chain is not specially disadvantaged.")
    else:
        print("    some patterns skip a servo in the middle, so individual replies go")
        print("    missing independently of position in the burst.")
    print("  when the last byte arrived, relative to the send:")
    if completion_ms:
        ordered = sorted(completion_ms)
        late = sum(1 for t in ordered if t > 20.0)
        print(f"    min {ordered[0]:.2f} ms   median {ordered[len(ordered) // 2]:.2f} ms   "
              f"max {ordered[-1]:.2f} ms")
        print(f"    arrived AFTER the driver's 20 ms timeout: {late} of {len(ordered)} "
              f"({100.0 * late / len(ordered):.1f}%)")
        if late == 0:
            print("      -> nothing is merely late. A longer timeout would change nothing,")
            print("         so this is genuine loss, not the lerobot #526 timeout case.")
        else:
            print("      -> some replies land after the driver has given up: raising")
            print("         FeetechBus(timeout=...) would recover exactly these.")
    else:
        print("    no trial received a single byte")
    print("-" * 78)
    echo = counts["echo"] + counts["echo-partial"]
    if echo:
        print(f"VERDICT: TX echo present in {100.0 * echo / total:.1f}% of trials.")
        print("  The adapter is not muting RX during TX. Every echoed query desyncs the")
        print("  reader, which is why the loss carries no UART error. Fixable in software")
        print("  by consuming exactly len(query) echo bytes after each send.")
    elif counts["nothing"]:
        print(f"VERDICT: {100.0 * counts['nothing'] / total:.1f}% of queries got NOTHING back.")
        print("  No echo, no reply: the query is not reaching the servos, or their reply")
        print("  is being suppressed outright. That is electrical, not framing.")
    else:
        print("VERDICT: no echo and every query answered -- the loss is in the framing")
        print("  layer above this dump, not on the wire.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
