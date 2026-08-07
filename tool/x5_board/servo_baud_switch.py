#!/usr/bin/env python3
"""Switch one Feetech servo's baud rate, with state probing and rollback.

WHY THIS IS SAFE ON A BUS THAT LOSES ~19% OF TRANSACTIONS

The register is a single byte (``Baud_Rate``, address 6), so it is either written
or it is not -- there is no half-written state.  That makes the state space two
elements wide and, crucially, both of them are *observable*: point the host at
each candidate rate and ping.  A ping storm settles it to any confidence wanted --
at 19% loss, 25 consecutive failures has probability 1e-18, so "no answer on
either rate" is a real finding rather than bad luck.

So this tool never assumes what it did; it always measures what happened.  It is
also re-entrant: it probes first, and if the motor already answers at the target
rate it simply confirms and exits.  A run interrupted anywhere can be repeated.

THE TRAP THIS IS BUILT AROUND

Writing ``Baud_Rate`` makes the servo change rate.  Its status packet for that
very write is then sent at a rate the host is no longer listening on, so the
write *appears* to fail even when it succeeded.  Retrying then re-sends at the
old rate, which the servo can no longer hear, and the whole thing looks like a
hard failure while actually being a success.

Hence: the write goes out exactly once with num_retry=0, its exception is
swallowed, and the outcome is decided purely by the probe that follows.

READ-ONLY EXCEPT FOR ``Baud_Rate``, ``Lock`` and ``Torque_Enable``.
"""

from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, "/userdata/x5/tinynav")

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError  # noqa: E402

# Feetech control table value -> actual rate. Only the two we use are listed;
# adding more would widen the probe for no benefit.
BAUD_CODES = {0: 1_000_000, 1: 500_000}


def probe(port: str, motor: int, attempts: int = 25):
    """Find which of BAUD_CODES the motor answers on.

    Returns (code, baud, readback) or (None, None, None). `readback` is the
    Baud_Rate register value if it could be read, else None -- a ping alone
    already proves the rate, the readback is corroboration.
    """
    for code, baud in BAUD_CODES.items():
        bus = FeetechBus(port, baudrate=baud)
        try:
            bus.connect()
        except Exception as exc:
            print(f"    [probe] cannot open {port} at {baud}: {exc}")
            continue
        try:
            hits = 0
            for _ in range(attempts):
                try:
                    if bus.ping(motor):
                        hits += 1
                        if hits >= 2:  # two hits, not one: rules out a fluke
                            break
                except FeetechBusError:
                    pass
            if hits >= 2:
                try:
                    readback = bus.read("Baud_Rate", motor, num_retry=10)
                except FeetechBusError:
                    readback = None
                print(f"    [probe] motor {motor} answers at {baud} "
                      f"({hits} hits), Baud_Rate reads {readback}")
                return code, baud, readback
        finally:
            bus.disconnect()
    print(f"    [probe] motor {motor} answered on NEITHER {list(BAUD_CODES.values())}")
    return None, None, None


def set_baud(port: str, motor: int, current_baud: int, target_code: int) -> None:
    """Unlock EEPROM, write Baud_Rate once, relock is NOT done here.

    The write deliberately does not retry -- see the module docstring. Its
    exception is expected and swallowed; the caller probes to find out what
    actually happened.
    """
    bus = FeetechBus(port, baudrate=current_baud)
    bus.connect()
    try:
        # EEPROM writes need torque off and the lock cleared.
        for name, value in (("Torque_Enable", 0), ("Lock", 0)):
            try:
                bus.write(name, motor, value, num_retry=12)
                print(f"    [write] {name}={value} ok")
            except FeetechBusError as exc:
                print(f"    [write] {name}={value} FAILED: {exc}")
                raise
        time.sleep(0.05)
        print(f"    [write] Baud_Rate={target_code} (single shot, no retry)")
        try:
            bus.write("Baud_Rate", motor, target_code, num_retry=0)
            print("    [write] servo acknowledged")
        except FeetechBusError as exc:
            print(f"    [write] no ack ({type(exc).__name__}) -- expected if it "
                  "switched rate before replying; the probe decides")
    finally:
        bus.disconnect()


def relock(port: str, motor: int, baud: int) -> bool:
    """Put the EEPROM lock back. Best effort, reported honestly."""
    bus = FeetechBus(port, baudrate=baud)
    try:
        bus.connect()
    except Exception as exc:
        print(f"    [relock] cannot open: {exc}")
        return False
    try:
        bus.write("Lock", motor, 1, num_retry=15)
        value = bus.read("Lock", motor, num_retry=15)
        print(f"    [relock] Lock reads {value}")
        return value == 1
    except FeetechBusError as exc:
        print(f"    [relock] FAILED: {exc}")
        return False
    finally:
        bus.disconnect()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS3")
    ap.add_argument("--motor", type=int, default=7)
    ap.add_argument("--target", type=int, choices=sorted(BAUD_CODES), required=True,
                    help="0 = 1 Mbaud (factory default), 1 = 500 kbaud")
    ap.add_argument("--attempts", type=int, default=25)
    ap.add_argument("--max-rounds", type=int, default=3)
    args = ap.parse_args()

    target_baud = BAUD_CODES[args.target]
    print(f"=== motor {args.motor} on {args.port} -> code {args.target} "
          f"({target_baud} baud) ===")

    print("  step 1: where is it now?")
    code, baud, _ = probe(args.port, args.motor, args.attempts)
    if code is None:
        print("  ABORT: motor did not answer on any known rate. Do NOT touch other "
              "motors. Recover over USB (jumper to USB-SERVO).")
        return 2
    if code == args.target:
        print(f"  already at code {code}; ensuring the lock is set and exiting.")
        relock(args.port, args.motor, baud)
        return 0

    for round_no in range(1, args.max_rounds + 1):
        print(f"  step 2: write attempt {round_no}/{args.max_rounds} (from {baud})")
        try:
            set_baud(args.port, args.motor, baud, args.target)
        except FeetechBusError:
            print("    could not even unlock; retrying the whole round")
            code, baud, _ = probe(args.port, args.motor, args.attempts)
            if code is None:
                print("  ABORT: lost the motor during unlock.")
                return 2
            continue

        print("  step 3: what actually happened?")
        code, baud, readback = probe(args.port, args.motor, args.attempts)
        if code is None:
            print("  motor silent on both rates -- it may need a power cycle for the "
                  "new rate to take effect. STOPPING so a human can decide.")
            return 3
        if code == args.target:
            print(f"  SUCCESS: now at {baud} baud (Baud_Rate reads {readback})")
            ok = relock(args.port, args.motor, baud)
            print(f"  lock restored: {ok}")
            return 0
        print(f"  still at {baud}; the write was lost. Retrying.")

    print(f"  FAILED after {args.max_rounds} rounds, motor still at {baud} baud "
          "-- unchanged and usable.")
    relock(args.port, args.motor, baud)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
