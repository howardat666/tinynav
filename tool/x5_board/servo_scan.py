#!/usr/bin/env python3
"""Find the Feetech servo bus. READ-ONLY: this script never writes a register.

This is the first thing to run once the Looper camera's UART is wired to the
Waveshare Bus Servo Adapter. It answers one question: which serial port, at
which baud rate, has which servo IDs on it?

    python3 tool/x5_board/servo_scan.py                     # ttyS3 + ttyS5 @ 1 Mbaud
    python3 tool/x5_board/servo_scan.py --all-baudrates     # sweep every rate
    python3 tool/x5_board/servo_scan.py --ports /dev/ttyACM0  # USB adapter on a laptop

WHY READ-ONLY MATTERS
---------------------
Every transaction here is a PING or a READ. No register is written, so nothing
can move and no EEPROM can be altered -- including by a wiring mistake. That is
deliberate: bring-up must be able to fail without damaging hardware. The wheels
only turn once you run the odometry node with ``enable_wheel_command:=true``,
which is a separate and explicit step.

WHY ttyS3 AND ttyS5 ARE THE DEFAULTS
------------------------------------
On the Looper's D-Robotics X5 those are the only two free UARTs, and both are
genuinely pinmuxed out to pads: ``uart3grp`` on pins 10/11 and ``uart5grp`` on
pins 28/29, unlike uart1/2/4/7 whose pins are left as GPIO. ttyS0 is the boot
console at 921600 and must not be touched. Which of the two reaches the camera's
10-pin GH1.25 connector is a PCB question that no datasheet we have answers, so
this script simply tries both and tells you which one replied.

It uses ``tinynav.platforms.feetech_bus``, the same bus implementation the
odometry node uses, so a successful scan here means the real driver will work
too -- and so that this tool adds no dependency beyond pyserial.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError

# Purely informational reads. All are read-only registers or registers we only
# ever read; nothing here is written.
PROBE_REGISTERS = (
    "Model_Number",
    "ID",
    "Operating_Mode",
    "Torque_Enable",
    "Lock",
    "Goal_Velocity",
    "Present_Position",
    "Present_Velocity",
    "Present_Voltage",
    "Present_Temperature",
)

MODEL_NAMES = {777: "sts3215", 2825: "sts3250", 11272: "sm8512bl", 1284: "scs0009"}

# LeKiwi's three base wheels. An arm, if fitted, is IDs 1-6.
LEKIWI_WHEEL_IDS = {7: "left", 8: "back", 9: "right"}

# The rates Feetech servos can be configured for. 1 Mbaud is the factory default
# and what LeKiwi ships with, so it is tried alone unless --all-baudrates.
FEETECH_BAUDRATES = (1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600, 38_400, 19_200)


def probe_one(bus: FeetechBus, motor_id: int) -> dict[str, int | None] | None:
    """PING then read the informational registers. None if the servo is absent."""
    if not bus.ping(motor_id, num_retry=2):
        return None

    info: dict[str, int | None] = {}
    for name in PROBE_REGISTERS:
        try:
            info[name] = bus.read(name, motor_id, num_retry=1)
        except FeetechBusError:
            # A servo that pings but will not answer a particular read is itself
            # a finding -- usually a marginal link. Record it and carry on.
            info[name] = None
    return info


def scan_port(port_name: str, baudrates, ids, verbose: bool) -> dict[int, dict[int, dict]]:
    """Return {baudrate: {motor_id: info}} for one port."""
    found: dict[int, dict[int, dict]] = {}
    for baud in baudrates:
        bus = FeetechBus(port=port_name, baudrate=baud)
        try:
            bus.connect()
        except (OSError, FeetechBusError) as exc:
            # In use by another process, or no permission. Normal at bring-up and
            # not a reason to abandon the remaining ports. (pyserial's
            # SerialException is an OSError subclass.)
            print(f"    {baud:>8} baud: cannot open ({type(exc).__name__}: {exc})")
            continue

        at_this_baud: dict[int, dict] = {}
        try:
            for motor_id in ids:
                info = probe_one(bus, motor_id)
                if info is None:
                    if verbose:
                        print(f"    {baud:>8} baud id {motor_id:>3}: no reply")
                    continue
                at_this_baud[motor_id] = info
                volts = info["Present_Voltage"]
                print(
                    f"    {baud:>8} baud id {motor_id:>3}: FOUND  "
                    f"model={MODEL_NAMES.get(info['Model_Number'], info['Model_Number'])}  "
                    f"pos={info['Present_Position']}  "
                    f"vel={info['Present_Velocity']}  "
                    f"mode={info['Operating_Mode']}  "
                    f"torque={info['Torque_Enable']}  "
                    f"lock={info['Lock']}  "
                    f"{'?' if volts is None else format(volts / 10.0, '.1f')}V  "
                    f"{info['Present_Temperature']}C"
                )
        finally:
            bus.disconnect()

        if at_this_baud:
            found[baud] = at_this_baud
    return found


def print_verdict(results: dict[str, dict[int, dict[int, dict]]]) -> int:
    live = {port: per_baud for port, per_baud in results.items() if per_baud}
    if not live:
        print("VERDICT: no servos answered on any port.")
        print()
        print("Work through these in order -- the first three cover almost every case:")
        print("  1. TX and RX swapped. By far the most common cause. Swap them, re-run.")
        print("  2. No common ground between the X5 and the adapter. Signal ground must be shared;")
        print("     a shared battery negative is not automatically a shared signal ground.")
        print("  3. Adapter not powered. The servo bus needs its DC input up -- the UART header")
        print("     alone does not power the servos, and an unpowered servo cannot reply.")
        print("  4. Wrong baud rate: re-run with --all-baudrates.")
        print("  5. Wrong port: if neither ttyS3 nor ttyS5 answers and both exist, the pins may")
        print("     not be routed to the connector at all. That is a PCB question, not software.")
        print("  6. IDs outside the scanned range: widen with --ids 1-253.")
        return 1

    print("VERDICT:")
    for port_name, per_baud in live.items():
        for baud, per_id in per_baud.items():
            found_ids = sorted(per_id)
            print(f"  {port_name} @ {baud} baud: ids {found_ids}")

            wheels = {i: LEKIWI_WHEEL_IDS[i] for i in found_ids if i in LEKIWI_WHEEL_IDS}
            if len(wheels) == 3:
                print(f"      all three LeKiwi base wheels present: {wheels}")
            elif wheels:
                missing = {i: n for i, n in LEKIWI_WHEEL_IDS.items() if i not in wheels}
                print(f"      PARTIAL base: found {wheels}, missing {missing}")
                print("      A missing wheel is usually a daisy-chain break after the last found ID.")
            else:
                print(f"      note: none of the expected LeKiwi wheel IDs {sorted(LEKIWI_WHEEL_IDS)} are here.")
                print("      Either the IDs were reassigned, or this is the arm and not the base.")

            locked = {i: per_id[i]["Lock"] for i in found_ids if per_id[i]["Lock"] not in (0, None)}
            if locked:
                print(f"      EEPROM locked on {sorted(locked)} -- normal, and expected.")
                print("      Operating_Mode lives in EEPROM, so the driver clears Lock, writes the")
                print("      mode, then re-locks. It reads the mode back to prove it took effect,")
                print("      because a locked write is acknowledged and silently discarded.")

            low_volts = {
                i: per_id[i]["Present_Voltage"] / 10.0
                for i in found_ids
                if per_id[i]["Present_Voltage"] is not None and per_id[i]["Present_Voltage"] < 60
            }
            if low_volts:
                print(f"      WARNING: bus voltage below 6.0 V on {low_volts} -- check the battery;")
                print("      servos brown out under load long before they stop answering pings.")

    print()
    first_port = next(iter(live))
    first_baud = next(iter(live[first_port]))
    print("Next steps, in order:")
    print("  1. Read-only odometry (wheels stay passive, push the chassis by hand and watch the pose):")
    print("       ros2 run tinynav wheel_odometry_node --ros-args \\")
    print(f"         -p port:={first_port} -p baudrate:={first_baud} -p publish_tf:=false")
    print("  2. Only then enable actuation, with the chassis up on blocks:")
    print("       ros2 run tinynav wheel_odometry_node --ros-args \\")
    print(f"         -p port:={first_port} -p baudrate:={first_baud} -p enable_wheel_command:=true")
    print("     and command it with small values on /lekiwi_control/cmd_vel.")
    print("  3. Calibrate wheel_radius and base_radius with tool/wheel_odom_calibrate.py before")
    print("     trusting the pose: the 0.05 / 0.125 defaults are upstream's, not measurements.")
    return 0


def parse_ids(spec: str) -> list[int]:
    ids: list[int] = []
    for chunk in spec.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "-" in chunk:
            lo, hi = chunk.split("-", 1)
            ids.extend(range(int(lo), int(hi) + 1))
        else:
            ids.append(int(chunk))
    return ids


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--ports",
        default="/dev/ttyS3,/dev/ttyS5",
        help="comma-separated ports to try (default: the X5's two free UARTs)",
    )
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="default 1000000 (Feetech factory)")
    parser.add_argument("--all-baudrates", action="store_true", help="sweep every Feetech rate")
    parser.add_argument("--ids", default="1-20", help="IDs to probe, e.g. '7-9' or '1,7,8,9' (default 1-20)")
    parser.add_argument("--timeout", type=float, default=0.02, help="per-transaction read timeout, seconds")
    parser.add_argument("-v", "--verbose", action="store_true", help="also print IDs that did not reply")
    args = parser.parse_args()

    ids = parse_ids(args.ids)
    baudrates = list(FEETECH_BAUDRATES) if args.all_baudrates else [args.baudrate]
    ports = [p.strip() for p in args.ports.split(",") if p.strip()]

    print("=" * 78)
    print("Feetech servo bus scan -- READ-ONLY: no register is written, nothing can move")
    print("=" * 78)
    print(f"ports     : {', '.join(ports)}")
    print(f"baudrates : {', '.join(str(b) for b in baudrates)}")
    print(f"ids       : {args.ids}  ({len(ids)} ids)")
    print()

    results: dict[str, dict[int, dict[int, dict]]] = {}
    for port_name in ports:
        print(f"--- {port_name} ---")
        if not os.path.exists(port_name):
            print("    does not exist, skipped")
            print()
            continue
        found = scan_port(port_name, baudrates, ids, args.verbose)
        if not found:
            print("    no servo answered")
        results[port_name] = found
        print()

    print("=" * 78)
    return print_verdict(results)


if __name__ == "__main__":
    sys.exit(main())
