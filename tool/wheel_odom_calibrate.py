"""Calibrate / self-check the LeKiwi wheel odometry geometry.

Why you cannot skip this
------------------------
``wheel_radius = 0.05`` and ``base_radius = 0.125`` are the *upstream LeRobot
defaults*, not measurements of your robot.  Both feed straight into the odometry
as scale factors:

* ``wheel_radius`` scales translation.  A 2% error is a 2% distance error --
  annoying but bounded, and a mapper can partly absorb it.
* ``base_radius`` scales yaw.  A 2% error is a permanent 2% heading scale error.
  Drive a 20 m loop and you come back pointing several degrees off, every single
  time, in the same direction.  Nothing in a dead-reckoning-only stack corrects
  it.

So: measure them.  This script does the arithmetic.

Three checks, in the order you should run them
----------------------------------------------
1. ``signs`` -- confirm the wheel order and the per-wheel sign.  Everything else
   is meaningless if a motor is wired backwards or the IDs are permuted.
2. ``straight`` -- push/drive the robot a tape-measured distance, recover
   ``wheel_radius``.
3. ``spin`` -- rotate the robot a whole number of turns in place, recover
   ``base_radius``.  Run this *after* ``straight``, because the yaw estimate
   depends on the wheel radius you just fixed.

Usage
-----
Manual mode (recommended first; nothing is energised, you push the robot)::

    python3 tool/wheel_odom_calibrate.py signs   --port /dev/ttyACM0
    python3 tool/wheel_odom_calibrate.py straight --port /dev/ttyACM0 --distance 2.0
    python3 tool/wheel_odom_calibrate.py spin     --port /dev/ttyACM0 --turns 5

Powered mode (the script drives the wheels itself; keep a hand on the power)::

    python3 tool/wheel_odom_calibrate.py straight --port /dev/ttyACM0 --distance 2.0 \
        --drive --speed 0.15
    python3 tool/wheel_odom_calibrate.py spin --port /dev/ttyACM0 --turns 5 \
        --drive --yaw-rate 0.6

Dry run with no hardware at all (exercises this script's own arithmetic; it
should recover ``--fake-wheel-radius`` / ``--fake-base-radius``)::

    python3 tool/wheel_odom_calibrate.py straight --fake --distance 2.0
    # the spin test needs an already-correct wheel_radius, so feed it the one
    # the straight test just recovered:
    python3 tool/wheel_odom_calibrate.py spin --fake --turns 5 --wheel-radius 0.0483

Practical notes
---------------
* Do ``straight`` on the same floor you will navigate on.  Carpet and vinyl give
  measurably different effective wheel radii on omniwheels.
* Do ``spin`` for as many turns as you can stand (5-10).  The estimate improves
  linearly with total rotation, and one turn is not enough to separate the answer
  from the start/stop transient.
* Pushing by hand is *better* than driving for the ``straight`` test: no torque
  means no slip, so you measure geometry rather than geometry plus slip.
* Repeat each run 3 times and check the spread.  If ``wheel_radius`` moves by
  more than ~1% between runs, something mechanical is loose.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tinynav.platforms.feetech_bus import FakeFeetechBus, FeetechBus  # noqa: E402
from tinynav.platforms.omni3_kinematics import (  # noqa: E402
    DEFAULT_BASE_RADIUS,
    DEFAULT_TICKS_PER_REV,
    DEFAULT_WHEEL_RADIUS,
    Omni3Kinematics,
    wrap_tick_delta,
)

WHEEL_NAMES = ("left", "back", "right")

# How long a --fake run pretends to take.  The synthesised wheel rates are then
# chosen so the robot really does travel --distance / rotate --turns in that
# window, which is what makes the dry run self-verifying: the script must recover
# --fake-wheel-radius / --fake-base-radius.
FAKE_DURATION_S = 2.0


class RealTime:
    """Wall clock: what a real calibration run uses."""

    monotonic = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


class VirtualTime:
    """Clock whose only advance is an explicit sleep().

    Used for ``--fake`` so the dry run is both instantaneous and *exact*: with a
    real clock the polling loop overshoots the deadline by one poll period, which
    shows up as a ~1% error in the recovered radius and makes it look as though
    the arithmetic were wrong.

    Time is held as integer nanoseconds rather than a float, because accumulating
    a 0.02 s step 100 times in binary floating point lands just *below* the 2 s
    deadline and buys an extra poll -- reintroducing exactly the off-by-one-period
    error this class exists to remove.
    """

    def __init__(self, t0: float = 1000.0) -> None:
        self._ns = round(t0 * 1e9)

    def monotonic(self) -> float:
        return self._ns / 1e9

    def sleep(self, dt: float) -> None:
        self._ns += round(dt * 1e9)


def make_bus(args, kin: Omni3Kinematics, clock):
    """Real bus, or a fake one synthesising exactly the ground-truth motion."""
    if not args.fake:
        return FeetechBus(port=args.port, baudrate=args.baudrate, timeout=args.timeout)

    # Synthesise a base whose true geometry differs from the nominal one, so the
    # dry run demonstrates the correction rather than printing 1.000.
    true_kin = Omni3Kinematics(
        wheel_radius=args.fake_wheel_radius,
        base_radius=args.fake_base_radius,
        ticks_per_rev=args.ticks_per_rev,
        wheel_signs=tuple(args.wheel_signs),
    )
    if args.command == "spin":
        rates = true_kin.body_to_wheel_ticks_per_s(0.0, 0.0, 2.0 * np.pi * args.turns / FAKE_DURATION_S)
    elif args.command == "straight":
        rates = true_kin.body_to_wheel_ticks_per_s(args.distance / FAKE_DURATION_S, 0.0, 0.0)
    else:
        rates = true_kin.body_to_wheel_ticks_per_s(args.speed, 0.0, 0.0)
    return FakeFeetechBus(
        args.motor_ids,
        dict(zip(args.motor_ids, rates)),
        args.ticks_per_rev,
        clock=clock.monotonic,
    )


def read_ticks(bus, motor_ids, retries: int, ticks_per_rev: int = DEFAULT_TICKS_PER_REV):
    res = bus.sync_read("Present_Position", motor_ids, num_retry=retries)
    return {i: int(res.values[i]) % ticks_per_rev for i in motor_ids}, res.t_sample


def accumulate(bus, motor_ids, retries, stop_predicate, clock, ticks_per_rev, poll_hz: float = 50.0):
    """Poll positions until ``stop_predicate()`` is true; return summed tick deltas.

    Deltas are accumulated per poll (rather than differencing only the first and
    last reading) precisely so that wheels turning through more than one
    revolution are handled: a single-turn encoder cannot tell 0.5 turns from 1.5
    turns, but the running sum of wrapped per-poll deltas can.
    """
    prev, t0 = read_ticks(bus, motor_ids, retries, ticks_per_rev)
    totals = dict.fromkeys(motor_ids, 0)
    period = 1.0 / poll_hz
    while not stop_predicate():
        clock.sleep(period)
        curr, _ = read_ticks(bus, motor_ids, retries, ticks_per_rev)
        for i in motor_ids:
            totals[i] += wrap_tick_delta(curr[i], prev[i], ticks_per_rev)
        prev = curr
    return [totals[i] for i in motor_ids], clock.monotonic() - t0


def make_stop_predicate(args, clock):
    """Either a fixed duration (``--duration``/``--fake``) or 'wait for ENTER'."""
    duration = args.duration if args.duration is not None else (FAKE_DURATION_S if args.fake else None)
    if duration is not None:
        deadline = clock.monotonic() + duration
        return lambda: clock.monotonic() >= deadline

    # Interactive: a background-free way to poll stdin without threads.
    import select

    print(">>> press ENTER when the motion is complete ...")

    def stopped() -> bool:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if ready:
            sys.stdin.readline()
            return True
        return False

    return stopped


def start_motion(args, bus, kin, motor_ids) -> None:
    if not args.drive or args.fake:
        return
    for motor_id in motor_ids:
        bus.write("Operating_Mode", motor_id, 1)
        bus.write("Torque_Enable", motor_id, 1)
        bus.write("Lock", motor_id, 1)
    if args.command == "spin":
        raw = kin.body_to_wheel_raw(0.0, 0.0, args.yaw_rate)
    else:
        raw = kin.body_to_wheel_raw(args.speed, 0.0, 0.0)
    print(f"driving wheels at Goal_Velocity {list(raw)} (ticks/s)")
    bus.sync_write("Goal_Velocity", dict(zip(motor_ids, (int(v) for v in raw))))


def stop_motion(args, bus, motor_ids) -> None:
    if not args.drive or args.fake:
        return
    bus.sync_write("Goal_Velocity", dict.fromkeys(motor_ids, 0))


def report(label: str, nominal: float, measured: float) -> None:
    dev = 100.0 * (measured - nominal) / nominal
    print(f"\n  {label}")
    print(f"    nominal (upstream default) : {nominal:.6f} m")
    print(f"    measured                   : {measured:.6f} m")
    print(f"    deviation                  : {dev:+.2f} %")
    if abs(dev) > 10.0:
        print("    !! >10% off. Suspect wrong ticks_per_rev, wrong wheel order, or a sign error.")
    elif abs(dev) > 2.0:
        print("    -> worth applying; >2% is a drift you will see on a 20 m loop.")
    else:
        print("    -> within 2% of nominal.")


# --------------------------------------------------------------------------- #
# Commands
# --------------------------------------------------------------------------- #


def cmd_signs(args, bus, kin, clock) -> int:
    ids = args.motor_ids
    print("SIGN / ORDER CHECK")
    print("  Push the robot straight FORWARD (robot +x) by ~0.5 m, then press ENTER.")
    print("  Expected for the stock LeKiwi layout (angles 150 / -90 / 30 deg):")
    nominal = kin.body_to_wheel_ticks_per_s(1.0, 0.0, 0.0)
    for name, motor_id, val in zip(WHEEL_NAMES, ids, nominal):
        print(f"    {name:<5} (id {motor_id}): sign {'+' if val >= 0 else '-'}  ({val:+.1f} ticks/s per m/s)")

    deltas, elapsed = accumulate(
        bus, ids, args.retries, make_stop_predicate(args, clock), clock, args.ticks_per_rev
    )
    print(f"\n  measured over {elapsed:.2f}s:")
    for name, motor_id, d, expect in zip(WHEEL_NAMES, ids, deltas, nominal):
        ok = "OK" if (d == 0 or np.sign(d) == np.sign(expect)) else "MISMATCH -> flip wheel_signs"
        print(f"    {name:<5} (id {motor_id}): {d:+7d} ticks   {ok}")

    body = kin.wheel_tick_delta_to_body_delta(deltas)
    print(f"\n  implied body displacement: dx={body[0]:+.4f} m  dy={body[1]:+.4f} m  dyaw={np.degrees(body[2]):+.2f} deg")
    if body[0] <= 0:
        print("  !! dx is not positive for a forward push: wheel order or signs are wrong.")
        return 1
    if abs(body[1]) > 0.3 * abs(body[0]):
        print("  !! large lateral component for a straight push: likely a permuted wheel order.")
        return 1
    if abs(np.degrees(body[2])) > 15.0:
        print("  !! large yaw for a straight push: likely a single flipped wheel.")
        return 1
    print("  signs and order look consistent.")
    return 0


def cmd_straight(args, bus, kin, clock) -> int:
    ids = args.motor_ids
    print(f"STRAIGHT TEST -> wheel_radius   (ground truth distance {args.distance:.3f} m)")
    if not args.drive:
        print("  Push the robot straight forward exactly that distance, then press ENTER.")
    start_motion(args, bus, kin, ids)
    deltas, elapsed = accumulate(
        bus, ids, args.retries, make_stop_predicate(args, clock), clock, args.ticks_per_rev
    )
    stop_motion(args, bus, ids)

    body = kin.wheel_tick_delta_to_body_delta(deltas)
    measured = float(np.hypot(body[0], body[1]))
    print(f"\n  elapsed {elapsed:.2f}s, tick deltas {deltas}")
    print(f"  odometry path length with nominal geometry: {measured:.4f} m")
    print(f"  lateral / yaw leakage: dy={body[1]:+.4f} m, dyaw={np.degrees(body[2]):+.2f} deg")
    if measured < 1e-6:
        print("  !! no motion detected.")
        return 1

    # Body displacement is exactly proportional to wheel_radius, so the
    # correction is a plain ratio.
    corrected = kin.wheel_radius * args.distance / measured
    report("wheel_radius", DEFAULT_WHEEL_RADIUS, corrected)
    print(f"\n  apply with:  -p wheel_radius:={corrected:.6f}")
    return 0


def cmd_spin(args, bus, kin, clock) -> int:
    ids = args.motor_ids
    truth_rad = 2.0 * np.pi * args.turns
    print(f"SPIN TEST -> base_radius   (ground truth {args.turns:g} turns = {np.degrees(truth_rad):.1f} deg)")
    print(f"  using wheel_radius = {kin.wheel_radius:.6f} m (run the straight test first!)")
    if not args.drive:
        print("  Rotate the robot in place by exactly that many turns, then press ENTER.")
        print("  Mark the floor and the chassis so you can hit the whole number of turns.")
    start_motion(args, bus, kin, ids)
    deltas, elapsed = accumulate(
        bus, ids, args.retries, make_stop_predicate(args, clock), clock, args.ticks_per_rev
    )
    stop_motion(args, bus, ids)

    body = kin.wheel_tick_delta_to_body_delta(deltas)
    measured_rad = abs(float(body[2]))
    print(f"\n  elapsed {elapsed:.2f}s, tick deltas {deltas}")
    print(f"  odometry yaw with nominal geometry: {np.degrees(measured_rad):.2f} deg "
          f"({measured_rad / (2 * np.pi):.4f} turns)")
    print(f"  translation leakage: dx={body[0]:+.4f} m, dy={body[1]:+.4f} m "
          "(should be near zero for a true spin-in-place)")
    if measured_rad < 1e-6:
        print("  !! no rotation detected.")
        return 1

    # yaw is proportional to wheel_radius / base_radius, so with wheel_radius
    # already fixed the correction is again a plain ratio.
    corrected = kin.base_radius * measured_rad / truth_rad
    report("base_radius", DEFAULT_BASE_RADIUS, corrected)
    print(f"\n  apply with:  -p base_radius:={corrected:.6f}")
    return 0


def cmd_monitor(args, bus, kin, clock) -> int:
    ids = args.motor_ids
    # A virtual clock makes sleep() free, so an unbounded --fake monitor would
    # spin at CPU speed forever. Cap it; on real hardware Ctrl-C is the exit.
    max_lines = 20 if args.fake else None
    print("MONITOR: live wheel readings, Ctrl-C to stop")
    prev, prev_t = read_ticks(bus, ids, args.retries, args.ticks_per_rev)
    lines = 0
    while max_lines is None or lines < max_lines:
        lines += 1
        clock.sleep(0.1)
        curr, curr_t = read_ticks(bus, ids, args.retries, args.ticks_per_rev)
        dt = max(curr_t - prev_t, 1e-6)
        deltas = [wrap_tick_delta(curr[i], prev[i], args.ticks_per_rev) for i in ids]
        vel = kin.wheel_ticks_per_s_to_body([d / dt for d in deltas])
        raw = [curr[i] for i in ids]
        print(
            f"  ticks {raw}  d {deltas}  ->  vx={vel[0]:+.3f} vy={vel[1]:+.3f} "
            f"w={np.degrees(vel[2]):+.1f}deg/s",
            flush=True,
        )
        prev, prev_t = curr, curr_t
    return 0


COMMANDS = {"signs": cmd_signs, "straight": cmd_straight, "spin": cmd_spin, "monitor": cmd_monitor}


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("command", choices=sorted(COMMANDS))
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baudrate", type=int, default=1_000_000)
    ap.add_argument("--timeout", type=float, default=0.02)
    ap.add_argument("--retries", type=int, default=2)
    ap.add_argument("--motor-ids", type=int, nargs=3, default=[7, 8, 9], metavar=("LEFT", "BACK", "RIGHT"))
    ap.add_argument("--wheel-signs", type=float, nargs=3, default=[1.0, 1.0, 1.0])
    ap.add_argument("--wheel-radius", type=float, default=DEFAULT_WHEEL_RADIUS)
    ap.add_argument("--base-radius", type=float, default=DEFAULT_BASE_RADIUS)
    ap.add_argument("--ticks-per-rev", type=int, default=DEFAULT_TICKS_PER_REV)
    ap.add_argument("--distance", type=float, default=2.0, help="straight test ground truth, metres")
    ap.add_argument("--turns", type=float, default=5.0, help="spin test ground truth, whole turns")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds instead of on ENTER")
    ap.add_argument("--drive", action="store_true", help="let this script command the wheels")
    ap.add_argument("--speed", type=float, default=0.15, help="--drive forward speed, m/s")
    ap.add_argument("--yaw-rate", type=float, default=0.6, help="--drive spin rate, rad/s")
    ap.add_argument("--fake", action="store_true", help="no hardware; synthesise motion")
    ap.add_argument("--fake-wheel-radius", type=float, default=0.0483)
    ap.add_argument("--fake-base-radius", type=float, default=0.1312)
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    kin = Omni3Kinematics(
        wheel_radius=args.wheel_radius,
        base_radius=args.base_radius,
        ticks_per_rev=args.ticks_per_rev,
        wheel_signs=tuple(args.wheel_signs),
    )
    clock = VirtualTime() if args.fake else RealTime()
    bus = make_bus(args, kin, clock)
    bus.connect()
    if args.fake:
        print(f"[fake] true wheel_radius={args.fake_wheel_radius:.6f} "
              f"base_radius={args.fake_base_radius:.6f} -- the script should recover these\n")
    try:
        return COMMANDS[args.command](args, bus, kin, clock)
    except KeyboardInterrupt:
        stop_motion(args, bus, args.motor_ids)
        return 130
    finally:
        bus.disconnect()


if __name__ == "__main__":
    sys.exit(main())
