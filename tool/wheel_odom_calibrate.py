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
   ``wheel_radius``.  Measure *two* numbers at the finish: how far forward it
   got (``--forward``) and how far sideways it ended up (``--lateral``, + left).
   The sideways miss hardly changes the recovered radius, but it is the only
   external check on the travel *direction*, and hence on whether the three
   wheels are scaled consistently relative to one another.
3. ``spin`` -- rotate the robot a whole number of turns in place, recover
   ``base_radius``.  Run this *after* ``straight``, because the yaw estimate
   depends on the wheel radius you just fixed.

Usage
-----
Step 1 is a hand push: nothing is energised and it only needs to establish a
sign, so slip does not matter::

    python3 tool/wheel_odom_calibrate.py signs --port /dev/ttyS3

Steps 2 and 3 drive the wheels -- keep a hand on the power.  Step 3 is
deliberately given no ``--duration``, so it turns until you press ENTER::

    python3 tool/wheel_odom_calibrate.py straight --port /dev/ttyS3 \
        --drive --speed 0.12 --duration 25 --measure-after
    python3 tool/wheel_odom_calibrate.py spin --port /dev/ttyS3 \
        --drive --yaw-rate 0.3 --turns 4 --wheel-radius <from step 2>
    python3 tool/wheel_odom_calibrate.py spin --port /dev/ttyS3 \
        --drive --yaw-rate -0.3 --turns 4 --wheel-radius <from step 2>

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
* Run ``spin`` with ``--drive`` and **no** ``--duration``, so it turns until you
  press ENTER, and press it at the instant your floor and chassis marks realign
  on a whole turn.  Then ``--turns`` is exact by construction and the only error
  is your reaction time -- 0.3 s at 0.3 rad/s is 5 degrees, i.e. 0.35%.  Driving
  for a fixed time and afterwards estimating "about 4 1/8 turns" is worth only
  about +/-5%: two such runs on this robot disagreed by 5.3%, and the tick totals
  proved the error was in the count, not the wheels.  Judging an *event* beats
  estimating a *quantity*.
* Run ``spin`` in both directions.  The two results check each other, and it
  unwinds the tether: four turns wraps a USB cable four times round the chassis,
  and its restoring torque resists the run that wound it while assisting the run
  that unwinds it, biasing the two estimates in opposite directions.
* **Do not push by hand.**  This file used to recommend it, reasoning that no
  torque means no slip.  On an omni base that is backwards: pushing at one point
  yaws the chassis and skids the rollers sideways.  Measured on this robot, a
  hand push gave 23.7 degrees of yaw and 10.8% left/right asymmetry where driving
  all three wheels gave 0.17 degrees and 0.06%, and two hand-pushed runs
  under-rotated by 11.4% and 11.0% -- enough to report a ``wheel_radius`` 12.8%
  *above* the geometric radius, which is impossible, since a loaded roller's
  effective radius can only be smaller.
* Repeat each run 3 times and check the spread.  If ``wheel_radius`` moves by
  more than ~1% between runs, something mechanical is loose.
* Sanity-check the result against a tape, but do not let the tape win a
  disagreement.  ``base_radius`` is not really a distance -- it is the
  coefficient mapping wheel rotation to base yaw -- and measuring to the base
  *centre*, an imaginary point, came out 5.4% off on this robot.  If you want an
  independent geometric check, measure between two wheels, where both ends are
  physical, and use ``base_radius = d / sqrt(3)``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tinynav.platforms.feetech_bus import (  # noqa: E402
    FakeFeetechBus,
    FeetechBus,
    configure_velocity_mode,
)
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
        # Synthesise the sideways miss too, so a --fake run with --lateral still
        # recovers --fake-wheel-radius exactly and the direction check reads 0.
        forward, lateral = straight_preset(args)
        rates = true_kin.body_to_wheel_ticks_per_s(
            forward / FAKE_DURATION_S, lateral / FAKE_DURATION_S, 0.0
        )
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
    # The EEPROM unlock sequence and its read-back live in feetech_bus, shared
    # with the odometry node and the yaw comparison tool; see that docstring for
    # why a locked write is acknowledged and then silently discarded.
    configure_velocity_mode(bus, motor_ids)
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


def driven_accumulate(args, bus, kin, motor_ids, clock):
    """start_motion -> accumulate -> stop_motion, with the stop guaranteed.

    With ``--drive`` the wheels are energised for the whole run, so the stop must
    survive anything raised in between: Ctrl-C at the ENTER prompt, a bus error
    mid-read, an unexpected exception. Without the finally, the base drives away
    while the traceback prints.
    """
    start_motion(args, bus, kin, motor_ids)
    try:
        return accumulate(
            bus, motor_ids, args.retries, make_stop_predicate(args, clock), clock, args.ticks_per_rev
        )
    finally:
        stop_motion(args, bus, motor_ids)


def prompt_float(label: str, unit: str, sign: str = "positive") -> float:
    """Read one number from the operator, re-asking until it parses.

    ``sign`` is ``"positive"``, ``"nonzero"`` or ``"any"``; a lateral offset must
    accept 0 and both signs, while a distance or a turn count must not be 0.
    """
    while True:
        raw = input(f"\n  {label} in {unit}: ").strip()
        try:
            value = float(raw)
        except ValueError:
            print("  not a number, try again")
            continue
        if sign == "positive" and value <= 0:
            print("  must be positive")
            continue
        if sign == "nonzero" and value == 0:
            print("  must be non-zero")
            continue
        return value


def ground_truth(args, label: str, unit: str, preset: float) -> float:
    """The externally measured truth, asked for after the motion if requested.

    With --drive you cannot reliably stop on a mark: by the time you react the
    base has moved on. Driving for a fixed time and *then* measuring what
    actually happened removes that error entirely, and it is the only part of
    this calibration that is not derived from the encoders -- so it is worth
    getting right rather than approximating.
    """
    if not args.measure_after:
        return preset
    return prompt_float(f"measure the actual {label} now and enter it", unit)


def straight_preset(args) -> tuple[float, float]:
    """Ground-truth displacement for the straight test, as (forward, left).

    ``--distance`` is kept as the one-dimensional spelling of ``--forward`` so
    older command lines keep working.
    """
    forward = args.distance if args.forward is None else args.forward
    return float(forward), float(args.lateral)


def straight_ground_truth(args, preset: tuple[float, float]) -> tuple[float, float]:
    """(forward, left) displacement measured with a tape after the motion.

    Two numbers rather than one because a real straight run does not end on the
    line it started on. The sideways miss barely changes the *length* of the
    displacement -- 80 mm across 3 m lengthens it by 0.03% -- so it hardly moves
    ``wheel_radius``. What it does give is the travel *direction*, which is the
    only external check on whether the three wheels are scaled consistently
    relative to one another; a magnitude-only comparison cannot see that at all.
    """
    if not args.measure_after:
        return preset
    forward = prompt_float(
        "measure the FORWARD distance travelled and enter it", "metres (negative if you drove in reverse)",
        sign="nonzero",
    )
    lateral = prompt_float(
        "now the SIDEWAYS miss at the finish", "metres, + = left, - = right, 0 if none", sign="any"
    )
    return forward, lateral


def log_run(args, record: dict) -> None:
    """Append one JSON line describing the encoder half of a run.

    Called *before* the ground truth is asked for, because the encoder deltas are
    the part that cannot be recovered: a run that ends at the measurement prompt
    -- interrupted, mistyped, or just abandoned -- used to lose them entirely,
    since the deltas were only printed afterwards. The tape measurement can
    always be re-entered later against a logged line; a drive cannot be un-driven.
    """
    if args.fake or args.no_log:
        return
    path = args.log or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "wheel_odom_runs.jsonl"
    )
    entry = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "command": args.command,
             "argv": sys.argv[1:], **record}
    try:
        with open(path, "a") as handle:
            handle.write(json.dumps(entry) + "\n")
    except OSError as exc:
        print(f"  !! could not append the run to {path}: {exc}")
        return
    print(f"  encoder result appended to {path}")


def report_command_tracking(args, kin, deltas, elapsed: float) -> None:
    """How closely the servos actually followed the Goal_Velocity they were given.

    This separates two error sources that a tape measurement alone cannot tell
    apart.  Wheel *slip* means the wheel turned but the robot did not move, so
    the robot travels less than the encoders say.  A Goal_Velocity *scale* error
    means the wheel did not even turn by the commanded amount -- a sagging
    battery, a saturating velocity loop, or the setpoint unit simply not being
    ticks/s.  The second kind is invisible in ``wheel_radius`` (which is derived
    from encoder counts, not from the setpoint) but it decides whether two runs
    at different battery voltages are comparable at all -- which matters as soon
    as you reuse a ground truth measured during an earlier run.
    """
    if not args.drive or args.fake:
        return
    if args.command == "spin":
        raw = kin.body_to_wheel_raw(0.0, 0.0, args.yaw_rate)
    else:
        raw = kin.body_to_wheel_raw(args.speed, 0.0, 0.0)
    print("  Goal_Velocity tracking (commanded vs actual wheel rotation):")
    for name, rate, delta in zip(WHEEL_NAMES, raw, deltas):
        expected = float(rate) * elapsed
        if abs(expected) < 1.0:
            print(f"    {name:<5}: commanded ~0 ticks, actual {delta:+7d}")
            continue
        print(f"    {name:<5}: commanded {expected:+9.0f} ticks, actual {delta:+7d}  "
              f"({100.0 * delta / expected - 100.0:+.1f}%)")


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
    # A wheel whose expected rate is ~0 for this motion carries no sign
    # information: the back wheel's rolling direction is perpendicular to +x, so
    # a pure forward push turns it only by whatever slip occurs. Comparing
    # np.sign(d) against np.sign(0.0) == 0 flagged that noise as a mismatch and
    # sent you looking for a wiring fault that was not there.
    scale = float(np.max(np.abs(nominal)))
    uninformative = np.abs(nominal) < 0.05 * scale if scale > 0 else np.ones(3, dtype=bool)
    for name, motor_id, d, expect, skip in zip(WHEEL_NAMES, ids, deltas, nominal, uninformative):
        if skip:
            ok = "(no constraint: nominally 0 for this motion, so any reading here is slip)"
        elif d == 0 or np.sign(d) == np.sign(expect):
            ok = "OK"
        else:
            ok = "MISMATCH -> flip this wheel's sign"
        print(f"    {name:<5} (id {motor_id}): {d:+7d} ticks   {ok}")

    # The two informative wheels should read equal magnitudes for a straight
    # push; the difference is the yaw that leaked in, and it is the single most
    # useful number for judging whether the run is worth keeping.
    informative = [abs(d) for d, skip in zip(deltas, uninformative) if not skip]
    if len(informative) == 2 and max(informative) > 0:
        asym = 100.0 * abs(informative[0] - informative[1]) / max(informative)
        print(f"\n  left/right magnitude asymmetry: {asym:.1f}%  (a clean straight push is under ~3%)")

    body = kin.wheel_tick_delta_to_body_delta(deltas)
    print(f"\n  implied body displacement: dx={body[0]:+.4f} m  dy={body[1]:+.4f} m  dyaw={np.degrees(body[2]):+.2f} deg")
    log_run(args, {"elapsed_s": elapsed, "tick_deltas": list(deltas),
                   "odo_dx": body[0], "odo_dy": body[1], "odo_dyaw_deg": float(np.degrees(body[2]))})
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
    preset = straight_preset(args)
    if args.measure_after:
        print("STRAIGHT TEST -> wheel_radius   (ground truth measured after the motion)")
    else:
        print(f"STRAIGHT TEST -> wheel_radius   (ground truth {preset[0]:+.3f} m forward, "
              f"{preset[1]:+.3f} m left)")
    if not args.drive:
        print("  Push the robot straight forward exactly that distance, then press ENTER.")
    deltas, elapsed = driven_accumulate(args, bus, kin, ids, clock)

    body = kin.wheel_tick_delta_to_body_delta(deltas)
    odo = np.array([body[0], body[1]])
    odo_chord = float(np.hypot(*odo))
    print(f"\n  elapsed {elapsed:.2f}s, tick deltas {deltas}")
    print(f"  odometry with nominal geometry: dx={body[0]:+.4f} m  dy={body[1]:+.4f} m  "
          f"dyaw={np.degrees(body[2]):+.2f} deg")
    print(f"  odometry path length: {odo_chord:.4f} m")
    report_command_tracking(args, kin, deltas, elapsed)
    log_run(args, {"elapsed_s": elapsed, "tick_deltas": list(deltas),
                   "wheel_radius_in": kin.wheel_radius, "base_radius_in": kin.base_radius,
                   "odo_dx": body[0], "odo_dy": body[1], "odo_dyaw_deg": float(np.degrees(body[2]))})
    if odo_chord < 1e-6:
        print("  !! no motion detected.")
        return 1

    forward, lateral = straight_ground_truth(args, preset)
    truth = np.array([forward, lateral])
    truth_chord = float(np.hypot(*truth))
    side = "left" if lateral > 0 else "right"
    print(f"\n  ground truth: {forward:+.4f} m forward, {abs(lateral) * 1000:.0f} mm to the {side}"
          f"  ->  displacement {truth_chord:.4f} m")
    if abs(forward) > 0.0:
        print(f"    the sideways miss lengthens that by only "
              f"{100.0 * (truth_chord / abs(forward) - 1.0):+.3f}%, so it barely moves "
              "wheel_radius; its real use is the direction check below")

    # Body displacement is exactly proportional to wheel_radius, so the
    # correction is a plain ratio -- of the two *lengths*, which is why the
    # sideways miss enters only through the hypotenuse.
    corrected = kin.wheel_radius * truth_chord / odo_chord
    report("wheel_radius", DEFAULT_WHEEL_RADIUS, corrected)
    print(f"\n  apply with:  -p wheel_radius:={corrected:.6f}")

    # Direction is a separate, independent check: the magnitude ratio above is
    # one equation and can always be satisfied by rescaling wheel_radius, so it
    # can never reveal a *relative* error between the three wheels. The angle
    # between the two displacement vectors can. Measured as the angle between
    # them rather than a difference of headings, so a reverse run (both vectors
    # near 180 degrees) needs no special case.
    unit_truth, unit_odo = truth / truth_chord, odo / odo_chord
    between = float(np.degrees(np.arccos(np.clip(float(unit_truth @ unit_odo), -1.0, 1.0))))
    crab = float(np.degrees(np.arctan2(lateral, forward)))
    print("\n  direction check (independent of wheel_radius)")
    # Both angles are reported in the same raw atan2 convention (so a reverse run
    # shows both near 180 rather than one near 180 and one near 0); the verdict
    # below uses the angle between the vectors, which needs no convention at all.
    print(f"    tape says the robot went : {crab:+.2f} deg from the body +x axis "
          f"({abs(lateral) * 1000:.0f} mm sideways over {abs(forward):.3f} m)")
    print(f"    odometry says            : {np.degrees(np.arctan2(body[1], body[0])):+.2f} deg")
    print(f"    mismatch                 : {between:.2f} deg")
    if between > 5.0:
        print("    !! >5 deg. The wheels disagree with the tape about which way the robot went.")
        print("       Suspect a permuted wheel order or one wheel scaled differently; a single")
        print("       wheel_radius cannot absorb this, and it will show up as a curved path.")
    elif between > 2.0:
        print("    -> 2-5 deg: a real but small cross-axis error. Worth a repeat run to see")
        print("       whether the sign is consistent (systematic) or not (just slip).")
    else:
        print("    -> under 2 deg: the three wheels are consistent with each other.")
    return 0


def cmd_spin(args, bus, kin, clock) -> int:
    ids = args.motor_ids
    truth_rad = 2.0 * np.pi * args.turns
    if args.measure_after:
        print("SPIN TEST -> base_radius   (turns to be entered after the motion)")
    else:
        print(f"SPIN TEST -> base_radius   (ground truth {args.turns:g} turns = {np.degrees(truth_rad):.1f} deg)")
    print(f"  using wheel_radius = {kin.wheel_radius:.6f} m (run the straight test first!)")
    if not args.drive:
        print("  Rotate the robot in place by exactly that many turns, then press ENTER.")
        print("  Mark the floor and the chassis so you can hit the whole number of turns.")
    deltas, elapsed = driven_accumulate(args, bus, kin, ids, clock)

    body = kin.wheel_tick_delta_to_body_delta(deltas)
    measured_rad = abs(float(body[2]))
    print(f"\n  elapsed {elapsed:.2f}s, tick deltas {deltas}")
    print(f"  odometry yaw with nominal geometry: {np.degrees(measured_rad):.2f} deg "
          f"({measured_rad / (2 * np.pi):.4f} turns)")
    print(f"  translation leakage: dx={body[0]:+.4f} m, dy={body[1]:+.4f} m "
          "(should be near zero for a true spin-in-place)")
    report_command_tracking(args, kin, deltas, elapsed)
    log_run(args, {"elapsed_s": elapsed, "tick_deltas": list(deltas),
                   "wheel_radius_in": kin.wheel_radius, "base_radius_in": kin.base_radius,
                   "odo_dx": body[0], "odo_dy": body[1], "odo_dyaw_deg": float(np.degrees(body[2]))})
    if measured_rad < 1e-6:
        print("  !! no rotation detected.")
        return 1

    turns = ground_truth(args, "number of turns completed", "turns (fractions ok)", args.turns)
    truth_rad = 2.0 * np.pi * turns

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
    ap.add_argument(
        "--forward",
        type=float,
        default=None,
        help="straight test ground truth: forward displacement in metres, overriding --distance. "
             "Negative for a run driven in reverse.",
    )
    ap.add_argument(
        "--lateral",
        type=float,
        default=0.0,
        help="straight test ground truth: how far sideways the robot ended up, metres. "
             "ROS REP-103 sign, so + is to the robot's LEFT and a rightward miss is NEGATIVE. "
             "Affects the recovered wheel_radius only through the hypotenuse (80 mm over 3 m is "
             "0.03%); it is there to check the travel direction.",
    )
    ap.add_argument("--turns", type=float, default=5.0, help="spin test ground truth, whole turns")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds instead of on ENTER")
    ap.add_argument("--drive", action="store_true", help="let this script command the wheels")
    ap.add_argument("--speed", type=float, default=0.15, help="--drive forward speed, m/s")
    ap.add_argument("--yaw-rate", type=float, default=0.6, help="--drive spin rate, rad/s")
    ap.add_argument(
        "--measure-after",
        action="store_true",
        help="ask for the ground truth after the motion instead of before. Use this with "
             "--drive: rather than trying to stop the base on a mark, drive for a fixed "
             "--duration, then tape-measure what actually happened.",
    )
    ap.add_argument(
        "--log",
        default=None,
        help="append each run's encoder result here as JSON lines "
             "(default: wheel_odom_runs.jsonl beside this script)",
    )
    ap.add_argument("--no-log", action="store_true", help="do not record runs to disk")
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
