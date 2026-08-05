#!/usr/bin/env python3
"""Compare two yaw sources against a counted ground truth: the gyro and the wheels.

Spin the base in place at a deliberately uneven rate, press ENTER on a whole turn,
and type how many turns it made.  The script reports what the gyro thought and
what the wheel encoders thought, so both are judged against the same external
truth in a single run instead of against each other.

    python3 tool/x5_board/yaw_source_compare.py --port /dev/ttyS3 --drive \
        --wheel-radius 0.050385 --base-radius 0.127083

WHY DRIVEN, AND WHY UNEVENLY
----------------------------
Use ``--drive``.  Hand-rotating the chassis seems like the more revealing test,
since skidding the rollers is exactly the error wheel odometry cannot see -- but
it measures a condition that never occurs in operation, and on this base hand
pushing is wildly unrepresentative: measured over 3 m, a hand push yawed the
chassis 23.7 deg and left the two driving wheels 10.8% apart, where driving all
three gave 0.17 deg and 0.06%.  A geometry calibration done by hand on this robot
reported an effective wheel radius 12.8% *above* the geometric one, which is
physically impossible.  Validate under the conditions the robot will actually run
in.

Uneven speed is the part worth keeping, and ``--drive`` varies the commanded yaw
rate on purpose.  At a constant rate a scale error and a bias error trade off
against each other and both look fine; a varying rate separates them, because
bias accumulates with *time* while scale error accumulates with *angle*.  The
report prints peak/mean yaw rate so a run that was not actually uneven is visible
rather than quietly believed.

Hand rotation is still available (omit ``--drive``) and still worth one run: it
sizes the worst case, i.e. what the wheels report when the robot is shoved,
stalled against an obstacle, or lifted.

WHAT MAKES THE GYRO USABLE WITHOUT ANY MOUNTING ASSUMPTION
----------------------------------------------------------
The camera's IMU publishes no orientation (the quaternion is all zeros), so there
is no absolute yaw reference -- only a rate to integrate.  Two things have to be
established before that integral means anything, and both come free from a few
seconds of standing still at the start:

* **Which axis is yaw.**  A stationary accelerometer measures proper acceleration,
  a vector of ~9.8 m/s^2 pointing *up*.  So the average of ``linear_acceleration``
  over the still window *is* the up direction, in IMU coordinates, and the yaw
  rate is the gyro projected onto it.  This needs no TF, no static extrinsic, and
  no assumption that the camera is mounted level -- it measures the tilt instead
  of trusting it.  (On this camera gravity reads y = -9.795, confirming the IMU's
  y axis points down like the optical frames, but nothing here depends on that.)
* **The gyro bias.**  This is the one step that cannot be skipped.  An
  uncorrected 0.5 deg/s bias is 2.5 deg over 5 s, which is *worse* than the
  wheels it is supposed to improve on.  The average gyro reading while stationary
  is that bias, and it is subtracted from every later sample.

The script prints how much of the final answer the bias correction was worth, so
a suspiciously large correction is visible rather than silently baked in.

A THIRD THING THIS CHECKS FOR FREE
----------------------------------
The two sources must agree on the *sign* of the rotation.  The gyro's sign comes
from the gravity projection above; the wheels' comes from ``wheel_signs`` and the
kinematic layout.  Nothing links them, so if they disagree, one of the two
conventions is wrong -- and a sign error is invisible in a magnitude comparison.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import time

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    print(f"needs rclpy and sensor_msgs on the PYTHONPATH: {exc}", file=sys.stderr)
    print("on the board:  source /userdata/x5/env.sh", file=sys.stderr)
    sys.exit(2)

from tinynav.platforms.feetech_bus import (
    FakeFeetechBus,
    FeetechBus,
    configure_velocity_mode,
)
from tinynav.platforms.omni3_kinematics import (
    DEFAULT_BASE_RADIUS,
    DEFAULT_TICKS_PER_REV,
    DEFAULT_WHEEL_RADIUS,
    Omni3Kinematics,
    wrap_tick_delta,
)

WHEEL_NAMES = ("left", "back", "right")

# Multipliers on --yaw-rate, cycled one per --profile-step-s. Chosen to span a
# ~4x range and to include a brief reversal: reversing is what makes a constant
# *bias* separable from a constant *scale* error, because the bias keeps
# accumulating in the same direction while the true rotation changes sign.
YAW_RATE_PROFILE = (1.0, 0.4, 1.0, 0.6, -0.5, 0.8, 1.0, 0.3)


class YawCompare(Node):
    def __init__(self, args) -> None:
        super().__init__("yaw_source_compare")
        self.args = args
        self.kin = Omni3Kinematics(
            wheel_radius=args.wheel_radius,
            base_radius=args.base_radius,
            ticks_per_rev=args.ticks_per_rev,
            wheel_signs=tuple(args.wheel_signs),
        )

        # -- IMU side ------------------------------------------------------- #
        # BEST_EFFORT with a shallow queue: this is a 346 Hz stream and we want
        # the freshest samples, not a backlog replayed after a slow poll.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(Imu, args.imu_topic, self._imu_cb, qos)
        self.gyro_samples: list[tuple[float, np.ndarray]] = []
        self.accel_samples: list[np.ndarray] = []
        self.gyro_bias = np.zeros(3)
        self.up_hat: np.ndarray | None = None
        self.imu_yaw_rad = 0.0
        self._prev_imu: tuple[float, float] | None = None  # (t, yaw_rate)
        self.imu_rate_peak = 0.0
        self.imu_rate_abs_sum = 0.0
        self.imu_rate_n = 0

        # -- wheel side ----------------------------------------------------- #
        self.bus = (
            FakeFeetechBus(
                args.motor_ids,
                dict(zip(args.motor_ids, self.kin.body_to_wheel_ticks_per_s(0.0, 0.0, 0.6))),
                args.ticks_per_rev,
            )
            if args.fake
            else FeetechBus(port=args.port, baudrate=args.baudrate, timeout=args.timeout)
        )
        self.bus.connect()
        self.wheel_totals = dict.fromkeys(args.motor_ids, 0)
        self.wheel_prev: dict[int, int] | None = None
        self.read_failures = 0
        self.driving = False
        self.profile_index = -1
        if args.drive and not args.fake:
            # Done here, before the still window, so a servo that refuses velocity
            # mode aborts the run before anyone has stood around waiting 4 s.
            configure_velocity_mode(self.bus, args.motor_ids)
            print("wheels are in velocity mode and energised -- they will NOT move "
                  "until the still window ends")

        self.phase = "bias"
        self.phase_started = time.monotonic()
        self.motion_started: float | None = None
        self.done = False
        self.create_timer(1.0 / args.poll_hz, self._tick)

        print(f"IMU topic   : {args.imu_topic}")
        print(f"servo bus   : {args.port if not args.fake else '(fake)'}")
        print(f"geometry    : wheel_radius={self.kin.wheel_radius:.6f}  "
              f"base_radius={self.kin.base_radius:.6f}")
        print()
        print(f"PHASE 1/2 -- hold still for {args.bias_seconds:.0f} s "
              "(measuring gyro bias and the gravity direction)")

    # -- callbacks ---------------------------------------------------------- #

    def _imu_cb(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if self.phase == "bias":
            self.gyro_samples.append((t, gyro))
            self.accel_samples.append(
                np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
            )
            return
        if self.phase != "motion" or self.up_hat is None:
            return
        rate = float((gyro - self.gyro_bias) @ self.up_hat)
        if self._prev_imu is not None:
            t0, r0 = self._prev_imu
            dt = t - t0
            # Trapezoid rather than rectangle: at 346 Hz the difference is small,
            # but a hand spin has real acceleration in it and the rectangle rule
            # biases consistently in whichever direction the rate is changing.
            if 0.0 < dt < 0.2:
                self.imu_yaw_rad += 0.5 * (rate + r0) * dt
            elif dt >= 0.2:
                self.get_logger().warning(f"IMU gap of {dt:.3f}s -- that interval is not integrated")
        self._prev_imu = (t, rate)
        self.imu_rate_peak = max(self.imu_rate_peak, abs(rate))
        self.imu_rate_abs_sum += abs(rate)
        self.imu_rate_n += 1

    def _read_wheels(self) -> dict[int, int] | None:
        try:
            res = self.bus.sync_read("Present_Position", self.args.motor_ids, num_retry=2)
        except Exception:  # noqa: BLE001 - a dropped frame must not end the run
            self.read_failures += 1
            return None
        return {i: int(res.values[i]) % self.args.ticks_per_rev for i in self.args.motor_ids}

    def _tick(self) -> None:
        if self.done:
            return
        curr = self._read_wheels()

        if self.phase == "bias":
            # --fake synthesises a constant spin, so the stillness check would
            # restart the window forever. Skip it there; on real hardware it is
            # the whole point.
            if curr is not None and not self.args.fake and self.wheel_prev is not None:
                moved = max(
                    abs(wrap_tick_delta(curr[i], self.wheel_prev[i], self.args.ticks_per_rev))
                    for i in self.args.motor_ids
                )
                if moved > self.args.still_tick_tolerance:
                    # Restart rather than warn: a bias measured while moving is
                    # worse than no bias measurement at all, and it would silently
                    # corrupt everything downstream.
                    print(f"  moved {moved} ticks during the still window -- restarting it")
                    self.gyro_samples.clear()
                    self.accel_samples.clear()
                    self.phase_started = time.monotonic()
            if curr is not None:
                self.wheel_prev = curr
            if time.monotonic() - self.phase_started >= self.args.bias_seconds:
                self._finish_bias()
            return

        if curr is not None:
            if self.wheel_prev is not None:
                for i in self.args.motor_ids:
                    self.wheel_totals[i] += wrap_tick_delta(
                        curr[i], self.wheel_prev[i], self.args.ticks_per_rev
                    )
            self.wheel_prev = curr

        elapsed = time.monotonic() - (self.motion_started or time.monotonic())
        commanded = self._update_drive(elapsed)
        wheel_yaw_deg = np.degrees(self._wheel_yaw_rad())
        imu_yaw_deg = np.degrees(self.imu_yaw_rad)
        print(f"\r  {elapsed:5.1f}s  {commanded}  gyro {imu_yaw_deg:+9.1f} deg ({imu_yaw_deg / 360.0:+6.3f} turns)"
              f"   wheels {wheel_yaw_deg:+9.1f} deg ({wheel_yaw_deg / 360.0:+6.3f} turns)   "
              "[ENTER to stop]", end="", flush=True)

        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if ready:
            sys.stdin.readline()
            # Stop before anything else, including before the report and the
            # ground-truth prompt: the prompt blocks on input(), and a base that
            # is still turning while it waits would invalidate the very number
            # being typed in.
            self.stop_wheels()
            self.done = True

    # -- phases ------------------------------------------------------------- #

    def _finish_bias(self) -> None:
        if len(self.gyro_samples) < 10 or len(self.accel_samples) < 10:
            print(f"\n!! only {len(self.gyro_samples)} IMU samples arrived in "
                  f"{self.args.bias_seconds:.0f}s -- is {self.args.imu_topic} publishing?")
            self.done = True
            return
        self.gyro_bias = np.mean([g for _, g in self.gyro_samples], axis=0)
        accel_mean = np.mean(self.accel_samples, axis=0)
        norm = float(np.linalg.norm(accel_mean))
        # A stationary accelerometer reads proper acceleration: ~9.8 m/s^2 pointing
        # UP. So the mean *is* the up direction, no negation.
        self.up_hat = accel_mean / norm

        gyro_std = np.std([g for _, g in self.gyro_samples], axis=0)
        print(f"\r  still window done: {len(self.gyro_samples)} IMU samples"
              f" ({len(self.gyro_samples) / self.args.bias_seconds:.0f} Hz)          ")
        print(f"    gravity      : {np.round(accel_mean, 3)} m/s^2, |g| = {norm:.3f}")
        if abs(norm - 9.81) > 0.5:
            print(f"    !! |g| is {norm:.2f}, not ~9.81 -- the robot was not still, "
                  "or the accelerometer is not in m/s^2")
        print(f"    yaw axis     : {np.round(self.up_hat, 4)} (up, in IMU coordinates)")
        print(f"    gyro bias    : {np.round(np.degrees(self.gyro_bias), 4)} deg/s")
        print(f"    gyro noise   : {np.round(np.degrees(gyro_std), 4)} deg/s (1 sigma)")
        bias_yaw = float(np.degrees(self.gyro_bias @ self.up_hat))
        print(f"    bias about yaw axis: {bias_yaw:+.4f} deg/s "
              f"-> {abs(bias_yaw) * 60:.2f} deg per minute of drift if left uncorrected")
        print()
        if self.args.drive:
            print("PHASE 2/2 -- DRIVING NOW. The base will spin, changing rate every "
                  f"{self.args.profile_step_s:.0f}s.")
            print("  Count the turns. Press ENTER at the instant your floor and chassis")
            print("  marks line up on a whole turn, so the ground truth is exact rather")
            print("  than an estimated fraction -- reaction time is worth ~0.4%, guessing")
            print("  a fraction is worth ~5%.")
            print("  !! The tether winds up one turn per turn. Stop by ~4 and run again")
            print("     with a negative --yaw-rate to unwind it.")
        else:
            print("PHASE 2/2 -- rotate the robot in place BY HAND, deliberately unevenly.")
            print("  Vary the speed, pause, back up a little. Count the turns as you go.")
            print("  Press ENTER at a whole-turn mark alignment.")
        print()
        self.phase = "motion"
        self.motion_started = time.monotonic()

    def _update_drive(self, elapsed: float) -> str:
        """Step the yaw-rate profile, writing Goal_Velocity when it changes.

        Written only on a step boundary rather than every poll: the setpoint is
        piecewise constant, and re-sending it 50 times a second would fill the
        half-duplex bus with writes competing against the position reads that this
        test actually depends on.
        """
        if not self.args.drive:
            return "              "
        index = int(elapsed // self.args.profile_step_s) % len(YAW_RATE_PROFILE)
        rate = self.args.yaw_rate * YAW_RATE_PROFILE[index]
        if index != self.profile_index:
            self.profile_index = index
            raw = self.kin.body_to_wheel_raw(0.0, 0.0, rate)
            self.bus.sync_write("Goal_Velocity", dict(zip(self.args.motor_ids, (int(v) for v in raw))))
            self.driving = True
        return f"cmd {rate:+.2f} rad/s"

    def stop_wheels(self) -> None:
        """Zero Goal_Velocity. Must run on every exit path, including exceptions.

        Without this an unhandled error anywhere above leaves the base spinning
        while the traceback prints.
        """
        if not self.driving:
            return
        try:
            self.bus.sync_write("Goal_Velocity", dict.fromkeys(self.args.motor_ids, 0))
            self.driving = False
        except Exception as exc:  # noqa: BLE001 - report, but keep unwinding
            print(f"\n!! FAILED TO STOP THE WHEELS: {exc}  -- cut the power")

    def _wheel_yaw_rad(self) -> float:
        deltas = [self.wheel_totals[i] for i in self.args.motor_ids]
        # Only the yaw component of this is valid for a large rotation. Yaw is a
        # linear function of the wheel arcs -- the yaw row of M^-1 is
        # [1,1,1]/(3*base_radius) -- so summing tick deltas first and converting
        # once is exact however far the base turned. The dx/dy components of the
        # same product are *not* valid here, because they are expressed in a body
        # frame that rotated during the run; they are deliberately unused.
        return float(self.kin.wheel_tick_delta_to_body_delta(deltas)[2])

    # -- report ------------------------------------------------------------- #

    def report(self) -> int:
        if self.phase != "motion":
            return 1
        duration = time.monotonic() - (self.motion_started or time.monotonic())
        wheel_yaw = self._wheel_yaw_rad()
        imu_yaw = self.imu_yaw_rad
        deltas = [self.wheel_totals[i] for i in self.args.motor_ids]

        print("\n")
        print("=" * 72)
        print(f"ran for {duration:.1f}s, {self.imu_rate_n} gyro samples, "
              f"{self.read_failures} wheel read failures")
        print("  wheel tick totals: " + ", ".join(
            f"{n}={d:+d}" for n, d in zip(WHEEL_NAMES, deltas)))
        mags = [abs(d) for d in deltas]
        if max(mags) > 0:
            print(f"  three-wheel spread: {100.0 * (max(mags) - min(mags)) / max(mags):.1f}%"
                  "   (a true spin-in-place turns all three equally; a large spread"
                  " means the base translated too)")
        if self.imu_rate_n:
            mean_rate = self.imu_rate_abs_sum / self.imu_rate_n
            print(f"  |yaw rate|: mean {np.degrees(mean_rate):.1f} deg/s, "
                  f"peak {np.degrees(self.imu_rate_peak):.1f} deg/s "
                  f"(peak/mean = {self.imu_rate_peak / mean_rate:.1f}x"
                  f"{' -- good, the rate really did vary' if self.imu_rate_peak > 2.5 * mean_rate else ' -- try harder to vary the speed'})")

        truth = self._ask_turns()
        if truth is None:
            return 1
        truth_rad = 2.0 * np.pi * truth

        print()
        print(f"{'source':<12} {'turns':>9} {'degrees':>11} {'error vs truth':>16}")
        print("-" * 52)
        print(f"{'ground truth':<12} {truth:>9.3f} {np.degrees(truth_rad):>11.1f} {'--':>16}")
        rows = []
        for label, value in (("gyro", imu_yaw), ("wheels", wheel_yaw)):
            err = 100.0 * (abs(value) - abs(truth_rad)) / abs(truth_rad) if truth_rad else float("nan")
            rows.append((label, value, err))
            print(f"{label:<12} {value / (2 * np.pi):>9.3f} {np.degrees(value):>11.1f} "
                  f"{err:>+15.1f}%")

        print()
        if np.sign(imu_yaw) != np.sign(wheel_yaw) and abs(imu_yaw) > 0.1 and abs(wheel_yaw) > 0.1:
            print("!! THE TWO SOURCES DISAGREE ON SIGN.")
            print("   One convention is inverted -- either wheel_signs / the kinematic")
            print("   layout, or the gravity projection. A magnitude-only comparison")
            print("   cannot see this, and it would send the robot the wrong way round")
            print("   every corner. Fix this before believing any number above.")
        else:
            print("signs agree: both sources call this rotation the same direction.")

        bias_contribution = float(np.degrees(self.gyro_bias @ self.up_hat)) * duration
        print(f"\nbias correction was worth {bias_contribution:+.2f} deg over this run "
              f"({100.0 * abs(bias_contribution) / max(abs(np.degrees(truth_rad)), 1e-9):.2f}% of the truth).")
        print("  Without it the gyro column would be off by that much. If this is a large")
        print("  fraction of the answer, redo the still window -- it was contaminated.")

        gyro_err, wheel_err = rows[0][2], rows[1][2]
        print()
        if abs(gyro_err) < abs(wheel_err):
            print(f"=> the gyro is the better yaw source here ({abs(gyro_err):.1f}% vs "
                  f"{abs(wheel_err):.1f}%). Expected: hand rotation skids the omniwheel")
            print("   rollers, and the encoders cannot see that.")
        else:
            print(f"=> the wheels beat the gyro this run ({abs(wheel_err):.1f}% vs "
                  f"{abs(gyro_err):.1f}%). Suspect the bias window, or too short a run:")
            print("   bias error grows with time while slip grows with angle.")
        return 0

    def _ask_turns(self) -> float | None:
        while True:
            raw = input("\nhow many turns did it actually make? "
                        "(fractions ok, negative for clockwise): ").strip()
            if not raw:
                return None
            try:
                value = float(raw)
            except ValueError:
                print("  not a number, try again")
                continue
            if value == 0.0:
                print("  must be non-zero")
                continue
            return value

    def shutdown(self) -> None:
        self.stop_wheels()
        try:
            self.bus.disconnect()
        except Exception as exc:  # noqa: BLE001 - best effort on the way out
            self.get_logger().warning(f"bus disconnect failed: {exc}")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--port", default="/dev/ttyS3")
    ap.add_argument("--baudrate", type=int, default=1_000_000)
    ap.add_argument("--timeout", type=float, default=0.02)
    ap.add_argument("--motor-ids", type=int, nargs=3, default=[7, 8, 9],
                    metavar=("LEFT", "BACK", "RIGHT"))
    ap.add_argument("--wheel-signs", type=float, nargs=3, default=[1.0, 1.0, 1.0])
    ap.add_argument("--wheel-radius", type=float, default=DEFAULT_WHEEL_RADIUS)
    ap.add_argument("--base-radius", type=float, default=DEFAULT_BASE_RADIUS)
    ap.add_argument("--ticks-per-rev", type=int, default=DEFAULT_TICKS_PER_REV)
    ap.add_argument("--imu-topic", default="/camera/camera/imu")
    ap.add_argument("--poll-hz", type=float, default=50.0, help="wheel encoder poll rate")
    ap.add_argument("--bias-seconds", type=float, default=4.0,
                    help="how long to stand still measuring gyro bias and gravity")
    ap.add_argument("--still-tick-tolerance", type=int, default=3,
                    help="per-poll tick movement tolerated during the still window")
    ap.add_argument("--drive", action="store_true",
                    help="let this script spin the base, varying the rate. Recommended: "
                         "hand rotation measures a condition the robot never operates in")
    ap.add_argument("--yaw-rate", type=float, default=0.45,
                    help="base yaw rate in rad/s; the profile scales this by 0.3x to 1.0x "
                         "and briefly reverses")
    ap.add_argument("--profile-step-s", type=float, default=4.0,
                    help="how long to hold each rate before stepping to the next")
    ap.add_argument("--fake", action="store_true", help="synthesise wheels (IMU still real)")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    rclpy.init(args=None)
    node = YawCompare(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rclpy.ok():
            # A signal landed mid-run. rclpy's handler invalidates the context
            # before the loop notices, so anything that touches it after this
            # point -- including creating the internal wait-set timer inside
            # spin_once -- raises RCLError rather than returning.
            print("\nshut down before the motion finished; nothing to report")
            return 130
        return node.report()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\ninterrupted")
        return 130
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
