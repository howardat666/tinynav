#!/usr/bin/env python3
"""Live servo-bus loss rate, the way ``ros2 topic hz`` is live.

WHY THIS AND NOT servo_failure_pattern.py

Every bus measurement in this investigation so far required stopping
``wheel_odometry_node`` first, because only one process may hold ``/dev/ttyS3``.
That makes a whole class of question unanswerable: what the bus does while the
robot is actually driving.  Motor current, PWM switching and the wheels'
own supply transients all share the 12 V rail that feeds the camera through the
GH1.25, and every number gathered so far was taken with the robot parked and the
wheels commanded to zero.

So this measures without touching the port at all.  ``wheel_odometry_node``
already knows both halves:

* failures -- it logs ``read failed (N total)`` with the running count, throttled
  to one line per 2 s, so consecutive lines give the delta
* successes -- every tick that reads cleanly publishes ``camera_pose_topic``, so
  counting those messages counts successful ticks

⚠️ THE DENOMINATOR IS THE HARD PART, SO THIS REPORTS TWO OF THEM
The obvious `dFailures / (dFailures + published)` is wrong here.  Measured with
zero failures, the pose topic arrives at 10-18 Hz, not the 50 Hz the timer is
configured for -- some mixture of the timer not keeping up under load and this
subscriber dropping messages.  Either way the published count is a lower bound on
attempted ticks, which would inflate the percentage exactly when the board is
busiest.  So the primary readout is **failures per second**, which needs no
denominator and is exact, and the percentage is shown against the nominal 50 Hz
tick with that assumption stated rather than hidden.  Pose Hz is shown too, since
a collapse there is worth seeing even when its cause is CPU rather than the bus.

READ THIS BEFORE COMPARING TO THE OTHER TOOLS
The number here is the END-TO-END rate after ``num_read_retries`` retries, which
is what navigation actually experiences.  ``servo_failure_pattern.py`` reports the
SINGLE-ATTEMPT rate, deliberately, because retries smear the structure it is
looking for.  The single-attempt figure is always the larger one; they are not
interchangeable.
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

FAIL_RE = re.compile(r"read failed \((\d+) total\)")
THERMAL = "/sys/class/thermal/thermal_zone0/temp"


class LogTail(threading.Thread):
    """Follow the newest wheel_odometry node log and keep its cumulative failure count.

    ⚠️ THE FILE IS NOT app.log. node_manager._launch_proc gives every node its own
    file under logs/nodes/<timestamp>_<name>.txt; app.log only has the backend's
    own output. The first version of this tool tailed app.log and therefore
    reported 0.0% forever while the bus was in fact losing 1377 reads -- a silent
    wrong answer, which is worse than a crash. Hence a glob over the directory and
    always the newest match, rather than a fixed path.

    Restarting the node starts a new file whose counter begins at zero again, so a
    decrease in the total is a restart, not a negative delta; report() rebaselines
    on it instead of printing a nonsense negative rate.
    """

    daemon = True

    def __init__(self, pattern: str) -> None:
        super().__init__()
        self.pattern = pattern
        self.total: int | None = None
        self.path: str | None = None
        self._stop = threading.Event()

    def _newest(self) -> str | None:
        matches = glob.glob(self.pattern)
        return max(matches, key=os.path.getmtime) if matches else None

    def run(self) -> None:
        fh = None
        last_check = 0.0
        while not self._stop.is_set():
            try:
                now = time.monotonic()
                # Re-glob occasionally so a node restart is picked up without
                # needing this tool to be restarted too.
                if fh is None or now - last_check > 5.0:
                    last_check = now
                    newest = self._newest()
                    if newest and newest != self.path:
                        if fh is not None:
                            fh.close()
                        fh = open(newest, "r", errors="replace")
                        fh.seek(0, os.SEEK_END)
                        self.path = newest
                if fh is None:
                    time.sleep(0.5)
                    continue
                line = fh.readline()
                if not line:
                    time.sleep(0.2)
                    continue
                m = FAIL_RE.search(line)
                if m:
                    self.total = int(m.group(1))
            except OSError:
                if fh is not None:
                    fh.close()
                fh = None
                time.sleep(1.0)

    def stop(self) -> None:
        self._stop.set()


def read_temp() -> float:
    try:
        with open(THERMAL) as fh:
            return int(fh.read().strip()) / 1000.0
    except (OSError, ValueError):
        return float("nan")


class BusLossMonitor(Node):
    def __init__(self, topic: str, log_path: str, interval: float, window_s: float,
                 tick_hz: float) -> None:
        super().__init__("bus_loss_live")
        self.interval = interval
        self.count = 0
        # BEST_EFFORT: the publisher is a sensor-style stream and a RELIABLE
        # subscription would silently fail to match it, showing 0 Hz forever.
        self.create_subscription(
            PoseStamped, topic, self._on_pose,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.tail = LogTail(log_path)
        self.tail.start()
        self.t0 = time.monotonic()
        self.prev_t = self.t0
        self.prev_count = 0
        self.prev_fail: int | None = None
        self.first_fail: int | None = None
        self.window: deque = deque()
        self.window_s = window_s
        self.create_timer(interval, self._report)
        self.tick_hz = tick_hz
        print(f"watching {topic} and {log_path}", flush=True)
        print(f"  -> newest log: {self.tail._newest()}", flush=True)
        print(f"fail/s is exact (the node's own counter). %% is against the nominal "
              f"{tick_hz:.0f} Hz tick.", flush=True)
        print(f"{'elapsed':>8} {'fail/s':>8} {'now%':>7} {'win%':>7} "
              f"{'fails':>7} {'poseHz':>7} {'temp':>7}", flush=True)

    def _on_pose(self, _msg: PoseStamped) -> None:
        self.count += 1

    def _report(self) -> None:
        now = time.monotonic()
        dt = now - self.prev_t
        ok = self.count - self.prev_count
        total_fail = self.tail.total
        if total_fail is not None and self.first_fail is None:
            # The first line seen carries a count accumulated before we attached,
            # so it is a baseline, not a delta.
            self.first_fail = total_fail
            self.prev_fail = total_fail
        bad = 0 if (total_fail is None or self.prev_fail is None) else total_fail - self.prev_fail
        if bad < 0:
            # The node restarted and its counter went back to zero.
            self.first_fail = total_fail
            self.prev_fail = total_fail
            bad = 0

        fail_hz = bad / dt if dt else 0.0
        inst = 100.0 * fail_hz / self.tick_hz
        self.window.append((now, dt, bad))
        while self.window and now - self.window[0][0] > self.window_s:
            self.window.popleft()
        w_dt = sum(x[1] for x in self.window)
        w_bad = sum(x[2] for x in self.window)
        w = (100.0 * (w_bad / w_dt) / self.tick_hz) if w_dt else float("nan")

        c_bad = 0 if (total_fail is None or self.first_fail is None) else total_fail - self.first_fail

        hint = ""
        if ok == 0:
            hint = "   <- no poses: is wheel_odometry_node running?"
        if total_fail is None:
            # Nothing has been seen in the log yet, which is NOT the same as zero
            # failures -- the tail starts at the end of the file, so the first
            # datum arrives with the first failure. Printing 0.0% here is exactly
            # the silent-wrong-answer that the app.log bug already produced once.
            print(f"{now - self.t0:7.0f}s {'--':>8} {'--':>7} {'--':>7} "
                  f"{'--':>7} {ok / dt if dt else 0:7.1f} {read_temp():6.1f}C"
                  f"   <- no failure logged yet (quiet bus, or none since start)",
                  flush=True)
        else:
            print(f"{now - self.t0:7.0f}s {fail_hz:8.1f} {inst:6.1f}% {w:6.1f}% "
                  f"{c_bad:7d} {ok / dt if dt else 0:7.1f} {read_temp():6.1f}C{hint}",
                  flush=True)

        self.prev_t = now
        self.prev_count = self.count
        if total_fail is not None:
            self.prev_fail = total_fail


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/wheel/camera_pose")
    ap.add_argument("--log", default="/userdata/x5/logs/nodes/*_wheel_odometry.txt",
                    help="glob; the NEWEST match is followed. Not app.log -- node output "
                         "goes to its own file under logs/nodes/")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="print period; not below 2 s, which is the log throttle")
    ap.add_argument("--window", type=float, default=10.0)
    ap.add_argument("--tick-hz", type=float, default=50.0,
                    help="wheel_odometry_node publish_rate_hz; the percentage denominator")
    args = ap.parse_args()

    rclpy.init()
    node = BusLossMonitor(args.topic, args.log, max(args.interval, 2.0), args.window,
                          args.tick_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tail.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
