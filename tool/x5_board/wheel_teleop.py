#!/usr/bin/env python3
"""Keyboard teleop for the LeKiwi base, for driving the mapping run.

    python3 tool/x5_board/wheel_teleop.py                    # -> /cmd_vel
    python3 tool/x5_board/wheel_teleop.py --topic /teleop/cmd_vel

Run it in its own ssh session alongside ``wheel_odometry_node`` started with
``-p enable_wheel_command:=true``.  That node owns the serial bus and applies the
``Twist``; this only publishes.

WHY DRIVE INSTEAD OF PUSH
    A mapping run recorded while hand-pushing the base is useless for the
    odometry-mapped comparison: pushing an omni base slips the rollers, and the
    calibration runs measured that at ~11%, enough to produce a physically
    impossible wheel radius.  Every metre of the mapping drive has to come from
    the wheels if the odometry-built map is to mean anything.

WHY IT REPEATS
    ``wheel_odometry_node`` has a command watchdog (``cmd_timeout_s``, 0.5 s by
    default) that zeroes ``Goal_Velocity`` when commands stop arriving, so a key
    press cannot latch the robot into motion if this process or the ssh session
    dies.  The flip side is that a held key has to be *re-sent*, which is what the
    fixed publish rate below does.

Dependencies are rclpy and the standard library only: this runs on the X5 inside
the camera, which has ~850 MB of RAM free and no swap.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
  w / s      forward / back
  a / d      strafe left / right   (the base is holonomic -- no yaw is commanded)
  q / e      rotate left / right
  space      stop
  - / =      speed scale down / up  (0.1 steps, clamped to [0.1, 1.5])
  h          reprint this help
  x or Ctrl-C  quit (stops the wheels first)
"""

# Modest by design. Keyframes are promoted every 3 cm of travel and the bridge's
# exact-stamp sync caps the keyframe rate at 4.83 Hz, so above ~0.15 m/s the map
# gets its keyframes from motion rather than from the threshold and the spacing
# stops being uniform. Slower also means less roller slip to charge to odometry.
DEFAULT_LINEAR = 0.15  # m/s
DEFAULT_ANGULAR = 0.4  # rad/s


class WheelTeleop(Node):
    def __init__(self, args) -> None:
        super().__init__("wheel_teleop")
        self.args = args
        self.scale = 1.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.pub = self.create_publisher(Twist, args.topic, 10)
        self.create_timer(1.0 / args.rate_hz, self._tick)
        print(f"teleop -> {args.topic} at {args.rate_hz:.0f} Hz")
        print(HELP)

    def handle_key(self, key: str) -> bool:
        """Apply one keypress. Returns False to quit."""
        if key in ("x", "\x03"):
            return False
        if key == "w":
            self.vx, self.vy, self.wz = self.args.linear, 0.0, 0.0
        elif key == "s":
            self.vx, self.vy, self.wz = -self.args.linear, 0.0, 0.0
        elif key == "a":
            self.vx, self.vy, self.wz = 0.0, self.args.linear, 0.0
        elif key == "d":
            self.vx, self.vy, self.wz = 0.0, -self.args.linear, 0.0
        elif key == "q":
            self.vx, self.vy, self.wz = 0.0, 0.0, self.args.angular
        elif key == "e":
            self.vx, self.vy, self.wz = 0.0, 0.0, -self.args.angular
        elif key == " ":
            self.vx = self.vy = self.wz = 0.0
        elif key in ("=", "+", "-", "_"):
            step = 0.1 if key in ("=", "+") else -0.1
            self.scale = min(1.5, max(0.1, self.scale + step))
        elif key == "h":
            print(HELP)
        else:
            return True
        print(
            f"  vx={self.vx * self.scale:+.3f} vy={self.vy * self.scale:+.3f} "
            f"wz={self.wz * self.scale:+.3f}  (scale {self.scale:.1f})"
        )
        return True

    def _tick(self) -> None:
        msg = Twist()
        msg.linear.x = self.vx * self.scale
        msg.linear.y = self.vy * self.scale
        msg.angular.z = self.wz * self.scale
        self.pub.publish(msg)

    def stop(self) -> None:
        self.vx = self.vy = self.wz = 0.0
        # Several, because a single message can be dropped and the watchdog then
        # takes cmd_timeout_s to notice. Cheap insurance against a rolling exit.
        for _ in range(5):
            self._tick()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic",
        default="/cmd_vel",
        help="Twist output. /cmd_vel is what wheel_odometry_node listens on when "
             "started with `-p cmd_vel_topic:=/cmd_vel`, i.e. the same topic "
             "cmd_vel_control drives -- so do not run this and an autonomous run "
             "at the same time, they would fight.",
    )
    parser.add_argument("--linear", type=float, default=DEFAULT_LINEAR)
    parser.add_argument("--angular", type=float, default=DEFAULT_ANGULAR)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not sys.stdin.isatty():
        raise SystemExit("wheel_teleop needs a terminal: stdin is not a tty")

    rclpy.init()
    node = WheelTeleop(args)
    old_attrs = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not select.select([sys.stdin], [], [], 0.0)[0]:
                continue
            if not node.handle_key(sys.stdin.read(1)):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()
        print("\nteleop stopped, zero Twist sent")


if __name__ == "__main__":
    main()
