#!/usr/bin/env python3
"""Driver for the ESP32-S3 + TB6612 differential-drive car.

Fills the same role `wheel_odometry_node` fills for LeKiwi: the single owner of the
serial port, consuming `/cmd_vel` and publishing odometry. It is a separate node
rather than a mode of that one because nothing is shared -- no Feetech bus, no
three-wheel kinematics, and the pose arrives already integrated.

The ESP32 firmware (see /home/dm/diffcar_esp32) does the closed loop, the encoder
decoding and the pose integration itself, so this is a protocol adapter:

    /cmd_vel  ->  `u <v> <w>`      chassis twist, m/s and rad/s
    `p`       ->  x/y/theta        firmware-integrated pose, drift < 0.05 deg/m

⚠️ The firmware's link-loss failsafe is 2 s and a serial command does NOT renew it,
so commands must be resent continuously -- see TICK_HZ.
"""
from __future__ import annotations

import os
import re
import termios
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Not omni-specific despite the module name: a generic planar-base -> camera-optical
# conversion. Duplicating it is how the two frames drift apart.
from tinynav.platforms.omni3_kinematics import base_pose_to_camera_pose

TICK_HZ = 20.0
_POSE_RE = re.compile(r"x=(-?[\d.]+)\s+y=(-?[\d.]+)\s+theta=(-?[\d.]+)")


class DiffCarLink:
    """Raw-termios serial link. Deliberately not pyserial: the board's Python has no
    guaranteed pyserial, and `looper/carlib.py` already proves this path works here."""

    def __init__(self, port: str, baud: int = termios.B115200):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0                       # raw
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = baud
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        self._buf = b""
        self._lines: list[str] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                d = os.read(self.fd, 512)
            except (BlockingIOError, OSError):
                d = b""
            if not d:
                time.sleep(0.005)
                continue
            with self._lock:
                self._buf += d
                while b"\n" in self._buf:
                    ln, self._buf = self._buf.split(b"\n", 1)
                    self._lines.append(ln.decode("utf-8", "replace").strip())
                    del self._lines[:-64]        # bound it; only the newest pose matters

    def send(self, cmd: str) -> None:
        os.write(self.fd, (cmd + "\n").encode())

    def drain(self) -> list[str]:
        with self._lock:
            out, self._lines = self._lines, []
        return out

    def close(self) -> None:
        self._stop.set()
        self.send("s")                            # exit closed loop and cut the drivers
        os.close(self.fd)


class DiffCarControlNode(Node):
    def __init__(self) -> None:
        super().__init__("diffcar_control")
        p = self.declare_parameter
        p("port", "/dev/ttyS3")
        # base_link -> camera as [forward, left, up]. base_link is the drive axle.
        # After the 2026-08-21 front-drive rebuild the camera sits 90 mm ahead of the
        # box centre and the axle 40 mm ahead, so only 50 mm separates them -- it was
        # 170 mm when the axle was at the back. Height 0.18 is the measured optical
        # centre. node_manager overrides all three from DIFFCAR_CONFIG; this default
        # only applies to a hand-launched node.
        p("camera_offset_xyz", [0.05, 0.05, 0.18])
        p("cmd_vel_topic", "/cmd_vel")
        p("odom_topic", "/wheel/odometry")
        p("camera_pose_topic", "/wheel/camera_pose")
        p("cmd_timeout_s", 0.5)
        # Same values as DIFFCAR_CONFIG, which node_manager overrides these with anyway;
        # duplicated rather than imported so this node stays free of the planning stack.
        p("max_vx", 0.3)
        p("max_yaw", 0.8)

        g = self.get_parameter
        self.offset = np.asarray([float(v) for v in g("camera_offset_xyz").value], dtype=float)
        if self.offset.shape != (3,):
            raise ValueError(f"camera_offset_xyz needs 3 elements, got {self.offset.tolist()}")
        self.cmd_timeout = float(g("cmd_timeout_s").value)
        self.max_vx = float(g("max_vx").value)
        self.max_yaw = float(g("max_yaw").value)

        self.link = DiffCarLink(str(g("port").value))
        self.odom_pub = self.create_publisher(Odometry, str(g("odom_topic").value), 10)
        pose_topic = str(g("camera_pose_topic").value)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10) if pose_topic else None
        self.create_subscription(Twist, str(g("cmd_vel_topic").value), self._on_cmd, 10)

        self._cmd = (0.0, 0.0)
        self._cmd_stamp = 0.0
        self._pose = None
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            f"diffcar_control up on {g('port').value}, cam_offset={self.offset.tolist()}"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = (
            float(np.clip(msg.linear.x, -self.max_vx, self.max_vx)),
            float(np.clip(msg.angular.z, -self.max_yaw, self.max_yaw)),
        )
        self._cmd_stamp = time.monotonic()

    def _tick(self) -> None:
        # Stale commands must become zero here, not just stop being refreshed: the
        # firmware's 2 s failsafe is far too slow to be the only brake.
        if time.monotonic() - self._cmd_stamp > self.cmd_timeout:
            self._cmd = (0.0, 0.0)
        v, w = self._cmd
        self.link.send(f"u {v:.3f} {w:.3f}")
        # Poll rather than wait for a reply: the firmware's `y` telemetry carries wheel
        # speeds but not the pose, and a blocking read would stall this timer.
        self.link.send("p")

        for line in self.link.drain():
            m = _POSE_RE.search(line)
            if m:
                x, y, theta_deg = (float(s) for s in m.groups())
                self._pose = (x, y, np.deg2rad(theta_deg))
        if self._pose is None:
            return
        self._publish(*self._pose)

    def _publish(self, x: float, y: float, theta: float) -> None:
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = float(np.sin(theta / 2.0))
        odom.pose.pose.orientation.w = float(np.cos(theta / 2.0))
        odom.twist.twist.linear.x = self._cmd[0]
        odom.twist.twist.angular.z = self._cmd[1]
        self.odom_pub.publish(odom)

        if self.pose_pub is None:
            return
        position, quat = base_pose_to_camera_pose(x, y, theta, self.offset)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(c) for c in position)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (float(c) for c in quat)
        self.pose_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self.link.close()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiffCarControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
