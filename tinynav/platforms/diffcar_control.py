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
import socket
import struct
import termios
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.node import Node

# Not omni-specific despite the module name: a generic planar-base -> camera-optical
# conversion. Duplicating it is how the two frames drift apart.
from tinynav.platforms.omni3_kinematics import base_pose_to_camera_pose

TICK_HZ = 20.0
_POSE_RE = re.compile(r"x=(-?[\d.]+)\s+y=(-?[\d.]+)\s+theta=(-?[\d.]+)")
# 电压只有 ESP32 知道，而串口只有本节点持有 —— 不采就没有任何电压时间序列，
# 而"电池带载塌下去"正是 2026-08-21 掉线唯一的可疑线索却又无法证实的东西。
_BATT_RE = re.compile(r"当前=([\d.]+)V 最低=([\d.]+)V")


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
        # Host that "the PC is reachable" means. Empty disables the heartbeat.
        p("link_probe_host", "192.168.19.51")
        p("link_probe_period_s", 3.0)
        p("battery_period_s", 2.0)
        # 固件的低压闭锁是 9.60 V；这里早一点叫，好在日志里留下"塌之前"的样子
        p("battery_warn_v", 10.0)

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

        # The ESP32 has no network, so it cannot tell whether the PC is reachable -- it
        # can only be told, and only by whoever owns the serial port, which is this node.
        # /dev/ttyS3 takes exactly one owner, so a separate daemon could not write here
        # while we run. When this node is down nobody writes at all and the firmware's
        # own "UART silent" detector lights amber, which is the honest signal.
        self._probe_host = str(g("link_probe_host").value).strip()
        self._pc_ok = None
        self._pc_sent = None
        self._pc_sent_at = 0.0
        self.batt_pub = self.create_publisher(Float32, "/battery", 10)
        self._batt_period = float(g("battery_period_s").value)
        self._batt_warn = float(g("battery_warn_v").value)
        self._batt_at = 0.0
        self._batt_warned = False
        if self._probe_host:
            threading.Thread(target=self._probe_loop, daemon=True).start()
        self.get_logger().info(
            f"diffcar_control up on {g('port').value}, cam_offset={self.offset.tolist()}"
        )

    def _icmp_echo(self, timeout: float = 2.5) -> bool:
        """One ICMP echo, raw socket. The board has no `ping` binary -- busybox is
        stripped -- and shelling out raised FileNotFoundError, which killed this thread
        silently on the first attempt. Root on the X5, so SOCK_RAW is available."""
        pid = os.getpid() & 0xFFFF
        hdr = struct.pack("!BBHHH", 8, 0, 0, pid, 1)
        payload = b"tinynav-link"
        chk = self._checksum(hdr + payload)
        pkt = struct.pack("!BBHHH", 8, 0, chk, pid, 1) + payload
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
            sock.settimeout(timeout)
            sock.sendto(pkt, (self._probe_host, 0))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                data, _ = sock.recvfrom(1024)
                # 20-byte IPv4 header, then type 0 = echo reply; match our id
                if len(data) >= 28 and data[20] == 0 and data[24:26] == struct.pack("!H", pid):
                    return True
            return False
        except (OSError, socket.timeout):
            return False
        finally:
            if sock is not None:
                sock.close()

    @staticmethod
    def _checksum(data: bytes) -> int:
        if len(data) % 2:
            data += b"\x00"
        total = sum(struct.unpack("!%dH" % (len(data) // 2), data))
        total = (total & 0xFFFF) + (total >> 16)
        return ~((total & 0xFFFF) + (total >> 16)) & 0xFFFF

    def _probe_loop(self) -> None:
        """Reachability verdict for _tick. Does not touch the serial port itself: every
        write goes through _tick, so the 20 Hz command stream never interleaves with a
        heartbeat mid-line. Never lets an exception out -- a dead probe thread would
        leave the LED claiming everything is fine."""
        period = max(1.0, float(self.get_parameter("link_probe_period_s").value))
        # Both numbers are measured, not guessed. 180 probes at 1 Hz over this WiFi:
        # 17.2 percent "lost" at a 1.0 s timeout while p95 RTT was 839 ms and the max
        # 991 ms -- so most of those were slow, not lost, hence the 2.5 s timeout above.
        # Real loss bursts were never longer than 2 (seen 7 times in 3 minutes), so 2
        # misses would false-positive; 3 leaves margin. One success clears it -- a false
        # "fine" is worse than a late warning, so recovery is not debounced.
        misses = 0
        while True:
            try:
                if self._icmp_echo():
                    misses = 0
                    self._pc_ok = True
                else:
                    misses += 1
                    if misses >= 3:
                        self._pc_ok = False
            except Exception as e:                       # noqa: BLE001
                self.get_logger().warning(f"link probe failed, giving up on it: {e}")
                self._pc_ok = None
                return
            time.sleep(period)

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
        self._send_link_state()
        now = time.monotonic()
        if now - self._batt_at > self._batt_period:
            self._batt_at = now
            self.link.send("e")          # 回复里带电压；下面的解析只挑自己认识的行

        for line in self.link.drain():
            m = _POSE_RE.search(line)
            if m:
                x, y, theta_deg = (float(s) for s in m.groups())
                self._pose = (x, y, np.deg2rad(theta_deg))
                continue
            b = _BATT_RE.search(line)
            if b:
                self._on_battery(float(b.group(1)), float(b.group(2)))
        if self._pose is None:
            return
        self._publish(*self._pose)

    def _on_battery(self, now_v: float, min_v: float) -> None:
        """min_v is the lowest since the firmware was last asked, so it catches a sag
        between two polls -- the instantaneous reading would miss exactly the dip that
        matters. The firmware clears it on every read, so each value is one interval."""
        self.batt_pub.publish(Float32(data=now_v))
        if min_v < self._batt_warn:
            self.get_logger().warning(
                f"battery sagged to {min_v:.2f} V (now {now_v:.2f}); "
                f"firmware cuts out at 9.60"
            )
            self._batt_warned = True
        elif self._batt_warned and now_v > self._batt_warn + 0.4:
            self.get_logger().info(f"battery back to {now_v:.2f} V")
            self._batt_warned = False

    def _send_link_state(self) -> None:
        """Tell the firmware whether the PC answers, so its LED can show it. Resent
        every few seconds because the firmware ages the report out (a stale verdict is
        worse than none) -- and immediately on any change."""
        if self._pc_ok is None:
            return
        now = time.monotonic()
        if self._pc_ok == self._pc_sent and now - self._pc_sent_at < 3.0:
            return
        self.link.send(f"K {1 if self._pc_ok else 0}")
        if self._pc_ok != self._pc_sent:
            self.get_logger().warning(
                f"PC {self._probe_host} "
                + ("reachable again" if self._pc_ok else "unreachable -- LED goes cyan")
            )
        self._pc_sent, self._pc_sent_at = self._pc_ok, now

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
