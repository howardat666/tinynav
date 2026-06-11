#!/usr/bin/env python3
import base64
import calendar
import json
import math
import os
import pty
import select
import socket
import termios
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
KNOT_TO_MPS = 0.5144444444444445


@dataclass
class LlaOrigin:
    lat_rad: float
    lon_rad: float
    alt_m: float
    ecef: np.ndarray
    ecef_to_enu: np.ndarray


def lla_to_ecef(lat_rad: float, lon_rad: float, alt_m: float) -> np.ndarray:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array(
        [
            (n + alt_m) * cos_lat * cos_lon,
            (n + alt_m) * cos_lat * sin_lon,
            (n * (1.0 - WGS84_E2) + alt_m) * sin_lat,
        ],
        dtype=np.float64,
    )


def make_origin(lat_deg: float, lon_deg: float, alt_m: float) -> LlaOrigin:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    ecef_to_enu = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )
    return LlaOrigin(lat, lon, alt_m, lla_to_ecef(lat, lon, alt_m), ecef_to_enu)


def lla_to_enu(lat_deg: float, lon_deg: float, alt_m: float, origin: LlaOrigin) -> np.ndarray:
    ecef = lla_to_ecef(math.radians(lat_deg), math.radians(lon_deg), alt_m)
    return origin.ecef_to_enu @ (ecef - origin.ecef)


def yaw_to_quat(yaw_rad: float):
    return R.from_euler("z", yaw_rad).as_quat()


def nmea_checksum_ok(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False
    body, checksum = line[1:].split("*", 1)
    value = 0
    for ch in body:
        value ^= ord(ch)
    try:
        expected = int(checksum[:2], 16)
    except ValueError:
        return False
    return value == expected


def nmea_latlon(value: str, hemi: str) -> float | None:
    if not value or not hemi:
        return None
    dot = value.find(".")
    deg_digits = (dot - 2) if dot >= 0 else (len(value) - 2)
    if deg_digits <= 0:
        return None
    deg = float(value[:deg_digits])
    minutes = float(value[deg_digits:])
    out = deg + minutes / 60.0
    if hemi in ("S", "W"):
        out = -out
    return out


def ros_time_from_utc(hhmmss: str, ddmmyy: str | None = None):
    if not hhmmss:
        return None
    now = datetime.now(timezone.utc)
    day = now.day
    month = now.month
    year = now.year
    if ddmmyy and len(ddmmyy) >= 6:
        day = int(ddmmyy[0:2])
        month = int(ddmmyy[2:4])
        year = 2000 + int(ddmmyy[4:6])
    hour = int(hhmmss[0:2])
    minute = int(hhmmss[2:4])
    sec_float = float(hhmmss[4:])
    second = int(sec_float)
    nanosec = int(round((sec_float - second) * 1e9))
    dt = datetime(year, month, day, hour, minute, second, tzinfo=timezone.utc)
    stamp = Time()
    stamp.sec = calendar.timegm(dt.utctimetuple())
    stamp.nanosec = nanosec
    return stamp


def set_serial_raw(fd: int, baud: int):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
    attrs[2] = attrs[2] & ~termios.CSIZE
    attrs[2] = attrs[2] | termios.CS8
    attrs[2] = attrs[2] & ~(termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
    attrs[3] = 0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    speed = getattr(termios, f"B{baud}", None)
    if speed is None:
        raise ValueError(f"Unsupported baud rate: {baud}")
    attrs[4] = speed
    attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class RtkBridgeNode(Node):
    def __init__(self):
        super().__init__("rtk_bridge_node")
        self._declare_params()
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.min_navsat_status = int(self.get_parameter("min_navsat_status").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.path_max_size = int(self.get_parameter("path_max_size").value)
        self.heading_offset = math.radians(float(self.get_parameter("heading_offset_deg").value))
        self.use_heading = bool(self.get_parameter("use_heading").value)
        self.heading_is_north_clockwise = bool(self.get_parameter("heading_is_north_clockwise").value)
        self.serial_enabled = bool(self.get_parameter("serial_enabled").value)
        self.serial_port = self.get_parameter("serial_port").value
        self.baud = int(self.get_parameter("baud").value)
        self.ntrip_enabled = bool(self.get_parameter("ntrip_enabled").value)
        self.raw_pty_enabled = bool(self.get_parameter("raw_pty_enabled").value)
        self.raw_pty_path = self.get_parameter("raw_pty_path").value

        self.origin = self._load_origin_from_params()
        self.latest_fix: NavSatFix | None = None
        self.latest_gga = ""
        self.latest_heading_yaw = 0.0
        self.latest_heading_stamp = None
        self.latest_velocity = np.zeros(3, dtype=np.float64)
        self.latest_velocity_stamp = None
        self.latest_time_reference = None
        self.serial_fd = None
        self.raw_pty_master = None
        self.stop_event = threading.Event()
        self.path = Path()
        self.path.header.frame_id = self.frame_id

        self.fix_pub = self.create_publisher(NavSatFix, self.get_parameter("fix_topic").value, 20)
        self.heading_pub = self.create_publisher(QuaternionStamped, self.get_parameter("heading_topic").value, 20)
        self.vel_pub = self.create_publisher(TwistStamped, self.get_parameter("vel_topic").value, 20)
        self.time_ref_pub = self.create_publisher(TimeReference, self.get_parameter("time_reference_topic").value, 20)
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 10)
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)
        self.raw_pub = self.create_publisher(String, self.get_parameter("raw_sentence_topic").value, 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.raw_pty_enabled:
            self._setup_raw_pty()
        if self.serial_enabled:
            self._open_serial()
            threading.Thread(target=self._serial_loop, daemon=True).start()
        if self.ntrip_enabled:
            threading.Thread(target=self._ntrip_loop, daemon=True).start()

        self.get_logger().info(
            f"RTK bridge ready. serial={self.serial_enabled}:{self.serial_port}, "
            f"ntrip={self.ntrip_enabled}, odom={self.get_parameter('odom_topic').value}"
        )

    def _declare_params(self):
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("heading_topic", "/heading")
        self.declare_parameter("vel_topic", "/vel")
        self.declare_parameter("time_reference_topic", "/time_reference")
        self.declare_parameter("odom_topic", "/rtk/odom")
        self.declare_parameter("path_topic", "/rtk/path")
        self.declare_parameter("status_topic", "/rtk/status")
        self.declare_parameter("raw_sentence_topic", "/rtk/nmea_sentence")
        self.declare_parameter("frame_id", "rtk_world")
        self.declare_parameter("child_frame_id", "rtk_base")
        self.declare_parameter("min_navsat_status", int(NavSatStatus.STATUS_FIX))
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("path_max_size", 2000)
        self.declare_parameter("heading_offset_deg", 0.0)
        self.declare_parameter("use_heading", True)
        self.declare_parameter("heading_is_north_clockwise", True)
        self.declare_parameter("origin_lat", float("nan"))
        self.declare_parameter("origin_lon", float("nan"))
        self.declare_parameter("origin_alt", float("nan"))
        self.declare_parameter("serial_enabled", True)
        self.declare_parameter("serial_port", "/dev/ttyCH341USB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("raw_pty_enabled", True)
        self.declare_parameter("raw_pty_path", "/tmp/rtk_nmea")
        self.declare_parameter("ntrip_enabled", True)
        self.declare_parameter("ntrip_host", os.environ.get("TINYNAV_NTRIP_HOST", "120.253.239.161"))
        self.declare_parameter("ntrip_port", int(os.environ.get("TINYNAV_NTRIP_PORT", "8002")))
        self.declare_parameter("ntrip_mountpoint", os.environ.get("TINYNAV_NTRIP_MOUNTPOINT", "RTCM33_GRCEJ"))
        self.declare_parameter("ntrip_user", os.environ.get("TINYNAV_NTRIP_USER", ""))
        self.declare_parameter("ntrip_password", os.environ.get("TINYNAV_NTRIP_PASSWORD", ""))
        self.declare_parameter("ntrip_initial_gga", os.environ.get("TINYNAV_NTRIP_INITIAL_GGA", "$GNGGA,085520.00,2246.89808758,N,11330.83046100,E,1,17,1.1,5.5866,M,-5.5511,M,,*6F"))
        self.declare_parameter("ntrip_gga_period_s", 5.0)
        self.declare_parameter("ntrip_reconnect_s", 3.0)

    def _setup_raw_pty(self):
        master, slave = pty.openpty()
        slave_name = os.ttyname(slave)
        try:
            if os.path.islink(self.raw_pty_path) or os.path.exists(self.raw_pty_path):
                os.unlink(self.raw_pty_path)
            os.symlink(slave_name, self.raw_pty_path)
        except OSError as exc:
            self.get_logger().warning(f"Could not create {self.raw_pty_path}: {exc}")
        self.raw_pty_master = master
        self.get_logger().info(f"Raw NMEA mirror: cat {self.raw_pty_path}")

    def _open_serial(self):
        self.serial_fd = os.open(self.serial_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        set_serial_raw(self.serial_fd, self.baud)
        self.get_logger().info(f"Opened RTK serial {self.serial_port} at {self.baud}")

    def _serial_loop(self):
        buf = b""
        while not self.stop_event.is_set():
            try:
                readable, _, _ = select.select([self.serial_fd], [], [], 0.2)
                if not readable:
                    continue
                data = os.read(self.serial_fd, 4096)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip(b"\r").decode("ascii", errors="ignore").strip()
                    if line:
                        self._handle_nmea_line(line)
            except Exception as exc:
                self.get_logger().error(f"Serial read error: {exc}")
                time.sleep(1.0)

    def _ntrip_loop(self):
        while not self.stop_event.is_set():
            sock = None
            try:
                sock = self._connect_ntrip()
                self._read_http_header(sock)
                last_gga = 0.0
                while not self.stop_event.is_set():
                    now = time.monotonic()
                    if now - last_gga >= float(self.get_parameter("ntrip_gga_period_s").value):
                        gga = self.latest_gga or self.get_parameter("ntrip_initial_gga").value
                        if gga:
                            sock.sendall((gga.strip() + "\r\n").encode("ascii"))
                        last_gga = now
                    data = sock.recv(4096)
                    if not data:
                        raise ConnectionError("NTRIP socket closed")
                    if self.serial_fd is not None:
                        os.write(self.serial_fd, data)
            except Exception as exc:
                self.get_logger().warning(f"NTRIP disconnected: {exc}")
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                time.sleep(float(self.get_parameter("ntrip_reconnect_s").value))

    def _connect_ntrip(self):
        host = self.get_parameter("ntrip_host").value
        port = int(self.get_parameter("ntrip_port").value)
        mount = self.get_parameter("ntrip_mountpoint").value.lstrip("/")
        user = self.get_parameter("ntrip_user").value
        password = self.get_parameter("ntrip_password").value
        if not host or not mount or not user or not password:
            raise ValueError(
                "NTRIP config is incomplete. Set ntrip_host, ntrip_mountpoint, "
                "ntrip_user, and ntrip_password via ROS params or TINYNAV_NTRIP_* env vars."
            )
        auth = base64.b64encode(f"{user}:{password}".encode("utf-8")).decode("ascii")
        req = (
            f"GET /{mount} HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP TinyNav/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"Connection: close\r\n\r\n"
        )
        sock = socket.create_connection((host, port), timeout=10.0)
        sock.settimeout(10.0)
        sock.sendall(req.encode("ascii"))
        self.get_logger().info(f"Connected NTRIP {host}:{port}/{mount}")
        return sock

    def _read_http_header(self, sock):
        data = b""
        while b"\r\n\r\n" not in data and len(data) < 4096:
            data += sock.recv(1)
        header = data.decode("latin1", errors="ignore")
        if "200" not in header and "ICY" not in header:
            raise ConnectionError(header.strip())

    def _handle_nmea_line(self, line: str):
        if self.raw_pty_master is not None:
            try:
                os.write(self.raw_pty_master, (line + "\n").encode("ascii", errors="ignore"))
            except OSError:
                pass
        self.raw_pub.publish(String(data=line))
        if not nmea_checksum_ok(line):
            return
        self.latest_gga = line if line[3:6] == "GGA" else self.latest_gga
        parts = line[1:].split("*")[0].split(",")
        msg_type = parts[0][2:]
        if msg_type == "GGA":
            self._parse_gga(parts)
        elif msg_type == "RMC":
            self._parse_rmc(parts)
        elif msg_type in ("HDT", "THS"):
            self._parse_heading(parts)

    def _parse_gga(self, p: list[str]):
        if len(p) < 15:
            return
        lat = nmea_latlon(p[2], p[3])
        lon = nmea_latlon(p[4], p[5])
        if lat is None or lon is None:
            return
        quality = int(p[6] or "0")
        alt = float(p[9] or "0.0")
        undulation = float(p[11] or "0.0") if len(p) > 11 and p[11] else 0.0
        stamp = ros_time_from_utc(p[1]) or self.get_clock().now().to_msg()
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "gps"
        fix.status.status = self._gga_quality_to_status(quality)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt + undulation
        sigma_h = 0.02 if quality >= 4 else 0.5
        sigma_v = 0.08 if quality >= 4 else 1.0
        fix.position_covariance = [sigma_h**2, 0.0, 0.0, 0.0, sigma_h**2, 0.0, 0.0, 0.0, sigma_v**2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(fix)
        self.latest_fix = fix
        self._publish_odom_from_fix(fix)

        time_ref = TimeReference()
        time_ref.header = fix.header
        time_ref.time_ref = stamp
        time_ref.source = "nmea_gga"
        self.latest_time_reference = stamp
        self.time_ref_pub.publish(time_ref)

    def _parse_rmc(self, p: list[str]):
        if len(p) < 10 or p[2] != "A":
            return
        stamp = ros_time_from_utc(p[1], p[9]) or self.get_clock().now().to_msg()
        speed = float(p[7] or "0.0") * KNOT_TO_MPS
        course = math.radians(float(p[8] or "0.0"))
        yaw = self._wrap_angle(math.pi / 2.0 - course)
        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.frame_id
        twist.twist.linear.x = speed * math.cos(yaw)
        twist.twist.linear.y = speed * math.sin(yaw)
        twist.twist.linear.z = 0.0
        self.latest_velocity = np.array([twist.twist.linear.x, twist.twist.linear.y, 0.0], dtype=np.float64)
        self.latest_velocity_stamp = stamp
        self.vel_pub.publish(twist)

    def _parse_heading(self, p: list[str]):
        if len(p) < 2 or not p[1]:
            return
        heading = math.radians(float(p[1]))
        yaw = self._wrap_angle(math.pi / 2.0 - heading + self.heading_offset)
        self.latest_heading_yaw = yaw
        self.latest_heading_stamp = self.get_clock().now().to_msg()
        quat = yaw_to_quat(yaw)
        msg = QuaternionStamped()
        msg.header.stamp = self.latest_heading_stamp
        msg.header.frame_id = self.frame_id
        msg.quaternion.x = float(quat[0])
        msg.quaternion.y = float(quat[1])
        msg.quaternion.z = float(quat[2])
        msg.quaternion.w = float(quat[3])
        self.heading_pub.publish(msg)

    def _publish_odom_from_fix(self, msg: NavSatFix):
        if msg.status.status < self.min_navsat_status:
            self._publish_status(msg, accepted=False, position=None)
            return
        if self.origin is None:
            self.origin = make_origin(msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(
                f"Initialized RTK ENU origin lat={msg.latitude:.9f}, "
                f"lon={msg.longitude:.9f}, alt={msg.altitude:.3f}"
            )
        position = lla_to_enu(msg.latitude, msg.longitude, msg.altitude, self.origin)
        yaw = self.latest_heading_yaw if self.use_heading else 0.0
        quat = yaw_to_quat(yaw)
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        odom.twist.twist.linear.x = float(self.latest_velocity[0])
        odom.twist.twist.linear.y = float(self.latest_velocity[1])
        odom.twist.twist.linear.z = float(self.latest_velocity[2])
        self._copy_position_covariance(msg, odom)
        self.odom_pub.publish(odom)
        self._publish_path(odom)
        self._publish_status(msg, accepted=True, position=position)
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(self._odom_to_tf(odom))

    def _load_origin_from_params(self):
        lat = float(self.get_parameter("origin_lat").value)
        lon = float(self.get_parameter("origin_lon").value)
        alt = float(self.get_parameter("origin_alt").value)
        if math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt):
            origin = make_origin(lat, lon, alt)
            self.get_logger().info(f"Using configured RTK origin lat={lat:.9f}, lon={lon:.9f}, alt={alt:.3f}")
            return origin
        return None

    def _copy_position_covariance(self, fix_msg: NavSatFix, odom_msg: Odometry):
        cov = list(odom_msg.pose.covariance)
        fix_cov = list(fix_msg.position_covariance)
        cov[0] = fix_cov[0]
        cov[1] = fix_cov[1]
        cov[2] = fix_cov[2]
        cov[6] = fix_cov[3]
        cov[7] = fix_cov[4]
        cov[8] = fix_cov[5]
        cov[12] = fix_cov[6]
        cov[13] = fix_cov[7]
        cov[14] = fix_cov[8]
        cov[35] = 0.1 if self.use_heading else 999.0
        odom_msg.pose.covariance = cov

    def _publish_path(self, odom: Odometry):
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.header.stamp = odom.header.stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.path_max_size:
            self.path.poses = self.path.poses[-self.path_max_size:]
        self.path_pub.publish(self.path)

    def _publish_status(self, msg: NavSatFix, accepted: bool, position):
        payload = {
            "accepted": accepted,
            "navsat_status": int(msg.status.status),
            "service": int(msg.status.service),
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "origin_ready": self.origin is not None,
            "heading_ready": self.latest_heading_stamp is not None,
            "velocity_ready": self.latest_velocity_stamp is not None,
            "ntrip_enabled": self.ntrip_enabled,
            "serial_enabled": self.serial_enabled,
            "enu": None if position is None else [float(v) for v in position],
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def _odom_to_tf(self, odom: Odometry):
        from geometry_msgs.msg import TransformStamped

        tf_msg = TransformStamped()
        tf_msg.header = odom.header
        tf_msg.child_frame_id = odom.child_frame_id
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z
        tf_msg.transform.rotation = odom.pose.pose.orientation
        return tf_msg

    @staticmethod
    def _gga_quality_to_status(quality: int) -> int:
        if quality in (4, 5):
            return NavSatStatus.STATUS_GBAS_FIX
        if quality > 0:
            return NavSatStatus.STATUS_FIX
        return NavSatStatus.STATUS_NO_FIX

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def destroy_node(self):
        self.stop_event.set()
        if self.serial_fd is not None:
            try:
                os.close(self.serial_fd)
            except OSError:
                pass
        if self.raw_pty_master is not None:
            try:
                os.close(self.raw_pty_master)
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
