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
        self.rtcm_serial_port = self.get_parameter("rtcm_serial_port").value or self.serial_port
        self.rtcm_baud = int(self.get_parameter("rtcm_baud").value) or self.baud
        self.ntrip_enabled = bool(self.get_parameter("ntrip_enabled").value)
        self.raw_pty_enabled = bool(self.get_parameter("raw_pty_enabled").value)
        self.raw_pty_path = self.get_parameter("raw_pty_path").value
        self.raw_sentence_types = self._parse_sentence_type_filter(self.get_parameter("raw_sentence_types").value)

        self.origin = self._load_origin_from_params()
        self.latest_fix: NavSatFix | None = None
        self.latest_gga = ""
        self.latest_position_gga = ""
        self.latest_gga_quality = 0
        self.latest_num_satellites = 0
        self.latest_hdop = float("nan")
        self.latest_gga_utc = ""
        self.latest_gga_differential_age = None
        self.latest_gga_station_id = ""
        self.latest_sentence = ""
        self.latest_sentence_type = ""
        self.last_gga_time = None
        self.nmea_checksum_fail_count = 0
        self.nmea_sentence_count = 0
        self.nmea_gga_count = 0
        self.nmea_rmc_count = 0
        self.nmea_heading_count = 0
        self.raw_sentence_publish_count = 0
        self.status_seq = 0
        self.latest_heading_yaw = 0.0
        self.latest_heading_stamp = None
        self.latest_velocity = np.zeros(3, dtype=np.float64)
        self.latest_velocity_stamp = None
        self.latest_time_reference = None
        self.last_nmea_time = None
        self.ntrip_connected = False
        self.ntrip_connect_count = 0
        self.ntrip_disconnect_count = 0
        self.last_rtcm_time = None
        self.rtcm_bytes = 0
        self.rtcm_written_bytes = 0
        self.rtcm_dropped_bytes = 0
        self.rtcm_write_fail_count = 0
        self.latest_ntrip_gga_source = "none"
        self.latest_enu = None
        self.serial_fd = None
        self.nmea_fd = None
        self.rtcm_fd = None
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
        self.io_status_pub = self.create_publisher(String, self.get_parameter("io_status_topic").value, 10)
        self.raw_pub = self.create_publisher(String, self.get_parameter("raw_sentence_topic").value, 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.status_timer = self.create_timer(1.0, self._publish_status_timer)

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
        self.declare_parameter("io_status_topic", "/rtk/io_status")
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
        self.declare_parameter("rtcm_serial_port", "")
        self.declare_parameter("rtcm_baud", 0)
        self.declare_parameter("split_same_serial_fd", True)
        self.declare_parameter("serial_read_only", True)
        self.declare_parameter("serial_init_commands", "")
        self.declare_parameter("rtcm_serial_init_commands", "")
        self.declare_parameter("raw_pty_enabled", True)
        self.declare_parameter("raw_pty_path", "/tmp/rtk_nmea")
        self.declare_parameter("raw_sentence_types", "GGA")
        self.declare_parameter("ntrip_enabled", True)
        self.declare_parameter("ntrip_host", os.environ.get("TINYNAV_NTRIP_HOST", "120.253.239.161"))
        self.declare_parameter("ntrip_port", int(os.environ.get("TINYNAV_NTRIP_PORT", "8002")))
        self.declare_parameter("ntrip_mountpoint", os.environ.get("TINYNAV_NTRIP_MOUNTPOINT", "RTCM33_GRCEJ"))
        self.declare_parameter("ntrip_user", os.environ.get("TINYNAV_NTRIP_USER", ""))
        self.declare_parameter("ntrip_password", os.environ.get("TINYNAV_NTRIP_PASSWORD", ""))
        self.declare_parameter("ntrip_request_version", os.environ.get("TINYNAV_NTRIP_REQUEST_VERSION", "1.0"))
        self.declare_parameter("ntrip_initial_gga", os.environ.get("TINYNAV_NTRIP_INITIAL_GGA", "$GNGGA,085520.00,2246.89808758,N,11330.83046100,E,1,17,1.1,5.5866,M,-5.5511,M,,*6F"))
        self.declare_parameter("ntrip_gga_period_s", 1.0)
        self.declare_parameter("ntrip_reconnect_s", 3.0)
        self.declare_parameter("ntrip_recv_timeout_s", 1.0)
        self.declare_parameter("rtcm_write_timeout_s", 0.25)
        self.declare_parameter("fix_stale_after_s", 2.0)
        self.declare_parameter("strict_nmea_checksum", False)

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
        if self.rtcm_serial_port == self.serial_port and self.rtcm_baud != self.baud:
            raise ValueError("rtcm_baud cannot differ from baud when both streams use the same serial port")
        split_same_port = self.rtcm_serial_port == self.serial_port and bool(
            self.get_parameter("split_same_serial_fd").value
        )
        read_only = bool(self.get_parameter("serial_read_only").value)
        nmea_flags = os.O_RDWR if self.rtcm_serial_port == self.serial_port and not split_same_port else (
            os.O_RDONLY if read_only else os.O_RDWR
        )
        self.nmea_fd = os.open(self.serial_port, nmea_flags | os.O_NOCTTY | os.O_NONBLOCK)
        set_serial_raw(self.nmea_fd, self.baud)
        self.serial_fd = self.nmea_fd
        self.get_logger().info(f"Opened RTK serial {self.serial_port} at {self.baud}")
        self._send_serial_init_commands(self.nmea_fd, "serial_init_commands", self.serial_port)
        if self.rtcm_serial_port == self.serial_port:
            if split_same_port:
                self.rtcm_fd = os.open(self.rtcm_serial_port, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
                set_serial_raw(self.rtcm_fd, self.rtcm_baud)
                self.get_logger().info(f"Opened RTCM writer on {self.rtcm_serial_port} at {self.rtcm_baud}")
            else:
                self.rtcm_fd = self.nmea_fd
        else:
            self.rtcm_fd = os.open(self.rtcm_serial_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            set_serial_raw(self.rtcm_fd, self.rtcm_baud)
            self.get_logger().info(f"Opened RTCM serial {self.rtcm_serial_port} at {self.rtcm_baud}")
            self._send_serial_init_commands(self.rtcm_fd, "rtcm_serial_init_commands", self.rtcm_serial_port)

    def _send_serial_init_commands(self, fd: int, param_name: str, port: str):
        commands = str(self.get_parameter(param_name).value or "")
        for command in commands.split(";"):
            command = command.strip()
            if not command:
                continue
            payload = command.encode("ascii") + b"\r\n"
            written = self._write_fd(fd, payload, timeout_s=0.5)
            if written == len(payload):
                self.get_logger().info(f"Sent init command to {port}: {command}")
            else:
                self.get_logger().warning(f"Could not fully send init command to {port}: {command}")

    def _serial_loop(self):
        buf = b""
        while not self.stop_event.is_set():
            try:
                readable, _, _ = select.select([self.nmea_fd], [], [], 0.2)
                if not readable:
                    continue
                data = os.read(self.nmea_fd, 4096)
                if not data:
                    continue
                buf += data

                while True:
                    cr_pos = buf.find(b"\r")
                    lf_pos = buf.find(b"\n")

                    if cr_pos == -1 and lf_pos == -1:
                        break

                    if cr_pos != -1 and (lf_pos == -1 or cr_pos < lf_pos):
                        split_pos = cr_pos
                    else:
                        split_pos = lf_pos

                    line = buf[:split_pos]
                    buf = buf[split_pos+1:]

                    if buf.startswith(b"\r") or buf.startswith(b"\n"):
                        buf = buf[1:]

                    line = line.decode("ascii", errors="ignore").strip()
                    if line and line.startswith("$"):
                        self._handle_nmea_line(line)

            except Exception as exc:
                self.get_logger().error(f"Serial read error: {exc}")
                time.sleep(1.0)

    def _ntrip_loop(self):
        while not self.stop_event.is_set():
            sock = None
            last_sent_gga_source = "none"
            try:
                sock, last_sent_gga_source = self._connect_ntrip()
                initial_data = self._read_http_header(sock)
                sock.settimeout(float(self.get_parameter("ntrip_recv_timeout_s").value))
                self.ntrip_connected = True
                self.ntrip_connect_count += 1
                last_sent_gga_source = self._send_ntrip_gga(sock)
                last_gga = time.monotonic()
                if initial_data:
                    self._handle_rtcm_data(initial_data)
                while not self.stop_event.is_set():
                    now = time.monotonic()
                    if now - last_gga >= float(self.get_parameter("ntrip_gga_period_s").value):
                        last_sent_gga_source = self._send_ntrip_gga(sock)
                        last_gga = now
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        raise ConnectionError("NTRIP socket closed")
                    self._handle_rtcm_data(data)

            except Exception as exc:
                self.ntrip_connected = False
                self.ntrip_disconnect_count += 1
                self.get_logger().warning(
                    f"NTRIP disconnected: {exc}; "
                    f"last_gga_source={last_sent_gga_source}, "
                    f"rtcm_bytes={self.rtcm_bytes}, written={self.rtcm_written_bytes}"
                )
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                time.sleep(float(self.get_parameter("ntrip_reconnect_s").value))

    def _handle_rtcm_data(self, data: bytes):
        self.rtcm_bytes += len(data)
        written = self._write_serial(data)
        self.rtcm_written_bytes += written
        dropped = len(data) - written
        if dropped > 0:
            self.rtcm_dropped_bytes += dropped
            self.rtcm_write_fail_count += 1
            self.get_logger().warning(f"Serial write timeout, dropped {dropped}/{len(data)} RTCM bytes")
        if written > 0:
            self.last_rtcm_time = time.monotonic()

    def _write_serial(self, data: bytes) -> int:
        if self.rtcm_fd is None:
            return 0
        return self._write_fd(self.rtcm_fd, data, float(self.get_parameter("rtcm_write_timeout_s").value))

    def _write_fd(self, fd: int, data: bytes, timeout_s: float) -> int:
        deadline = time.monotonic() + timeout_s
        written = 0
        while written < len(data) and not self.stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            _, writable, _ = select.select([], [fd], [], remaining)
            if not writable:
                break
            try:
                chunk_written = os.write(fd, data[written:])
            except BlockingIOError:
                time.sleep(0.001)
                continue
            except OSError:
                break
            if chunk_written <= 0:
                break
            written += chunk_written
        return written

    def _select_ntrip_gga(self):
        if self.latest_position_gga:
            return self.latest_position_gga, "live"
        return self.get_parameter("ntrip_initial_gga").value, "initial"

    def _send_ntrip_gga(self, sock):
        gga, gga_source = self._select_ntrip_gga()
        if gga:
            sock.sendall((gga.strip() + "\r\n").encode("ascii"))
        self.latest_ntrip_gga_source = gga_source
        return gga_source

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
        version = str(self.get_parameter("ntrip_request_version").value)
        if version == "2.0":
            req = (
                f"GET /{mount} HTTP/1.1\r\n"
                f"Host: {host}:{port}\r\n"
                f"Ntrip-Version: Ntrip/2.0\r\n"
                f"User-Agent: NTRIP TinyNav/1.0\r\n"
                f"Authorization: Basic {auth}\r\n"
                f"Connection: keep-alive\r\n\r\n"
            )
        else:
            req = (
                f"GET /{mount} HTTP/1.0\r\n"
                f"User-Agent: NTRIP TinyNav/1.0\r\n"
                f"Authorization: Basic {auth}\r\n\r\n"
            )
        sock = socket.create_connection((host, port), timeout=10.0)
        sock.settimeout(10.0)
        sock.sendall(req.encode("ascii"))
        gga_source = self._send_ntrip_gga(sock)
        self.get_logger().info(
            f"Opened NTRIP TCP {host}:{port}/{mount} request_version={version}, "
            f"initial_gga_source={gga_source}"
        )
        return sock, gga_source

    def _read_http_header(self, sock):
        data = b""
        max_header_bytes = 8192
        start = time.monotonic()
        while time.monotonic() - start < 10.0:
            try:
                chunk = sock.recv(256)
                if not chunk:
                    header = data.decode("latin1", errors="ignore").strip()
                    raise ConnectionError(f"NTRIP socket closed while reading header: {header!r}")
                data += chunk
                status_line, payload_after_status = self._split_status_line(data)
                if status_line is None:
                    if len(data) > max_header_bytes:
                        raise ConnectionError("NTRIP header too large before status line")
                    continue
                if "200" not in status_line and "ICY" not in status_line:
                    raise ConnectionError(f"Bad NTRIP response: {status_line}")
                if status_line.startswith("ICY"):
                    self.get_logger().info(f"NTRIP header: {status_line!r}")
                    return payload_after_status
                header, payload = self._split_http_header(data)
                if header is not None:
                    self.get_logger().info(f"NTRIP header: {header[:100]!r}")
                    return payload
                if len(data) > max_header_bytes:
                    raise ConnectionError(f"NTRIP HTTP header too large: {status_line}")
            except socket.timeout:
                continue

        header = data.decode("latin1", errors="ignore").strip()
        raise TimeoutError(f"Timed out reading NTRIP header: {header[:100]!r}")

    @staticmethod
    def _split_status_line(data: bytes):
        lf = data.find(b"\n")
        if lf < 0:
            return None, b""
        line = data[: lf + 1].decode("latin1", errors="ignore").strip()
        return line, data[lf + 1 :]

    @staticmethod
    def _split_http_header(data: bytes):
        for sep in (b"\r\n\r\n", b"\n\n"):
            pos = data.find(sep)
            if pos >= 0:
                header = data[:pos].decode("latin1", errors="ignore")
                return header, data[pos + len(sep) :]
        return None, b""

    def _handle_nmea_line(self, line: str):
        self.last_nmea_time = time.monotonic()
        self.nmea_sentence_count += 1
        if self.raw_pty_master is not None:
            try:
                os.write(self.raw_pty_master, (line + "\n").encode("ascii", errors="ignore"))
            except OSError:
                pass
        has_checksum = "*" in line
        if has_checksum and not nmea_checksum_ok(line):
            self.nmea_checksum_fail_count += 1
            self.get_logger().warning(f"NMEA checksum failed: {line}")
            return
        if not has_checksum and bool(self.get_parameter("strict_nmea_checksum").value):
            self.nmea_checksum_fail_count += 1
            return
        parts = line[1:].split("*")[0].split(",")
        msg_type = parts[0][2:]
        self.latest_sentence = line
        self.latest_sentence_type = msg_type
        if self._should_publish_raw_sentence(msg_type):
            self.raw_pub.publish(String(data=line))
            self.raw_sentence_publish_count += 1
        if msg_type == "GGA":
            self.nmea_gga_count += 1
            self.latest_gga = line
            self.last_gga_time = time.monotonic()
            if len(parts) > 5 and parts[2] and parts[3] and parts[4] and parts[5]:
                self.latest_position_gga = line
            self._parse_gga(parts)
        elif msg_type == "RMC":
            self.nmea_rmc_count += 1
            self._parse_rmc(parts)
        elif msg_type in ("HDT", "THS"):
            self.nmea_heading_count += 1
            self._parse_heading(parts)

    def _should_publish_raw_sentence(self, msg_type: str) -> bool:
        return not self.raw_sentence_types or msg_type in self.raw_sentence_types

    def _parse_gga(self, p: list[str]):
        if len(p) < 15:
            return
        lat = nmea_latlon(p[2], p[3])
        lon = nmea_latlon(p[4], p[5])
        if lat is None or lon is None:
            return
        quality = int(p[6] or "0")
        num_satellites = int(p[7] or "0")
        hdop = float(p[8] or "nan")
        alt = float(p[9] or "0.0")
        undulation = float(p[11] or "0.0") if len(p) > 11 and p[11] else 0.0
        differential_age = float(p[13]) if len(p) > 13 and p[13] else None
        station_id = p[14] if len(p) > 14 else ""
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
        self.latest_gga_quality = quality
        self.latest_num_satellites = num_satellites
        self.latest_hdop = hdop
        self.latest_gga_utc = p[1]
        self.latest_gga_differential_age = differential_age
        self.latest_gga_station_id = station_id
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
            self.latest_enu = None
            return
        if self.origin is None:
            self.origin = make_origin(msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(
                f"Initialized RTK ENU origin lat={msg.latitude:.9f}, "
                f"lon={msg.longitude:.9f}, alt={msg.altitude:.3f}"
            )
        position = lla_to_enu(msg.latitude, msg.longitude, msg.altitude, self.origin)
        self.latest_enu = [float(v) for v in position]
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

    def _publish_status_timer(self):
        if self.latest_fix is None:
            self._publish_status(None, accepted=False, position=None)
        else:
            accepted = self.latest_fix.status.status >= self.min_navsat_status and not self._fix_is_stale()
            self._publish_status(self.latest_fix, accepted=accepted, position=self.latest_enu)

    def _fix_is_stale(self) -> bool:
        if self.last_nmea_time is None:
            return True
        return time.monotonic() - self.last_nmea_time > float(self.get_parameter("fix_stale_after_s").value)

    def _publish_status(self, msg: NavSatFix | None, accepted: bool, position):
        now = time.monotonic()
        self.status_seq += 1
        nmea_age = None if self.last_nmea_time is None else now - self.last_nmea_time
        gga_age = None if self.last_gga_time is None else now - self.last_gga_time
        rtcm_age = None if self.last_rtcm_time is None else now - self.last_rtcm_time
        io_status = {
            "seq": self.status_seq,
            "ntrip_connected": self.ntrip_connected,
            "last_nmea_age_s": nmea_age,
            "last_gga_age_s": gga_age,
            "last_rtcm_age_s": rtcm_age,
            "latest_sentence_type": self.latest_sentence_type or None,
            "nmea_sentence_count": self.nmea_sentence_count,
            "nmea_gga_count": self.nmea_gga_count,
            "nmea_rmc_count": self.nmea_rmc_count,
            "nmea_heading_count": self.nmea_heading_count,
            "raw_sentence_publish_count": self.raw_sentence_publish_count,
            "nmea_checksum_fail_count": self.nmea_checksum_fail_count,
            "rtcm_bytes": self.rtcm_bytes,
            "rtcm_written_bytes": self.rtcm_written_bytes,
            "rtcm_dropped_bytes": self.rtcm_dropped_bytes,
            "rtcm_write_fail_count": self.rtcm_write_fail_count,
            "ntrip_gga_source": self.latest_ntrip_gga_source,
            "gga_quality": self.latest_gga_quality,
            "fix_stale": self._fix_is_stale(),
        }
        status = {
            "seq": self.status_seq,
            "accepted": accepted,
            "navsat_status": None if msg is None else int(msg.status.status),
            "service": None if msg is None else int(msg.status.service),
            "fix_stale": self._fix_is_stale(),
            "last_gga_age_s": gga_age,
            "last_rtcm_age_s": rtcm_age,
            "gga_quality": self.latest_gga_quality,
            "gga_utc": self.latest_gga_utc or None,
            "gga_differential_age_s": self.latest_gga_differential_age,
            "gga_station_id": self.latest_gga_station_id or None,
            "num_satellites": self.latest_num_satellites,
            "hdop": None if not math.isfinite(self.latest_hdop) else self.latest_hdop,
            "latitude": None if msg is None else msg.latitude,
            "longitude": None if msg is None else msg.longitude,
            "altitude": None if msg is None else msg.altitude,
            "enu": position,
            "latest_gga": self.latest_gga or None,
            "origin_ready": self.origin is not None,
            "heading_ready": self.latest_heading_stamp is not None,
            "velocity_ready": self.latest_velocity_stamp is not None,
        }
        self.status_pub.publish(String(data=json.dumps(status, separators=(",", ":"))))
        self.io_status_pub.publish(String(data=json.dumps(io_status, separators=(",", ":"))))

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

    @staticmethod
    def _parse_sentence_type_filter(value):
        text = str(value or "").strip().upper()
        if not text or text in ("*", "ALL"):
            return set()
        return {part.strip() for part in text.split(",") if part.strip()}

    def destroy_node(self):
        self.stop_event.set()
        for fd in {self.nmea_fd, self.rtcm_fd}:
            if fd is None:
                continue
            try:
                os.close(fd)
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
