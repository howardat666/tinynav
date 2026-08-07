"""Minimal Feetech / SCS servo bus driver (pyserial + numpy only).

Why this exists
---------------
The upstream LeRobot LeKiwi driver talks to the Feetech STS3215 servos through
``lerobot.motors.feetech.FeetechMotorsBus``, which imports the whole ``lerobot``
package and therefore pulls in ``torch``.  The D-Robotics X5 inside the Looper
camera only has ~1.3 GB of writable rootfs, so ``torch`` is simply not an
option.  This module re-implements *only* the parts of the protocol that a wheel
odometry node needs:

* ``ping``
* single-register ``read`` / ``write``
* ``sync_read`` of one register across several motor IDs (one bus transaction)
* ``sync_write`` of one register across several motor IDs (used only when this
  process also owns wheel commands)

Everything else (calibration, normalisation, homing offsets, model scanning) is
deliberately left out.

Protocol notes (Feetech "SCS" protocol, a Dynamixel-1.0 derivative)
-------------------------------------------------------------------
Instruction packet::

    0xFF 0xFF  ID  LENGTH  INSTRUCTION  PARAM...  CHECKSUM

* ``LENGTH`` counts everything after itself: ``len(params) + 2``.
* ``CHECKSUM = (~(ID + LENGTH + INSTRUCTION + sum(params))) & 0xFF``.

Status packet returned by each motor::

    0xFF 0xFF  ID  LENGTH  ERROR  DATA...  CHECKSUM

A ``SYNC_READ`` is addressed to the broadcast ID (0xFE); every listed motor then
answers with its own status packet, back to back, in the order the IDs were
listed.  Total expected reply size is ``n_motors * (6 + data_length)``.

Two gotchas that are easy to get wrong and that cost real debugging time:

1. **Byte order.**  The STS/SMS series (protocol 0, which is what the STS3215
   is) is *little endian*: low byte first.  The older SCS series (protocol 1)
   is big endian.  Controlled here by ``protocol_end``.
2. **Sign encoding.**  ``Present_Position``, ``Present_Velocity`` and
   ``Goal_Velocity`` are **sign-magnitude**, not two's complement: bit 15 is the
   direction bit and bits 0..14 are the magnitude.  Decoding them as two's
   complement gives garbage for every negative value (e.g. a wheel spinning
   backwards at 100 ticks/s reports 0x8064 = 32868, which as two's complement
   is -32668 instead of -100).

Also note that protocol 1 (SCS series) has no ``SYNC_READ`` instruction at all;
``sync_read`` falls back to sequential ``read`` calls in that case.

Units
-----
* ``Present_Position``: encoder ticks, 4096 ticks per revolution on the STS3215.
* ``Present_Velocity``: encoder ticks per second (so deg/s = ticks/s / (4096/360)).

References: the register addresses and the sign-magnitude bit indices were taken
from ``lerobot/motors/feetech/tables.py`` (Apache-2.0, HuggingFace Inc.) which
in turn follows the Feetech STS/SMS e-manual.
"""

from __future__ import annotations

import errno
import logging
import termios
import time
from dataclasses import dataclass, field

import serial

logger = logging.getLogger(__name__)

# How many times to retry tcdrain() when a signal interrupts it. Signals arrive
# in bursts at process teardown, not continuously, so a small count is enough;
# see FeetechBus._drain for why exhausting them is not fatal.
_DRAIN_EINTR_RETRIES = 8

# --------------------------------------------------------------------------- #
# Protocol constants
# --------------------------------------------------------------------------- #

BROADCAST_ID = 0xFE

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83

# Length in bytes of the fixed part of a status packet: FF FF ID LEN ERR ... CHK
STATUS_OVERHEAD = 6

DEFAULT_BAUDRATE = 1_000_000

# data_name: (address, size_in_bytes).  Subset of STS_SMS_SERIES_CONTROL_TABLE.
STS_CONTROL_TABLE: dict[str, tuple[int, int]] = {
    "Model_Number": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Phase": (18, 1),
    "Operating_Mode": (33, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Velocity": (46, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
}

# First SRAM address.  Everything below this lives in EEPROM and can only be
# written while the motor's ``Lock`` register is 0 -- a locked write is
# acknowledged and then silently discarded, with no error byte set.  40 is
# ``Torque_Enable``, the first SRAM register on the STS/SMS series.
FIRST_SRAM_ADDRESS = 40

# Retries for the one-shot startup configuration sequence, which has no next tick to
# recover on. Deliberately much larger than the steady-state default: see
# configure_velocity_mode's docstring for why this is safe rather than papering over
# a fault.
_CONFIG_RETRIES = 12

# data_name: index of the sign bit (sign-magnitude encoded registers).
STS_SIGN_MAGNITUDE_BITS: dict[str, int] = {
    "Present_Load": 10,
    "Homing_Offset": 11,
    "Goal_Position": 15,
    "Goal_Velocity": 15,
    "Present_Position": 15,
    "Present_Velocity": 15,
}

# Encoder resolution of the STS3215 (ticks per full turn).
STS3215_RESOLUTION = 4096

# Velocity registers are expressed in ticks/s; this is the conversion the
# upstream LeKiwi code calls ``steps_per_deg``.
TICKS_PER_DEG = STS3215_RESOLUTION / 360.0


class FeetechBusError(Exception):
    """Base class for all bus level failures."""


class FeetechTimeoutError(FeetechBusError):
    """No (or not enough) bytes came back before the serial timeout."""


class FeetechChecksumError(FeetechBusError):
    """A status packet arrived but its checksum did not match."""


class FeetechStatusError(FeetechBusError):
    """A motor answered with a non-zero error byte."""


class FeetechIOError(FeetechBusError):
    """The serial layer itself failed, as opposed to the exchange timing out.

    pyserial raises SerialException out of read() when poll() reports the fd
    readable but the following read() returns zero bytes -- "device reports
    readiness to read but returned no data". On this bus that is simply another
    way for one transaction to fail, so it has to live *inside* FeetechBusError
    for the retry loops in read/write/sync_read to absorb it like any other bad
    attempt.

    It did not, and the consequence was severe: the exception escaped every
    retry, escaped wheel_odometry_node._tick's `except FeetechBusError`, and
    rclpy re-raised it out of the timer callback, so rclpy.spin() returned and
    the whole odometry node exited. Observed 2026-08-07 after 9 samples, which
    took /wheel/camera_pose down with it and left planning_node with no pose
    source at all. On a bus that loses ~19% of single attempts, a serial-layer
    hiccup must cost one sample, not the node.
    """


# --------------------------------------------------------------------------- #
# Sign-magnitude helpers
# --------------------------------------------------------------------------- #


def decode_sign_magnitude(encoded: int, sign_bit_index: int) -> int:
    """Decode a sign-magnitude integer (bit ``sign_bit_index`` is the sign)."""
    magnitude = encoded & ((1 << sign_bit_index) - 1)
    return -magnitude if (encoded >> sign_bit_index) & 1 else magnitude


def encode_sign_magnitude(value: int, sign_bit_index: int) -> int:
    """Encode an integer as sign-magnitude (bit ``sign_bit_index`` is the sign)."""
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = abs(int(value))
    if magnitude > max_magnitude:
        raise ValueError(f"magnitude {magnitude} exceeds {max_magnitude} for sign bit {sign_bit_index}")
    return ((1 if value < 0 else 0) << sign_bit_index) | magnitude


def checksum(payload: bytes | list[int]) -> int:
    """Feetech checksum: bitwise-not of the sum of every byte after the header."""
    return (~sum(payload)) & 0xFF


def configure_velocity_mode(bus, motor_ids) -> None:
    """Put servos into continuous-velocity mode with torque on, and verify it.

    ``Operating_Mode`` 1 is the STS3215's constant-speed mode, the one that obeys
    ``Goal_Velocity``.  It lives at address 33, inside the EEPROM region, so it
    can only be written while the motor has torque **off** and its ``Lock``
    register **cleared**.  A locked write is not refused: the servo acknowledges
    the packet with error byte 0 and silently discards it.  So both the order
    below and the read-back matter -- without the read-back the wheels stay in
    position mode, every ``Goal_Velocity`` write reports success, and nothing
    turns, with no error anywhere to explain it.

    Note the sequence ends with ``Lock`` set again, which is what makes the wrong
    version so convincing: writing ``Operating_Mode`` before unlocking works
    exactly once on a factory-fresh servo and then fails silently forever after.

    Lives here, rather than in whichever caller needs it, because there are now
    three -- the odometry node, the geometry calibration tool and the yaw-source
    comparison -- and a safety-relevant sequence that must not be got wrong is a
    bad thing to have three copies of.

    Every write here is retried far harder than the default, because this sequence
    is fifteen one-shot packets on a bus that drops roughly one in ten and drops
    them in bursts (docs/x5/servo_bus.md). With the default three attempts, one
    unlucky packet aborts the whole sequence and wheel_odometry_node exits at
    startup -- so the chassis has no driver at all, observed three times on
    2026-08-06 and once misattributed to a flat battery. Retrying costs nothing when
    the bus is healthy and does not weaken any guarantee: a retried write is
    idempotent, and the read-back below is still what decides success, so a servo
    that genuinely did not take the mode is still caught.

    Raises:
        FeetechBusError: if any servo does not read back mode 1 afterwards. An
            unreadable mode counts as failure too: commanding a wheel whose mode
            could not be confirmed is how a runaway starts.
    """
    for motor_id in motor_ids:
        bus.write("Torque_Enable", motor_id, 0, num_retry=_CONFIG_RETRIES)  # EEPROM needs torque off
        bus.write("Lock", motor_id, 0, num_retry=_CONFIG_RETRIES)  # ...and the lock cleared
        bus.write("Operating_Mode", motor_id, 1, num_retry=_CONFIG_RETRIES)
        bus.write("Lock", motor_id, 1, num_retry=_CONFIG_RETRIES)
        bus.write("Torque_Enable", motor_id, 1, num_retry=_CONFIG_RETRIES)

    wrong = {}
    for motor_id in motor_ids:
        try:
            mode = bus.read("Operating_Mode", motor_id, num_retry=_CONFIG_RETRIES)
        except FeetechBusError as exc:
            wrong[motor_id] = f"read failed: {exc}"
            continue
        if mode != 1:
            wrong[motor_id] = f"Operating_Mode={mode}, expected 1"
    if wrong:
        raise FeetechBusError(
            "base servos did not enter velocity mode: "
            + "; ".join(f"id {i}: {why}" for i, why in sorted(wrong.items()))
            + ". The EEPROM unlock did not take effect, so Goal_Velocity would be ignored."
        )


# --------------------------------------------------------------------------- #
# Read results
# --------------------------------------------------------------------------- #


@dataclass
class SyncReadResult:
    """One bus transaction, with enough timing info to integrate odometry.

    ``t_request`` / ``t_reply`` are ``time.monotonic()`` stamps taken right after
    the request was flushed and right after the last reply byte was read.  The
    servos latch their encoders somewhere in between, so ``t_sample`` (the
    midpoint) is the best single-instant estimate available without hardware
    timestamping.  At 1 Mbaud a 3-motor / 2-byte sync read spans roughly
    ``3 * 8 bytes * 10 bits / 1e6 = 240 us`` of wire time, so the residual skew
    between the first and last wheel is well under a millisecond -- two orders of
    magnitude below a 20 ms control period, hence ignorable.
    """

    values: dict[int, int]
    t_request: float
    t_reply: float
    retries: int = 0
    missing_ids: tuple[int, ...] = field(default_factory=tuple)

    @property
    def t_sample(self) -> float:
        """Best estimate of the instant the encoders were latched (monotonic)."""
        return 0.5 * (self.t_request + self.t_reply)

    @property
    def duration(self) -> float:
        """Wall time the transaction took, in seconds."""
        return self.t_reply - self.t_request

    @property
    def complete(self) -> bool:
        return not self.missing_ids


# --------------------------------------------------------------------------- #
# The bus
# --------------------------------------------------------------------------- #


class FeetechBus:
    """A very small synchronous Feetech servo bus.

    Only depends on ``pyserial``.  Not thread safe: the Feetech bus is half
    duplex, so exactly one process/thread may drive a given serial port.  If
    another process (e.g. a LeRobot ``lekiwi_host``) already holds the port, do
    not open it here as well -- the transactions will collide.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = 0.02,
        protocol_end: int = 0,
        control_table: dict[str, tuple[int, int]] | None = None,
        sign_magnitude_bits: dict[str, int] | None = None,
    ) -> None:
        """
        Args:
            port: serial device, e.g. ``/dev/ttyACM0``.
            baudrate: 1 Mbaud is the Feetech factory default.
            timeout: per-transaction read timeout in seconds.  20 ms is already
                ~80x the wire time of a 3-motor sync read.
            protocol_end: 0 for STS/SMS servos (little endian, this is the
                STS3215 used by LeKiwi), 1 for the older SCS series (big
                endian, and no sync read instruction).
        """
        if protocol_end not in (0, 1):
            raise ValueError("protocol_end must be 0 (STS/SMS, little endian) or 1 (SCS, big endian)")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.protocol_end = protocol_end
        self.control_table = dict(control_table or STS_CONTROL_TABLE)
        self.sign_magnitude_bits = dict(
            sign_magnitude_bits if sign_magnitude_bits is not None else STS_SIGN_MAGNITUDE_BITS
        )
        self._serial: serial.Serial | None = None

    # -- lifecycle ---------------------------------------------------------- #

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        if self.is_connected:
            return
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def disconnect(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.disconnect()

    def _require_serial(self) -> serial.Serial:
        if self._serial is None or not self._serial.is_open:
            raise FeetechBusError(f"bus on '{self.port}' is not connected; call connect() first")
        return self._serial

    # -- word packing ------------------------------------------------------- #

    def _split_word(self, value: int) -> list[int]:
        lo, hi = value & 0xFF, (value >> 8) & 0xFF
        return [lo, hi] if self.protocol_end == 0 else [hi, lo]

    def _join_word(self, data: bytes) -> int:
        lo, hi = (data[0], data[1]) if self.protocol_end == 0 else (data[1], data[0])
        return (hi << 8) | lo

    def _pack(self, value: int, length: int) -> list[int]:
        if length == 1:
            return [value & 0xFF]
        if length == 2:
            return self._split_word(value)
        raise ValueError(f"unsupported register size {length}")

    def _unpack(self, data: bytes, length: int) -> int:
        if length == 1:
            return data[0]
        if length == 2:
            return self._join_word(data)
        raise ValueError(f"unsupported register size {length}")

    def _address_of(self, data_name: str) -> tuple[int, int]:
        try:
            return self.control_table[data_name]
        except KeyError as exc:
            raise FeetechBusError(f"unknown register '{data_name}'") from exc

    def _decode(self, data_name: str, raw: int) -> int:
        sign_bit = self.sign_magnitude_bits.get(data_name)
        return raw if sign_bit is None else decode_sign_magnitude(raw, sign_bit)

    def _encode(self, data_name: str, value: int) -> int:
        sign_bit = self.sign_magnitude_bits.get(data_name)
        return int(value) if sign_bit is None else encode_sign_magnitude(int(value), sign_bit)

    # -- raw framing -------------------------------------------------------- #

    def _drain(self, ser) -> None:
        """``ser.flush()``, but tolerant of being interrupted by a signal.

        pyserial's ``flush()`` is ``termios.tcdrain()``, which blocks until the
        UART has actually shifted the bytes out.  The ``termios`` module is not
        covered by PEP 475, so a signal delivered while it blocks raises
        ``termios.error(EINTR)`` instead of the call being retried -- unlike
        almost every other blocking call in the standard library.

        Observed on the X5: SIGTERM arriving mid-transaction propagated out of a
        ROS timer callback as an unhandled traceback.  The dangerous part is not
        the ugly exit, it is that ``WheelOdometryNode.destroy_node()`` zeroes
        ``Goal_Velocity`` on the way out -- so a Ctrl-C landing inside tcdrain
        could skip the stop command and leave the base driving.

        Draining is only a barrier for tidiness: ``write()`` has already handed
        the bytes to the kernel and the UART will transmit them regardless.  So
        if the retries are exhausted, carry on rather than raise; the worst case
        is a stale byte at the head of the next read, which the checksum catches.
        """
        for _ in range(_DRAIN_EINTR_RETRIES):
            try:
                ser.flush()
                return
            except (termios.error, OSError) as exc:
                if (exc.args[0] if exc.args else None) != errno.EINTR:
                    # SerialException subclasses OSError, and its args[0] is a
                    # message rather than an errno, so it lands here. Re-raise it
                    # as a bus error for the same reason the read path does:
                    # otherwise it escapes every retry and kills the node.
                    if isinstance(exc, serial.SerialException):
                        raise FeetechIOError(
                            f"serial drain failed on '{self.port}': {exc}"
                        ) from exc
                    raise
        logger.debug("tcdrain interrupted %d times in a row; proceeding without the barrier",
                     _DRAIN_EINTR_RETRIES)

    def _send(self, motor_id: int, instruction: int, params: list[int]) -> None:
        ser = self._require_serial()
        length = len(params) + 2
        body = [motor_id & 0xFF, length, instruction, *params]
        packet = bytes([0xFF, 0xFF, *body, checksum(body)])
        try:
            ser.reset_input_buffer()
            ser.write(packet)
        except serial.SerialException as exc:
            raise FeetechIOError(f"serial write failed on '{self.port}': {exc}") from exc
        self._drain(ser)

    def _read_exactly(self, n: int) -> bytes:
        """Read exactly ``n`` bytes or raise :class:`FeetechTimeoutError`."""
        ser = self._require_serial()
        buf = bytearray()
        deadline = time.monotonic() + self.timeout
        while len(buf) < n:
            try:
                chunk = ser.read(n - len(buf))
            except serial.SerialException as exc:
                raise FeetechIOError(f"serial read failed on '{self.port}': {exc}") from exc
            if chunk:
                buf.extend(chunk)
                continue
            if time.monotonic() >= deadline:
                raise FeetechTimeoutError(f"expected {n} bytes on '{self.port}', got {len(buf)}")
        return bytes(buf)

    @staticmethod
    def _parse_status(packet: bytes) -> tuple[int, int, bytes]:
        """Validate one status packet, returning ``(motor_id, error, data)``."""
        if len(packet) < STATUS_OVERHEAD or packet[0] != 0xFF or packet[1] != 0xFF:
            raise FeetechChecksumError(f"malformed status packet header: {packet.hex(' ')}")
        body = packet[2:-1]
        if checksum(body) != packet[-1]:
            raise FeetechChecksumError(f"bad checksum on status packet: {packet.hex(' ')}")
        motor_id, length, error = packet[2], packet[3], packet[4]
        if length != len(packet) - 4:
            raise FeetechChecksumError(f"length field {length} inconsistent with {len(packet)} byte packet")
        return motor_id, error, packet[5:-1]

    def _receive_status(self, data_length: int) -> tuple[int, int, bytes]:
        """Read one status packet carrying ``data_length`` payload bytes.

        Resynchronises on the 0xFF 0xFF header so that a stray byte left over
        from a previous aborted transaction does not desynchronise the stream
        permanently.
        """
        ser = self._require_serial()
        deadline = time.monotonic() + self.timeout
        # Hunt for the header.
        window = bytearray()
        while True:
            if time.monotonic() >= deadline:
                raise FeetechTimeoutError(f"no status packet header on '{self.port}'")
            try:
                byte = ser.read(1)
            except serial.SerialException as exc:
                raise FeetechIOError(f"serial read failed on '{self.port}': {exc}") from exc
            if not byte:
                continue
            window.extend(byte)
            if len(window) > 2:
                del window[0]
            if len(window) == 2 and window[0] == 0xFF and window[1] == 0xFF:
                break
        rest = self._read_exactly(data_length + 4)  # ID LEN ERR DATA... CHK
        return self._parse_status(b"\xff\xff" + rest)

    # -- public single-motor API -------------------------------------------- #

    def ping(self, motor_id: int, num_retry: int = 0) -> bool:
        """Return True if ``motor_id`` answers a PING."""
        for _ in range(1 + num_retry):
            try:
                self._send(motor_id, INST_PING, [])
                got_id, error, _ = self._receive_status(0)
            except FeetechBusError:
                continue
            if got_id == motor_id and error == 0:
                return True
        return False

    def read(self, data_name: str, motor_id: int, num_retry: int = 2) -> int:
        """Read one register from one motor, sign-decoded."""
        addr, length = self._address_of(data_name)
        last: Exception | None = None
        for _ in range(1 + num_retry):
            try:
                self._send(motor_id, INST_READ, [addr, length])
                got_id, error, data = self._receive_status(length)
                if got_id != motor_id:
                    raise FeetechBusError(f"reply from id {got_id}, expected {motor_id}")
                if error != 0:
                    raise FeetechStatusError(f"motor {motor_id} error byte 0x{error:02x}")
                return self._decode(data_name, self._unpack(data, length))
            except FeetechBusError as exc:
                last = exc
        raise FeetechBusError(f"read '{data_name}' from motor {motor_id} failed: {last}") from last

    def write(self, data_name: str, motor_id: int, value: int, num_retry: int = 2) -> None:
        """Write one register on one motor and wait for the acknowledgement."""
        addr, length = self._address_of(data_name)
        params = [addr, *self._pack(self._encode(data_name, value), length)]
        last: Exception | None = None
        for _ in range(1 + num_retry):
            try:
                self._send(motor_id, INST_WRITE, params)
                got_id, error, _ = self._receive_status(0)
                if got_id != motor_id:
                    raise FeetechBusError(f"reply from id {got_id}, expected {motor_id}")
                if error != 0:
                    raise FeetechStatusError(f"motor {motor_id} error byte 0x{error:02x}")
                return
            except FeetechBusError as exc:
                last = exc
        raise FeetechBusError(f"write '{data_name}' to motor {motor_id} failed: {last}") from last

    # -- public group API --------------------------------------------------- #

    def sync_read(
        self,
        data_name: str,
        motor_ids: list[int] | tuple[int, ...],
        num_retry: int = 2,
        allow_partial: bool = False,
    ) -> SyncReadResult:
        """Read the same register from several motors in one bus transaction.

        Args:
            data_name: register name, e.g. ``"Present_Position"``.
            motor_ids: motor IDs to query, in any order.
            num_retry: extra attempts if the transaction fails.  Feetech buses
                occasionally return a corrupted status packet, especially while
                several motors are moving; upstream LeKiwi defaults to 2 retries
                for the same reason.  Retries are immediate (no sleep) so the
                steady-state cost is unchanged.
            allow_partial: if True, return whatever motors did answer and list
                the rest in ``missing_ids`` instead of raising.

        Returns:
            A :class:`SyncReadResult` with sign-decoded values keyed by motor ID
            and the timestamps needed to turn position deltas into velocities.
        """
        motor_ids = list(motor_ids)
        if not motor_ids:
            raise ValueError("motor_ids must not be empty")
        addr, length = self._address_of(data_name)

        last: Exception | None = None
        for attempt in range(1 + num_retry):
            try:
                result = self._sync_read_once(addr, length, motor_ids, attempt)
            except FeetechBusError as exc:
                last = exc
                continue
            if result.complete or (allow_partial and result.values):
                result.values = {
                    motor_id: self._decode(data_name, raw) for motor_id, raw in result.values.items()
                }
                return result
            last = FeetechBusError(f"no reply from motor ids {result.missing_ids}")
        raise FeetechBusError(
            f"sync_read '{data_name}' on ids {motor_ids} failed after {num_retry + 1} tries: {last}"
        ) from last

    def _sync_read_once(
        self, addr: int, length: int, motor_ids: list[int], attempt: int
    ) -> SyncReadResult:
        if self.protocol_end == 1:
            # SCS series has no SYNC_READ instruction; fall back to sequential
            # reads.  Slower and the samples are staggered, but functional.
            t_request = time.monotonic()
            values: dict[int, int] = {}
            missing: list[int] = []
            for motor_id in motor_ids:
                self._send(motor_id, INST_READ, [addr, length])
                try:
                    got_id, error, data = self._receive_status(length)
                except FeetechBusError:
                    missing.append(motor_id)
                    continue
                if got_id != motor_id or error != 0:
                    missing.append(motor_id)
                    continue
                values[motor_id] = self._unpack(data, length)
            return SyncReadResult(values, t_request, time.monotonic(), attempt, tuple(missing))

        params = [addr, length, *(mid & 0xFF for mid in motor_ids)]
        self._send(BROADCAST_ID, INST_SYNC_READ, params)
        t_request = time.monotonic()

        values = {}
        for _ in motor_ids:
            try:
                got_id, error, data = self._receive_status(length)
            except FeetechBusError:
                # Stop early: a missing/garbled reply desynchronises the rest of
                # the burst anyway, and the caller will retry the whole thing.
                break
            if error != 0 or got_id not in motor_ids:
                continue
            values[got_id] = self._unpack(data, length)
        t_reply = time.monotonic()
        missing = tuple(mid for mid in motor_ids if mid not in values)
        return SyncReadResult(values, t_request, t_reply, attempt, missing)

    def sync_write(self, data_name: str, id_to_value: dict[int, int]) -> None:
        """Write the same register on several motors at once (no acknowledgement).

        Feetech sync writes are fire-and-forget: no motor answers, so a lost
        packet is silently dropped.  That is the accepted trade-off upstream too
        (see ``SerialMotorsBus.sync_write``) because it keeps the control loop
        fast.  Callers that care should re-send at their control rate.
        """
        if not id_to_value:
            return
        addr, length = self._address_of(data_name)
        params: list[int] = [addr, length]
        for motor_id, value in id_to_value.items():
            params.append(motor_id & 0xFF)
            params.extend(self._pack(self._encode(data_name, value), length))
        self._send(BROADCAST_ID, INST_SYNC_WRITE, params)

    # -- convenience -------------------------------------------------------- #

    def read_present_positions(
        self, motor_ids: list[int] | tuple[int, ...], num_retry: int = 2
    ) -> SyncReadResult:
        """Sync-read ``Present_Position`` (encoder ticks, 0..4095 per turn)."""
        return self.sync_read("Present_Position", motor_ids, num_retry=num_retry)

    def read_present_velocities(
        self, motor_ids: list[int] | tuple[int, ...], num_retry: int = 2
    ) -> SyncReadResult:
        """Sync-read ``Present_Velocity`` (encoder ticks per second, signed)."""
        return self.sync_read("Present_Velocity", motor_ids, num_retry=num_retry)


# --------------------------------------------------------------------------- #
# Fake bus, for tests and for bringing the node up without hardware
# --------------------------------------------------------------------------- #


class FakeFeetechBus:
    """Drop-in stand-in for :class:`FeetechBus` that synthesises wheel motion.

    Used by the unit self-checks and by ``wheel_odometry_node.py --fake-bus`` so
    that the ROS plumbing (kinematics, SE(2) integration, TF, covariance) can be
    exercised on a laptop with no servos attached.

    ``wheel_ticks_per_s`` maps motor ID -> constant velocity in encoder ticks per
    second.  ``Present_Position`` is integrated from those velocities and wrapped
    into ``[0, resolution)`` so the wrap-around handling in the odometry node is
    actually exercised.
    """

    def __init__(
        self,
        motor_ids: list[int] | tuple[int, ...],
        wheel_ticks_per_s: dict[int, float] | None = None,
        resolution: int = STS3215_RESOLUTION,
        initial_ticks: dict[int, float] | None = None,
        clock=time.monotonic,
    ) -> None:
        self.motor_ids = list(motor_ids)
        self.wheel_ticks_per_s = dict(wheel_ticks_per_s or dict.fromkeys(self.motor_ids, 0.0))
        self.resolution = resolution
        self._clock = clock
        self._t0 = clock()
        self._initial = dict(initial_ticks or dict.fromkeys(self.motor_ids, 0.0))
        self.is_connected = False
        self.written: list[tuple[str, dict[int, int]]] = []

        # Register state, so that ``read`` can see what ``write`` did.  The
        # defaults are deliberately the pessimistic ones: a servo that has been
        # used before comes up with its EEPROM ``Lock`` set and, unless someone
        # changed it, ``Operating_Mode`` at 0 (position).  Starting the fake in
        # the convenient state instead would hide the exact bug this models.
        self.registers: dict[int, dict[str, int]] = {
            motor_id: {"Lock": 1, "Operating_Mode": 0, "Torque_Enable": 0} for motor_id in self.motor_ids
        }
        self.rejected_eeprom_writes: list[tuple[str, int, int]] = []

    def connect(self) -> None:
        self.is_connected = True
        self._t0 = self._clock()

    def disconnect(self) -> None:
        self.is_connected = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.disconnect()

    def set_wheel_ticks_per_s(self, wheel_ticks_per_s: dict[int, float]) -> None:
        self.wheel_ticks_per_s.update(wheel_ticks_per_s)

    def sync_read(
        self,
        data_name: str,
        motor_ids: list[int] | tuple[int, ...],
        num_retry: int = 2,
        allow_partial: bool = False,
    ) -> SyncReadResult:
        t_request = self._clock()
        elapsed = t_request - self._t0
        values: dict[int, int] = {}
        for motor_id in motor_ids:
            rate = self.wheel_ticks_per_s.get(motor_id, 0.0)
            if data_name == "Present_Position":
                ticks = self._initial.get(motor_id, 0.0) + rate * elapsed
                values[motor_id] = round(ticks) % self.resolution
            elif data_name == "Present_Velocity":
                # Real servos quantise their velocity report to whole ticks/s.
                values[motor_id] = round(rate)
            else:
                values[motor_id] = 0
        return SyncReadResult(values, t_request, self._clock())

    def read_present_positions(self, motor_ids, num_retry: int = 2) -> SyncReadResult:
        return self.sync_read("Present_Position", motor_ids, num_retry=num_retry)

    def read_present_velocities(self, motor_ids, num_retry: int = 2) -> SyncReadResult:
        return self.sync_read("Present_Velocity", motor_ids, num_retry=num_retry)

    def sync_write(self, data_name: str, id_to_value: dict[int, int]) -> None:
        self.written.append((data_name, dict(id_to_value)))
        for motor_id, value in id_to_value.items():
            self.registers.setdefault(motor_id, {})[data_name] = int(value)

    def write(self, data_name: str, motor_id: int, value: int, num_retry: int = 2) -> None:
        # Every attempt is logged, including the ones the servo ignores, so a
        # test can distinguish "was not attempted" from "was attempted and
        # dropped".
        self.written.append((data_name, {motor_id: int(value)}))
        regs = self.registers.setdefault(motor_id, {})

        addr, _length = STS_CONTROL_TABLE.get(data_name, (None, None))
        is_eeprom = addr is not None and addr < FIRST_SRAM_ADDRESS
        if is_eeprom and regs.get("Lock", 1) != 0:
            # Real STS3215 behaviour: a write to the EEPROM region while Lock is
            # set is acknowledged with error byte 0 and then discarded.  No
            # exception here, because raising would make the fake *easier* to
            # pass than the hardware.
            self.rejected_eeprom_writes.append((data_name, motor_id, int(value)))
            return

        regs[data_name] = int(value)

    def read(self, data_name: str, motor_id: int, num_retry: int = 2) -> int:
        if motor_id not in self.motor_ids:
            raise FeetechBusError(f"no motor with id {motor_id} on this fake bus")
        if data_name in ("Present_Position", "Present_Velocity"):
            return self.sync_read(data_name, [motor_id], num_retry=num_retry).values[motor_id]
        return self.registers.get(motor_id, {}).get(data_name, 0)

    def ping(self, motor_id: int, num_retry: int = 0) -> bool:
        return motor_id in self.motor_ids
