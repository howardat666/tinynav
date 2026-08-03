"""Self-checks for the LeKiwi wheel odometry stack.

Run directly (repo style, no pytest needed):

    python3 tests/test_wheel_odometry.py

Covers, in order of how much they would hurt if wrong:
  * omni kinematics forward/inverse round-trip
  * encoder wrap-around differencing across the 0/4095 seam
  * SE(2) integration on a straight line and a closed circle
  * Feetech packet framing, checksums and sign-magnitude decoding
  * the full ROS node driven by a fake bus (skipped if rclpy is unavailable)
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tinynav.platforms.feetech_bus import (  # noqa: E402
    BROADCAST_ID,
    INST_SYNC_READ,
    FakeFeetechBus,
    FeetechBus,
    FeetechBusError,
    FeetechChecksumError,
    checksum,
    decode_sign_magnitude,
    encode_sign_magnitude,
)
from tinynav.platforms.omni3_kinematics import (  # noqa: E402
    Omni3Kinematics,
    integrate_se2,
    wrap_angle,
    wrap_tick_delta,
    yaw_to_quaternion,
)

TICKS = 4096


def test_kinematics_roundtrip():
    """body -> wheel -> body must be exact to float precision, not just close."""
    rng = np.random.default_rng(20260803)
    configs = [
        Omni3Kinematics(),
        Omni3Kinematics(wheel_signs=(1.0, -1.0, -1.0)),
        Omni3Kinematics(wheel_radius=0.0483, base_radius=0.1312, wheel_signs=(-1.0, 1.0, -1.0)),
        Omni3Kinematics(wheel_radius=0.075, base_radius=0.22, ticks_per_rev=2048),
    ]
    worst_vel = 0.0
    worst_delta = 0.0
    for kin in configs:
        for _ in range(5000):
            v = rng.uniform(-2.0, 2.0, 3)
            back = kin.wheel_ticks_per_s_to_body(kin.body_to_wheel_ticks_per_s(*v))
            worst_vel = max(worst_vel, float(np.max(np.abs(back - v))))
            # the tick-delta map is the same linear map, so it round-trips too
            back_d = kin.wheel_tick_delta_to_body_delta(kin.body_to_wheel_ticks_per_s(*v))
            worst_delta = max(worst_delta, float(np.max(np.abs(back_d - v))))
    print(f"  velocity round-trip worst abs error : {worst_vel:.3e}")
    print(f"  tick-delta round-trip worst error   : {worst_delta:.3e}")
    assert worst_vel < 1e-6, worst_vel
    assert worst_delta < 1e-6, worst_delta

    # A pure spin must move no wheel differently from any other.
    kin = Omni3Kinematics()
    spin = kin.body_to_wheel_radps(0.0, 0.0, 1.0)
    print(f"  pure spin wheel speeds (rad/s)      : {np.round(spin, 6)}")
    assert np.allclose(spin, spin[0]), spin
    # ... and equals base_radius * omega / wheel_radius
    assert abs(spin[0] - kin.base_radius / kin.wheel_radius) < 1e-12

    # A pure forward push must produce zero net rotation.
    fwd = kin.wheel_radps_to_body(kin.body_to_wheel_radps(0.7, 0.0, 0.0))
    print(f"  forward 0.7 m/s -> body             : {np.round(fwd, 9)}")
    assert abs(fwd[0] - 0.7) < 1e-12 and abs(fwd[1]) < 1e-12 and abs(fwd[2]) < 1e-12

    # Degenerate geometry must be rejected rather than silently inverted.
    for bad in ({"mount_angles_deg": (0.0, 0.0, 0.0)}, {"wheel_radius": 0.0}, {"base_radius": -1.0}):
        try:
            Omni3Kinematics(**bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"expected ValueError for {bad}")
    print("  degenerate geometries rejected      : OK")


def test_wrap_tick_delta():
    """Wrap-around differencing across the 0/4095 seam, both directions."""
    cases = [
        # (prev, curr, expected)
        (0, 0, 0),
        (100, 150, 50),
        (150, 100, -50),
        (4090, 5, 11),  # forward across the seam
        (5, 4090, -11),  # backward across the seam
        (4095, 0, 1),
        (0, 4095, -1),
        # The half-revolution boundary: +2048 and -2048 are indistinguishable, so
        # the range [-2048, +2048) resolves both to -2048. Documented aliasing,
        # ~33x beyond the STS3215's top speed at 50 Hz.
        (2047, 4095, -2048),
        (0, 2048, -2048),
        (0, 2047, 2047),  # just inside the limit, still correct
        (0, 2049, -2047),  # just outside, aliases
    ]
    for prev, curr, expected in cases:
        got = wrap_tick_delta(curr, prev, TICKS)
        print(f"  prev={prev:5d} curr={curr:5d} -> {got:+6d} (expected {expected:+6d})")
        assert got == expected, (prev, curr, got, expected)

    # Property: a running sum of wrapped deltas must recover many full turns.
    rng = np.random.default_rng(7)
    for _ in range(200):
        step = int(rng.integers(1, 400))  # ticks per sample, well under 2048
        n = int(rng.integers(1, 500))
        pos = int(rng.integers(0, TICKS))
        total = 0
        for _ in range(n):
            nxt = (pos + step) % TICKS
            total += wrap_tick_delta(nxt, pos, TICKS)
            pos = nxt
        assert total == step * n, (step, n, total)
    print(f"  running-sum property over 200 random runs (up to {500 * 400} ticks): OK")

    # Non-4096 resolutions (SCS series is 1024).
    assert wrap_tick_delta(2, 1020, 1024) == 6
    assert wrap_tick_delta(1020, 2, 1024) == -6
    print("  1024-tick resolution seam           : OK")


def test_se2_integration():
    """Straight line and closed circle, plus the small-angle branch."""
    # Straight: 1.0 m/s forward for 2 s at 100 Hz, heading 30 deg.
    x, y, theta = 0.0, 0.0, np.radians(30.0)
    dt, steps, v = 0.01, 200, 1.0
    for _ in range(steps):
        x, y, theta = integrate_se2(x, y, theta, v * dt, 0.0, 0.0)
    expect = np.array([2.0 * np.cos(np.radians(30.0)), 2.0 * np.sin(np.radians(30.0))])
    err = float(np.hypot(x - expect[0], y - expect[1]))
    print(f"  straight 2 m @30deg  err={err:.3e} m, heading drift={np.degrees(theta - np.radians(30)):+.2e} deg")
    assert err < 1e-12 and abs(theta - np.radians(30.0)) < 1e-12

    # Circle: vx=0.4 m/s, omega=0.8 rad/s -> radius 0.5 m, period 2pi/0.8.
    vx, omega = 0.4, 0.8
    radius = vx / omega
    period = 2.0 * np.pi / omega
    n = 2000
    dt = period / n
    x, y, theta = 0.0, 0.0, 0.0
    radii = []
    for _ in range(n):
        x, y, theta = integrate_se2(x, y, theta, vx * dt, 0.0, omega * dt)
        radii.append(np.hypot(x - 0.0, y - radius))  # ICC is at (0, +radius)
    radii = np.array(radii)
    close_err = float(np.hypot(x, y))
    radius_err = float(np.max(np.abs(radii - radius)))
    print(f"  circle R={radius} m: max radius error={radius_err:.3e} m, loop closure error={close_err:.3e} m")
    print(f"  final heading after one full turn: {np.degrees(theta):+.3e} deg (wrapped)")
    assert radius_err < 1e-9, radius_err
    assert close_err < 1e-9, close_err

    # Exactness check: a single big step must land on the arc, not the chord.
    # Quarter circle of radius 1 from the origin heading +x: ends at (1, 1).
    x, y, theta = integrate_se2(0.0, 0.0, 0.0, np.pi / 2, 0.0, np.pi / 2)
    print(f"  single quarter-turn step -> ({x:.12f}, {y:.12f}), expected (1, 1)")
    assert abs(x - 1.0) < 1e-12 and abs(y - 1.0) < 1e-12

    # Small-angle branch must agree with the general branch at the crossover.
    for dtheta in (1e-9, 1.0000001e-9, 1e-8, 1e-7):
        a = integrate_se2(0.0, 0.0, 0.0, 0.01, 0.003, dtheta)
        b = integrate_se2(0.0, 0.0, 0.0, 0.01, 0.003, dtheta * (1 + 1e-12))
        assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) < 1e-15
    print("  small-angle branch continuity       : OK")

    assert abs(wrap_angle(np.pi + 0.1) - (-np.pi + 0.1)) < 1e-12
    qx, qy, qz, qw = yaw_to_quaternion(np.radians(90.0))
    print(f"  yaw 90deg -> quat ({qx:.4f}, {qy:.4f}, {qz:.6f}, {qw:.6f})")
    assert abs(qz - np.sqrt(0.5)) < 1e-12 and abs(qw - np.sqrt(0.5)) < 1e-12


def test_sign_magnitude():
    """Feetech reports position/velocity as sign-magnitude, not two's complement."""
    cases = [(0, 0), (100, 100), (-100, 0x8064), (4095, 4095), (-4095, 0x8FFF), (32767, 32767)]
    for value, encoded in cases:
        got = encode_sign_magnitude(value, 15)
        print(f"  {value:+7d} <-> 0x{got:04X} (expected 0x{encoded:04X})")
        assert got == encoded, (value, got, encoded)
        assert decode_sign_magnitude(encoded, 15) == value

    # This is the bug you get for free if you assume two's complement:
    raw = 0x8064  # a wheel spinning backwards at 100 ticks/s
    twos = raw - 0x10000
    print(f"  raw 0x8064: sign-magnitude={decode_sign_magnitude(raw, 15):+d}, "
          f"two's complement would give {twos:+d}")
    assert decode_sign_magnitude(raw, 15) == -100 and twos == -32668

    try:
        encode_sign_magnitude(40000, 15)
    except ValueError:
        print("  magnitude overflow rejected         : OK")
    else:
        raise AssertionError("expected ValueError on magnitude overflow")


def test_feetech_framing():
    """Packet construction, checksum, byte order and status parsing."""
    bus = FeetechBus("/dev/null")  # never connected; we only exercise framing

    # Checksum of a known sync-read request for ids 7,8,9 @ addr 56 len 2.
    params = [56, 2, 7, 8, 9]
    body = [BROADCAST_ID, len(params) + 2, INST_SYNC_READ, *params]
    chk = checksum(body)
    total = sum(body)
    print(f"  sync-read body {body} sum=0x{total:02X} checksum=0x{chk:02X}")
    assert chk == (~total) & 0xFF
    assert (total + chk) & 0xFF == 0xFF  # the classic self-check

    # STS/SMS (protocol 0) is little endian; SCS (protocol 1) is big endian.
    assert bus._split_word(0x1234) == [0x34, 0x12]
    assert bus._join_word(bytes([0x34, 0x12])) == 0x1234
    big = FeetechBus("/dev/null", protocol_end=1)
    assert big._split_word(0x1234) == [0x12, 0x34]
    assert big._join_word(bytes([0x12, 0x34])) == 0x1234
    print("  endianness: STS little / SCS big     : OK")

    # A well-formed status packet: id 7, no error, Present_Position = 0x0ABC.
    data = [0xBC, 0x0A]
    sbody = [7, len(data) + 2, 0x00, *data]
    packet = bytes([0xFF, 0xFF, *sbody, checksum(sbody)])
    motor_id, error, payload = FeetechBus._parse_status(packet)
    value = bus._unpack(payload, 2)
    print(f"  status packet {packet.hex(' ')} -> id={motor_id} err={error} value={value}")
    assert (motor_id, error, value) == (7, 0, 0x0ABC)

    # A corrupted checksum must raise, not return garbage.
    bad = bytearray(packet)
    bad[-1] ^= 0xFF
    try:
        FeetechBus._parse_status(bytes(bad))
    except FeetechChecksumError as exc:
        print(f"  corrupted checksum rejected         : {type(exc).__name__}")
    else:
        raise AssertionError("expected a checksum error")

    # An inconsistent length field must also raise.
    try:
        FeetechBus._parse_status(bytes([0xFF, 0xFF, 7, 99, 0, 0xBC, 0x0A, checksum([7, 99, 0, 0xBC, 0x0A])]))
    except FeetechChecksumError as exc:
        print(f"  bad length field rejected           : {type(exc).__name__}")
    else:
        raise AssertionError("expected a length error")

    # Unknown register names must fail loudly.
    try:
        bus._address_of("Nope")
    except FeetechBusError as exc:
        print(f"  unknown register rejected           : {type(exc).__name__}")
    else:
        raise AssertionError("expected an unknown-register error")


def test_fake_bus_wraps():
    """The fake bus must actually cross the encoder seam, or it tests nothing."""

    class Clock:
        def __init__(self):
            self.t = 1000.0

        def __call__(self):
            return self.t

    clk = Clock()
    ids = [7, 8, 9]
    bus = FakeFeetechBus(ids, dict.fromkeys(ids, 5000.0), TICKS, clock=clk)
    bus.connect()
    prev = {i: bus.sync_read("Present_Position", ids).values[i] for i in ids}
    total = dict.fromkeys(ids, 0)
    seams = 0
    for _ in range(100):
        clk.t += 0.02  # 100 ticks per sample at 5000 ticks/s
        curr = bus.sync_read("Present_Position", ids).values
        for i in ids:
            d = wrap_tick_delta(curr[i], prev[i], TICKS)
            if curr[i] < prev[i]:
                seams += 1
            total[i] += d
        prev = dict(curr)
    print(f"  fake bus crossed the seam {seams} times; accumulated {total[7]} ticks (expected ~10000)")
    assert seams >= 2, "fake bus never wrapped, the wrap test is vacuous"
    assert abs(total[7] - 10000) <= 2, total


def _run_node_with_fake_bus(vx, vy, omega, source, seconds=4.0, rate=50.0):
    """Drive WheelOdometryNode off a fake bus on a fake clock; return the pose track."""
    import rclpy

    from tinynav.core.wheel_odometry_node import WheelOdometryNode

    class Clock:
        def __init__(self):
            self.t = 5000.0

        def __call__(self):
            return self.t

    rclpy.init(
        args=[
            "--ros-args",
            "-p", "fake_bus:=true",
            "-p", f"velocity_source:={source}",
            "-p", f"publish_rate_hz:={rate}",
            "-p", "odom_topic:=/test/wheel_odometry",
            "-p", "publish_tf:=false",
        ]
    )
    try:
        node = WheelOdometryNode()
        node.timer.cancel()  # step manually instead of relying on the executor
        clk = Clock()
        ticks = node.kin.body_to_wheel_ticks_per_s(vx, vy, omega)
        node.bus = FakeFeetechBus(node.motor_ids, dict(zip(node.motor_ids, ticks)), node.ticks_per_rev, clock=clk)
        node.bus.connect()
        node._mono = clk

        dt = 1.0 / rate
        track = []
        for _ in range(int(seconds * rate)):
            clk.t += dt
            node._tick()
            track.append((node.x, node.y, node.theta))
        cov = node.pose_cov.copy()
        sys_cov = node._systematic_covariance()
        node.destroy_node()
        return np.array(track), cov, sys_cov
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_node_straight_and_circle():
    """End-to-end: synthetic wheel motion in, correct SE(2) trajectory out."""
    try:
        import rclpy  # noqa: F401
    except ImportError:
        print("  rclpy unavailable, skipping node integration test")
        return

    # --- straight line, position differencing --------------------------------
    vx, seconds = 0.30, 4.0
    track, cov, sys_cov = _run_node_with_fake_bus(vx, 0.0, 0.0, "position", seconds=seconds)
    x, y, theta = track[-1]
    # one sample is consumed establishing the baseline
    expect = vx * (seconds - 1.0 / 50.0)
    print(f"  straight/position: x={x:.6f} (expected {expect:.6f}), y={y:.3e}, yaw={np.degrees(theta):+.3e} deg")
    print(f"                     max |y| along the path = {np.max(np.abs(track[:, 1])):.3e} m")
    assert abs(x - expect) < 2e-3, (x, expect)
    assert np.max(np.abs(track[:, 1])) < 1e-3
    assert abs(np.degrees(theta)) < 0.05
    # Covariance: the random-walk part grows as sqrt(N), the systematic part
    # linearly with distance, and the systematic part must dominate.
    rx, ry, ryaw = np.sqrt(np.diag(cov))
    tx, ty, tyaw = np.sqrt(np.diag(cov + sys_cov))
    print(f"                     random-walk 1-sigma: x={rx * 100:.2f} cm, y={ry * 100:.2f} cm, "
          f"yaw={np.degrees(ryaw):.2f} deg")
    print(f"                     total       1-sigma: x={tx * 100:.2f} cm, y={ty * 100:.2f} cm, "
          f"yaw={np.degrees(tyaw):.2f} deg  (after {expect:.2f} m)")
    assert rx > 0.0 and ryaw > 0.0, "random-walk covariance never grew"
    assert tyaw > ryaw, "systematic term must add yaw uncertainty"
    # systematic yaw is rot_bias_per_m (0.05 rad/m) * path length, and must
    # dominate. Compare against the integrated path length, not the nominal
    # distance: they differ by the encoder quantisation residual.
    assert abs(np.sqrt(sys_cov[2, 2]) - 0.05 * x) < 1e-9, np.sqrt(sys_cov[2, 2])
    assert abs(np.sqrt(sys_cov[0, 0]) - 0.03 * x) < 1e-9, np.sqrt(sys_cov[0, 0])
    assert np.sqrt(sys_cov[2, 2]) > 2.0 * ryaw, "systematic yaw should dominate the random walk"

    # --- circle, position differencing --------------------------------------
    vx, omega = 0.30, 0.50
    radius = vx / omega
    period = 2.0 * np.pi / omega
    track, _, _ = _run_node_with_fake_bus(vx, 0.0, omega, "position", seconds=period, rate=100.0)
    r = np.hypot(track[:, 0] - 0.0, track[:, 1] - radius)
    print(f"  circle/position: R={radius} m, measured radius {r.mean():.6f} +/- {r.std():.2e}, "
          f"max deviation {np.max(np.abs(r - radius)):.3e} m")
    print(f"                   loop closure error {np.hypot(track[-1, 0], track[-1, 1]):.4f} m "
          f"after {2 * np.pi * radius:.3f} m of arc")
    assert np.max(np.abs(r - radius)) < 5e-3, np.max(np.abs(r - radius))
    assert np.hypot(track[-1, 0], track[-1, 1]) < 1e-2

    # --- same circle via reported velocity ----------------------------------
    track_v, _, _ = _run_node_with_fake_bus(vx, 0.0, omega, "velocity", seconds=period, rate=100.0)
    r_v = np.hypot(track_v[:, 0] - 0.0, track_v[:, 1] - radius)
    print(f"  circle/velocity: measured radius {r_v.mean():.6f}, "
          f"loop closure error {np.hypot(track_v[-1, 0], track_v[-1, 1]):.4f} m")
    print("                   (velocity readings are quantised to whole ticks/s, hence the larger error)")
    assert np.hypot(track_v[-1, 0], track_v[-1, 1]) < 0.1

    # --- pure lateral motion, which only an omni base can do ----------------
    track_y, _, _ = _run_node_with_fake_bus(0.0, 0.25, 0.0, "position", seconds=2.0)
    print(f"  strafe/position: x={track_y[-1, 0]:.3e}, y={track_y[-1, 1]:.6f} "
          f"(expected {0.25 * (2.0 - 0.02):.6f}), yaw={np.degrees(track_y[-1, 2]):+.3e} deg")
    assert abs(track_y[-1, 1] - 0.25 * (2.0 - 0.02)) < 2e-3
    assert abs(track_y[-1, 0]) < 1e-3


TESTS = [
    test_kinematics_roundtrip,
    test_wrap_tick_delta,
    test_se2_integration,
    test_sign_magnitude,
    test_feetech_framing,
    test_fake_bus_wraps,
    test_node_straight_and_circle,
]


if __name__ == "__main__":
    failures = 0
    for test in TESTS:
        print(f"\n=== {test.__name__} ===")
        try:
            test()
        except Exception as exc:  # noqa: BLE001 - summary runner: report every failure, not just the first
            failures += 1
            print(f"  FAILED: {type(exc).__name__}: {exc}")
            import traceback

            traceback.print_exc()
        else:
            print(f"  {test.__name__}: PASS")
    print(f"\n{len(TESTS) - failures}/{len(TESTS)} passed")
    sys.exit(1 if failures else 0)
