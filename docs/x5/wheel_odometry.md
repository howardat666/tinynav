# LeKiwi wheel odometry

Wheel odometry for the LeKiwi three-wheel omnidirectional base, intended to
replace or supplement VIO on the D-Robotics X5 inside the Looper camera.

**TL;DR before you put this on the robot:** `wheel_radius` and `base_radius`
default to the upstream LeRobot values, which are *not* measurements of your
robot. Run `tool/wheel_odom_calibrate.py` first. An uncalibrated `base_radius`
produces a permanent yaw scale error that nothing in a dead-reckoning-only stack
will ever correct.

## Files

| File | Role |
| --- | --- |
| `tinynav/platforms/feetech_bus.py` | Minimal Feetech/SCS serial protocol driver. `pyserial` only. Sync read/write, retries, sampling timestamps. Includes `FakeFeetechBus` for hardware-free bring-up. |
| `tinynav/platforms/omni3_kinematics.py` | 3-wheel omni kinematics, encoder wrap-around differencing, exact SE(2) integration. `numpy` only. |
| `tinynav/core/wheel_odometry_node.py` | The ROS 2 node: read wheels, integrate, publish `nav_msgs/Odometry` + TF. |
| `tool/wheel_odom_calibrate.py` | Sign check, `wheel_radius` and `base_radius` calibration. |
| `tests/test_wheel_odometry.py` | Self-checks. `python3 tests/test_wheel_odometry.py`. |

### Why not just use LeRobot?

`tinynav/platforms/lekiwi_control.py` does `from lerobot.robots.lekiwi...`, and
`lerobot` depends on `torch`. The X5 has ~1.3 GB of writable rootfs, so that is
not installable. The kinematics were therefore lifted from
`lerobot/robots/lekiwi/lekiwi.py` (`_body_to_wheel_raw` / `_wheel_raw_to_body`,
Apache-2.0) and the register map from `lerobot/motors/feetech/tables.py`, and
re-expressed with no dependency beyond `numpy` and `pyserial`.

For the same reason this code does **not** import `tinynav/core/math_utils.py`:
that module pulls in numba, cv2 and fufpy, which is a lot of rootfs to spend on
building one quaternion. `yaw_to_quaternion` is four lines instead.

What upstream does not have, and this adds: pose integration, a ROS interface,
sampling timestamps, covariance, encoder-position differencing, and calibration.

## Kinematics

The three omniwheels sit at 120-degree intervals on a circle of radius
`base_radius`. Upstream mounts them at `[240, 0, 120]` degrees for
(left, back, right) with a -90 degree offset, giving rolling-direction angles
`[150, -90, 30]` degrees.

For a wheel rolling along direction `a`, at distance `base_radius` from the
centre, the contact-point speed along its rolling direction is

```
v_i = cos(a_i) * vx + sin(a_i) * vy + base_radius * omega
```

Stacking all three wheels:

```
        [ cos(a_i)  sin(a_i)  base_radius ]              [ -0.866   0.500   0.125 ]
v = M @ [vx, vy, omega]^T ,        M  =                  [  0.000  -1.000   0.125 ]
                                                         [  0.866   0.500   0.125 ]
```

`M` is square with `det(M) = 0.3248`, so the forward kinematics that odometry
needs is a plain inverse, not a least-squares fit:

```
                       [ -0.577   0.000   0.577 ]
[vx, vy, omega]^T =    [  0.333  -0.667   0.333 ]  @ v
                       [  2.667   2.667   2.667 ]
```

Three wheels, three DOF, exactly determined. That is convenient for the
arithmetic and terrible for error detection: **there is no redundancy, so wheel
slip is completely unobservable.** Any slip maps straight into a body velocity
error with nothing left over to notice it. A differential drive with a caster at
least has a consistency check between its two wheels; this base has none.

Note the yaw row of `M^-1` is `[1, 1, 1] / (3 * base_radius)`, i.e. yaw is
recovered from the *sum* of the three wheel speeds divided by three times the
lever arm. With `base_radius = 0.125 m` that coefficient is 2.667 rad/s of yaw
per m/s of wheel error.

### Integration

Body increments are composed onto the pose with the exact SE(2) exponential map,
not a Euler step:

```
V(w) = (1/w) * [[ sin(w),      -(1 - cos(w)) ],
                [ 1 - cos(w),   sin(w)       ]]

[x, y]  +=  R(theta) @ V(dtheta) @ [dx, dy]
theta   +=  dtheta
```

`V` is the arc-versus-chord correction; `V -> I` as `w -> 0`, so straight-line
motion falls out of the same expression. A Euler step systematically undershoots
turns, and since the bias has a consistent sign it accumulates -- exactly the
error wheel odometry can least afford. Cost is one extra `sin` and `cos` per
sample. Verified in the tests: a single 90-degree step lands on the arc endpoint
`(1, 1)` to 1e-12 rather than on the chord.

## Two sampling paths

`velocity_source` selects which register is read.

### `position` (default)

Sync-read `Present_Position` and difference it. The STS3215 encoder is 4096
ticks/rev, so:

- 1 tick = 0.0879 deg of wheel rotation
- 1 tick = 76.7 um of travel at the contact point (50 mm wheel)

Differencing an absolute position gives exactly the displacement that occurred.
Quantisation error does **not** accumulate: the running sum of deltas telescopes
back to `round(final) - round(initial)`, so it stays bounded at ~1 tick no matter
how far you drive.

### `velocity`

Sync-read `Present_Velocity`, which is what upstream LeKiwi does
(`lekiwi.py:347`). It is quantised to whole ticks/s and the servo derives it
internally over an unspecified window, so it is both coarse and lagged.
Integrating it feeds that bias directly into the pose. Kept for cross-checking;
not recommended for navigation.

Measured on the fake-bus circle test (1 full circle of radius 0.6 m, 3.77 m of
arc): loop closure error 4.9 mm via position differencing, and the velocity path
recovers a radius of 0.599842 m instead of 0.600000 m. On real hardware the gap
will be much wider than this synthetic comparison suggests, because the fake bus
models only quantisation, not the servo's internal filtering.

### Encoder wrap-around

`Present_Position` is a single-turn value in `[0, 4095]`, so a continuously
spinning wheel wraps about once per revolution. Naive differencing gives a -4095
spike at every wrap. The fix is the standard modular one, assuming the wheel took
the shorter path:

```python
delta = (curr - prev + 2048) % 4096 - 2048
```

| prev | curr | delta | note |
| --- | --- | --- | --- |
| 4090 | 5 | +11 | forward across the seam |
| 5 | 4090 | -11 | backward across the seam |
| 4095 | 0 | +1 | |
| 0 | 4095 | -1 | |
| 0 | 2047 | +2047 | just inside the limit |
| 0 | 2048 | -2048 | at the limit: aliases |
| 0 | 2049 | -2047 | past the limit: aliases |

**Aliasing limit:** motion faster than half a revolution per sample is
indistinguishable from motion the other way. At 50 Hz that ceiling is
2048 ticks / 0.02 s = 9000 deg/s, versus an STS3215 no-load top speed around
270 deg/s -- a ~33x margin, safe even at 5 Hz. The residual risk is a *stalled
loop*: if sampling pauses for more than ~0.6 s at full wheel speed the delta
aliases silently. The node therefore drops any interval longer than `max_dt`
(default 0.5 s) and re-baselines instead of integrating it.

## Topics and frames

Verified topology in this repo:

| Topic | Type | Published by | Consumed by |
| --- | --- | --- | --- |
| `/slam/odometry` | `nav_msgs/Odometry` | `perception_node.py` | `map_node.py`, `build_map_node.py`, `imu_propagator_node.py` |
| `/slam/keyframe_odom` | `nav_msgs/Odometry` | `perception_node.py`, `looper_bridge_node.py` | `map_node.py`, `build_map_node.py` |
| `/slam/odometry_visual` | `nav_msgs/Odometry` | `looper_bridge_node.py` | nothing in `tinynav/core` |
| `/insight/vio_20hz` | `geometry_msgs/PoseStamped` | Insight VIO (external) | `planning_node.py` |
| `/wheel/odometry` | `nav_msgs/Odometry` | **this node** | nothing yet |

Two things worth knowing before wiring this in:

- `map_node.py` consumes `/slam/odometry`, **not** `/slam/odometry_visual`. The
  `_visual` topic that `looper_bridge_node.py` publishes has no consumer inside
  `tinynav/core`.
- `planning_node.py` does not take `Odometry` at all for its pose input; it takes
  `PoseStamped` on `/insight/vio_20hz`. Feeding planning from wheel odometry
  would need a small `Odometry -> PoseStamped` republisher, which this node does
  not provide.

The default `odom_topic` is `/wheel/odometry` so that starting this node can
never silently fight the VIO. To drive mapping from wheels instead:

```bash
ros2 run ... wheel_odometry_node --ros-args -p odom_topic:=/slam/odometry
# and do NOT run perception_node's odometry at the same time
```

### Frames

The repo convention is `world` as the root with `camera` as the moving child
(`perception_node.py`, `math_utils.np2tf`). There is no `odom` or `base_link`
frame anywhere in the existing tree.

This node measures **base** motion, not camera motion, so it publishes
`world -> base_link` by default. If the existing `world -> camera` consumers
should see wheel odometry, you must publish the fixed base-to-camera extrinsic
yourself:

```bash
ros2 run tf2_ros static_transform_publisher X Y Z QX QY QZ QW base_link camera
```

The node deliberately does not guess that extrinsic. Publishing base motion
labelled as `camera` would be wrong by whatever the mounting offset is, and a
lever-arm error shows up as spurious translation during every turn.

Body axes follow REP-103: +x forward, +y left, +z up, `omega` positive
counter-clockwise from above. The `twist` field is in `child_frame_id` (the body
frame), which is what the kinematics natively produces.

## Parameters

All are ROS parameters (`--ros-args -p name:=value`). This differs from the
argparse convention used elsewhere in `tinynav/core`, because the topic name and
the geometry need to be overridable from a launcher without editing code.

### Hardware and geometry

| Parameter | Default | Notes |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | Feetech bus serial device |
| `baudrate` | `1000000` | Feetech factory default |
| `wheel_motor_ids` | `[7, 8, 9]` | left, back, right. Order matters. |
| `wheel_signs` | `[1.0, 1.0, 1.0]` | per-wheel +1/-1, absorbs reversed wiring |
| `wheel_radius` | `0.05` | **CALIBRATE.** Scales translation. |
| `base_radius` | `0.125` | **CALIBRATE.** Scales yaw. |
| `wheel_mount_angles_deg` | `[240, 0, 120]` | before the offset |
| `wheel_mount_offset_deg` | `-90.0` | |
| `ticks_per_rev` | `4096` | STS3215; SCS series is 1024 |

### Loop

| Parameter | Default | Notes |
| --- | --- | --- |
| `publish_rate_hz` | `50.0` | |
| `velocity_source` | `position` | or `velocity` |
| `num_read_retries` | `2` | matches upstream `num_read_retries` |
| `read_timeout_s` | `0.02` | ~80x the wire time of a 3-motor sync read |
| `max_dt` | `0.5` | longer gaps are dropped, not integrated |

### ROS interface

| Parameter | Default |
| --- | --- |
| `odom_topic` | `/wheel/odometry` |
| `odom_frame` | `world` |
| `base_frame` | `base_link` |
| `publish_tf` | `true` |
| `qos_depth` | `50` |

### Covariance

| Parameter | Default | Meaning |
| --- | --- | --- |
| `pos_quant_ticks` | `1.0` | tick quantisation, `position` mode |
| `vel_noise_ticks` | `8.0` | reported-velocity noise, `velocity` mode |
| `wheel_slip_frac` | `0.05` | slip as a fraction of wheel speed |
| `trans_scale_error` | `0.03` | systematic, fraction of distance |
| `rot_scale_error` | `0.05` | systematic, fraction of rotation |
| `rot_bias_per_m` | `0.05` | systematic yaw drift, rad per metre driven |
| `planar_variance` | `1e-4` | prior on the unmeasured z / roll / pitch |

### Optional wheel command

The Feetech bus is half duplex and a serial port has one owner, so this node
**cannot share `/dev/ttyACM0`** with `lekiwi_control.py` or a LeRobot
`lekiwi_host`. Both would transmit onto the same wire. Run one or the other.

If you want this node to be the sole bus owner, it can also consume `Twist` and
drive `Goal_Velocity` itself:

| Parameter | Default | Notes |
| --- | --- | --- |
| `enable_wheel_command` | `false` | sets Operating_Mode 1 + torque on at startup |
| `cmd_vel_topic` | `/lekiwi_control/cmd_vel` | what `lekiwi_control.py` publishes |
| `cmd_timeout_s` | `0.5` | watchdog: stop the wheels if commands go stale |
| `max_wheel_raw` | `3000` | ticks/s cap, scaled proportionally across wheels |

### Offline

| Parameter | Default | Notes |
| --- | --- | --- |
| `fake_bus` | `false` | synthesise wheel motion, no serial port needed |
| `fake_wheel_ticks_per_s` | `[0, 0, 0]` | per-wheel constant rate |

```bash
# bring the node up on a laptop with no hardware, driving a 0.6 m circle
# (vx = 0.3 m/s, omega = 0.5 rad/s)
ros2 run ... wheel_odometry_node --ros-args \
  -p fake_bus:=true -p 'fake_wheel_ticks_per_s:=[-2572.5, 814.9, 4202.2]'
```

### Service

`~/reset` (`std_srvs/Empty`) zeroes the pose, the covariance and the path-length
accumulators. Useful between calibration runs.

## Covariance model

Two error mechanisms that grow at *different rates*. Conflating them is the usual
way odometry covariance ends up wrong.

### 1. Random per-sample noise -> random walk

Propagated through the SE(2) Jacobians, `P <- Gx P Gx' + Gu Q Gu'`, so
`sigma ~ sqrt(N)`. `Q` comes from a per-wheel noise model with two parts:

- **reading noise**, white. In `position` mode this is `pos_quant_ticks / dt`
  ticks/s -- note it *shrinks* as `dt` grows, which is the real reason a slower
  loop dead-reckons more smoothly. In `velocity` mode it is a fixed
  `vel_noise_ticks`, much larger, because the servo's internal estimate is coarse
  and lagged.
- **slip noise**, proportional to that wheel's own speed (`wheel_slip_frac`).
  Omniwheel rollers skid under load; a faster wheel slips more.

These are projected into the body frame with `M^-1`, which makes the geometry do
the work instead of a fudge factor:

- yaw row of `M^-1` is `[1,1,1]/(3*base_radius)` = 2.667, RSS over three
  independent wheels = **4.62** rad/s per m/s of per-wheel error.
- a differential drive with a 0.5 m track has `yaw = (v_r - v_l)/track`, per-wheel
  coefficient 2.0, RSS = **2.83**.

So the geometry alone makes this base ~1.6x noisier in yaw than a diff drive.
The rest of the gap is slip, which here is continuous and unobservable.

The same projection makes lateral velocity noisier than longitudinal (rows of
`M^-1` have RSS 0.82 for vy vs 0.71 for vx, because the back wheel contributes to
vy alone) without any hand-tuning. Measured: at 50 Hz with 1-tick quantisation,
body sigma is vx 3.1 mm/s, vy 3.1 mm/s, omega 1.01 deg/s.

### 2. Systematic scale error -> linear in distance

A miscalibrated `wheel_radius` or `base_radius` is the *same* error every step,
not a fresh random draw, so it grows linearly with distance. White per-step noise
cannot represent this: at 50 Hz over a 20 m run, a random-walk model understates
it by more than 30x. It is therefore accumulated from path-length counters and
added at publish time:

```
sigma_trans = trans_scale_error * path_length
sigma_yaw   = rot_scale_error * abs_rotation + rot_bias_per_m * path_length
```

`rot_bias_per_m = 0.05` rad/m (2.9 deg/m) is roughly 5x what a well calibrated
differential drive would claim. Reported as a diagonal; the true error is
correlated (a yaw bias also displaces position), so this understates the
off-diagonals while staying conservative on the diagonal.

Measured after 1.19 m of straight driving at 50 Hz: random-walk 1-sigma is
x 0.31 cm / y 1.16 cm / yaw 0.84 deg, total 1-sigma is x 3.60 cm / y 3.76 cm /
yaw 3.52 deg. The systematic term dominates by ~4x, as it should.

**The total covariance grows without bound.** There is no loop closure and no
absolute reference here, so that is the honest answer. Whoever consumes it must
supply the correction.

### z / roll / pitch

Not measured, but the base is rigid on a floor, so `planar_variance = 1e-4`
(1 cm / 0.57 deg 1-sigma) is published as a planar prior rather than the
conventional 1e6 "unknown". A downstream filter is better served by being told
the robot is on the ground than by being told nothing.

## Calibration

Run in this order. Steps 2 and 3 both need step 1 to be correct first.

### 1. Signs and wheel order

```bash
python3 tool/wheel_odom_calibrate.py signs --port /dev/ttyACM0
```

Push the robot straight forward ~0.5 m. Expected for the stock layout, per m/s
of forward motion:

| wheel | id | ticks/s |
| --- | --- | --- |
| left | 7 | -11291.2 |
| back | 8 | 0.0 |
| right | 9 | +11291.2 |

The back wheel reading zero for pure forward motion is a good free sanity check:
its rolling direction is perpendicular to +x. The script flags a wrong sign, a
permuted wheel order (large lateral component) and a single flipped wheel (large
yaw) separately.

### 2. `wheel_radius`, from a straight run

```bash
python3 tool/wheel_odom_calibrate.py straight --port /dev/ttyACM0 --distance 2.0
```

Push the robot a tape-measured 2 m, press ENTER. Body displacement is exactly
proportional to `wheel_radius`, so the correction is a plain ratio.

Push by hand rather than using `--drive`: no torque means no slip, so you measure
geometry instead of geometry plus slip.

### 3. `base_radius`, from a spin

```bash
python3 tool/wheel_odom_calibrate.py spin --port /dev/ttyACM0 --turns 5 \
    --wheel-radius <value from step 2>
```

Rotate in place a whole number of turns. Yaw is proportional to
`wheel_radius / base_radius`, so with `wheel_radius` already fixed this is again
a plain ratio. Mark the floor and the chassis so you can hit the turn count.

Use as many turns as you can stand (5-10): the estimate improves linearly with
total rotation, and one turn does not separate the answer from the start/stop
transient.

### Also

- `monitor` streams live wheel readings and the implied body velocity.
- `--fake` runs the whole thing with no hardware and a virtual clock, and must
  recover `--fake-wheel-radius` / `--fake-base-radius`. Verified: it recovers
  0.048301 against a true 0.048300 (the 1e-6 residual is one encoder tick), and
  `base_radius` exactly.
- Calibrate on the floor you will navigate on. Carpet and vinyl give measurably
  different effective wheel radii on omniwheels.
- Repeat each run 3 times. If `wheel_radius` moves more than ~1% between runs,
  something mechanical is loose.

## Practical limits

**Top speed.** 1 m/s forward needs 11291 ticks/s per wheel. The STS3215 tops out
around 45 rpm at 12 V, i.e. ~3070 ticks/s, so **maximum forward speed is roughly
0.27 m/s**. A 1 rad/s spin needs only 1630 ticks/s and is comfortable. Plan
trajectories accordingly; `lekiwi_control.py` currently clips `linear.x` to
2.0 m/s, which is about 7x more than the base can do.

**Bus budget.** A 3-motor 2-byte sync read is ~240 us of wire time at 1 Mbaud, so
50 Hz uses well under 2% of the bus. The 50 Hz loop is limited by serial latency
and scheduling, not bandwidth.

## Known limitations

1. **Slip is unobservable.** Three wheels, three DOF, no redundancy. This is the
   dominant error source and there is no way to detect it from wheel data alone.
   Omniwheel rollers skid sideways continuously during any turn.
2. **No absolute reference, no loop closure.** Error accumulates monotonically.
   This is dead reckoning; it is a *complement* to VIO or a map, not a
   replacement, over any distance that matters.
3. **Yaw is the weak axis.** Short lever arm (0.125 m) plus continuous slip.
   Budget for yaw drift, and prefer to correct heading from another source.
4. **Position differencing vs reported velocity.** Position differencing is
   strictly better for odometry (no accumulated quantisation, no servo filter
   lag) and is the default. The `velocity` path exists for comparison and because
   it is what upstream does. Reported velocity's only real advantage is that a
   single dropped sample does not matter, whereas position differencing needs a
   baseline; the node handles that by re-baselining after a `max_dt` gap.
5. **Bus contention.** Cannot co-exist with `lekiwi_control.py` on the same
   serial port. See `enable_wheel_command`.
6. **No base-to-camera extrinsic.** Publishes `world -> base_link`; the existing
   consumers want `world -> camera`. Supply the static transform yourself.
7. **Yaw-only pose.** A planar SE(2) model. Ramps and thresholds are not
   represented; `z`, `roll` and `pitch` are always zero with a planar prior.
8. **Timestamps are estimated, not hardware.** The servos have no timestamping,
   so the sample instant is taken as the midpoint of the request/reply pair. The
   residual skew between the first and last wheel in a sync read is under a
   millisecond, two orders of magnitude below a 20 ms control period.

## Verification without hardware

```bash
python3 tests/test_wheel_odometry.py
```

Covers: kinematics round-trip (worst error 3.0e-15 over 20000 random twists and
4 geometries), wrap-around differencing including a running-sum property test up
to 200000 ticks, SE(2) integration (2 m straight to 5.2e-15 m, closed circle to
1.7e-13 m), sign-magnitude coding including the two's-complement trap, packet
framing and checksum rejection, and the full node driven by a fake bus on a
virtual clock (straight line, circle, pure strafe, covariance growth).
