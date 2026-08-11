#!/usr/bin/env python3
"""What does the obstacle z band actually cost, and is the floor in it? Runs ON THE BOARD.

    python3 tool/x5_board/obstacle_band_probe.py --seconds 25

Recomputes build_obstacle_map's band-and-z-span test off /planning/occupied_voxels and
sweeps (bottom, top, min_wall_span_m). Read the sweep instead of assuming monotonicity --
both directions have a trap. See docs/x5/nav_obstacle_tuning.md section 4.
"""

from __future__ import annotations

import argparse
import time
from collections import Counter, defaultdict

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

RES = 0.1
# The local grid is rolled to stay centred on the robot in all three axes, so z_rel
# always lands on this lattice and a band edge effectively snaps to it.
LAYER_Z_REL = [-0.45 + k * RES for k in range(10)]


class Probe(Node):
    def __init__(self):
        super().__init__('obstacle_band_probe')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.robot_z = None
        self.frames = []
        self.mask_counts = []
        self.create_subscription(Odometry, '/slam/odometry_visual', self._on_odom, qos)
        self.create_subscription(PointCloud2, '/planning/occupied_voxels', self._on_cloud, qos)
        self.create_subscription(OccupancyGrid, '/planning/obstacle_mask', self._on_mask, qos)

    def _on_odom(self, msg):
        self.robot_z = msg.pose.pose.position.z

    def _on_mask(self, msg):
        self.mask_counts.append(int(np.count_nonzero(np.asarray(msg.data, dtype=np.int8) > 0)))

    def _on_cloud(self, msg):
        if self.robot_z is None:
            return
        pts = np.asarray(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if len(pts) == 0:
            self.frames.append((self.robot_z, np.zeros((0, 3))))
            return
        if pts.dtype.names:
            pts = np.stack([pts['x'], pts['y'], pts['z']], axis=-1)
        self.frames.append((self.robot_z, pts.astype(np.float64)))


def columns_per_frame(frames):
    """Per frame, the occupied layer indices of every (x, y) column, plus a histogram.

    The cloud emits the cell's low corner while build_obstacle_map uses its centre, so
    the z_fix below is load-bearing: without it every band edge reads one layer wrong.
    """
    z_fix = 0.5 * RES
    hist = Counter()
    out = []
    for robot_z, pts in frames:
        if len(pts) == 0:
            out.append(None)
            continue
        layer = np.rint((pts[:, 2] - robot_z + z_fix - LAYER_Z_REL[0]) / RES).astype(int)
        xi = np.rint(pts[:, 0] / RES).astype(int)
        yi = np.rint(pts[:, 1] / RES).astype(int)
        cols = defaultdict(list)
        for a, b, k in zip(xi, yi, layer):
            hist[k] += 1
            cols[(a, b)].append(k)
        out.append(cols)
    return hist, out


def obstacle_cells(cols, bottom, top, span_m):
    """build_obstacle_map's verdict, recomputed for one (band, span) candidate."""
    n = 0
    for layers in cols.values():
        in_band = [k for k in layers
                   if bottom - 1e-9 <= LAYER_Z_REL[0] + k * RES <= top + 1e-9]
        if in_band and (max(in_band) - min(in_band)) * RES >= span_m - 1e-9:
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=25.0)
    ap.add_argument('--camera-height', type=float, default=0.18,
                    help='camera above the floor, for the world-z column only')
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    frames = [f for f in node.frames if f[1] is not None]
    mask_counts = node.mask_counts
    node.destroy_node()
    rclpy.shutdown()

    if not frames:
        print('no occupied_voxels or odometry_visual -- is planning_node running?')
        return 1
    hist, per_frame = columns_per_frame(frames)
    live = [c for c in per_frame if c]
    if not live:
        print('every frame was empty -- the camera sees nothing to raycast onto')
        return 1

    h = args.camera_height
    print(f'{len(frames)} cloud frames, {len(mask_counts)} mask frames, '
          f'robot_z={frames[-1][0]:+.3f} m')
    if mask_counts:
        print(f"planning's own obstacle-cell count: p50={int(np.median(mask_counts))} "
              f"min={min(mask_counts)} max={max(mask_counts)}")

    print(f'\noccupied voxels per z layer (z_rel is relative to the camera; '
          f'the floor is at {-h:+.2f})')
    print(f'{"layer":>5s} {"z_rel":>7s} {"world z band":>16s} {"voxels/frame":>13s}')
    for k in sorted(hist):
        zr = LAYER_Z_REL[0] + k * RES
        print(f'{k:5d} {zr:+7.2f} {f"[{zr + h - RES / 2:+.2f},{zr + h + RES / 2:+.2f}]":>16s} '
              f'{hist[k] / len(live):13.0f}')

    # min_wall_span_m only changes behaviour at multiples of the layer height, so
    # anything between 0.1 and 0.2 is 0.1. Sweeping the in-between values once makes
    # that visible instead of inviting a false precision.
    spans = (0.0, 0.1, 0.2, 0.3)
    cases = [
        ('GO2 defaults', -0.40, 0.40),
        ('LeKiwi shipped', -0.20, 0.20),
        ('swept height only', -0.20, 0.10),
        ('floor layer excluded', -0.10, 0.20),
        ('floor excluded, low', -0.10, 0.10),
    ]
    print('\nobstacle cells per frame, by (band, min_wall_span_m):')
    print(f'{"case":<22s}{"band":>15s}{"layers":>7s}'
          + ''.join(f'{f"span={s:.1f}":>10s}' for s in spans))
    for name, bottom, top in cases:
        n_layers = sum(1 for z in LAYER_Z_REL if bottom - 1e-9 <= z <= top + 1e-9)
        row = ''.join(f'{np.mean([obstacle_cells(c, bottom, top, s) for c in live]):10.1f}'
                      for s in spans)
        print(f'{name:<22s}{f"[{bottom:+.2f},{top:+.2f}]":>15s}{n_layers:7d}{row}')

    shipped = np.mean([obstacle_cells(c, -0.20, 0.20, 0.2) for c in live])
    if mask_counts and shipped > 0:
        print(f'\ndilation: {shipped:.1f} cells undilated -> '
              f'{int(np.median(mask_counts))} reported = x{np.median(mask_counts) / shipped:.1f}')
    print('\nHow to read it: compare the floor-excluded rows against the ones that keep '
          'it.\nA small gap means min_wall_span_m is already rejecting the floor and the '
          'band\nbottom buys nothing. A column that collapses toward zero means that '
          'band is too\nnarrow for that span, and the robot would see no obstacles at '
          'all.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
