#!/usr/bin/env python3
"""Why does the bridge's keyframe sync drop a third of the depth frames? Runs ON THE BOARD.

    python3 tool/x5_board/sync_loss_probe.py --seconds 30

Separates the two candidates, which need different fixes: STAMP INFEASIBLE (no partner
within `slop`, so a bigger queue changes nothing) from QUEUE EVICTION (the partner
existed but was pushed out). Caveat -- it observes from a lightly loaded process, and the
bridge it reasons about is at 101% CPU, which is how it once supported the wrong answer.
"""

from __future__ import annotations

import argparse
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

DEPTH = '/camera/camera/depth/image_rect_raw'
INFRA1 = '/camera/camera/infra1/image_rect_raw'
POSE = '/wheel/camera_pose'


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Probe(Node):
    def __init__(self, slop: float):
        super().__init__('sync_loss_probe')
        self.slop = slop
        # Deep enough that the probe's own history never limits the answer -- the
        # point is to measure what the bridge's queue of 3 misses, so this one must
        # not miss anything itself.
        self.infra = deque(maxlen=400)   # (stamp, arrival)
        self.pose = deque(maxlen=400)
        self.rows = []
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, INFRA1, self._on_infra, qos)
        self.create_subscription(PoseStamped, POSE, self._on_pose, qos)
        self.create_subscription(Image, DEPTH, self._on_depth, qos)

    def _on_infra(self, msg):
        self.infra.append((stamp_s(msg), time.time()))

    def _on_pose(self, msg):
        self.pose.append((stamp_s(msg), time.time()))

    def _on_depth(self, msg):
        now = time.time()
        ds = stamp_s(msg)
        if not self.infra or not self.pose:
            return
        bi = min(self.infra, key=lambda e: abs(e[0] - ds))
        bp = min(self.pose, key=lambda e: abs(e[0] - ds))
        # How many newer infra1 frames arrived after the matching one? That is the
        # per-input queue depth this depth frame needed to still find its partner.
        need_infra = sum(1 for s, a in self.infra if a > bi[1])
        need_pose = sum(1 for s, a in self.pose if a > bp[1])
        self.rows.append({
            'd_infra': abs(bi[0] - ds), 'd_pose': abs(bp[0] - ds),
            'arrival_lag': now - bi[1],
            'need': max(need_infra, need_pose),
            'depth_own_lag': now - ds,
        })


def pct(v, q):
    v = sorted(v)
    return v[min(int(q * len(v)), len(v) - 1)] if v else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--slop', type=float, default=0.06, help="the bridge's --pose-sync-slop")
    ap.add_argument('--queue-size', type=int, default=3, help="the bridge's --sync-queue-size")
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.slop)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    rows = node.rows
    node.destroy_node()
    rclpy.shutdown()

    if not rows:
        print('no depth frames seen -- is the camera publishing?')
        return 1

    n = len(rows)
    print(f'\n{n} depth frames in {args.seconds:.0f}s = {n / args.seconds:.2f} Hz')
    print(f"bridge settings assumed: slop={args.slop}s queue_size={args.queue_size}\n")

    stamp_bad = [r for r in rows if r['d_infra'] > args.slop or r['d_pose'] > args.slop]
    queue_bad = [r for r in rows if r not in stamp_bad and r['need'] >= args.queue_size]
    print(f"  STAMP INFEASIBLE : {len(stamp_bad):4d} / {n}  ({100 * len(stamp_bad) / n:5.1f}%)"
          f"   no partner within slop")
    print(f"  QUEUE EVICTION   : {len(queue_bad):4d} / {n}  ({100 * len(queue_bad) / n:5.1f}%)"
          f"   partner existed but needed queue_size >= {args.queue_size}")
    print(f"  SHOULD MATCH     : {n - len(stamp_bad) - len(queue_bad):4d} / {n}  "
          f"({100 * (n - len(stamp_bad) - len(queue_bad)) / n:5.1f}%)")

    print(f"\n  stamp distance to nearest infra1 : p50 {1000 * pct([r['d_infra'] for r in rows], .5):6.1f}ms"
          f"  p95 {1000 * pct([r['d_infra'] for r in rows], .95):6.1f}ms")
    print(f"  stamp distance to nearest pose   : p50 {1000 * pct([r['d_pose'] for r in rows], .5):6.1f}ms"
          f"  p95 {1000 * pct([r['d_pose'] for r in rows], .95):6.1f}ms")
    print(f"  depth arrival lag behind its own stamp : p50 "
          f"{1000 * pct([r['depth_own_lag'] for r in rows], .5):6.1f}ms"
          f"  p95 {1000 * pct([r['depth_own_lag'] for r in rows], .95):6.1f}ms")
    print(f"  newer partners arrived meanwhile      : p50 "
          f"{pct([r['need'] for r in rows], .5):4.0f}"
          f"  p95 {pct([r['need'] for r in rows], .95):4.0f}"
          f"  max {max(r['need'] for r in rows):4d}")

    print("\n  queue_size needed to keep this fraction of depth frames:")
    for q in (3, 5, 8, 10, 15, 20, 30):
        keep = sum(1 for r in rows if r['need'] < q
                   and r['d_infra'] <= args.slop and r['d_pose'] <= args.slop)
        print(f"    {q:3d} -> {100 * keep / n:5.1f}%")
    print("\n  Raising queue_size costs latency, not memory: a synchroniser that falls "
          "behind\n  emits its oldest matched set, and map_node discards keyframes over "
          "0.5 s old. At\n  20 Hz, 10 slots is a 0.5 s window -- the same bound.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
