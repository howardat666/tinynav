#!/usr/bin/env python3
"""把一段深度序列 + 位姿 + 内参存成一个 npz，好在 PC 上离线复现障碍图。

板上每换一组障碍参数要重启 planning、等 30 s，一轮 A/B 六组就是四五分钟，还得赌板子
不重启。存下来之后 `replay_obstacle_map.py` 在 PC 上几秒扫完整个参数空间，而且**同一份
数据**可以反复比，不受场景漂移干扰。

    # 板上（车停稳，别动）
    python3 tool/x5_board/capture_depth_scene.py --out /userdata/x5/scene_empty.npz --frames 40
    # PC 上
    scp root@<板子>:/userdata/x5/scene_empty.npz .

⚠️ 必须存**序列**不是单帧：占据栅格是多帧累积 + 0.99/周期衰减（约 16 s 半衰），
单帧复现不出真实的 z 层结构 —— 假障碍正是累积出来的。

🔴 **而且要够长**：第一次只存了 40 帧 / 7.8 s，比半衰期还短，离线复现出来的障碍图
是 0 个近处格子，而板上同一配置是 70 个 —— **复现不出来，结论就不能用**。
默认 250 帧 ≈ 50 s（约 3 个半衰期）才追得上板上的稳态。
拿到数据先用 `replay_obstacle_map.py` 跑一遍现役配置，对得上板上的数再往下扫。
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image


def pose_to_T(msg: PoseStamped) -> np.ndarray:
    """和 planning 的 pose_msg2np 逐字一致 —— 差一点这份数据就复现不了。"""
    T = np.eye(4)
    o, p = msg.pose.orientation, msg.pose.position
    T[:3, :3] = R.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    T[:3, 3] = (p.x, p.y, p.z)
    return T


class Capture(Node):
    def __init__(self, args):
        super().__init__("capture_depth_scene")
        self.args = args
        self.bridge = CvBridge()
        self.K = None
        self.poses: list[tuple[float, np.ndarray]] = []
        self.frames: list[tuple[float, np.ndarray, np.ndarray]] = []
        self.infra = None
        self.create_subscription(CameraInfo, args.info_topic, self._on_info, 1)
        self.create_subscription(PoseStamped, args.pose_topic, self._on_pose, 50)
        self.create_subscription(Image, args.depth_topic, self._on_depth, 5)
        self.create_subscription(Image, args.infra_topic, self._on_infra, 1)

    def _on_info(self, m):
        self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)

    def _on_pose(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
        self.poses.append((t, pose_to_T(m)))
        del self.poses[:-400]

    def _on_infra(self, m):
        if self.infra is None:
            self.infra = np.asarray(
                self.bridge.imgmsg_to_cv2(m, desired_encoding="passthrough")).copy()

    def _on_depth(self, m):
        if self.K is None or not self.poses or len(self.frames) >= self.args.frames:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
        # 近似同步，和 planning 一样的 slop
        best = min(self.poses, key=lambda p: abs(p[0] - t))
        if abs(best[0] - t) > self.args.slop:
            return
        if m.encoding in ("mono16", "16UC1"):
            d = np.asarray(self.bridge.imgmsg_to_cv2(
                m, desired_encoding="passthrough")).astype(np.uint16)
        else:
            d = (np.asarray(self.bridge.imgmsg_to_cv2(m, desired_encoding="32FC1"))
                 * 1000.0).astype(np.uint16)
        self.frames.append((t, d.copy(), best[1]))
        if len(self.frames) % 10 == 0:
            print(f"  已存 {len(self.frames)}/{self.args.frames} 帧", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=250)
    ap.add_argument("--depth-topic", default="/slam/depth")
    ap.add_argument("--pose-topic", default="/wheel/camera_pose")
    ap.add_argument("--info-topic", default="/camera/camera/infra1/camera_info")
    ap.add_argument("--infra-topic", default="/camera/camera/infra1/image_rect_raw")
    ap.add_argument("--slop", type=float, default=0.06)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    rclpy.init()
    n = Capture(args)
    t0 = time.time()
    while rclpy.ok() and len(n.frames) < args.frames and time.time() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.frames:
        print(f"一帧都没存到 (K={n.K is not None}, 位姿 {len(n.poses)} 条) —— "
              f"检查话题名和 slop")
        n.destroy_node(); rclpy.shutdown(); return

    stamps = np.array([f[0] for f in n.frames])
    depths = np.stack([f[1] for f in n.frames])
    poses = np.stack([f[2] for f in n.frames])
    np.savez_compressed(
        args.out, stamps=stamps, depth_mm=depths, poses=poses, K=n.K,
        infra1=(n.infra if n.infra is not None else np.zeros((1, 1), np.uint8)),
        note=np.array(args.note), depth_topic=np.array(args.depth_topic),
        pose_topic=np.array(args.pose_topic))
    dt = np.diff(stamps)
    print(f"存好 {args.out}")
    print(f"  {len(n.frames)} 帧 {depths.shape[1]}x{depths.shape[2]}  "
          f"跨时 {stamps[-1]-stamps[0]:.1f}s  帧间隔 p50={np.median(dt) if len(dt) else 0:.3f}s")
    print(f"  位姿平移范围 {np.ptp(poses[:, :3, 3], axis=0)} m（车该是不动的）")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
