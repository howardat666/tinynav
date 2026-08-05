import argparse
import copy

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from tinynav.core.math_utils import np2msg, pose_msg2np


class LooperBridgeNode(Node):
    def __init__(self, args):
        super().__init__("looper_bridge_node")
        self.args = args
        self.bridge = CvBridge()

        self.cached_camera_info = None
        self.last_keyframe_pose = None
        self.last_keyframe_time = None
        self.last_pose = None
        self.last_pose_time = None
        self._missing_input_counter = 0
        self._last_sync_log_stamp = None

        self.sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.fast_pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.fast_depth_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fast_pose_group = MutuallyExclusiveCallbackGroup()
        self.fast_depth_group = MutuallyExclusiveCallbackGroup()
        self.sync_group = MutuallyExclusiveCallbackGroup()
        self.misc_group = MutuallyExclusiveCallbackGroup()

        self._exact_pose_prefixes = ("/camera/camera/vio",)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera/infra1/camera_info", self.camera_info_callback,
            self.sensor_qos, callback_group=self.misc_group
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage, "/tf_static", self.tf_callback, self.tf_static_qos,
            callback_group=self.misc_group
        )
        self.pose_visual_sub = self.create_subscription(
            PoseStamped, args.pose_topic, self.pose_visual_callback,
            self.fast_pose_qos, callback_group=self.fast_pose_group
        )
        self.depth_direct_sub = self.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw", self.depth_callback,
            self.fast_depth_qos, callback_group=self.fast_depth_group
        )

        # The 100 Hz pose stays unbridged; cmd_vel_control consumes it directly.
        self.depth_sub = message_filters.Subscriber(
            self, Image, "/camera/camera/depth/image_rect_raw",
            qos_profile=self.sensor_qos, callback_group=self.sync_group
        )
        self.pose_sub = message_filters.Subscriber(
            self, PoseStamped, args.pose_topic, callback_group=self.sync_group
        )
        self.image_sub = message_filters.Subscriber(
            self, Image, "/camera/camera/infra1/image_rect_raw",
            qos_profile=self.sensor_qos, callback_group=self.sync_group
        )
        # Exact vs approximate, and why this is not a style choice.
        #
        # message_filters.TimeSynchronizer matches on the exact (sec, nanosec)
        # pair. That works for the camera's own VIO because the firmware stamps
        # /camera/camera/vio_image with the *image's* timestamp -- hence the name --
        # so depth, infra1 and pose all carry byte-identical stamps; 58/58 depth
        # frames matched over a 12 s measurement against a live camera.
        #
        # A wheel-odometry pose has no such relationship: wheel_odometry_node
        # stamps with its own clock minus the measured bus read latency, so its
        # nanoseconds never coincide with an image's. Exact matching then yields
        # *zero* synced frames, and TimeSynchronizer reports nothing at all -- no
        # warning, no error, just permanent silence on /slam/keyframe_*. That is
        # the same silent-failure shape as the /insight/* topics that stopped
        # existing, which cost a debugging session to find.
        #
        # So `auto` uses exact only for a pose topic that is stamp-locked to the
        # images, approximate otherwise, and logs which branch it took.
        use_exact = self._pose_sync_is_exact()
        if use_exact:
            self.sync = message_filters.TimeSynchronizer(
                [self.depth_sub, self.pose_sub, self.image_sub], queue_size=20
            )
        else:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.depth_sub, self.pose_sub, self.image_sub],
                queue_size=20,
                slop=args.pose_sync_slop,
            )
        self.sync.registerCallback(self.sync_callback)

        self.odom_visual_pub = self.create_publisher(
            Odometry, "/slam/odometry_visual", 10
        )
        self.depth_pub = self.create_publisher(Image, "/slam/depth", 10)
        self.disparity_pub_vis = self.create_publisher(Image, "/slam/disparity_vis", 10)
        self.slam_camera_info_pub = self.create_publisher(CameraInfo, "/slam/camera_info", 10)
        # Off unless asked for: see --publish-camera-info-alias. Against a live
        # Looper this topic already has a publisher carrying the real stereo Tx,
        # and adding a second one that carries Tx = 0 silently zeroes everyone's
        # baseline about half the time.
        self.camera_info_alias_pub = (
            self.create_publisher(CameraInfo, "/camera/camera/infra2/camera_info", 10)
            if args.publish_camera_info_alias
            else None
        )
        self.keyframe_pose_visual_pub = self.create_publisher(
            Odometry, "/slam/keyframe_odom", 10
        )
        self.keyframe_image_pub = self.create_publisher(Image, "/slam/keyframe_image", 10)
        self.keyframe_depth_pub = self.create_publisher(Image, "/slam/keyframe_depth", 10)

        self.get_logger().info(
            f"Bridging {args.pose_topic} + /camera/camera/depth/image_rect_raw + "
            "/camera/camera/infra1/image_rect_raw into TinyNav /slam topics."
        )
        self.get_logger().info(
            f"keyframe sync: {'exact' if use_exact else f'approximate slop={args.pose_sync_slop}s'}"
        )

    def _pose_sync_is_exact(self) -> bool:
        """Whether the pose topic's stamps are byte-identical to the image stamps."""
        mode = self.args.pose_sync
        if mode == "exact":
            return True
        if mode == "approx":
            return False
        return self.args.pose_topic.startswith(self._exact_pose_prefixes)

    def camera_info_callback(self, msg: CameraInfo):
        self.cached_camera_info = msg
        self.get_logger().info(
            f"Received camera info from /camera/camera/infra1/camera_info with frame {msg.header.frame_id}.",
            once=True,
        )

    def tf_callback(self, msg: TFMessage):
        self.get_logger().info("Received TF_STATIC for Looper bridge.", once=True)

    def log_missing_inputs(self):
        self._missing_input_counter += 1
        if self._missing_input_counter % 30 != 1:
            return
        if self.cached_camera_info is None:
            self.get_logger().info("Waiting for Looper bridge inputs: /camera/camera/infra1/camera_info")

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def should_add_keyframe(self, T_world_camera: np.ndarray, stamp) -> bool:
        if self.last_keyframe_pose is None or self.last_keyframe_time is None:
            return True
        current_time = self.stamp_to_sec(stamp)
        translation = np.linalg.norm(
            T_world_camera[:3, 3] - self.last_keyframe_pose[:3, 3]
        )
        relative_rotation = self.last_keyframe_pose[:3, :3].T @ T_world_camera[:3, :3]
        rotation_angle = np.arccos(
            np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0)
        )
        return (
            translation >= self.args.keyframe_translation
            or rotation_angle >= np.deg2rad(self.args.keyframe_rotation_deg)
            or current_time - self.last_keyframe_time >= self.args.keyframe_static_interval
        )

    def make_odom_msg(self, T_world_camera: np.ndarray, stamp, velocity=None) -> Odometry:
        return np2msg(
            T_world_camera,
            stamp,
            "world",
            "camera",
            velocity=velocity,
        )

    def build_odom(self, T_world_camera: np.ndarray, stamp) -> Odometry:
        velocity = None
        current_time = stamp.sec + stamp.nanosec * 1e-9
        if self.last_pose is not None and self.last_pose_time is not None:
            dt = current_time - self.last_pose_time
            if dt > 1e-3:
                velocity = (T_world_camera[:3, 3] - self.last_pose[:3, 3]) / dt

        odom_msg = self.make_odom_msg(T_world_camera, stamp, velocity=velocity)
        self.last_pose = T_world_camera.copy()
        self.last_pose_time = current_time
        return odom_msg

    def pose_visual_callback(self, pose_msg: PoseStamped):
        T_world_camera = pose_msg2np(pose_msg)
        self.odom_visual_pub.publish(self.build_odom(T_world_camera, pose_msg.header.stamp))

    def depth_callback(self, depth_msg: Image):
        depth_m = self.decode_depth_meters(depth_msg)
        self.depth_pub.publish(self.build_depth_msg(depth_m, depth_msg.header.stamp))

    def decode_depth_meters(self, depth_msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth = np.asarray(depth)
        if depth_msg.encoding in ("mono16", "16UC1"):
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)
        return depth

    def build_depth_msg(self, depth_m: np.ndarray, stamp) -> Image:
        depth_out = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
        depth_out.header.stamp = stamp
        depth_out.header.frame_id = "camera"
        return depth_out

    def build_disparity_vis(self, depth_m: np.ndarray, stamp) -> Image:
        depth = np.asarray(depth_m, dtype=np.float32)

        valid = np.isfinite(depth) & (depth > 1e-3)
        disparity_u8 = np.zeros(depth.shape, dtype=np.uint8)
        if np.any(valid):
            inv_depth = np.zeros(depth.shape, dtype=np.float32)
            inv_depth[valid] = 1.0 / depth[valid]
            disp_min = float(np.min(inv_depth[valid]))
            disp_max = float(np.max(inv_depth[valid]))
            if disp_max > disp_min:
                disparity_u8[valid] = np.clip(
                    255.0 * (inv_depth[valid] - disp_min) / (disp_max - disp_min),
                    0.0,
                    255.0,
                ).astype(np.uint8)
            else:
                disparity_u8[valid] = 255

        disp_color = cv2.applyColorMap(disparity_u8, cv2.COLORMAP_PLASMA)
        disp_color[~valid] = 0
        disp_color_msg = self.bridge.cv2_to_imgmsg(disp_color, encoding="bgr8")
        disp_color_msg.header.stamp = stamp
        disp_color_msg.header.frame_id = "camera"
        return disp_color_msg

    def sync_callback(self, depth_msg: Image, pose_msg: PoseStamped, image_msg: Image):
        if self.cached_camera_info is None:
            self.log_missing_inputs()
            return

        T_world_camera = pose_msg2np(pose_msg)
        stamp = pose_msg.header.stamp

        odom_msg = self.make_odom_msg(T_world_camera, stamp)
        depth_m = self.decode_depth_meters(depth_msg)
        depth_out = self.build_depth_msg(depth_m, stamp)
        disparity_vis_msg = self.build_disparity_vis(depth_m, stamp)

        stamp_s = self.stamp_to_sec(stamp)
        if self._last_sync_log_stamp is None or stamp_s - self._last_sync_log_stamp >= 1.0:
            self._last_sync_log_stamp = stamp_s
            self.get_logger().info(
                "sync_callback: "
                f"t={stamp_s:.3f}, "
                f"depth={depth_m.shape}, image={image_msg.height}x{image_msg.width}"
            )

        image_out = copy.deepcopy(image_msg)
        image_out.header.stamp = stamp
        image_out.header.frame_id = "camera"

        camera_info_out = copy.deepcopy(self.cached_camera_info)
        camera_info_out.header.stamp = stamp
        camera_info_out.header.frame_id = "camera"

        self.disparity_pub_vis.publish(disparity_vis_msg)
        self.slam_camera_info_pub.publish(camera_info_out)
        if self.camera_info_alias_pub is not None:
            self.camera_info_alias_pub.publish(camera_info_out)

        if self.should_add_keyframe(T_world_camera, stamp):
            self.keyframe_pose_visual_pub.publish(odom_msg)
            self.keyframe_image_pub.publish(image_out)
            self.keyframe_depth_pub.publish(depth_out)
            self.last_keyframe_pose = T_world_camera.copy()
            self.last_keyframe_time = self.stamp_to_sec(stamp)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframe-translation", type=float, default=0.03)
    parser.add_argument("--keyframe-rotation-deg", type=float, default=1.0)
    parser.add_argument("--keyframe-static-interval", type=float, default=1.0)
    parser.add_argument(
        "--pose-topic",
        default="/camera/camera/vio_image",
        help="Camera pose input. The default is what current Looper firmware "
             "actually publishes (PoseStamped, measured 19.99 Hz). It used to be "
             "/insight/vio_20hz, which does not exist on the camera any more. "
             "Point this at a wheel-odometry pose topic to drive mapping from "
             "odometry instead of from VIO.",
    )
    parser.add_argument(
        "--pose-sync",
        choices=("auto", "exact", "approx"),
        default="auto",
        help="How the keyframe synchroniser matches the pose against depth and "
             "infra1. `auto` -- the default -- uses exact (sec, nanosec) matching "
             "for a /camera/camera/vio* topic, which the firmware stamps with the "
             "image's own timestamp, and approximate matching for anything else. "
             "Forcing `exact` on a pose from a different clock, such as wheel "
             "odometry, produces zero keyframes with no diagnostic whatsoever.",
    )
    parser.add_argument(
        "--pose-sync-slop",
        type=float,
        default=0.02,
        help="Maximum stamp difference for approximate matching, seconds. 0.02 is "
             "one wheel-odometry period at the default 50 Hz; at a 0.15 m/s "
             "mapping speed a 10 ms mismatch is 1.5 mm of position and 0.23 deg "
             "of yaw, both well under the 3 cm keyframe threshold.",
    )
    parser.add_argument(
        "--publish-camera-info-alias",
        action="store_true",
        help="Republish infra1's CameraInfo under the infra2 topic name. OFF by "
             "default, and it must stay off against a live Looper: the camera "
             "already publishes /camera/camera/infra2/camera_info carrying the "
             "stereo Tx (P[3] = -31.009, a 100.2 mm baseline) while infra1's P[3] "
             "is 0. Two publishers on one topic is last-writer-wins, so the alias "
             "intermittently zeroes the baseline that build_map_node hands to "
             "generate_occupancy_map() and that planning_node uses for its local "
             "map. Enable it only for a source that has no infra2 info at all.",
    )
    return parser.parse_args()


def main(args=None):
    rclpy.init(args=args)
    node = LooperBridgeNode(parse_args())
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
