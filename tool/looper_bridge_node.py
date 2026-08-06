import argparse
import copy
import os
import time

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

# Same switch and same default as planning_node's, so one environment variable turns on
# the whole pipeline's stage timing. None means the timing is not emitted at all rather
# than emitted and filtered: node_manager._launch_proc redirects stdout into an
# unrotated file on the board's eMMC, and this callback runs at up to 5 Hz.
_TIMER_LOGGER = print if os.environ.get('TINYNAV_VERBOSE_TIMER', '0') == '1' else None


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
        # rclpy's own subscription queue sits *underneath* message_filters and delivers
        # strictly FIFO, so --sync-queue-size does not bound it: a depth-50 reader hands
        # the synchroniser 50 backlogged frames oldest-first the moment the sync thread
        # stalls, and the keyframe that comes out is seconds old again. Capping the
        # matcher alone fixed only half of the measured 1.49 s keyframe age; this is the
        # other half.
        #
        # Sized as a duration, not a slot count. Slots are not comparable across these
        # streams -- depth runs at ~5 Hz on this board while infra1 and the pose run at
        # ~20 Hz -- so giving all three the same depth would silently shrink the fast
        # streams' matching window to a quarter of depth's and cost matches that the
        # exact-stamp synchroniser needs.
        #
        # The window itself is a live-vs-offline decision, and getting it wrong costs
        # opposite things in the two modes -- so it is a flag, not a constant, mirroring
        # the split --sync-queue-size already makes for the matcher above it.
        #
        # Live, a deep reader queue hands the synchroniser backlogged frames oldest-first
        # and the keyframe that comes out is seconds old; 1 s is what fixed the measured
        # 1.49 s keyframe age. Offline the bag is paced and latency is meaningless, but
        # the bridge runs ~3.4x slower than realtime on this board (measured: 240 s of
        # wall clock for the 71 s dryrun_probe bag), so BagPlayer keeps handing it depth
        # frames faster than it drains them. A 1 s depth window is 5 slots, and every
        # frame that overflows it is a keyframe missing from the map -- a silent hole in
        # coverage, not an error. 10 s offline costs ~34 MB of reader history for the
        # duration of a build and nothing at all during navigation.
        #
        # NOT a measured regression: the 224-keyframe build this was first blamed for was
        # a contaminated run whose bridge was fed 912 s of live camera instead of the bag.
        self.sync_depth_qos = QoSProfile(
            depth=max(1, round(args.sync_window_s * 5.0)), reliability=ReliabilityPolicy.RELIABLE
        )
        self.sync_fast_qos = QoSProfile(
            depth=max(1, round(args.sync_window_s * 20.0)), reliability=ReliabilityPolicy.RELIABLE
        )
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
            qos_profile=self.sync_depth_qos, callback_group=self.sync_group
        )
        self.pose_sub = message_filters.Subscriber(
            self, PoseStamped, args.pose_topic,
            qos_profile=self.sync_fast_qos, callback_group=self.sync_group
        )
        self.image_sub = message_filters.Subscriber(
            self, Image, "/camera/camera/infra1/image_rect_raw",
            qos_profile=self.sync_fast_qos, callback_group=self.sync_group
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
                [self.depth_sub, self.pose_sub, self.image_sub],
                queue_size=args.sync_queue_size,
            )
        else:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.depth_sub, self.pose_sub, self.image_sub],
                queue_size=args.sync_queue_size,
                slop=args.pose_sync_slop,
            )
        self.sync.registerCallback(self.sync_callback)

        self.odom_visual_pub = self.create_publisher(
            Odometry, "/slam/odometry_visual", 10
        )
        self.depth_pub = self.create_publisher(Image, "/slam/depth", 10)
        self.disparity_pub_vis = (
            self.create_publisher(Image, "/slam/disparity_vis", 10)
            if args.publish_disparity_vis
            else None
        )
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

        # Timed, and split by whether this set became a keyframe, because that split is
        # the measurement. Before the keyframe-only restructuring below, every set paid
        # the keyframe path's cost, so the gap between the two buckets is exactly what
        # stopped being spent on the other four sets in five -- an internal comparison,
        # immune to the run-to-run spread that made process CPU% useless here (three
        # 60 s windows on identical code measured 103.6%, 108.3% and 107.6%).
        #
        # Emitted in planning_node's codetiming format on purpose, so
        # tool/x5_board/planning_timer_stats.py aggregates it with no changes.
        t_cb = time.perf_counter()

        T_world_camera = pose_msg2np(pose_msg)
        stamp = pose_msg.header.stamp

        stamp_s = self.stamp_to_sec(stamp)
        if self._last_sync_log_stamp is None or stamp_s - self._last_sync_log_stamp >= 1.0:
            self._last_sync_log_stamp = stamp_s
            self.get_logger().info(
                "sync_callback: "
                f"t={stamp_s:.3f}, "
                f"depth={depth_msg.height}x{depth_msg.width}, image={image_msg.height}x{image_msg.width}"
            )

        # This callback fires on every synced set, up to 5 Hz, but keyframes land at
        # roughly 1 Hz -- so four of every five sets used to pay for work nobody read: a
        # depth decode over 544x640, a 1.39 MB float32 Image built from it, a deepcopy of
        # the infra1 image and an Odometry, all constructed unconditionally above and then
        # used only inside the keyframe branch at the bottom. The latency of this callback
        # is what decides whether map_node accepts the keyframe at all, so the waste was
        # not merely CPU.
        #
        # should_add_keyframe is a pure predicate -- it reads last_keyframe_pose/time and
        # the caller does the mutating -- so hoisting the call is safe, and it has to be
        # hoisted for anything below to become conditional on it.
        is_keyframe = self.should_add_keyframe(T_world_camera, stamp)

        # Decoded at most once per callback, and only when something will read it.
        depth_m = None
        if is_keyframe or self.disparity_pub_vis is not None:
            depth_m = self.decode_depth_meters(depth_msg)

        camera_info_out = copy.deepcopy(self.cached_camera_info)
        camera_info_out.header.stamp = stamp
        camera_info_out.header.frame_id = "camera"

        # Built and published only if asked for. It runs on every synced set rather
        # than every keyframe, and it is pure rviz decoration -- the only reference
        # to /slam/disparity_vis anywhere is docs/vis.rviz. The work is a masked
        # reciprocal over 544x640, a colour map, and serialising ~1 MB into DDS at
        # up to 5 Hz, all of it inside the callback whose latency decides whether
        # map_node accepts the keyframe at all.
        if self.disparity_pub_vis is not None:
            self.disparity_pub_vis.publish(self.build_disparity_vis(depth_m, stamp))
        self.slam_camera_info_pub.publish(camera_info_out)
        if self.camera_info_alias_pub is not None:
            self.camera_info_alias_pub.publish(camera_info_out)

        if is_keyframe:
            image_out = copy.deepcopy(image_msg)
            image_out.header.stamp = stamp
            image_out.header.frame_id = "camera"
            self.keyframe_pose_visual_pub.publish(self.make_odom_msg(T_world_camera, stamp))
            self.keyframe_image_pub.publish(image_out)
            self.keyframe_depth_pub.publish(self.build_depth_msg(depth_m, stamp))
            self.last_keyframe_pose = T_world_camera.copy()
            self.last_keyframe_time = self.stamp_to_sec(stamp)

        if _TIMER_LOGGER is not None:
            name = "sync:keyframe" if is_keyframe else "sync:plain"
            _TIMER_LOGGER(
                f"[{name}] Elapsed time: {(time.perf_counter() - t_cb) * 1000.0:.0f} ms"
            )


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
        default=0.06,
        help="Maximum stamp difference for approximate matching, seconds. Sized "
             "from the pose rate actually observed, not the configured one: the "
             "LeKiwi's servo bus loses reads, and /wheel/camera_pose arrived at "
             "28 Hz against a configured 50 Hz in a measured recording, in bursts "
             "rather than evenly. 0.02 -- one nominal period -- silently drops "
             "every depth frame whose nearest pose fell in a burst, and a dropped "
             "keyframe looks exactly like a keyframe that was never worth keeping. "
             "0.06 covers a two-period gap. The cost is bounded: at a 0.15 m/s "
             "mapping speed 60 ms is 9 mm of position and 1.4 deg of yaw, still "
             "under the 3 cm keyframe threshold. Ignored on the VIO path, which is "
             "matched by exact stamp.",
    )
    parser.add_argument(
        "--publish-disparity-vis",
        action="store_true",
        help="Publish the colour-mapped inverse-depth image on /slam/disparity_vis. "
             "Off by default because nothing in the stack subscribes to it -- the only "
             "reference is docs/vis.rviz -- and it is not cheap: a masked reciprocal "
             "over 544x640, an OpenCV colour map, and ~1 MB serialised into DDS, on "
             "every synced set rather than every keyframe. Turn it on when driving "
             "rviz by hand, and expect the keyframe latency to rise.",
    )
    parser.add_argument(
        "--sync-queue-size",
        type=int,
        default=3,
        help="Depth of the keyframe synchroniser queue. This is a latency knob, not "
             "a throughput one, and the right value depends on whether the output is "
             "consumed live. A deep queue turns a CPU shortfall into unbounded delay: "
             "message_filters delivers matched sets in order, so a node that falls "
             "behind keeps emitting the *oldest* set it still holds. Measured on the "
             "X5 with queue_size 20 and the board at load 14: /camera/camera/vio_image "
             "arrived 40 ms old, and /slam/keyframe_image left this node 1.49 s old. "
             "map_node drops keyframes older than max_keyframe_age_s (0.5 s), so every "
             "single one was discarded and relocalization was never attempted once -- a "
             "total failure that looked like a relocalization accuracy problem. "
             "3 bounds the added delay to about two depth periods. Use a deep queue "
             "only for an offline map build, where the bag is paced, latency is "
             "meaningless, and dropping a matched set loses a keyframe for good.",
    )
    parser.add_argument(
        "--sync-window-s",
        type=float,
        default=1.0,
        help="Seconds of history the rclpy reader queues underneath the synchroniser "
             "hold, converted to slots per stream from its own rate (depth ~5 Hz, "
             "infra1 and the pose ~20 Hz) so the fast streams do not silently get a "
             "quarter of depth's matching window. Same live-vs-offline split as "
             "--sync-queue-size and for the same reason, one layer down: 1 s live, so a "
             "stalled sync thread cannot serve up seconds-old frames, and 10 s for an "
             "offline map build, where the bridge runs ~3.4x slower than realtime and a "
             "depth frame dropped by an overflowing reader queue is a keyframe missing "
             "from the map with nothing logged.",
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
