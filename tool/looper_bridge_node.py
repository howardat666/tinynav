import argparse
import collections
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
        # stamp_ns -> the depth Image with that stamp, newest few only. Depth now reaches
        # the keyframe path by lookup rather than by being a sync input; 12 entries is
        # ~2.8 s at the board's 4.3 Hz, far longer than the sub-100 ms the two paths can
        # drift apart, and 12 x 680 kB is 8 MB held at worst.
        self._depth_by_stamp = collections.OrderedDict()
        self._depth_cache_max = 12
        self._depth_lookup_miss = 0
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
        # VIO 原点重置会毒化 map->odom：约束跨了两个坐标系，解出来的 yaw 会阶跃几十度。
        # 判据用 z 不用速度 —— 触发场景是原地快转，线速度本来就小。
        self._vio_prev = None
        self._vio_resets = 0
        self._vio_gaps = 0
        self._vio_dz_m = float(os.environ.get("TINYNAV_VIO_RESET_DZ_M", "0.15"))

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
        # Pose + infra1 only. main syncs depth in as well, and on its target that is free:
        # all three inputs run at the camera's 20 Hz. The X5 sets depth_frame_skip=4 in the
        # firmware for thermal reasons (0.86 core, 89% BPU and 10 C, see
        # docs/x5/depth_frame_skip.md), which silently made depth the rate limiter of a
        # three-way exact-stamp sync -- keyframes could never exceed 4.3 Hz, and depth was
        # a third input that could starve the match. Measured 2026-08-24: keyframes at
        # 0.161 Hz, sync_callback idle for up to 39.6 s at a stretch, 61% of the run.
        #
        # Depth was only ever feeding /slam/keyframe_depth (measured Subscription count: 0)
        # and the optional /slam/disparity_vis. What map_node relocalizes on is
        # /slam/keyframe_image, which comes from infra1, with its pose -- both 20 Hz. So
        # depth leaves the sync and is looked up by stamp from the single subscription that
        # already receives it for /slam/depth. That also ends the duplicate subscription:
        # the same 680 kB frame was being delivered and deserialized twice per frame.
        #
        # ...except when /slam/keyframe_depth actually has a consumer. The stamp lookup
        # below cannot serve one: depth runs at 4.3 Hz against pose+infra1's 20 Hz and
        # arrives on a different callback group, so a keyframe's stamp is usually not in
        # the cache yet -- measured 168 misses out of 168 keyframes on 2026-08-25, which
        # left build_map_node's four-way sync one input short and produced a map with zero
        # poses. A synchronizer waits for the input; a cache lookup cannot wait. So for a
        # map build depth goes back in as a sync input, and only navigation gets the
        # uncapped keyframe rate that taking it out was for.
        self._depth_in_sync = args.keyframe_depth == 'always'
        if self._depth_in_sync:
            self.depth_sub = message_filters.Subscriber(
                self, Image, "/camera/camera/depth/image_rect_raw",
                qos_profile=self.sync_depth_qos, callback_group=self.sync_group
            )
            sync_inputs = [self.depth_sub, self.pose_sub, self.image_sub]
        else:
            self.depth_sub = None
            sync_inputs = [self.pose_sub, self.image_sub]

        use_exact = self._pose_sync_is_exact()
        if use_exact:
            self.sync = message_filters.TimeSynchronizer(
                sync_inputs, queue_size=args.sync_queue_size,
            )
        else:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                sync_inputs, queue_size=args.sync_queue_size,
                slop=args.pose_sync_slop,
            )
        if self._depth_in_sync:
            self.sync.registerCallback(
                lambda d, p_, i: self.sync_callback(p_, i, depth_msg=d))
        else:
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
        # /slam/keyframe_depth has no navigation-time consumer: map_node deliberately does
        # not subscribe (its keyframe_callback takes image and odom only) and build_map_node
        # is not running during a drive. Producing it anyway costs a mono16 -> float32
        # conversion over 544x640 plus a 1.39 MB message, per keyframe, in the callback
        # whose latency decides whether map_node accepts the keyframe at all.
        #
        # 'always' for a map build and 'auto' for navigation, rather than the subscription
        # count alone: node_manager launches this node *before* build_map_node, so during
        # the discovery window the count is legitimately 0 while the bag is already
        # playing, and every keyframe published without its depth partner is one that
        # build_map_node's exact-stamp synchroniser can never match -- a silent hole at the
        # start of every map. The count still gates 'auto', so attaching rviz mid-drive
        # works.
        self.keyframe_depth_mode = args.keyframe_depth

        self.get_logger().info(
            f"Bridging {args.pose_topic} + /camera/camera/depth/image_rect_raw + "
            "/camera/camera/infra1/image_rect_raw into TinyNav /slam topics."
        )
        self.get_logger().info(
            f"keyframe sync: {'exact' if use_exact else f'approximate slop={args.pose_sync_slop}s'}"
            f" on {'depth+pose+infra1' if self._depth_in_sync else 'pose+infra1 (depth by stamp lookup)'}; "
            f"gate: >={args.keyframe_translation}m or >={args.keyframe_rotation_deg}deg or "
            f">={args.keyframe_static_interval}s, capped at one per {args.keyframe_min_interval}s"
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
        elapsed = current_time - self.last_keyframe_time
        # 上限先判：门限满足了也不能比这更快。见 --keyframe-min-interval。
        if elapsed < self.args.keyframe_min_interval:
            return False
        return (
            translation >= self.args.keyframe_translation
            or rotation_angle >= np.deg2rad(self.args.keyframe_rotation_deg)
            or elapsed >= self.args.keyframe_static_interval
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
        self._check_vio_continuity(T_world_camera, pose_msg.header.stamp)
        self.odom_visual_pub.publish(self.build_odom(T_world_camera, pose_msg.header.stamp))

    def _check_vio_continuity(self, T, stamp):
        """检测在这里做，因为本回调已经收到每一帧 —— 单开一个 Python 订阅者在这块板上
        实测要 15% 一个核（rclpy 没有零拷贝）。"""
        t_ns = stamp.sec * 10**9 + stamp.nanosec
        z = float(T[2, 3])
        if self._vio_prev is not None:
            pt, pz = self._vio_prev
            dt = (t_ns - pt) / 1e9
            # z 恰好归零是实测到的重置签名，但固件开着 use_zupt（零速更新），有可能
            # 自己把 z 吸到 0 —— 所以它只报 WARNING，够门限的真跳变才报 ERROR。
            # 只有相邻两帧（没丢帧）才能判跳变。本节点的位姿 QoS 是 depth=1 而执行器只有
            # 2 线程，实测会丢到 150~400 ms 的洞 —— 跨洞采样平滑运动也像跳变。
            contiguous = dt < 0.12
            big = contiguous and abs(z - pz) > self._vio_dz_m
            snapped = contiguous and abs(z) < 1e-9 and abs(pz) > 0.05
            if big or snapped:
                self._vio_resets += 1
                where = (f"z={pz:+.3f}->{z:+.3f} pos=[{T[0, 3]:+.2f},{T[1, 3]:+.2f},{z:+.2f}] "
                         f"yaw={np.degrees(np.arctan2(T[1, 0], T[0, 0])):+.1f}deg dt={dt:.3f}s")
                if big:
                    self.get_logger().error(f"vio origin reset #{self._vio_resets}: {where}")
                else:
                    self.get_logger().warning(
                        f"vio z snapped to zero #{self._vio_resets}: {where}")
            # 固件的 rotation_prior_max_interval=0.15：超过它就丢 IMU 旋转先验，
            # 而快速转头最依赖那个先验
            if dt > 0.15:
                self._vio_gaps += 1
                self.get_logger().warning(
                    f"vio frame gap {dt * 1000:.0f}ms (#{self._vio_gaps})")
        self._vio_prev = (t_ns, z)

    def depth_callback(self, depth_msg: Image):
        # Forwarded in the camera's own mono16 millimetres, not converted to 32FC1
        # metres. The conversion doubled the message -- 696 kB becomes 1.39 MB -- and
        # `ros2 topic bw` measured /slam/depth at 4.2 MB/s against 3.0 MB/s for the
        # camera's own topic, for 5 Hz of the entire navigation session. It also cost
        # a decode plus a rebuild per frame (2.6 + 2.6 ms measured on the X5).
        #
        # Nobody wanted the float32: planning_node immediately divides to metres
        # itself, which is one line there instead of a doubled topic here. DDR
        # bandwidth is not free on this board either -- it is one of the remaining
        # suspects for the servo bus loss (see docs/x5/servo_bus.md), which app load
        # modulates by ~3x while pure CPU load does not.
        #
        # Consumers handle both encodings: planning_node and planning_bag_viser
        # branch on msg.encoding, and perception_node still publishes 32FC1 here in
        # RealSense mode, so this topic was never single-encoding to begin with.
        self.depth_pub.publish(self.relabel_depth(depth_msg))
        key = depth_msg.header.stamp.sec * 1_000_000_000 + depth_msg.header.stamp.nanosec
        self._depth_by_stamp[key] = depth_msg
        while len(self._depth_by_stamp) > self._depth_cache_max:
            self._depth_by_stamp.popitem(last=False)

    def relabel_depth(self, depth_msg: Image) -> Image:
        """Re-header the camera depth for /slam/depth, sharing the payload.

        A new message rather than mutating the input, which message_filters also
        holds. `.data` is shared, not copied: rebuilding this way measured 0.13 ms
        against 0.70 ms for copy.deepcopy.
        """
        out = Image()
        out.header.stamp = depth_msg.header.stamp
        out.header.frame_id = "camera"
        out.height = depth_msg.height
        out.width = depth_msg.width
        out.encoding = depth_msg.encoding
        out.is_bigendian = depth_msg.is_bigendian
        out.step = depth_msg.step
        out.data = depth_msg.data
        return out

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

    def sync_callback(self, pose_msg: PoseStamped, image_msg: Image, depth_msg: Image = None):
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
            # depth is no longer a sync input, so it cannot be reported here. depth_cache
            # is what says whether the lookup below will find anything, which is the thing
            # worth knowing now.
            self.get_logger().info(
                "sync_callback: "
                f"t={stamp_s:.3f}, "
                f"image={image_msg.height}x{image_msg.width}, "
                + (f"depth={depth_msg.height}x{depth_msg.width}, " if depth_msg is not None
                   else f"depth_cache={len(self._depth_by_stamp)}, ")
                + f"depth_misses={self._depth_lookup_miss}"
            )

        # This callback fires on every synced set -- up to the camera's 20 Hz now that depth
        # is not a sync input, where it used to be capped at depth's 4.3 -- but keyframes
        # still land at roughly 1 Hz, so most sets pay for work nobody reads: a
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

        # Checked before the decode, not just before the publish: the float32 conversion is
        # the expensive half. See keyframe_depth_mode for why 'auto' is not the default.
        want_keyframe_depth = is_keyframe and (
            self.keyframe_depth_mode == 'always'
            or self.keyframe_depth_pub.get_subscription_count() > 0
        )

        # Decoded at most once per callback, and only when something will read it. Depth
        # comes from the cache now: the pose/image pair runs at 20 Hz and depth at 4.3, so
        # most callbacks have no depth for their stamp -- that is expected, and the two
        # consumers below are the only things that care.
        depth_m = None
        if want_keyframe_depth or self.disparity_pub_vis is not None:
            if depth_msg is None:
                depth_msg = self._depth_by_stamp.get(
                    stamp.sec * 1_000_000_000 + stamp.nanosec)
            if depth_msg is not None:
                depth_m = self.decode_depth_meters(depth_msg)
            else:
                self._depth_lookup_miss += 1
                self.get_logger().info(
                    f"no depth for keyframe stamp (total {self._depth_lookup_miss}); "
                    "keyframe_depth and disparity_vis skipped for this one",
                    throttle_duration_sec=10.0,
                )

        camera_info_out = copy.deepcopy(self.cached_camera_info)
        camera_info_out.header.stamp = stamp
        camera_info_out.header.frame_id = "camera"

        # Built and published only if asked for. It runs on every synced set rather
        # than every keyframe, and it is pure rviz decoration -- the only reference
        # to /slam/disparity_vis anywhere is docs/vis.rviz. The work is a masked
        # reciprocal over 544x640, a colour map, and serialising ~1 MB into DDS at
        # up to 5 Hz, all of it inside the callback whose latency decides whether
        # map_node accepts the keyframe at all.
        if self.disparity_pub_vis is not None and depth_m is not None:
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
            if want_keyframe_depth and depth_m is not None:
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
    # 显式的关键帧上限，x5 专有。main 的门限（3 cm / 1 度）在它的目标机上没问题：depth 也是
    # 20 Hz，关键帧最多 20 Hz，而 Jetson 追得上。X5 上 map_node 每个关键帧实测 477 ms，
    # 0.6 m/s 下 3 cm 门限就是 20 Hz = 954% 一个核。
    #
    # 以前这个上限是"意外"存在的：depth 被固件降到 4.3 Hz，又是三路精确同步的一员，所以
    # 顺带把关键帧压在 4.3 Hz。把 depth 移出同步之后那个意外上限没了，必须换成显式的。
    # 按 map_node 的每帧成本定，不是拍脑袋：vlad 实测 876 ms/帧（bow 是 477）。1.0 s 的间隔
    # 就是 87.6% 利用率 —— 排队一抖就撞上 map_node 的 max_keyframe_age_s=1.0，关键帧被当成
    # 超龄丢掉（判据：日志里的 "dropping stale keyframes: N so far"）。1.5 s 留到 58%。
    # 路径重算因此是约 1.5 s，仍然比现在实测的 6~17 s 好一个量级。
    # 换回 bow 或者 map_node 变快之后可以往下调。
    parser.add_argument("--keyframe-min-interval", type=float, default=1.5)
    parser.add_argument(
        "--keyframe-depth", choices=("always", "auto"), default="always",
        help="Whether to produce /slam/keyframe_depth. 'auto' skips it -- decode "
             "included -- while nothing is subscribed, which during navigation is "
             "always. Defaults to 'always' so a hand-launched map build cannot lose "
             "depth to a discovery race; see keyframe_depth_mode.",
    )
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
    # 4 个回调组（pose / depth / sync / misc）抢 2 个线程时，位姿回调等不到调度，
    # 而它的队列只有 1 条 -> 等待期间到达的位姿被覆盖。实测丢掉 25.7%%，突发空白 2.15 s。
    # 工作量没变，变的只是调度。
    executor = MultiThreadedExecutor(num_threads=4)
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
