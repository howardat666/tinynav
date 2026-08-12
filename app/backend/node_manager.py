"""
BackendNode — extends Ros2NodeManager with extra subscriptions for pose and
mapping progress, plus a NodeRunner that spins it in a background thread.
"""
from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import traceback

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import base64

import rclpy
import rclpy.time
import tf2_ros
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import CompressedImage, Image, PointCloud, PointCloud2
from std_msgs.msg import Bool, Float32, String

from tinynav.core.robot_config import robot_config
from tool.ros2_node_manager import Ros2NodeManager

_DEFAULT_TINYNAV_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_TINYNAV_ROOT = os.environ.get('TINYNAV_ROOT', _DEFAULT_TINYNAV_ROOT)
_LOCAL_PREFIX = os.environ.get('LOCAL_PREFIX', '/userdata/local')
_DEVICE_VENV = os.environ.get('TINYNAV_VENV', '/userdata/junlinp/venv')
_DEFAULT_LOG_DIR = os.environ.get('TINYNAV_LOG_DIR', '/userdata/junlinp/logs')
# Fallback only. tool/x5_board/env.sh exports NUMBA_CACHE_DIR and its value wins; this
# exists so a backend started by a launcher that does not source env.sh still gets a
# warm JIT cache. Matches env.sh's path so the two share one cache rather than each
# paying the cold compile.
_NUMBA_CACHE_DIR = os.environ.get('TINYNAV_NUMBA_CACHE_DIR', '/userdata/x5/cache/numba')
_REALSENSE_SCRIPT = os.path.join(_TINYNAV_ROOT, 'scripts', 'run_realsense_sensor.sh')
_VENV_SITE = os.path.join(_TINYNAV_ROOT, '.venv', 'lib', 'python3.10', 'site-packages')
_MAP_BUILD_DOMAIN_LOOPER = '231'  # isolated domain to avoid live looper topic collision during map build
class _SensorModeDecided(Exception):
    """Control-flow marker: the sensor mode was set explicitly, skip probing.

    Raised rather than returned so the shared preview-topic initialisation at the
    end of _detect_and_init_sensor still runs exactly once, in one place.
    """


_ROS2_NODE_LIST_TIMEOUT = float(os.environ.get('TINYNAV_ROS2_NODE_LIST_TIMEOUT', '8'))
_ROS2_NODE_LIST_RETRIES = int(os.environ.get('TINYNAV_ROS2_NODE_LIST_RETRIES', '3'))

# build_map_node.py emits "MAPPING_PERCENT:<float>" lines on stdout so the
# parent process can track progress without a separate bridge subprocess.
_MAPPING_PERCENT_PREFIX = 'MAPPING_PERCENT:'

# ---------------------------------------------------------------------------- #
# Platform, actuator and odometry source
# ---------------------------------------------------------------------------- #
# Environment variables rather than HTTP settings because they decide which
# processes exist, and every one of them has to be fixed before the first node
# starts. Set them in the unit file / launch wrapper, one config per run.
#
#   TINYNAV_ROBOT_TYPE    go2 | b2 | lekiwi   robot geometry (see robot_config.py)
#   TINYNAV_ACTUATOR      unitree | wheel | none   what consumes /cmd_vel
#   TINYNAV_ODOM_SOURCE   vio | wheel         what the pose consumers read
#
# TINYNAV_ACTUATOR is a separate knob from TINYNAV_ODOM_SOURCE on purpose: the
# LeKiwi comparison runs need wheel *actuation* with VIO *odometry*, which is the
# combination that isolates the odometry change from everything else.
_ROBOT_TYPE = os.environ.get('TINYNAV_ROBOT_TYPE', 'go2')
# Safe to import here: robot_config deliberately depends on nothing but dataclasses and
# numpy, so this does not pull the planning stack into the backend.
_ROBOT = robot_config(_ROBOT_TYPE)
_ACTUATOR = os.environ.get('TINYNAV_ACTUATOR', 'unitree')
_ODOM_SOURCE = os.environ.get('TINYNAV_ODOM_SOURCE', 'vio')

# Which odometry the *offline map build* replays out of the bag, independent of
# what navigation then runs on. Separate because the interesting comparison needs
# all three combinations, and a single knob can only express two of them:
#
#   MAP_ODOM_SOURCE=vio    ODOM_SOURCE=vio      VIO map, VIO navigation (baseline)
#   MAP_ODOM_SOURCE=vio    ODOM_SOURCE=wheel    VIO map, odometry navigation
#   MAP_ODOM_SOURCE=wheel  ODOM_SOURCE=wheel    odometry throughout
#
# One recording serves all three, as long as it carries both pose topics: the
# build takes its pose from a topic in the bag, so only this variable changes.
_MAP_ODOM_SOURCE = os.environ.get('TINYNAV_MAP_ODOM_SOURCE', _ODOM_SOURCE)

# cmd_vel_control does the closed-loop path following and publishes /cmd_vel.
# unitree_control does its own, so it stays off for a Unitree base; a wheel base
# has no equivalent and needs it. Still overridable for bring-up.
_ENABLE_CMD_VEL_NODE = os.environ.get(
    'TINYNAV_ENABLE_CMD_VEL_NODE', '1' if _ACTUATOR == 'wheel' else '0'
) == '1'

# Pose sources. The camera stamps /camera/camera/vio_image with the image's own
# timestamp, which is what lets looper_bridge_node match keyframes exactly;
# vio_100hz is the faster unbridged one the controller closes its loop on.
# /wheel/camera_pose is wheel_odometry_node's integrated pose already pushed
# through the base_link -> camera extrinsic and rotated into optical axes, so it
# is a drop-in for either.
_POSE_TOPIC_VIO_KEYFRAME = '/camera/camera/vio_image'
_POSE_TOPIC_VIO_CONTROL = '/camera/camera/vio_100hz'
_POSE_TOPIC_WHEEL = '/wheel/camera_pose'

# Measured on this robot, not nominal. wheel_radius came from a driven 3 m
# straight run (tape 3.030 m), base_radius from two driven spins agreeing to
# 0.123%; camera offset is [forward, left, up] from the base centre in metres.
_WHEEL_PORT = os.environ.get('TINYNAV_WHEEL_PORT', '/dev/ttyS3')
_WHEEL_RADIUS = os.environ.get('TINYNAV_WHEEL_RADIUS', '0.050385')
_WHEEL_BASE_RADIUS = os.environ.get('TINYNAV_BASE_RADIUS', '0.127083')
_WHEEL_CAMERA_OFFSET = os.environ.get('TINYNAV_WHEEL_CAMERA_OFFSET', '0.06,0.05,0.18')

# ---------------------------------------------------------------------------- #
# Offline map build resources
# ---------------------------------------------------------------------------- #
# All four exist because an unmodified build is not survivable on the X5, whose
# 1307 MB is shared with the camera firmware and has no swap. Defaults reproduce
# the previous behaviour exactly, so nothing changes on a workstation.
#
# The vocabulary is the sharpest edge. ORBvoc.txt needs 1735 MB to load -- more
# than the board physically has, so the build is OOM-killed before the first
# keyframe -- and it is not even shipped to the board, which makes the failure a
# missing file rather than an obvious out-of-memory. A k10L5 vocabulary trained on
# office footage loads in 407 MB.
_DBOW3_VOCAB = os.environ.get(
    'TINYNAV_DBOW3_VOCAB', os.path.join(_TINYNAV_ROOT, 'docs/Vocabulary/ORBvoc.txt')
)
# Unpaced playback fills the keyframe sync queue faster than the board drains it.
# Measured: OOM at 0.5% progress, 645 MiB resident. A positive rate paces the bag.
_MAP_PLAY_RATE = os.environ.get('TINYNAV_MAP_PLAY_RATE', '')
# Each queue slot holds three full-resolution images, ~1.4 MB at 544x640, so the
# 200 default reserves 280 MB the board does not have.
_MAP_SYNC_QUEUE = os.environ.get('TINYNAV_MAP_SYNC_QUEUE', '')
# The rviz-only publishes dominated per-keyframe cost on the board: 13351 ms fell
# to 321-774 ms with them off, and nothing in the saved map depends on them. Default
# off, because a 17-40x slowdown is not a sensible thing to opt out of -- tool/x5_board/
# app_start.sh pinned it to 0 for exactly that reason, which left the two supported
# launch paths behaving differently. The cost is quadratic, not just large:
# pose_graph_trajectory_publish sends every pose so far on every keyframe, ~165600
# PoseStamped over a 575-keyframe build.
_MAP_VISUALIZATION = os.environ.get('TINYNAV_MAP_VISUALIZATION', '0') == '1'
# See _build_map_argv: 114 s of h264 per build, for videos only PC-side tools read.
_MAP_SAVE_VIDEOS = os.environ.get('TINYNAV_MAP_SAVE_VIDEOS', '0') == '1'


# Bag topics with no consumer in a *looper* offline build, and only there.
#
# The looper build launches exactly two processes -- looper_bridge_node and
# build_map_node -- so the full set of consumed topics is known. imu goes to nobody
# (perception_node is what integrates IMU, and it is the realsense branch); infra2's
# image stream goes to nobody because Looper supplies hardware depth and no stereo runs;
# vio_100hz is the controller's pose source and there is no controller in a map build.
# Measured on a 61847-message bag: imu alone is 69.8% of messages and infra2's images are
# 27% of the bytes, and every one of them costs a deserialize plus three publishes.
#
# Deliberately *not* applied to realsense builds: perception_node runs stereo and needs
# infra2. And deliberately an exclude list -- an allow list derived from build_map_node's
# own subscriptions would omit the raw camera topics the bridge needs in its separate
# process, and the build would finish with an empty map and no error.
_MAP_SKIP_TOPICS_LOOPER = (
    '/camera/camera/imu',
    '/camera/camera/infra2/image_rect_raw',
    '/camera/camera/vio_100hz',
)


def _build_map_argv(map_save_path: str, bag_file: str, skip_topics: tuple = ()) -> list[str]:
    """build_map_node argv, with the board's resource limits applied if set."""
    argv = [
        'python3', os.path.join(_TINYNAV_ROOT, 'tinynav/core/build_map_node.py'),
        '--map_save_path', map_save_path,
        '--bag_file', bag_file,
        '--loop-closure-mode', 'bow',
        '--loop-closure-use-bow',
        '--dbow3-vocabulary-path', _DBOW3_VOCAB,
    ]
    if _MAP_PLAY_RATE:
        argv += ['--play-rate', _MAP_PLAY_RATE]
    if _MAP_SYNC_QUEUE:
        argv += ['--sync-queue-size', _MAP_SYNC_QUEUE]
    if not _MAP_VISUALIZATION:
        argv.append('--no-visualization')
    if skip_topics:
        argv += ['--skip-topics', ','.join(skip_topics)]
    # The two h264 encodes are 95% of the save_image_and_depth stage on the board --
    # measured 95.3 s (rgb) and 18.5 s (infra1) over a 575-keyframe build, against 5.9 s
    # for the depth shelve write that this stage's cost had been attributed to. Neither
    # video is read by anything on the navigation path; the consumers are PC-side offline
    # tools (convert_to_nerf_format for rgb, poi_editor for infra1). A map built through
    # the app is a map for navigating, so it does not pay 114 s for them. Set
    # TINYNAV_MAP_SAVE_VIDEOS=1 when the map is going to be exported instead.
    if not _MAP_SAVE_VIDEOS:
        argv += ['--no-rgb-video', '--no-infra1-video']
    return argv


def _keyframe_pose_topic() -> str:
    """Pose the bridge builds /slam/keyframe_odom from."""
    return _POSE_TOPIC_WHEEL if _ODOM_SOURCE == 'wheel' else _POSE_TOPIC_VIO_KEYFRAME


def _control_pose_topic() -> str:
    """Pose cmd_vel_control closes its loop on."""
    return _POSE_TOPIC_WHEEL if _ODOM_SOURCE == 'wheel' else _POSE_TOPIC_VIO_CONTROL


# The launch argv lives in these builders rather than inline at each call site
# because most of these nodes are started from two or three places -- planning
# from _launch_sensor_procs and cmd_restart_nav_nodes, cmd_vel_control from
# cmd_start_nav_nodes and cmd_restart_nav_nodes, the bridge from
# _launch_sensor_procs and _start_rosbag_build_map -- and the copies had already
# drifted. A parameter added to one copy and not the others is a run that
# silently uses different geometry after an emergency stop.

def _node_argv(rel_path: str) -> list[str]:
    return ['python3', os.path.join(_TINYNAV_ROOT, rel_path)]


def _bridge_argv(*, for_map_build: bool = False) -> list[str]:
    source = _MAP_ODOM_SOURCE if for_map_build else _ODOM_SOURCE
    pose_topic = _POSE_TOPIC_WHEEL if source == 'wheel' else _POSE_TOPIC_VIO_KEYFRAME
    # The synchroniser queue means opposite things in the two modes. Live, it is a
    # latency budget: a deep queue makes a node that falls behind emit its oldest
    # matched set, and map_node discards anything older than 0.5 s, so depth 20 on
    # the X5 produced keyframes 1.49 s old and relocalization never ran at all.
    # Offline, the bag is paced and latency is meaningless, but a dropped matched
    # set is a keyframe lost from the map, so depth is what matters.
    queue_size = '20' if for_map_build else '3'
    # The same split one layer down, in the rclpy reader queues the synchroniser sits on
    # top of. Live, a deep reader queue is what let a stalled sync thread emit seconds-old
    # frames even after the matcher was capped. Offline it is the opposite: the bridge runs
    # ~3.4x slower than realtime on the X5 (240 s for a 71 s bag), so a 1 s depth window --
    # 5 slots at 5 Hz -- overflows, and each dropped frame is a keyframe that never makes
    # it into the map, silently.
    sync_window_s = '10.0' if for_map_build else '1.0'
    return _node_argv('tool/looper_bridge_node.py') + [
        '--pose-topic', pose_topic,
        '--sync-queue-size', queue_size,
        '--sync-window-s', sync_window_s,
        # Only build_map_node consumes /slam/keyframe_depth, and this process starts
        # before it does, so the build gets 'always' and cannot lose a keyframe to the
        # discovery window. Navigation gets 'auto', where nothing subscribes and the
        # decode is skipped outright.
        '--keyframe-depth', 'always' if for_map_build else 'auto',
    ]


def _planning_argv() -> list[str]:
    return _node_argv('tinynav/core/planning_node.py') + [
        '--ros-args',
        '-p', f'robot_type:={_ROBOT_TYPE}',
        '-p', f'pose_topic:={_keyframe_pose_topic()}',
    ]


def _cmd_vel_control_argv() -> list[str]:
    return _node_argv('tinynav/platforms/cmd_vel_control.py') + [
        '--ros-args',
        '-p', f'robot_type:={_ROBOT_TYPE}',
        '-p', f'pose_topic:={_control_pose_topic()}',
    ]


def _wheel_odometry_argv() -> list[str]:
    # enable_wheel_command makes this node the sole owner of the Feetech bus: it
    # both reads the encoders and writes Goal_Velocity. That is not a convenience
    # -- the bus is half duplex and a serial port has one owner, so lekiwi_control
    # cannot run alongside it. This node is the LeKiwi counterpart of
    # unitree_control: the thing that turns /cmd_vel into motion.
    return _node_argv('tinynav/core/wheel_odometry_node.py') + [
        '--ros-args',
        '-p', f'port:={_WHEEL_PORT}',
        '-p', f'wheel_radius:={_WHEEL_RADIUS}',
        '-p', f'base_radius:={_WHEEL_BASE_RADIUS}',
        '-p', f'camera_offset_xyz:=[{_WHEEL_CAMERA_OFFSET}]',
        '-p', 'enable_wheel_command:=true',
        '-p', 'cmd_vel_topic:=/cmd_vel',
    ]

_COLOR_TOPIC_REALSENSE = '/camera/camera/color/image_raw'
_COLOR_TOPIC_LOOPER = '/camera/camera/color/image_rect_raw/compressed'

_IMAGE_TOPICS_REALSENSE = [
    _COLOR_TOPIC_REALSENSE,
    '/camera/camera/infra1/image_rect_raw',
    '/camera/camera/infra2/image_rect_raw',
    '/slam/depth',
]
_IMAGE_TOPICS_LOOPER = [
    _COLOR_TOPIC_LOOPER,
    '/camera/camera/infra1/image_rect_raw',
    '/camera/camera/infra2/image_rect_raw',
    '/slam/depth',
]
_IMAGE_TOPICS_ALL = _IMAGE_TOPICS_REALSENSE  # fallback
_PREVIEW_MIN_INTERVAL = 0.2  # 5 fps


class BackendNode(Ros2NodeManager):
    """Ros2NodeManager + subscriptions needed by the HTTP/WS layer."""

    def __init__(
        self,
        tinynav_db_path: str | None = None,
        *,
        manage_processes: bool = True,
        telemetry_enabled: bool = True,
    ):
        if tinynav_db_path is None:
            tinynav_db_path = os.path.join(_TINYNAV_ROOT, 'tinynav_db')
        super().__init__(
            tinynav_db_path=tinynav_db_path,
            node_name='ros2_node_manager' if manage_processes else 'tinynav_display_backend',
            enable_service_control=manage_processes,
        )

        self._lock = threading.Lock()
        self._destroyed = False
        self._manage_processes = manage_processes
        self.telemetry_enabled = telemetry_enabled
        self.mapping_percent: float = 0.0
        self.current_pose: dict | None = None   # latest pose from SLAM or map

        # Callbacks invoked (in the rclpy spin thread) on new data.
        # Keep them cheap — just put data on a queue or set an event.
        self.pose_callbacks: list = []
        self.state_callbacks: list = []
        self.preview_callbacks: dict[str, list] = {}  # topic -> [callbacks]

        # Planning / localization state (read via get_planning_snapshot)
        self._odom_pose: dict | None = None
        # The local view's canvas origin. Every other layer is world coordinates around
        # the control centre, but /slam/odometry_visual reports the camera -- 78 mm off on
        # LeKiwi, which put the arrow outside the centre of its own footprint.
        self._control_pose: dict | None = None
        self._odom_pose_at_kf: dict | None = None  # odom pose snapshotted at last mapPose update
        self._map_pose: dict | None = None
        self._planning_diag: dict | None = None
        self._localized: bool = False
        self._esdf_bytes: bytes = b''
        self._obstacle_bytes: bytes = b''
        self._trajectory: list = []
        self._trajectory_ref: np.ndarray | None = None  # columns: x, y, t_abs
        self._global_path: list = []
        self._footprint: list = []
        self._voxel_points: list = []
        self._grid_info: dict | None = None
        self._nav_target_pose: dict | None = None
        self._relocalization_stats: dict | None = None
        self._poi_status: dict | None = None
        self._nav_progress: dict | None = None
        self.nav_progress_callbacks: list = []

        self._tf_buffer = None
        self._tf_listener = None
        if self.telemetry_enabled:
            self.create_subscription(Float32, '/mapping/percent', self._on_mapping_percent, 10)
            self.create_subscription(Odometry, '/slam/odometry_visual', self._on_slam_odom, 10)
            self.create_subscription(
                Odometry, '/mapping/current_pose_in_map', self._on_pose_in_map, 10
            )
            # Mark localized as soon as any relocalization succeeds (published unconditionally
            # by map_node, unlike current_pose_in_map which requires POIs to be set).
            self.create_subscription(
                Odometry, '/map/relocalization', self._on_relocalization, 10
            )
            # Rolling relocalization counters: how often it runs, how often it succeeds,
            # and which layer the failures stop at. /map/relocalization only fires on
            # success, so on its own it cannot distinguish "not trying" from "trying and
            # failing" -- which is the question that matters when the robot has stopped.
            # Latched, so a browser connecting mid-run sees the current numbers rather
            # than waiting up to 10 s for the next window.
            self.create_subscription(
                String, '/map/relocalization_stats', self._on_relocalization_stats,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            # Arrival. map_node decides it and previously told nobody: /mapping/poi_change
            # is a topic this class publishes and never subscribes to, and navStatus is
            # derived from this backend's own state machine, which stays 'navigation'
            # until someone disables the nav nodes. So the robot arrived and the UI went
            # on saying "Navigating..." forever.
            self.create_subscription(
                String, '/mapping/poi_status', self._on_poi_status,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            # Per-POI progress and the end-of-run signal. The frontend's progress
            # overlay reads these; nav_done is also what returns rawState to 'idle',
            # which is how the UI knows to stop showing "Navigating".
            self.create_subscription(Bool, '/mapping/nav_done', self._on_nav_done, 10)
            self.create_subscription(String, '/mapping/nav_progress', self._on_nav_progress, 10)
            self.create_subscription(Image, '/planning/height_map', self._on_height_map, 1)
            self.create_subscription(
                OccupancyGrid, '/planning/obstacle_mask', self._on_obstacle_mask, 1
            )
            self.create_subscription(Path, '/planning/trajectory_path', self._on_trajectory_path, 1)
            self.create_subscription(Path, '/mapping/global_plan', self._on_global_plan, 1)
            # TRANSIENT_LOCAL to match map_node's publisher. The nav target is state and
            # is published once per POI transition, so a volatile subscriber that
            # reconnects -- a backend restart, or this node coming up after map_node --
            # reports "no target" for a robot that has one.
            self.create_subscription(
                Odometry, '/control/target_pose', self._on_nav_target_pose,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.create_subscription(PointCloud, '/planning/footprint', self._on_footprint, 1)
            self.create_subscription(PointCloud2, '/planning/occupied_voxels', self._on_occupied_voxels, 1)
            self.create_subscription(String, '/planning/diagnostics', self._on_planning_diag, 1)

            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Publisher for POI nav target consumed by map_node via /mapping/cmd_pois.
        #
        # Latched, because this topic carries *state* -- "the current nav target" --
        # not an event, and it is published exactly once per user click. map_node is
        # started by the same request that then sends the POI, and it needs about
        # 15 s before it subscribes: 11 s to load 575 map keyframes plus 4 s to
        # compile the path-search kernels. A VOLATILE one-shot publish inside that
        # window is gone for good. Measured on the board: 61 consecutive nav-path
        # attempts logged skip_no_poi, planning fell back to a hold-position local
        # trajectory, cmd_vel_control correctly commanded zero, and the robot sat
        # still with a trajectory drawn on screen and no error anywhere.
        #
        # map_node's subscription must be TRANSIENT_LOCAL too: a TRANSIENT_LOCAL
        # publisher and a VOLATILE subscriber still connect, but the late joiner
        # gets no history, which is the entire point here.
        latched_poi_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cmd_pois_pub = self.create_publisher(String, '/mapping/cmd_pois', latched_poi_qos)
        self._poi_change_pub = self.create_publisher(Odometry, '/mapping/poi_change', 10)

        # Latched publisher — new subscribers (cmd_vel_control) get current state immediately on connect
        _latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pause_pub = self.create_publisher(Bool, '/nav/paused', _latched_qos)
        self._nav_paused = False

        # Tells planning_node whether anyone is watching the local view, so it can skip
        # the overlay layers and the voxel cloud when nobody is. Latched for the same
        # reason as the pause flag: planning_node restarts more often than this node.
        self._ui_active_pub = self.create_publisher(Bool, '/planning/ui_active', _latched_qos)
        self._ui_clients = 0
        self._ui_active_sent: bool | None = None
        # Whether any watching client is in 3D mode. Decoding /planning/occupied_voxels is
        # a Python-level read_points loop and the points are 78% of the planning socket's
        # bytes, and the local view starts in 2D, where none of it is drawn.
        self._want_voxels = False
        self._publish_ui_active()

        # Manually placed local-planner target (frontend long-press on the local map).
        # TRANSIENT_LOCAL is mandatory, not a nicety: planning_node's subscription and
        # map_node's publisher for this topic are both TRANSIENT_LOCAL, and DDS requires
        # offered >= requested durability -- a volatile publisher here would simply never
        # match, so the target would be published to nobody with nothing logged.
        self._target_pose_pub = self.create_publisher(
            Odometry, '/control/target_pose', _latched_qos
        )

        # Publisher for robot action commands (sit / stand)
        self._action_pub = self.create_publisher(String, '/service/command', 10)

        # Publisher for teleop velocity commands
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Sensor mode detection and image subscriptions
        self._sensor_mode: str = 'unknown'  # 'looper' | 'realsense' | 'unknown'
        self._image_subs: dict = {}
        # Preview subscriptions are created and destroyed on demand as browsers attach,
        # but the request arrives on uvicorn's asyncio thread while rclpy.spin() is
        # building its wait set on another. rclpy is not thread-safe across that
        # boundary: destroy_subscription from the foreign thread raised
        # `InvalidHandle: cannot use Destroyable because destruction was requested`
        # inside the executor and *killed the spin thread*, after which the backend ran
        # no ROS callbacks at all -- no preview frames, no /planning telemetry, and
        # /map/current and /map/pois answering 503. Observed 5 times across 12 sessions
        # on the board, each immediately after a /ws/preview connect or disconnect.
        #
        # So the websocket handlers only record what they want, and a timer applies it.
        # Timer callbacks run on the spin thread, which is the only thread allowed to
        # touch these handles.
        self._preview_sub_wanted: dict[str, bool] = {}
        # 0.2 s: a browser opening a preview waits at most one tick for its first frame,
        # and the reconcile is a dict comparison over a handful of topics.
        self._preview_sub_timer = self.create_timer(0.2, self._apply_preview_subs)
        self._last_frame: dict[str, bytes] = {}   # topic -> latest JPEG bytes
        self._last_frame_time: dict[str, float] = {}
        self._looper_bridge_proc: subprocess.Popen | None = None
        self._realsense_proc: subprocess.Popen | None = None
        self._perception_proc: subprocess.Popen | None = None
        self._planning_proc: subprocess.Popen | None = None
        self._unitree_proc: subprocess.Popen | None = None
        # LeKiwi base driver: reads the wheel encoders and writes Goal_Velocity.
        # Long-lived, see _launch_wheel_odometry_if_configured.
        self._wheel_odom_proc: subprocess.Popen | None = None

        # Battery level from /battery topic (published by unitree_control)
        self._battery: float | None = None

        # Path to the last successfully verified bag (after stop + ros2 bag info check)
        self._last_verified_bag: str | None = self._find_latest_bag()

        # Nav nodes (map_node + cmd_vel_control) managed independently of _stop_all
        self._nav_nodes_running: bool = False
        # Latch so the "nav node died" error is logged once, not at the UI poll rate.
        self._reported_dead_procs: bool = False
        self._map_node_proc: subprocess.Popen | None = None
        self._cmd_vel_proc: subprocess.Popen | None = None

        if self._manage_processes:
            self.create_subscription(Float32, '/battery', self._on_battery, 10)
        self._detect_and_init_sensor()
        if self._manage_processes:
            self._start_unitree_if_configured()

    def _cmd_cb(self, msg):
        if self._manage_processes:
            super()._cmd_cb(msg)

    # ------------------------------------------------------------------ #
    # ROS callbacks                                                        #
    # ------------------------------------------------------------------ #

    def _on_battery(self, msg: Float32):
        with self._lock:
            self._battery = float(msg.data)

    def _on_mapping_percent(self, msg: Float32):
        with self._lock:
            self.mapping_percent = float(msg.data)

    def _on_slam_odom(self, msg: Odometry):
        pose = self._odom_to_dict(msg, source='slam')
        with self._lock:
            self.current_pose = pose
            self._odom_pose = pose
            self._control_pose = self._to_control_centre(pose)
        for cb in self.pose_callbacks:
            try:
                cb(pose)
            except Exception:
                pass

    def _on_pose_in_map(self, msg: Odometry):
        pose = self._odom_to_dict(msg, source='map')
        with self._lock:
            self.current_pose = pose
            self._map_pose = pose
            self._odom_pose_at_kf = self._odom_pose  # freeze odom at this keyframe
            self._localized = True
        for cb in self.pose_callbacks:
            try:
                cb(pose)
            except Exception:
                pass

    def _on_relocalization(self, msg: Odometry):
        with self._lock:
            self._localized = True

    def _on_relocalization_stats(self, msg: String):
        try:
            stats = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            # Logged rather than passed: silence here would make a malformed payload
            # indistinguishable from map_node not publishing at all.
            self.get_logger().warn(f'bad /map/relocalization_stats payload: {e}')
            return
        with self._lock:
            self._relocalization_stats = stats

    def _on_poi_status(self, msg: String):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'bad /mapping/poi_status payload: {e}')
            return
        with self._lock:
            previously_all = bool((self._poi_status or {}).get('allVisited'))
            self._poi_status = status
        if status.get('allVisited') and not previously_all:
            self.get_logger().info(
                f"arrived: all {status.get('total')} POI(s) visited"
            )

    def _on_nav_done(self, msg: Bool):
        # This is what makes the UI stop saying "Navigating": the frontend watches
        # rawState leaving 'navigation'. Without it the state stays 'navigation'
        # until the user disables the nav nodes by hand.
        if msg.data and self.state == 'navigation':
            self.state = 'idle'
            self._pub_state()

    def _on_nav_progress(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'bad /mapping/nav_progress payload: {e}')
            return
        with self._lock:
            self._nav_progress = data
        # Snapshot before iterating: /ws/nav-progress sockets register and drop from
        # the asyncio thread while this runs on the spin thread.
        for cb in list(self.nav_progress_callbacks):
            cb(data)

    def _on_nav_target_pose(self, msg: Odometry):
        with self._lock:
            self._nav_target_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
            }

    def _on_height_map(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            # Grid is (X_dim, Y_dim, 3): rows=X, cols=Y.
            # Transpose + flipud → rows=Y(inverted), cols=X, so canvas X=right Y=up matches painter.
            arr = np.flipud(arr.transpose(1, 0, 2))
            # Invert JET colormap so dangerous (near obstacle) = red, safe = blue.
            arr = arr[:, :, ::-1]
            _, buf = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with self._lock:
                self._esdf_bytes = buf.tobytes()
        except Exception:
            pass

    def _on_obstacle_mask(self, msg: OccupancyGrid):
        try:
            # Planning node stores OccupancyGrid in Fortran (column-major) order.
            arr = np.array(msg.data, dtype=np.int8)
            grid = arr.reshape(msg.info.height, msg.info.width, order='F')  # (X_dim, Y_dim)
            img = np.where(grid > 50, 255, 0).astype(np.uint8)
            # Transpose + flipud → rows=Y(inverted), cols=X, matching painter (X=right, Y=up).
            img = np.flipud(img.T)
            _, buf = cv2.imencode('.png', img)
            info = {
                'origin_x': float(msg.info.origin.position.x),
                'origin_y': float(msg.info.origin.position.y),
                'resolution': float(msg.info.resolution),
                'width': int(msg.info.height),   # X_dim → image cols (horizontal)
                'height': int(msg.info.width),   # Y_dim → image rows (vertical)
            }
            with self._lock:
                self._obstacle_bytes = buf.tobytes()
                self._grid_info = info
        except Exception:
            pass

    # Anything below this span is planning's zero-motion placeholder: coincident points
    # carrying stamps spread seconds into the future. The history keeps hold of them,
    # and drawn together with a real trajectory they read as a start point sitting
    # where the robot was when navigation was enabled -- visible until the last of them
    # ages past the new path's start, then gone on its own. It is the history of a
    # robot that was not moving, so there is nothing worth keeping.
    _DEGENERATE_PATH_SPAN_M = 1e-3

    def _on_trajectory_path(self, msg: Path):
        new_ref = self._rebuild_trajectory_ref(msg)
        with self._lock:
            if new_ref is None:
                self._trajectory_ref = None
                self._trajectory = []
                return
            span = float(np.max(np.linalg.norm(
                new_ref[:, :2] - new_ref[0, :2], axis=1))) if len(new_ref) > 1 else 0.0
            if span <= self._DEGENERATE_PATH_SPAN_M:
                self._trajectory_ref = new_ref
                self._trajectory = self._trajectory_ref_to_points(new_ref)
                return
            if self._trajectory_ref is None or len(self._trajectory_ref) == 0:
                self._trajectory_ref = new_ref
            else:
                new_start_t = float(new_ref[0, 2])
                kept = self._trajectory_ref[self._trajectory_ref[:, 2] < new_start_t]
                self._trajectory_ref = new_ref if len(kept) == 0 else np.vstack((kept, new_ref))
            self._trajectory = self._trajectory_ref_to_points(self._trajectory_ref)

    def _rebuild_trajectory_ref(self, path_msg: Path):
        n = len(path_msg.poses)
        if n == 0:
            return None
        ref = np.zeros((n, 3), dtype=np.float64)
        for i, pose in enumerate(path_msg.poses):
            ref[i, 0] = pose.pose.position.x
            ref[i, 1] = pose.pose.position.y
            ref[i, 2] = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9

        if n > 1 and np.min(np.diff(ref[:, 2])) <= 0.0:
            start_t = ref[0, 2]
            if start_t <= 0.0:
                start_t = path_msg.header.stamp.sec + path_msg.header.stamp.nanosec * 1e-9
            ref[:, 2] = start_t + np.arange(n, dtype=np.float64) * 0.2
        return ref

    @staticmethod
    def _trajectory_ref_to_points(ref: np.ndarray):
        return [{'x': float(x), 'y': float(y)} for x, y in ref[:, :2]]

    def _on_global_plan(self, msg: Path):
        pts = [
            {'x': p.pose.position.x, 'y': p.pose.position.y}
            for p in msg.poses
        ]
        with self._lock:
            self._global_path = pts

    def _on_planning_diag(self, msg: String):
        try:
            diag = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'bad /planning/diagnostics payload: {e}')
            return
        with self._lock:
            self._planning_diag = diag

    def _on_footprint(self, msg: PointCloud):
        n = len(msg.points)
        if n == 0:
            return
        if n >= 84 and n % 21 == 0:
            corners = [{'x': msg.points[i * 21].x, 'y': msg.points[i * 21].y} for i in range(n // 21)]
        else:
            corners = [{'x': p.x, 'y': p.y} for p in msg.points]
        with self._lock:
            self._footprint = corners

    def _on_occupied_voxels(self, msg: PointCloud2):
        # Only the 3D local view draws these, and the view opens in 2D. Bailing here skips
        # the read_points loop as well as the bytes -- the decode is Python-level and runs
        # at the planning rate.
        if not self._want_voxels:
            return
        try:
            import sensor_msgs_py.point_cloud2 as pc2

            point_count = len(msg.data) // max(1, msg.point_step)
            step = max(1, point_count // 2500)
            points = []
            for i, p in enumerate(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)):
                if i % step != 0:
                    continue
                # Rounded to the 0.1 m grid these came off, so json.dumps writes "-1.2"
                # instead of "-1.2000000000000002" -- same picture, ~40% fewer bytes.
                points.append({'x': round(float(p[0]), 2), 'y': round(float(p[1]), 2),
                               'z': round(float(p[2]), 2)})
                if len(points) >= 2500:
                    break
            with self._lock:
                self._voxel_points = points
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _odom_to_dict(msg: Odometry, source: str) -> dict:
        q = msg.pose.pose.orientation
        # SLAM outputs camera-convention poses (body Z = forward).
        # Project body Z-axis onto world XY to get the true forward heading,
        # which is robust to pitch oscillations during the walking gait.
        fwd_x = 2.0 * (q.x * q.z + q.w * q.y)
        fwd_y = 2.0 * (q.y * q.z - q.w * q.x)
        yaw = math.atan2(fwd_y, fwd_x) if (abs(fwd_x) > 1e-9 or abs(fwd_y) > 1e-9) else 0.0
        return {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w,
            'yaw': yaw,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'source': source,
        }

    @classmethod
    def _to_control_centre(cls, pose: dict | None) -> dict | None:
        """The same pose moved from the camera to the robot's control centre, matching
        planning's camera_to_robot_center and cmd_vel_control's robot_pos."""
        if pose is None:
            return None
        R = cls._quat_to_rot(pose['qx'], pose['qy'], pose['qz'], pose['qw'])
        c = np.array([pose['x'], pose['y'], pose['z']]) - R @ _ROBOT.cam_offset_3d
        return {**pose, 'x': float(c[0]), 'y': float(c[1]), 'z': float(c[2])}

    @staticmethod
    def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
        return np.array([
            [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
            [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)],
        ])

    @classmethod
    def _rot_to_quat(cls, R: np.ndarray) -> tuple[float, float, float, float]:
        """Rotation matrix -> (qx, qy, qz, qw), branching on the largest diagonal so the
        square root stays well conditioned near a 180 degree turn."""
        t = float(np.trace(R))
        if t > 0.0:
            s = math.sqrt(t + 1.0) * 2.0
            return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                    (R[1, 0] - R[0, 1]) / s, 0.25 * s)
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2.0
        q = [0.0, 0.0, 0.0]
        q[i], q[j], q[k] = 0.25 * s, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
        return (q[0], q[1], q[2], (R[k, j] - R[j, k]) / s)

    @classmethod
    def _extrapolated_map_pose(cls, map_kf: dict | None, odom_kf: dict | None,
                               odom_now: dict | None) -> dict | None:
        """The map pose carried forward to now on wheel odometry.

        map_pose is recomputed once per keyframe, and the keyframe is already about a
        second old when map_node gets to it: measured 2026-08-12 at 0.97 s median age
        on top of a ~1 s keyframe interval, so the marker trailed the robot by 20-30 cm
        at 0.22 m/s, always backwards. odom_pose_at_kf was snapshotted for exactly this
        and had no consumer. Composes map_kf * inv(odom_kf) * odom_now.
        """
        if map_kf is None:
            return None
        if odom_kf is None or odom_now is None:
            return map_kf
        R_kf = cls._quat_to_rot(odom_kf['qx'], odom_kf['qy'], odom_kf['qz'], odom_kf['qw'])
        R_now = cls._quat_to_rot(odom_now['qx'], odom_now['qy'], odom_now['qz'], odom_now['qw'])
        R_map = cls._quat_to_rot(map_kf['qx'], map_kf['qy'], map_kf['qz'], map_kf['qw'])
        p_kf = np.array([odom_kf['x'], odom_kf['y'], odom_kf['z']])
        p_now = np.array([odom_now['x'], odom_now['y'], odom_now['z']])
        # The motion since the keyframe, expressed in the keyframe's own body frame.
        dR = R_kf.T @ R_now
        dp = R_kf.T @ (p_now - p_kf)
        R_out = R_map @ dR
        p_out = np.array([map_kf['x'], map_kf['y'], map_kf['z']]) + R_map @ dp
        qx, qy, qz, qw = cls._rot_to_quat(R_out)
        fwd = R_out @ np.array([0.0, 0.0, 1.0])
        return {
            **map_kf,
            'x': float(p_out[0]), 'y': float(p_out[1]), 'z': float(p_out[2]),
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'yaw': math.atan2(float(fwd[1]), float(fwd[0])),
            'timestamp': odom_now.get('timestamp', map_kf.get('timestamp')),
        }

    def _transform_path_via_tf(self, path: list) -> list:
        """Transform map-frame path points to odom (world) frame via TF lookup.

        Returns [] rather than the untransformed input when the transform is
        missing: the local view draws this over the ESDF heatmap in odom
        coordinates, so map coordinates land a metre or two off and read as a
        stray path. Drawing nothing for the second before relocalization
        publishes world<-map is the honest answer.
        """
        if not path or self._tf_buffer is None:
            return []
        try:
            t = self._tf_buffer.lookup_transform('world', 'map', rclpy.time.Time())
            tr = t.transform.translation
            rot = t.transform.rotation
            R = self._quat_to_rot(rot.x, rot.y, rot.z, rot.w)
            trans = np.array([tr.x, tr.y, tr.z])
            result = []
            for pt in path:
                p = R @ np.array([pt['x'], pt['y'], 0.0]) + trans
                result.append({'x': float(p[0]), 'y': float(p[1])})
            return result
        except Exception:
            now = time.monotonic()
            if now - getattr(self, '_last_tf_path_warn', 0.0) > 5.0:
                self._last_tf_path_warn = now
                self.get_logger().warning(
                    'world<-map transform unavailable; global path withheld from the '
                    'local view this cycle (it would be drawn in map coordinates)'
                )
            return []

    # ------------------------------------------------------------------ #
    # Sensor / camera                                                      #
    # ------------------------------------------------------------------ #

    def _detect_and_init_sensor(self):
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        self.get_logger().info(f'BackendNode ROS_DOMAIN_ID={domain}')
        try:
            forced = os.environ.get('TINYNAV_SENSOR_MODE', '').strip().lower()
            if forced in ('looper', 'realsense'):
                # An explicit answer beats probing on a machine whose sensor is not
                # going to change. Guessing wrong is not a degraded mode, it launches
                # an entirely different pipeline: observed on the X5, a mis-detect
                # started the realsense driver and perception against a Looper, so no
                # looper_bridge ran, no /slam keyframes existed, and navigation sat
                # there relocalizing against nothing with no error anywhere.
                self._sensor_mode = forced
                self.get_logger().info(
                    f'Sensor mode: {forced} (forced by TINYNAV_SENSOR_MODE)'
                )
                if self._manage_processes:
                    _env = os.environ.copy()
                    _env['PYTHONPATH'] = _VENV_SITE + ':' + _env.get('PYTHONPATH', '')
                    self._launch_sensor_procs(_env)
                raise _SensorModeDecided

            result = None
            last_err = None
            # Retry while /insight_full is *absent*, not only when the command
            # raises. DDS discovery is asynchronous, and the ros2 daemon caches a
            # view that has been observed on this board to come back empty while the
            # firmware was publishing 15 topics -- a successful call proves nothing
            # about completeness. --no-daemon skips that cache entirely.
            for attempt in range(max(1, _ROS2_NODE_LIST_RETRIES)):
                try:
                    result = subprocess.run(
                        ['ros2', 'node', 'list', '--no-daemon'],
                        capture_output=True,
                        text=True,
                        timeout=_ROS2_NODE_LIST_TIMEOUT,
                    )
                    if '/insight_full' in result.stdout.splitlines():
                        break
                    self.get_logger().info(
                        f'/insight_full not in node list (attempt {attempt + 1}/'
                        f'{_ROS2_NODE_LIST_RETRIES}), waiting for discovery'
                    )
                except Exception as e:
                    last_err = e
                time.sleep(2.0)
            if result is None:
                raise RuntimeError(f'ros2 node list failed after retries: {last_err}')

            if '/insight_full' in result.stdout.splitlines():
                self._sensor_mode = 'looper'
                action = 'launching looper bridge + planning' if self._manage_processes else 'using existing looper topics'
                self.get_logger().info(f'Sensor mode: looper — {action}')
            else:
                self._sensor_mode = 'realsense'
                action = 'launching driver + perception + planning' if self._manage_processes else 'using existing realsense topics'
                self.get_logger().info(f'Sensor mode: realsense — {action}')

            if self._manage_processes and self._sensor_mode in ('looper', 'realsense'):
                _env = os.environ.copy()
                _env['PYTHONPATH'] = _VENV_SITE + ':' + _env.get('PYTHONPATH', '')
                self._launch_sensor_procs(_env)
        except _SensorModeDecided:
            # The forced path is done; fall through to the shared preview-topic
            # setup below rather than duplicating it or returning early.
            pass
        except Exception as e:
            self.get_logger().warn(f'Sensor detection failed: {e}')
            self._sensor_mode = 'unknown'

        topics = _IMAGE_TOPICS_LOOPER if self._sensor_mode == 'looper' else _IMAGE_TOPICS_REALSENSE
        for topic in topics:
            self._last_frame[topic] = b''
            self._last_frame_time[topic] = 0.0
            self.preview_callbacks[topic] = []

    def add_preview_callback(self, topic: str, cb) -> bool:
        """Register a frame callback. The ROS subscription follows on the spin thread."""
        if topic not in self.preview_callbacks:
            return False
        with self._lock:
            self.preview_callbacks[topic].append(cb)
            self._preview_sub_wanted[topic] = True
        return True

    def remove_preview_callback(self, topic: str, cb):
        """Unregister a frame callback; the subscription is dropped once none are left."""
        if topic not in self.preview_callbacks:
            return
        with self._lock:
            try:
                self.preview_callbacks[topic].remove(cb)
            except ValueError:
                pass
            self._preview_sub_wanted[topic] = len(self.preview_callbacks[topic]) > 0

    def _apply_preview_subs(self):
        """Reconcile subscriptions with what the websocket handlers asked for.

        Runs on the spin thread (timer callback), which is the only thread that may
        create or destroy rclpy entities while rclpy.spin() owns the executor.
        Reconciling desired-vs-actual rather than replaying events also means a
        connect/disconnect pair that lands between two ticks cancels out instead of
        churning a subscription.
        """
        with self._lock:
            wanted = dict(self._preview_sub_wanted)
        for topic, want in wanted.items():
            have = topic in self._image_subs
            if want and not have:
                try:
                    self._create_image_sub(topic)
                except Exception as e:
                    self.get_logger().warn(f'preview subscribe {topic} failed: {e}')
            elif have and not want:
                self._destroy_image_sub(topic)

    def _create_image_sub(self, topic: str):
        if topic in self._image_subs:
            return
        if topic == _COLOR_TOPIC_LOOPER:
            self._image_subs[topic] = self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self._on_compressed_image(msg, t),
                1,
            )
        else:
            self._image_subs[topic] = self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self._on_image(msg, t),
                1,
            )

    def _destroy_image_sub(self, topic: str):
        sub = self._image_subs.pop(topic, None)
        if sub is not None:
            try:
                self.destroy_subscription(sub)
            except Exception as e:
                self.get_logger().warn(f'Failed to destroy preview subscription {topic}: {e}')

    def _on_compressed_image(self, msg: CompressedImage, topic: str):
        # monotonic, not time.time(): this board has no RTC, so the wall clock starts
        # near zero every boot and jumps forward when sync_board_time.sh runs. A
        # backward jump makes this delta negative and freezes the preview stream until
        # the wall clock catches back up to the stored timestamp.
        now = time.monotonic()
        if now - self._last_frame_time.get(topic, 0.0) < _PREVIEW_MIN_INTERVAL:
            return
        self._last_frame_time[topic] = now
        frame = bytes(msg.data)
        with self._lock:
            self._last_frame[topic] = frame
        for cb in list(self.preview_callbacks.get(topic, [])):
            try:
                cb(frame)
            except Exception:
                pass

    def _on_image(self, msg: Image, topic: str):
        now = time.monotonic()  # see _on_compressed_image: no RTC on this board
        if now - self._last_frame_time.get(topic, 0.0) < _PREVIEW_MIN_INTERVAL:
            return
        self._last_frame_time[topic] = now

        try:
            if msg.encoding in ('32FC1', 'mono16', '16UC1'):
                if msg.encoding == '32FC1':
                    arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
                    arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
                else:
                    # mono16 millimetres. looper_bridge_node forwards the camera's depth
                    # on /slam/depth unchanged now instead of doubling it to 32FC1;
                    # perception_node still sends 32FC1 in RealSense mode, so both
                    # encodings reach this preview. Integers cannot carry NaN or inf,
                    # so the nan_to_num above has nothing to do here.
                    arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                    arr = arr.astype(np.float32) / 1000.0
                # The 95th percentile is a normalisation constant for a 5 fps preview,
                # so it does not need every pixel. Taken over the full 544x640 it was
                # a compacting copy of up to 348k floats followed by a full sort --
                # the most expensive line in this path, and the main reason uvicorn
                # goes from 36% to 103% CPU while a browser holds the depth preview
                # open. Every 4th pixel in each axis is 1/16 the work and moves the
                # constant by far less than the JET colormap can show.
                sample = arr[::4, ::4]
                sample = sample[sample > 0]
                if sample.size > 0:
                    p95 = float(np.percentile(sample, 95))
                    arr = np.clip(arr / (p95 + 1e-6), 0.0, 1.0)
                arr = (arr * 255).astype(np.uint8)
                arr = cv2.applyColorMap(arr, cv2.COLORMAP_JET)
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
                if arr.shape[2] == 1:
                    arr = arr[:, :, 0]
                elif msg.encoding == 'rgb8':
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            _, buf = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 50])
            frame = buf.tobytes()
        except Exception:
            return

        with self._lock:
            self._last_frame[topic] = frame

        for cb in list(self.preview_callbacks.get(topic, [])):
            try:
                cb(frame)
            except Exception:
                pass

    def get_planning_snapshot(self) -> dict:
        with self._lock:
            path_snapshot = list(self._global_path)
            snapshot = {
                'localized': self._localized,
                # The control centre, not the camera -- see _control_pose. The frontend
                # uses this as the canvas origin for every layer and for the long-press
                # manual target, so all of them shift together.
                'odom_pose': self._control_pose or self._odom_pose,
                'odom_pose_at_kf': self._odom_pose_at_kf,
                # Aged forward on odometry and moved to the control centre, so the
                # marker sits where the chassis is rather than where the camera was a
                # keyframe ago. See _extrapolated_map_pose.
                'map_pose': self._to_control_centre(self._extrapolated_map_pose(
                    self._map_pose, self._odom_pose_at_kf, self._odom_pose)),
                'esdf_image': base64.b64encode(self._esdf_bytes).decode() if self._esdf_bytes else None,
                'obstacle_image': base64.b64encode(self._obstacle_bytes).decode() if self._obstacle_bytes else None,
                'trajectory': list(self._trajectory),
                'global_path': None,  # filled after TF transform (odom frame)
                'map_global_path': path_snapshot,
                'grid_info': self._grid_info,
                'nav_target_pose': self._nav_target_pose,
                'footprint': list(self._footprint),
                'voxel_points': list(self._voxel_points),
                # front clearance / obstacle cells / cycle time, for the app's
                # diagnostics panel. See planning_node._publish_diagnostics.
                'diag': self._planning_diag,
            }
        snapshot['global_path'] = self._transform_path_via_tf(path_snapshot)
        return snapshot

    def _clear_nav_snapshot_locked(self):
        self._localized = False
        self._odom_pose_at_kf = None
        self._map_pose = None
        self._trajectory = []
        self._trajectory_ref = None
        self._global_path = []
        self._footprint = []
        self._voxel_points = []
        self._nav_target_pose = None
        self._planning_diag = None
        # Latched on the wire, so without this a cancelled or restarted run would keep
        # reporting the previous run's arrival.
        self._poi_status = None
        # Not latched, but a cancelled run leaves the last frame sitting here and the
        # next run's overlay would open on the old POI's percentage.
        self._nav_progress = None

    def _publish_nav_target_clear(self):
        """Clear map_node POIs and directly notify planning_node to drop its target."""
        self._cmd_pois_pub.publish(String(data='{}'))
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'map'
        self._poi_change_pub.publish(msg)
        self.get_logger().info('Published nav target clear on /mapping/cmd_pois and /mapping/poi_change')

    def cmd_manual_target_pose(self, x: float, y: float, z: float):
        """Publish a manually selected local-planner target pose.

        planning_node subscribes to /control/target_pose and only reads the position
        vector, so Odometry is used here to match that existing API.
        """
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation.w = 1.0
        self._target_pose_pub.publish(msg)
        with self._lock:
            self._nav_target_pose = {'x': float(x), 'y': float(y)}
        self.get_logger().info(f'manual target published: ({x:.2f}, {y:.2f}, {z:.2f})')

    def _start_unitree_if_configured(self):
        # "if configured" used to mean "always". On a wheel base unitree_control
        # would publish onto the same /cmd_vel that cmd_vel_control drives and
        # fight it, so the actuator choice has to gate this.
        if _ACTUATOR != 'unitree':
            self.get_logger().info(
                f'unitree_control not started: TINYNAV_ACTUATOR={_ACTUATOR}'
            )
            return
        _env = os.environ.copy()
        _env['PYTHONPATH'] = _VENV_SITE + ':' + _env.get('PYTHONPATH', '')
        self._unitree_proc = self._launch_proc(
            'unitree',
            ['python3', os.path.join(_TINYNAV_ROOT, 'tinynav/platforms/unitree_control.py')],
            env=_env,
        )
        self.get_logger().info('unitree_control started')

    def get_sensor_mode(self) -> str:
        return self._sensor_mode

    def get_platform_config(self) -> dict:
        """The wiring this backend actually launched, for /device/platform.

        Worth an endpoint because the three comparison runs differ only in
        environment variables, and every one of them fails *quietly* when set
        wrong: the wrong robot_type just tracks badly, the wrong pose topic just
        never relocalizes. Reading it back beats inferring it from behaviour.
        """
        return {
            'robotType': _ROBOT_TYPE,
            'actuator': _ACTUATOR,
            'odomSource': _ODOM_SOURCE,
            'mapOdomSource': _MAP_ODOM_SOURCE,
            'keyframePoseTopic': _keyframe_pose_topic(),
            'controlPoseTopic': _control_pose_topic(),
            'cmdVelNodeEnabled': _ENABLE_CMD_VEL_NODE,
            'sensorMode': self._sensor_mode,
            'wheelOdometryRunning': self._proc_alive(getattr(self, '_wheel_odom_proc', None)),
            'wheel': {
                'port': _WHEEL_PORT,
                'wheelRadius': float(_WHEEL_RADIUS),
                'baseRadius': float(_WHEEL_BASE_RADIUS),
                'cameraOffsetForwardLeftUp': [float(v) for v in _WHEEL_CAMERA_OFFSET.split(',')],
            } if _ACTUATOR == 'wheel' else None,
            # Reported for the same reason as the rest: a build that dies for lack
            # of memory looks like a build that failed for an unknown reason, and
            # the vocabulary path is the single most likely cause.
            'mapBuild': {
                'vocabulary': _DBOW3_VOCAB,
                'vocabularyExists': os.path.exists(_DBOW3_VOCAB),
                'playRate': _MAP_PLAY_RATE or None,
                'syncQueueSize': _MAP_SYNC_QUEUE or None,
                'visualization': _MAP_VISUALIZATION,
            },
        }

    def get_image_topics(self) -> list[str]:
        if self._sensor_mode == 'looper':
            return _IMAGE_TOPICS_LOOPER
        return _IMAGE_TOPICS_REALSENSE

    def get_preview_frame(self, topic: str) -> bytes:
        with self._lock:
            return self._last_frame.get(topic, b'')

    # ------------------------------------------------------------------ #
    # Command API (called from FastAPI handlers — thread-safe enough)     #
    # ------------------------------------------------------------------ #

    def set_active_bag(self, bag_name: str):
        """Select a bag from rosbags/ by name for map building."""
        path = os.path.join(self.tinynav_db_path, 'rosbags', bag_name)
        if os.path.isdir(path):
            with self._lock:
                self._last_verified_bag = path

    def _find_latest_bag(self) -> str | None:
        rosbags_dir = os.path.join(self.tinynav_db_path, 'rosbags')
        if not os.path.isdir(rosbags_dir):
            return None
        candidates = []
        for name in os.listdir(rosbags_dir):
            path = os.path.join(rosbags_dir, name)
            if os.path.isdir(path) and os.path.exists(os.path.join(path, 'bag_0.db3')):
                candidates.append(path)
        if not candidates:
            return None
        now = time.time() + 24 * 60 * 60.0
        candidates_not_future = [p for p in candidates if os.path.getmtime(p) <= now]
        if candidates_not_future:
            candidates = candidates_not_future
        return max(candidates, key=os.path.getmtime)

    @property
    def active_bag_path(self) -> str | None:
        """Most recently verified bag folder, ready for map building."""
        lvb = self._last_verified_bag
        if lvb and os.path.isdir(lvb):
            return lvb
        return None

    def is_bag_recording(self) -> bool:
        """Return true if the backend or OS process table shows an active recorder."""
        return self._proc_alive(self.processes.get('bag_record')) or bool(self._find_bag_record_processes())

    def recover_stale_error_state(self):
        """Clear terminal error states once no managed process is still active."""
        if not self.state.startswith('error:'):
            return
        if self.is_bag_recording():
            return
        live = [
            name for name, proc in self.processes.items()
            if proc is not None and proc.poll() is None
        ]
        if live:
            return
        old_state = self.state
        self.processes.clear()
        self.state = 'idle'
        self._pub_state()
        self.get_logger().warn(f'Recovered stale backend state {old_state} to idle')

    def get_bag_status(self) -> dict:
        self.recover_stale_error_state()
        bag_file = os.path.join(self.bag_path, 'bag_0.db3')
        active_bag = self.active_bag_path
        return {
            'status': 'recording' if self.is_bag_recording() else 'idle',
            'bagFileReady': os.path.exists(bag_file) or active_bag is not None,
            'bagPath': self.bag_path,
            'activeBagPath': active_bag,
        }

    def _report_dead_nav_procs(self):
        """Log nav children that exited, and stop claiming navigation is running.

        There is no supervision anywhere in this class: _launch_sensor_procs is the only
        thing that ever (re)starts a node, and its only callers are sensor detection and
        the post-map-build restart. A planning_node that dies mid-navigation therefore
        stays dead -- and navNodesRunning is read off the _nav_nodes_running *flag*, not
        off poll(), so the UI kept reporting a navigating robot with no planner. That is
        the same class of silent failure as the lost POI: everything looks fine and
        nothing moves.

        Detection and honest reporting only. Relaunching a nav node by itself would put
        the robot back under control of a stack whose state the operator cannot see, and
        that is their call to make, not this function's.

        Reads the dedicated attributes rather than self.processes: that dict only ever
        holds bag_record, the sensor source node and build_map, so the nav children are
        not in it. recover_stale_error_state iterates the same dict, which is part of
        why nothing noticed a dead planner.
        """
        dead = []
        for name, proc in (
            ('map_node', self._map_node_proc),
            ('planning', self._planning_proc),
            ('cmd_vel_control', self._cmd_vel_proc),
            ('looper_bridge', self._looper_bridge_proc),
        ):
            if proc is not None and proc.poll() is not None:
                dead.append(f'{name}(rc={proc.returncode})')
        if not dead:
            return False
        if not self._reported_dead_procs:
            self._reported_dead_procs = True
            self.get_logger().error(
                'nav nodes exited and nothing restarts them: ' + ', '.join(dead)
                + ' -- navigation is not running; restart it from the UI'
            )
        return True

    def get_status(self) -> dict:
        self.recover_stale_error_state()
        with self._lock:
            raw = self.state
            pct = self.mapping_percent
            battery = self._battery
            nav_nodes = self._nav_nodes_running
            nav_paused = self._nav_paused
            reloc_stats = self._relocalization_stats
            poi_status = self._poi_status
        if nav_nodes and self._report_dead_nav_procs():
            nav_nodes = False
        reloc_stats = self._age_reloc_success(reloc_stats)
        bag_recording = self.is_bag_recording()
        bag_files_exist = self.active_bag_path is not None
        map_files_exist = os.path.exists(os.path.join(self.map_path, 'occupancy_grid.npy'))
        return {
            'battery': battery,
            'bagStatus': 'recording' if bag_recording else 'idle',
            'bagFileReady': bag_files_exist,
            'mapStatus': self._derive_map_status(raw, pct, map_files_exist),
            'mappingPercent': pct,
            # 'arrived' rather than 'navigating' once map_node reports every POI visited.
            # raw stays 'navigation' until the nav nodes are disabled, so on its own it
            # can never express arrival.
            'navStatus': self._derive_nav_status(raw, poi_status),
            'rawState': raw,
            'navNodesRunning': nav_nodes,
            'navPaused': nav_paused,
            # {total, index, visited, allVisited, distanceXyM, arrivalRadiusXyM}.
            # distanceXyM is what makes "closing in" visible instead of only the binary
            # arrival, which matters when the arrival radius is 0.5 m and the operator is
            # trying to work out why a robot 0.6 m away is not done.
            'poiStatus': poi_status,
            # None until map_node's first 10 s window closes. Shape is
            # {window, total, lastFailureCode, secondsSinceLastSuccess}, each of window
            # and total carrying keyframes/attempts/success/failure, attemptHz,
            # successHz, successRate and byCode. 'localized' alone cannot tell you
            # whether relocalization is failing or simply not being attempted.
            'relocalization': reloc_stats,
        }

    @staticmethod
    def _age_reloc_success(stats: dict | None) -> dict | None:
        """Re-age secondsSinceLastSuccess against now.

        map_node only republishes this every 10 s, so the field as received says how
        long ago the success was *when that snapshot was taken* -- a panel showing it
        raw sits frozen and then jumps by 10. Same machine, same clock, so wall time
        is safe here.
        """
        if not stats or stats.get('lastSuccessEpoch') is None:
            return stats
        return {**stats,
                'secondsSinceLastSuccess': round(time.time() - stats['lastSuccessEpoch'], 1)}

    @staticmethod
    def _derive_nav_status(raw: str, poi_status: dict | None) -> str:
        if raw != 'navigation':
            return 'idle'
        if poi_status and poi_status.get('allVisited') and poi_status.get('total'):
            return 'arrived'
        return 'navigating'

    @staticmethod
    def _derive_map_status(raw: str, pct: float, files_exist: bool) -> str:
        if raw == 'rosbag_build_map':
            return 'building'
        if raw.startswith('error:'):
            return 'failed'
        if files_exist and raw == 'idle':
            return 'success'
        return 'idle'

    # ------------------------------------------------------------------ #
    # Sensor proc helpers                                                  #
    # ------------------------------------------------------------------ #

    def _kill_proc(self, proc: subprocess.Popen | None):
        if proc and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), 15)
                proc.wait(timeout=2)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    def _make_log(self, name: str):
        """Open a timestamped log file under tinynav_db/logs/. Safe to close in parent
        after Popen — the child process inherits its own fd copy at fork time."""
        from datetime import datetime
        logs_dir = _DEFAULT_LOG_DIR
        os.makedirs(logs_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        path = os.path.join(logs_dir, f'{ts}_{name}.txt')
        return open(path, 'w')

    def _launch_proc(self, name: str, cmd: list[str], env: dict | None = None,
                      cwd: str = _TINYNAV_ROOT) -> subprocess.Popen:
        """Spawn a subprocess with standard logging and process-group setup."""
        lf = self._make_log(name)
        run_env = self._normalize_run_env(env)
        proc = subprocess.Popen(
            cmd, preexec_fn=os.setsid, cwd=cwd,
            env=run_env,
            stdout=lf, stderr=subprocess.STDOUT,
        )
        lf.close()
        return proc

    def _normalize_run_env(self, env: dict | None) -> dict:
        run_env = (env or os.environ.copy()).copy()
        # Ensure in-repo modules and compiled extension (tinynav_cpp_bind) are importable.
        run_env['PYTHONPATH'] = f"{_TINYNAV_ROOT}:" + run_env.get('PYTHONPATH', '')
        if os.path.exists(os.path.join(_DEVICE_VENV, 'bin', 'python3')):
            run_env['PATH'] = f"{_DEVICE_VENV}/bin:" + run_env.get('PATH', '')
            run_env['VIRTUAL_ENV'] = _DEVICE_VENV
        run_env.setdefault('TMPDIR', '/userdata/tmp')
        run_env.setdefault('TEMP', run_env['TMPDIR'])
        run_env.setdefault('TMP', run_env['TMPDIR'])
        run_env['PATH'] = f"{_LOCAL_PREFIX}/bin:" + run_env.get('PATH', '')
        # Keep /userdata/junlinp/local first so pydbow3 resolves against rebuilt DBoW3.
        run_env['LD_LIBRARY_PATH'] = (
            f"{_LOCAL_PREFIX}/lib:/userdata/junlinp/local/lib:/userdata/opencv-release/lib:"
            + run_env.get('LD_LIBRARY_PATH', '')
        )
        run_env['CMAKE_PREFIX_PATH'] = f"{_LOCAL_PREFIX}:" + run_env.get('CMAKE_PREFIX_PATH', '')
        run_env['PKG_CONFIG_PATH'] = f"{_LOCAL_PREFIX}/lib/pkgconfig:" + run_env.get('PKG_CONFIG_PATH', '')
        # Every child node JITs numba kernels at import; without a cache directory they
        # recompile from cold every launch, measured at 66.1 s of startup against 34.2 s
        # with the cache warm. setdefault rather than assignment because tool/x5_board/
        # env.sh already exports this, and that value should win -- but run_backend.sh
        # does not source env.sh, so relying on the launcher meant the two supported
        # launch paths differed by 32 seconds with nothing to indicate why. Numba writes
        # .nbi/.nbc here rather than into __pycache__, so an empty __pycache__ is not
        # evidence either way.
        run_env.setdefault('NUMBA_CACHE_DIR', _NUMBA_CACHE_DIR)
        os.makedirs(run_env['NUMBA_CACHE_DIR'], exist_ok=True)
        return run_env

    def _launch_wheel_odometry_if_configured(self, env: dict):
        """Bring up the LeKiwi base driver, once, and leave it up.

        Deliberately absent from _stop_sensor_procs and from the nav-node toggle.
        Two reasons, both learned the hard way elsewhere in this file:

        * It owns the serial bus.  Killing and respawning it churns the EEPROM
          unlock sequence and leaves a window where nothing zeroes Goal_Velocity,
          i.e. a moving robot with no controller.
        * It is the odometry origin.  A restart resets the integrated pose to
          zero, which teleports every consumer -- including, mid-recording, the
          /wheel/camera_pose being written into the bag.

        So it behaves like a driver: started with the sensors, torn down only on
        full backend shutdown.
        """
        if not self._manage_processes or _ACTUATOR != 'wheel':
            return
        if self._proc_alive(getattr(self, '_wheel_odom_proc', None)):
            return
        self._wheel_odom_proc = self._launch_proc(
            'wheel_odometry', _wheel_odometry_argv(), env=env,
        )
        self.get_logger().info(
            f'wheel_odometry started (port={_WHEEL_PORT}, odom_source={_ODOM_SOURCE})'
        )

    def _stop_sensor_procs(self):
        if not self._manage_processes:
            return
        for attr in ('_looper_bridge_proc', '_realsense_proc', '_perception_proc', '_planning_proc'):
            self._kill_proc(getattr(self, attr))
            setattr(self, attr, None)

    def _stop_backend_procs(self):
        if not self._manage_processes:
            return
        for attr in (
            '_looper_bridge_proc', '_realsense_proc', '_perception_proc',
            '_planning_proc', '_unitree_proc', '_map_node_proc', '_cmd_vel_proc',
            '_wheel_odom_proc',
        ):
            self._kill_proc(getattr(self, attr, None))
            if hasattr(self, attr):
                setattr(self, attr, None)
        if hasattr(self, '_lock'):
            with self._lock:
                self._nav_nodes_running = False
            self._set_nav_paused(False)

    def destroy_node(self):
        if getattr(self, '_destroyed', False):
            return
        self._destroyed = True
        self._stop_backend_procs()
        super().destroy_node()

    @staticmethod
    def _proc_alive(proc: subprocess.Popen | None) -> bool:
        return proc is not None and proc.poll() is None

    def _find_bag_record_processes(self) -> list[tuple[int, int, str]]:
        """Find live rosbag recorder or wrapper processes for self.bag_path.

        This is a safety net for cases where the backend state was reset but the
        OS-level recorder kept running.
        """
        try:
            result = subprocess.run(
                ['ps', '-ww', '-eo', 'pid=,pgid=,command='],
                capture_output=True,
                text=True,
                timeout=3,
            )
        except Exception:
            return []
        if result.returncode != 0:
            return []

        matches: list[tuple[int, int, str]] = []
        for raw in result.stdout.splitlines():
            parts = raw.strip().split(None, 2)
            if len(parts) != 3:
                continue
            try:
                pid = int(parts[0])
                pgid = int(parts[1])
            except ValueError:
                continue
            cmdline = parts[2]
            if pid == os.getpid() or self.bag_path not in cmdline:
                continue
            is_recorder = 'ros2 bag record' in cmdline
            is_wrapper = 'run_rosbag_record.sh' in cmdline
            if is_recorder or is_wrapper:
                matches.append((pid, pgid, cmdline))
        return matches

    def _signal_bag_recorders(self, sig: signal.Signals):
        current_pgid = os.getpgrp()
        targets: set[tuple[str, int]] = set()

        proc = self.processes.get('bag_record')
        if self._proc_alive(proc):
            try:
                pgid = os.getpgid(proc.pid)
                if pgid != current_pgid:
                    targets.add(('pgid', pgid))
                else:
                    targets.add(('pid', proc.pid))
            except Exception:
                targets.add(('pid', proc.pid))

        for pid, pgid, _ in self._find_bag_record_processes():
            if pgid > 0 and pgid != current_pgid:
                targets.add(('pgid', pgid))
            else:
                targets.add(('pid', pid))

        for kind, value in targets:
            try:
                if kind == 'pgid':
                    os.killpg(value, sig)
                else:
                    os.kill(value, sig)
            except ProcessLookupError:
                pass
            except Exception as exc:
                self.get_logger().warn(f'Failed to signal bag recorder {kind}={value}: {exc}')

    def _wait_for_bag_recorders_exit(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.is_bag_recording():
                return True
            time.sleep(0.1)
        return not self.is_bag_recording()

    def _terminate_bag_recorders(self) -> bool:
        if not self.is_bag_recording():
            self.processes.pop('bag_record', None)
            return True
        self._signal_bag_recorders(signal.SIGINT)
        if not self._wait_for_bag_recorders_exit(4.0):
            self._signal_bag_recorders(signal.SIGTERM)
            self._wait_for_bag_recorders_exit(2.0)
        if self.is_bag_recording():
            self._signal_bag_recorders(signal.SIGKILL)
            self._wait_for_bag_recorders_exit(1.0)
        self.processes.pop('bag_record', None)
        stopped = not self.is_bag_recording()
        if not stopped:
            self.get_logger().error('Failed to stop live bag recorder process')
        return stopped

    def _stop_all(self):
        if not self._manage_processes:
            return
        super()._stop_all()
        self._terminate_bag_recorders()

    def _launch_sensor_procs(self, env: dict):
        """Start sensor procs based on current _sensor_mode."""
        if not self._manage_processes:
            return
        self._launch_wheel_odometry_if_configured(env)
        if self._sensor_mode == 'looper':
            if not self._proc_alive(self._looper_bridge_proc):
                self._looper_bridge_proc = self._launch_proc(
                    'looper_bridge', _bridge_argv(), env=env,
                )
            if not self._proc_alive(self._planning_proc):
                self._planning_proc = self._launch_proc(
                    'planning', _planning_argv(), env=env,
                )
        elif self._sensor_mode == 'realsense':
            if not self._proc_alive(self._realsense_proc):
                self._realsense_proc = self._launch_proc(
                    'realsense',
                    ['bash', _REALSENSE_SCRIPT],
                )
            if not self._proc_alive(self._perception_proc):
                self._perception_proc = self._launch_proc(
                    'perception',
                    ['python3', os.path.join(_TINYNAV_ROOT, 'tinynav/core/perception_node.py')],
                    env=env,
                )
            if not self._proc_alive(self._planning_proc):
                self._planning_proc = self._launch_proc('planning', _planning_argv(), env=env)

    def _restart_sensor_procs(self):
        if not self._manage_processes:
            return
        _env = os.environ.copy()
        _env['PYTHONPATH'] = _VENV_SITE + ':' + _env.get('PYTHONPATH', '')
        self._launch_sensor_procs(_env)
        self.get_logger().info('Sensor procs restarted after map build')

    # ------------------------------------------------------------------ #
    # Nav nodes toggle                                                     #
    # ------------------------------------------------------------------ #

    def cmd_start_nav_nodes(self):
        if not self._manage_processes:
            raise RuntimeError('Nav node lifecycle is disabled in display backend role')
        # Enable has to be idempotent. It was not: this launched map_node and
        # cmd_vel_control unconditionally and overwrote the handles, so a second
        # /nav/nodes/enable orphaned the first pair with nothing left holding a
        # reference to kill. Measured on the board: a cmd_vel_control from 32 minutes
        # earlier still running at ppid 1, 78 MB and 68% of a core, and -- because it
        # still had a publisher -- /cmd_vel had three publishers and two controllers
        # issuing wheel commands from different trajectories. Kill first.
        for name, proc in (('map_node', self._map_node_proc), ('cmd_vel_control', self._cmd_vel_proc)):
            if proc is not None and proc.poll() is None:
                self.get_logger().warn(f'{name} already running (pid {proc.pid}), replacing it')
                self._kill_proc(proc)
        self._map_node_proc = None
        self._cmd_vel_proc = None
        _env = os.environ.copy()
        _env['PYTHONPATH'] = _VENV_SITE + ':' + _env.get('PYTHONPATH', '')
        self._map_node_proc = self._launch_proc(
            'map_node',
            [
                'python3', os.path.join(_TINYNAV_ROOT, 'tinynav/core/map_node.py'),
                '--tinynav_map_path', self.map_path,
                '--loop-closure-mode', 'bow',
                '--loop-closure-use-bow',
                # Same vocabulary as the build, and for the same reason: relocalization
                # has to query the descriptors the map was indexed with, and ORBvoc.txt
                # would OOM the board here exactly as it does there.
                '--dbow3-vocabulary-path', _DBOW3_VOCAB,
            ],
            env=_env,
        )
        if _ENABLE_CMD_VEL_NODE:
            self._cmd_vel_proc = self._launch_proc(
                'cmd_vel_control', _cmd_vel_control_argv(), env=_env,
            )
        else:
            self._cmd_vel_proc = None
        with self._lock:
            self._nav_nodes_running = True
            self._reported_dead_procs = False
        self.get_logger().info('Nav nodes started')

    def cmd_stop_nav_nodes(self):
        if not self._manage_processes:
            raise RuntimeError('Nav node lifecycle is disabled in display backend role')
        self._publish_nav_target_clear()
        self.publish_cmd_vel(0.0, 0.0, 0.0)
        self._kill_proc(self._map_node_proc)
        self._kill_proc(self._cmd_vel_proc)
        self._map_node_proc = None
        self._cmd_vel_proc = None
        with self._lock:
            self._nav_nodes_running = False
            self._clear_nav_snapshot_locked()
        self._set_nav_paused(False)
        self.get_logger().info('Nav nodes stopped')

    def cmd_restart_nav_nodes(self):
        if not self._manage_processes:
            raise RuntimeError('Nav node lifecycle is disabled in display backend role')
        self._kill_proc(self._map_node_proc)
        self._kill_proc(self._planning_proc)
        self._kill_proc(self._cmd_vel_proc)
        self._map_node_proc = None
        self._planning_proc = None
        self._cmd_vel_proc = None

        _env = os.environ.copy()
        _env['PYTHONPATH'] = _VENV_SITE + ':' + _env.get('PYTHONPATH', '')

        self._launch_wheel_odometry_if_configured(_env)
        self._planning_proc = self._launch_proc(
            'planning', _planning_argv(), env=_env,
        )
        self._map_node_proc = self._launch_proc(
            'map_node',
            ['python3', os.path.join(_TINYNAV_ROOT, 'tinynav/core/map_node.py'),
             '--tinynav_map_path', self.map_path,
             '--loop-closure-mode', 'bow',
             '--loop-closure-use-bow',
             # _DBOW3_VOCAB, not a literal. cmd_start_nav_nodes uses it; this path did
             # not, so a nav *restart* loaded ORBvoc.txt -- 1735 MB on a 1307 MB board --
             # and map_node was OOM-killed where a nav *start* had worked.
             '--dbow3-vocabulary-path', _DBOW3_VOCAB],
            env=_env,
        )
        if _ENABLE_CMD_VEL_NODE:
            self._cmd_vel_proc = self._launch_proc(
                'cmd_vel_control', _cmd_vel_control_argv(), env=_env,
            )
        else:
            self._cmd_vel_proc = None
        with self._lock:
            self._nav_nodes_running = True
            self._reported_dead_procs = False
            self._localized = False
            self._map_pose = None
            self._trajectory = []
            self._trajectory_ref = None
            self._global_path = []
            self._footprint = []
            self._voxel_points = []
            self._nav_target_pose = None
        self.state = 'idle'
        self._pub_state()
        self.get_logger().info('Nav nodes restarted (emergency stop)')

    def _start_realsense_bag_record(self):
        import shutil

        if not self._terminate_bag_recorders():
            raise RuntimeError('Cannot start bag recording while previous recorder is still running')

        if os.path.exists(self.bag_path):
            shutil.rmtree(self.bag_path)

        script = os.path.join(_TINYNAV_ROOT, 'scripts', 'run_rosbag_record.sh')
        # The topic set differs by sensor: the RealSense path records both infra
        # images because perception_node runs stereo itself, while the Looper path
        # takes depth from the camera and records the wheel pose instead.
        self.processes['bag_record'] = self._launch_proc(
            'bag_record',
            ['bash', script, '--output', self.bag_path,
             '--sensor', 'looper' if self._sensor_mode == 'looper' else 'realsense'],
        )

    def cmd_bag_start(self):
        if not self._manage_processes:
            raise RuntimeError('Bag recording is disabled in display backend role')
        if self._sensor_mode == 'looper':
            self._stop_sensor_procs()
        self._stop_all()
        self._start('realsense_bag_record')

    def cmd_bag_stop(self) -> bool:
        if self.state != 'realsense_bag_record' and not self.is_bag_recording():
            return False

        bag_path = self.bag_path
        stopped = self._terminate_bag_recorders()
        self.processes.pop('bag_record', None)
        if self.state != 'idle':
            self.state = 'idle'
            self._pub_state()
        if not stopped:
            return False

        if self._sensor_mode == 'looper':
            threading.Thread(
                target=lambda bp: (self._finalize_bag(bp), self._restart_sensor_procs()),
                args=(bag_path,), daemon=True,
            ).start()
        else:
            threading.Thread(target=self._finalize_bag, args=(bag_path,), daemon=True).start()
        return True


    def _finalize_bag(self, bag_path: str):
        import shutil
        from datetime import datetime
        time.sleep(1.5)  # wait for ros2 bag to flush
        if not os.path.isdir(bag_path):
            return
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', bag_path],
                capture_output=True,
                timeout=30,
                env={**os.environ},
            )
            if result.returncode != 0:
                return  # bag corrupted — leave in place
            output = result.stdout.decode('utf-8', errors='replace')
            match = re.search(r'Messages:\s+(\d+)', output)
            if not match or int(match.group(1)) == 0:
                return  # empty bag — leave in place
        except Exception:
            return
        rosbags_dir = os.path.join(os.path.dirname(bag_path), 'rosbags')
        os.makedirs(rosbags_dir, exist_ok=True)
        ts = datetime.now().strftime('bag_%Y_%m_%d_%H_%M_%S')
        dest = os.path.join(rosbags_dir, ts)
        shutil.move(bag_path, dest)
        with self._lock:
            self._last_verified_bag = dest

    def _start_rosbag_build_map(self):
        """Override to use the last verified bag instead of the default bag_path."""
        active = self.active_bag_path
        if active is None:
            self.get_logger().warn('No verified bag available for map building')
            return
        bag_file = os.path.join(active, 'bag_0.db3')
        if not os.path.exists(bag_file):
            self.get_logger().warn(f'bag_0.db3 not found in {active}')
            return
        # Remove existing map path so build_map_node creates a fresh real directory.
        # If map_path is a symlink, shutil.move would rename the symlink (not the target),
        # and build_map_node would write through the symlink into the old map directory.
        import shutil as _shutil
        if os.path.islink(self.map_path) or os.path.isfile(self.map_path):
            os.remove(self.map_path)
        elif os.path.isdir(self.map_path):
            _shutil.rmtree(self.map_path)

        _env = os.environ.copy()
        if self._sensor_mode == 'looper':
            _env['ROS_DOMAIN_ID'] = _MAP_BUILD_DOMAIN_LOOPER
        _env['PYTHONPATH'] = _VENV_SITE + ':' + _env.get('PYTHONPATH', '')
        source_node_cmd = (
            _bridge_argv(for_map_build=True)
            if self._sensor_mode == 'looper'
            else _node_argv('tinynav/core/perception_node.py')
        )
        if self._sensor_mode == 'looper':
            self.get_logger().info(
                f'map build pose source: {_MAP_ODOM_SOURCE} '
                f'({source_node_cmd[-1]}); navigation will run on {_ODOM_SOURCE}'
            )
        source_node_name = 'looper_bridge' if self._sensor_mode == 'looper' else 'perception'
        self.processes[source_node_name] = self._launch_proc(
            source_node_name,
            source_node_cmd,
            env=_env,
        )
        build_argv = _build_map_argv(
            self.map_path, bag_file,
            skip_topics=_MAP_SKIP_TOPICS_LOOPER if self._sensor_mode == 'looper' else (),
        )
        self.get_logger().info(f'map build: {" ".join(build_argv[2:])}')
        self.processes['build_map'] = self._launch_proc_tee(
            'build_map_node',
            build_argv,
            env=_env,
        )

        threading.Thread(target=self._on_build_map_done, daemon=True).start()

    def _launch_proc_tee(self, name: str, cmd: list[str], env: dict | None = None,
                          cwd: str = _TINYNAV_ROOT) -> subprocess.Popen:
        """Like _launch_proc, but also tees stdout to a pipe so the caller can
        scan for MAPPING_PERCENT: lines while still logging everything to file."""
        lf = self._make_log(name)
        run_env = self._normalize_run_env(env)
        proc = subprocess.Popen(
            cmd, preexec_fn=os.setsid, cwd=cwd,
            env=run_env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        threading.Thread(
            target=self._tee_and_read_percent,
            args=(proc, lf),
            daemon=True,
        ).start()
        return proc

    def _tee_and_read_percent(self, proc: subprocess.Popen, log_file):
        """Read lines from proc.stdout, write to log_file, and extract
        MAPPING_PERCENT:<float> values into self.mapping_percent."""
        try:
            for raw in proc.stdout:
                line = raw.decode('utf-8', errors='replace') if isinstance(raw, bytes) else raw
                log_file.write(line)
                log_file.flush()
                if _MAPPING_PERCENT_PREFIX in line:
                    try:
                        pct = float(line.split(_MAPPING_PERCENT_PREFIX, 1)[1].strip())
                        with self._lock:
                            self.mapping_percent = pct
                    except (ValueError, AttributeError):
                        pass
        finally:
            log_file.close()

    def _on_build_map_done(self):
        """Wait for build_map to finish, then archive and restart."""
        import shutil
        from datetime import datetime
        proc_build = self.processes.get('build_map')
        build_ret = proc_build.wait() if proc_build else 1
        if build_ret != 0:
            self.get_logger().error(f'build_map process failed with code {build_ret}')
            self._stop_all()
            self.state = 'idle'
            self._pub_state()
            self._restart_sensor_procs()
            return
        if not os.path.isdir(self.map_path):
            self.get_logger().error(f'map output not found: {self.map_path}')
            self._stop_all()
            self.state = 'idle'
            self._pub_state()
            self._restart_sensor_procs()
            return
        # mv map → maps/map_YYYY_MM_DD_HH_MM_SS, symlink back
        maps_dir = os.path.join(self.tinynav_db_path, 'maps')
        os.makedirs(maps_dir, exist_ok=True)
        ts = datetime.now().strftime('map_%Y_%m_%d_%H_%M_%S')
        dest = os.path.join(maps_dir, ts)
        shutil.move(self.map_path, dest)
        os.symlink(dest, self.map_path)

        # Auto-create a home POI at the SLAM origin (0,0,0) if none exist.
        # map_node requires at least one POI as a global localization anchor.
        pois_path = os.path.join(dest, 'pois.json')
        if not os.path.exists(pois_path):
            with open(pois_path, 'w') as _f:
                json.dump(
                    {'0': {'id': 0, 'name': 'home', 'position': [0.0, 0.0, 0.0]}},
                    _f, indent=2,
                )
            self.get_logger().info('Auto-created home POI at (0,0,0)')

        self._stop_all()
        self.state = 'idle'
        self._pub_state()
        self._restart_sensor_procs()


    def cmd_map_build(self):
        if not self._manage_processes:
            raise RuntimeError('Map building is disabled in display backend role')
        self._stop_sensor_procs()
        self._stop_all()
        self._start('rosbag_build_map')

    def _publish_cmd_pois(self, poi_id: int | None):
        """Publish the selected POI to map_node as JSON on /mapping/cmd_pois.
        Sending an empty dict clears the current nav target."""
        if poi_id is None:
            self._cmd_pois_pub.publish(String(data='{}'))
            return
        pois_file = os.path.join(self.map_path, 'pois.json')
        if not os.path.exists(pois_file):
            self.get_logger().warn('No pois.json found, cannot publish cmd_pois')
            return
        with open(pois_file) as f:
            pois = json.load(f)
        key = str(poi_id)
        if key not in pois:
            self.get_logger().warn(f'POI {poi_id} not found in pois.json')
            return
        # Re-index as "0" to match pub_pois.py convention expected by map_node
        payload = {'0': pois[key]}
        self._cmd_pois_pub.publish(String(data=json.dumps(payload)))

    def cmd_send_pois(self, poi_ids: list[int]):
        """Publish selected POIs to map_node and transition to navigation state."""
        if not poi_ids:
            self._cmd_pois_pub.publish(String(data='{}'))
        else:
            pois_file = os.path.join(self.map_path, 'pois.json')
            if not os.path.exists(pois_file):
                self.get_logger().warn('No pois.json found, cannot publish cmd_pois')
                return
            with open(pois_file) as f:
                all_pois = json.load(f)
            payload = {str(pid): all_pois[str(pid)] for pid in poi_ids if str(pid) in all_pois}
            self._cmd_pois_pub.publish(String(data=json.dumps(payload)))
            # Logged because the silence here cost real debugging time: the clear
            # path logs, the two failure paths log, and the success path did not, so
            # a POST that published a target and a POST that published nothing left
            # identical traces.
            missing = [p for p in poi_ids if str(p) not in all_pois]
            self.get_logger().info(
                f'Published nav target on /mapping/cmd_pois: {sorted(payload)}'
                + (f' (requested but absent from pois.json: {missing})' if missing else '')
            )
        with self._lock:
            nav_running = self._nav_nodes_running
        if nav_running:
            self.state = 'navigation'
            self._pub_state()
        else:
            self._stop_all()
            self._start('navigation')

    def cmd_nav_start(self, poi_id: str | None = None):
        if poi_id is not None:
            self._publish_cmd_pois(int(poi_id))
        with self._lock:
            nav_running = self._nav_nodes_running
        if nav_running:
            # Nav nodes already running — just send the target, don't spawn duplicates.
            self.state = 'navigation'
            self._pub_state()
        else:
            self._stop_all()
            self._start('navigation')

    def cmd_nav_cancel(self):
        if self.state != 'navigation':
            return
        self._publish_nav_target_clear()
        self.publish_cmd_vel(0.0, 0.0, 0.0)
        with self._lock:
            nav_running = self._nav_nodes_running
        if nav_running:
            self.state = 'idle'
            self._pub_state()
        else:
            self._stop_all()
        with self._lock:
            self._clear_nav_snapshot_locked()
        self._set_nav_paused(False)

    def _set_nav_paused(self, paused: bool):
        """Set the pause flag and the latched topic together, never one alone.

        /nav/paused is TRANSIENT_LOCAL, so a flag cleared without a publish leaves
        cmd_vel_control paused forever -- and a controller restarted afterwards still
        picks up the stale True. Call outside self._lock; it is not reentrant.
        """
        with self._lock:
            if self._nav_paused == paused:
                return
            self._nav_paused = paused
        pub = getattr(self, '_pause_pub', None)
        if pub is not None and not getattr(self, '_destroyed', False):
            pub.publish(Bool(data=paused))

    def _publish_ui_active(self):
        """Republish /planning/ui_active only when the answer changed."""
        active = self._ui_clients > 0
        if active == self._ui_active_sent:
            return
        self._ui_active_sent = active
        pub = getattr(self, '_ui_active_pub', None)
        if pub is not None and not getattr(self, '_destroyed', False):
            pub.publish(Bool(data=active))

    def ui_client_attach(self):
        """A /ws/planning client connected."""
        with self._lock:
            self._ui_clients += 1
        self._publish_ui_active()

    def ui_client_detach(self):
        """A /ws/planning client went away; drop its 3D vote with it."""
        with self._lock:
            self._ui_clients = max(0, self._ui_clients - 1)
            if self._ui_clients == 0:
                self._want_voxels = False
                self._voxel_points = []
        self._publish_ui_active()

    def set_want_voxels(self, want: bool):
        """Latest client's local-view mode. Last writer wins, which is right for the one
        browser this robot is driven from and harmless for a second read-only viewer."""
        with self._lock:
            if self._want_voxels == want:
                return
            self._want_voxels = want
            if not want:
                self._voxel_points = []

    def cmd_nav_pause(self):
        self._set_nav_paused(True)

    def cmd_nav_resume(self):
        self._set_nav_paused(False)

    def cmd_action(self, action: str):
        self._action_pub.publish(String(data=f'play {action}'))

    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self._cmd_vel_pub.publish(msg)


class NodeRunner:
    """Manages the rclpy lifecycle; spins BackendNode in a daemon thread."""

    def __init__(
        self,
        tinynav_db_path: str | None = None,
        *,
        manage_processes: bool = True,
        telemetry_enabled: bool = True,
    ):
        self._db_path = tinynav_db_path or os.path.join(_TINYNAV_ROOT, 'tinynav_db')
        self._manage_processes = manage_processes
        self._telemetry_enabled = telemetry_enabled
        self.node: BackendNode | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._stopping = threading.Event()
        self._lock = threading.Lock()
        self._restart_delay_sec = 2.0

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stopping.clear()
        self._ready.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name='rclpy-spin')
        self._thread.start()
        if not self._ready.wait(timeout=15.0):
            raise RuntimeError('rclpy node did not start in time')

    def _run(self):
        restart_nav_nodes = False
        while not self._stopping.is_set():
            node: BackendNode | None = None
            try:
                rclpy.init()
                node = BackendNode(
                    tinynav_db_path=self._db_path,
                    manage_processes=self._manage_processes,
                    telemetry_enabled=self._telemetry_enabled,
                )
                if self._manage_processes and restart_nav_nodes:
                    try:
                        node.cmd_start_nav_nodes()
                    except Exception:
                        print('Failed to restore nav nodes after backend restart', flush=True)
                        traceback.print_exc()
                    restart_nav_nodes = False
                with self._lock:
                    self.node = node
                    self._ready.set()
                rclpy.spin(node)
                if not self._stopping.is_set():
                    print('BackendNode rclpy spin returned unexpectedly; restarting', flush=True)
            except Exception:
                if not self._stopping.is_set():
                    print('BackendNode rclpy spin failed; restarting', flush=True)
                    traceback.print_exc()
            finally:
                with self._lock:
                    if self.node is node:
                        self.node = None
                        self._ready.clear()
                if node is not None:
                    if not self._stopping.is_set():
                        try:
                            with node._lock:
                                restart_nav_nodes = bool(
                                    self._manage_processes and node._nav_nodes_running
                                )
                        except Exception:
                            restart_nav_nodes = False
                    try:
                        node.destroy_node()
                    except Exception:
                        if not self._stopping.is_set():
                            print('BackendNode destroy failed during restart', flush=True)
                            traceback.print_exc()
                try:
                    rclpy.shutdown()
                except Exception:
                    if not self._stopping.is_set():
                        print('rclpy shutdown failed during backend restart', flush=True)
                        traceback.print_exc()
            if not self._stopping.is_set():
                time.sleep(self._restart_delay_sec)

    def stop(self):
        self._stopping.set()
        with self._lock:
            node = self.node
            self.node = None
            self._ready.clear()
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self.node = None
        self._thread = None
        self._ready.clear()
