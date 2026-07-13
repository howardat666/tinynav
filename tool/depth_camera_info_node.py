#!/usr/bin/env python3
"""
Fixes up a raw depth stream so it's usable by rviz2 DepthCloud.

Launch:
  ros2 run looper_vio depth_camera_info_node.py --ros-args \\
    -p depth_image_topic:=/camera/camera/depth/image_rect_raw \\
    -p source_camera_info_topic:=/camera/camera/infra1/camera_info \\
    -p depth_camera_info_topic:=/camera/camera/depth/camera_info \\
    -p depth_image_fixed_topic:=/camera/camera/depth/image_rect_raw_16uc1 \\
    -p vio_pose_topic:=/camera/camera/vio_image \\
    -p camera_tf_frame:=camera_camera_imu
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class DepthCameraInfoNode(Node):
    def __init__(self):
        super().__init__('depth_camera_info_node')

        self.declare_parameter('depth_image_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('source_camera_info_topic', '/camera/camera/infra1/camera_info')
        self.declare_parameter('depth_camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('depth_image_fixed_topic', '/camera/camera/depth/image_rect_raw_16uc1')
        self.declare_parameter('vio_pose_topic', '/camera/camera/vio_image')
        self.declare_parameter('camera_tf_frame', 'camera_camera_imu')

        depth_topic  = self.get_parameter('depth_image_topic').value
        source_topic = self.get_parameter('source_camera_info_topic').value
        out_topic    = self.get_parameter('depth_camera_info_topic').value
        depth_fixed_topic = self.get_parameter('depth_image_fixed_topic').value
        pose_topic   = self.get_parameter('vio_pose_topic').value
        self._camera_tf_frame = self.get_parameter('camera_tf_frame').value

        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        source_info_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        pub_info_qos = QoSProfile(depth=5,
                                   reliability=QoSReliabilityPolicy.RELIABLE,
                                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self._source_info = None
        self._info_sub = self.create_subscription(
            CameraInfo, source_topic, self._on_source_info, source_info_qos)
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth_image, sensor_qos)
        self._pub = self.create_publisher(CameraInfo, out_topic, pub_info_qos)
        self._depth_fixed_pub = self.create_publisher(Image, depth_fixed_topic, sensor_qos)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, 50)

        self.get_logger().info(
            f'Copying intrinsics from {source_topic} -> {out_topic} '
            f'(stamped per-frame from {depth_topic}); '
            f'relabeling encoding on {depth_topic} -> {depth_fixed_topic}; '
            f'broadcasting TF from {pose_topic} -> {self._camera_tf_frame}')

    def _on_source_info(self, msg: CameraInfo):
        self._source_info = msg

    def _on_depth_image(self, msg: Image):
        if msg.encoding == 'mono16':
            fixed = Image()
            fixed.header = msg.header
            fixed.height = msg.height
            fixed.width = msg.width
            fixed.encoding = '16UC1'
            fixed.is_bigendian = msg.is_bigendian
            fixed.step = msg.step
            fixed.data = msg.data
            self._depth_fixed_pub.publish(fixed)
        else:
            self._depth_fixed_pub.publish(msg)

        if self._source_info is None:
            self.get_logger().warn('Waiting for source camera_info...', once=True)
            return

        info = CameraInfo()
        info.header = msg.header
        info.height = self._source_info.height
        info.width = self._source_info.width
        info.distortion_model = self._source_info.distortion_model
        info.d = self._source_info.d
        info.k = self._source_info.k
        info.r = self._source_info.r
        info.p = self._source_info.p
        info.binning_x = self._source_info.binning_x
        info.binning_y = self._source_info.binning_y
        info.roi = self._source_info.roi
        self._pub.publish(info)

    def _on_pose(self, msg: PoseStamped):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = msg.header.frame_id or 'world'
        tf_msg.child_frame_id = self._camera_tf_frame
        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation = msg.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
