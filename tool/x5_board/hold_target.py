#!/usr/bin/env python3
"""往 /control/target_pose 喂一个【就在车当前位置】的目标，让整条决策链满速跑起来
但车不会动（距离≈0，进展奖励≈0，静止代价在 0.5 m 内已归零）。
用于在零位移条件下测量 cycle / stamp_lag。用法: hold_target.py <秒数>"""
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 70.0
Q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

class P(Node):
    def __init__(s):
        super().__init__('hold_target')
        s.pose = None
        s.create_subscription(PoseStamped, '/camera/camera/vio_image',
                              lambda m: setattr(s, 'pose', m), Q)
        # 订阅端是 TRANSIENT_LOCAL，发布端不给就零投递且完全静默
        s.pub = s.create_publisher(Odometry, '/control/target_pose',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

rclpy.init(); n = P()
t = time.monotonic()
while time.monotonic() - t < 15 and n.pose is None:
    rclpy.spin_once(n, timeout_sec=0.1)
if n.pose is None:
    print("收不到位姿"); sys.exit(1)
m = Odometry(); m.header.frame_id = 'world'
m.pose.pose.orientation.w = 1.0
print("目标锁在车当前位置，持续 %.0f s（车不会动）" % DUR)
t = time.monotonic()
while time.monotonic() - t < DUR:
    p = n.pose.pose.position          # 跟着车走，永远保持距离 0
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = p.x, p.y, p.z
    m.header.stamp = n.get_clock().now().to_msg()
    n.pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.15)
print("停止发布（不发空消息——空 Odometry 的位置是世界原点，那是个真实的点）")
rclpy.shutdown()
