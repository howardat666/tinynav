#!/usr/bin/env python3
"""模拟 map_node 的大消息流量：订阅深度+红外图并各做一次 numpy 拷贝。
不碰 BPU、不占大内存，只制造 DDS 传输 + 内存带宽。用法: bighog.py <秒数>"""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
Q = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
n_ = [0]
class H(Node):
    def __init__(s):
        super().__init__('bighog')
        def cb(m):
            a = np.frombuffer(m.data, dtype=np.uint8).copy()
            n_[0] += int(a[::4096].sum() > -1)
        s.create_subscription(Image, '/camera/camera/depth/image_rect_raw', cb, Q)
        s.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', cb, Q)
        s.create_subscription(Image, '/camera/camera/infra2/image_rect_raw', cb, Q)
rclpy.init(); n = H()
t = time.monotonic()
while time.monotonic() - t < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
rclpy.shutdown()
