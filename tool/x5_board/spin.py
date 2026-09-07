#!/usr/bin/env python3
"""直接往 /cmd_vel 发原地转（vx=0），把相机推进 TRACKING 状态。
圆盘底盘 + 驱动轴过圆心，原地转占据区域不变，几何上不会碰到东西。
退出时一定发零，且 diffcar_control 自带 0.5 s 看门狗兜底。用法: spin.py <秒数> [rad/s]"""
import sys, time, signal
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
W = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
rclpy.init()
n = rclpy.create_node('spin')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
stop = Twist()
def bye(*a):
    for _ in range(10):
        pub.publish(stop); time.sleep(0.02)
    rclpy.shutdown(); sys.exit(0)
signal.signal(signal.SIGTERM, bye); signal.signal(signal.SIGINT, bye)
m = Twist(); m.angular.z = W
t = time.monotonic()
try:
    while time.monotonic() - t < DUR:
        pub.publish(m); time.sleep(0.05)
finally:
    for _ in range(10):
        pub.publish(stop); time.sleep(0.02)
    rclpy.shutdown()
