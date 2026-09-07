#!/usr/bin/env python3
"""抖动版原地转：模拟规划器实际发出的指令序列（走走停停、幅度跳变）。
仍然 vx=0，只有角速度在变，圆盘原地转不会碰到东西。用法: spin_jerky.py <秒数> [周期s]"""
import sys, time, signal, random
import rclpy
from geometry_msgs.msg import Twist
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
PER = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0   # 每 PER 秒换一次指令，模拟重规划周期
rclpy.init()
n = rclpy.create_node('spin_jerky')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
stop = Twist()
def bye(*a):
    for _ in range(10):
        pub.publish(stop); time.sleep(0.02)
    rclpy.shutdown(); sys.exit(0)
signal.signal(signal.SIGTERM, bye); signal.signal(signal.SIGINT, bye)
# 实测过的序列形状：满速 / 近零 / 中速 反复跳，没有加速度限制
SEQ = [0.6, 0.0, 0.05, 0.5, 0.0, 0.6, 0.1, 0.0, 0.45, 0.0]
m = Twist()
t0 = time.monotonic(); k = 0
try:
    while time.monotonic() - t0 < DUR:
        w = SEQ[k % len(SEQ)] * (1 if (k // len(SEQ)) % 2 == 0 else -1)
        k += 1
        m.angular.z = w
        tc = time.monotonic()
        while time.monotonic() - tc < PER and time.monotonic() - t0 < DUR:
            pub.publish(m); time.sleep(0.05)
finally:
    for _ in range(10):
        pub.publish(stop); time.sleep(0.02)
    rclpy.shutdown()
