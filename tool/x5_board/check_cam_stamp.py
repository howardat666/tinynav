#!/usr/bin/env python3
"""量相机深度图的 header 时间戳落后墙钟多少秒。退出码 0=正常，1=偏了，2=没收到。

为什么需要：相机固件 insight_full 在启动时缓存一份墙钟基准，**之后系统时钟跳变它不跟**。
而板子没有 RTC，开机先恢复旧时间、等网络通了再 NTP 一跳（实测 +208.88 s）—— 相机若比
对时先起，它的时间戳就永远停在跳变前。planning 的同步 slop 只有 0.06 s，差 209 s 等于
零回调：局部视图全空、**导航也完全不工作**，而且一行报错都没有。2026-09-22 实测。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

TOPIC = "/slam/depth"
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0


def main() -> int:
    rclpy.init()
    node = Node("cam_stamp_check")
    lags = []
    qos = QoSProfile(depth=5)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT   # 可视化/图像都是 BEST_EFFORT，用默认的 RELIABLE 是零投递且静默

    def cb(msg):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lags.append(time.time() - st)

    node.create_subscription(Image, TOPIC, cb, qos)
    end = time.monotonic() + SECS
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if not lags:
        print("没收到 %s，判不了" % TOPIC)
        return 2
    lag = sorted(lags)[len(lags) // 2]
    print("%s 收到 %d 条，header 戳滞后墙钟中位 %.3f s (门限 %.1f s)" % (TOPIC, len(lags), lag, LIMIT))
    return 0 if abs(lag) <= LIMIT else 1


if __name__ == "__main__":
    sys.exit(main())
