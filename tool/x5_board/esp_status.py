#!/usr/bin/env python3
"""不插 COM 线查 ESP32 状态。ttyS3 被 diffcar_control 独占，所以走它的中继话题。

    . /userdata/x5/env.sh && export ROS_LOCALHOST_ONLY=1
    python3 esp_status.py          # 默认发 N
    python3 esp_status.py Nv 3     # 命令 + 收集秒数

只放行只读查询（N / Nv / Nl），白名单在 diffcar_control.py 那一侧，这里发什么都不会越权。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "N"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

    rclpy.init()
    node = Node("esp_status")
    lines: list[str] = []
    node.create_subscription(String, "/diffcar/esp_reply", lambda m: lines.append(m.data), 50)
    pub = node.create_publisher(String, "/diffcar/esp_cmd", 10)

    # 订阅要先和发布端握上手，否则第一批回复会丢在建立连接的空窗里
    deadline = time.monotonic() + 3.0
    while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() < 1:
        print("🔴 /diffcar/esp_cmd 没有订阅者 —— diffcar_control 没在跑，或者它是旧版没有中继")
        return 1

    pub.publish(String(data=cmd))
    end = time.monotonic() + secs
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()

    if not lines:
        print(f"⚠️ 发了 {cmd!r} 但 {secs}s 内没收到任何回复")
        return 1
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
