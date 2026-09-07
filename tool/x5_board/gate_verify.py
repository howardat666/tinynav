#!/usr/bin/env python3
"""零位移验证：nav 关着（cmd_vel_control 不在跑），只给 planning 喂一个正前方 2 m 的目标，
让它打决策日志。先自查 /cmd_vel 有没有发布者，有就直接退出，绝不冒让车动起来的风险。"""
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

Q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

class P(Node):
    def __init__(s):
        super().__init__('gate_verify')
        s.pose = None
        s.create_subscription(PoseStamped, '/camera/camera/vio_image', lambda m: setattr(s,'pose',m), Q)
        # 订阅端要 TRANSIENT_LOCAL，发布端不给就 QoS 不兼容、零投递且完全静默
        s.pub = s.create_publisher(Odometry, '/control/target_pose',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

rclpy.init(); n = P()
time.sleep(1.5); rclpy.spin_once(n, timeout_sec=0.5)
time.sleep(2.0)
pubs = n.get_publishers_info_by_topic('/cmd_vel')
names = [i.node_name for i in pubs]
print(f"/cmd_vel 发布者: {names or '无'}")
# 只有 cmd_vel_control 会自己持续发指令。app 后端那个是遥感用的，不动摇杆不发。
bad = [x for x in names if 'cmd_vel_control' in x]
if bad:
    print(f"❌ {bad} 在跑，车会动 —— 退出"); sys.exit(1)
print("✅ 没有 cmd_vel_control，没人会自动驱动轮子")
t = time.monotonic()
while time.monotonic()-t < 15 and n.pose is None:
    rclpy.spin_once(n, timeout_sec=0.1)
if n.pose is None:
    print("❌ 收不到位姿"); sys.exit(1)
p = n.pose.pose; q = p.orientation
fx = 2*(q.x*q.z + q.w*q.y); fy = 2*(q.y*q.z - q.w*q.x)
yaw = math.atan2(fy, fx)
R = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
tx, ty = p.position.x + R*math.cos(yaw), p.position.y + R*math.sin(yaw)
print(f"车在 ({p.position.x:+.2f},{p.position.y:+.2f}) 朝向 {math.degrees(yaw):+.0f}°"
      f" → 目标放到正前方 {R:.2f} m ({tx:+.2f},{ty:+.2f})")
m = Odometry(); m.header.frame_id = 'world'
m.pose.pose.position.x = tx; m.pose.pose.position.y = ty; m.pose.pose.position.z = p.position.z
m.pose.pose.orientation.w = 1.0
t = time.monotonic()
while time.monotonic()-t < 20:
    m.header.stamp = n.get_clock().now().to_msg()
    n.pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.2)
print("发了 20 秒，停止发布（不发空消息 —— 空 Odometry 的位置是世界原点，那是个真实的点）")
rclpy.shutdown()
