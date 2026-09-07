#!/usr/bin/env python3
"""验证「POI 在身后」这一路：给规划器喂一个车身后方的目标，看它往哪转。

修对了应该是：escape=heading 触发 -> 原地转（零位移）-> yaw 朝目标收敛 -> |goal_err| 掉进
12° 窗口 -> 脱困退出。修错了就是一路转开、|goal_err| 卡在 158° 不动（2026-09-01 实测）。

⚠️ 会让车原地转。位移理论为零。跑完自己把目标撤掉。"""
import math, sys, time
import numpy as np, rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

BEHIND_M = 1.2
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 18.0
Q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class T(Node):
    def __init__(s):
        super().__init__('rear_target_test')
        s.pose = None; s.cmd = None
        s.create_subscription(PoseStamped, '/camera/camera/vio_image',
                              lambda m: setattr(s, 'pose', m), Q)
        s.create_subscription(Odometry, '/wheel/odometry',
                              lambda m: setattr(s, 'cmd', m), Q)
        s.pub = s.create_publisher(Odometry, '/control/target_pose', LATCH)

def yaw_of(q):
    return math.atan2(2*(q.y*q.z - q.w*q.x), 2*(q.x*q.z + q.w*q.y))

rclpy.init(); n = T()
t = time.monotonic()
while time.monotonic()-t < 15 and n.pose is None:
    rclpy.spin_once(n, timeout_sec=0.1)
if n.pose is None:
    print("❌ 收不到 VIO 位姿"); sys.exit(1)
p0 = n.pose.pose.position; y0 = yaw_of(n.pose.pose.orientation)
tx = p0.x - BEHIND_M*math.cos(y0); ty = p0.y - BEHIND_M*math.sin(y0)
m = Odometry(); m.header.frame_id = 'world'
m.pose.pose.position.x = tx; m.pose.pose.position.y = ty; m.pose.pose.position.z = p0.z
m.pose.pose.orientation.w = 1.0
print(f"车在 ({p0.x:+.2f},{p0.y:+.2f}) 朝向 {math.degrees(y0):+.0f}°")
print(f"目标放在正后方 {BEHIND_M} m: ({tx:+.2f},{ty:+.2f}) —— 方位 {math.degrees(y0)+180:+.0f}°，即 heading_err 180°\n")
print("   t     yaw(deg)  到目标方位(deg)  heading_err(deg)  |位移|(m)")
rows = []
t0 = time.monotonic(); nxt = 0.0
try:
    while time.monotonic()-t0 < DUR:
        if time.monotonic()-t0 >= nxt:
            nxt += 0.25
            m.header.stamp = n.get_clock().now().to_msg()
            n.pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.pose is None: continue
        pp = n.pose.pose.position; yy = yaw_of(n.pose.pose.orientation)
        bearing = math.atan2(ty-pp.y, tx-pp.x)
        he = (bearing-yy+math.pi) % (2*math.pi) - math.pi
        rows.append((time.monotonic()-t0, yy, bearing, he, math.hypot(pp.x-p0.x, pp.y-p0.y)))
except KeyboardInterrupt:
    pass
finally:
    for _ in range(5):
        n.pub.publish(Odometry())      # 空目标 = 撤掉
        time.sleep(0.05)
last = -1
for r in rows:
    if r[0] - last < 1.0: continue
    last = r[0]
    print(f" {r[0]:5.1f}   {math.degrees(r[1]):+7.0f}      {math.degrees(r[2]):+7.0f}         "
          f"{math.degrees(r[3]):+7.0f}        {r[4]:.3f}")
if rows:
    he = np.array([abs(math.degrees(r[3])) for r in rows])
    print(f"\n|heading_err|: 起 {he[0]:.0f}°  最小 {he.min():.0f}°  末 {he[-1]:.0f}°")
    print(f"最大位移 {max(r[4] for r in rows):.3f} m")
    print("✅ 朝目标收敛了" if he.min() < 60 else "🔴 没有朝目标收敛（还是转开）")
n.destroy_node(); rclpy.shutdown()
