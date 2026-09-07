#!/usr/bin/env python3
"""在实测障碍图上，把 _prefix_clearance 的两套门限都算一遍，数有多少条前进轨迹被放行。
只订阅，不发布运动指令。"""
import math, sys, time
import numpy as np, rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from scipy.ndimage import distance_transform_edt

DT, DUR, NPREF = 0.1, 3.0, 21
VX_MAX, OM_MAX = 0.25, 0.6
RADIUS, HULL, SAFETY = 0.1175, 0.1375, 0.05   # HULL = 绕驱动轴的扫掠半径（轴后置 20 mm）
HARD = HULL + SAFETY          # 0.172，圆盘的正确门限
MARGIN = 0.10                 # 现在代码里的 prefix_margin_m

Q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

class P(Node):
    def __init__(s):
        super().__init__('gate_probe')
        s.g = s.pose = None
        s.create_subscription(OccupancyGrid, '/planning/obstacle_mask', lambda m: setattr(s,'g',m), Q)
        s.create_subscription(PoseStamped, '/camera/camera/vio_image', lambda m: setattr(s,'pose',m), Q)
        s.ui = s.create_publisher(Bool, '/planning/ui_active',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

rclpy.init(); n = P()
for _ in range(12):
    n.ui.publish(Bool(data=True)); rclpy.spin_once(n, timeout_sec=0.05); time.sleep(0.1)
t = time.monotonic()
while time.monotonic()-t < 25 and (n.g is None or n.pose is None):
    rclpy.spin_once(n, timeout_sec=0.1)
if n.g is None or n.pose is None:
    print("缺:", 'mask' if n.g is None else '', 'pose' if n.pose is None else ''); sys.exit(1)

inf_ = n.g.info; res = inf_.resolution
occ = np.array(n.g.data, dtype=np.int16).reshape(inf_.height, inf_.width) > 0
# ESDF：自由空间到最近障碍的距离。栅格是 [i=y, j=x]
esdf = distance_transform_edt(~occ, sampling=(res, res)).astype(np.float32)
ox, oy = inf_.origin.position.x, inf_.origin.position.y

ps = n.pose.pose; q = ps.orientation
fx = 2*(q.x*q.z + q.w*q.y); fy = 2*(q.y*q.z - q.w*q.x)
yaw0 = math.atan2(fy, fx)
rx, ry = ps.position.x, ps.position.y
print(f"栅格 {inf_.width}x{inf_.height} res={res:.3f} 标记 {int(occ.sum())} 格")
print(f"车在 ({rx:+.2f},{ry:+.2f}) 朝向 {math.degrees(yaw0):+.0f}°  中心 ESDF={esdf[int((ry-oy)/res), int((rx-ox)/res)]:.3f} m\n")

def look(xs, ys):
    j = np.clip(((np.asarray(xs)-ox)/res).astype(int), 0, inf_.width-1)
    i = np.clip(((np.asarray(ys)-oy)/res).astype(int), 0, inf_.height-1)
    return esdf[i, j]

def traj(v, om):
    """常扭矩轨迹。om 是【相机 Y 轴】的角速度，世界 yaw 角速度 = -om。"""
    x, y, th = rx, ry, yaw0
    P_ = [(x, y, th)]
    for _ in range(int(DUR/DT)):
        x += v*DT*math.cos(th); y += v*DT*math.sin(th); th -= om*DT
        P_.append((x, y, th))
    return np.array(P_[:NPREF])

def gates(pts):
    cx, cy, th = pts[:,0], pts[:,1], pts[:,2]
    c = look(cx, cy).min()                       # 新：只查中心
    fwx, fwy = np.cos(th), np.sin(th)
    lx, ly = -fwy, fwx
    fl = rl = hw = RADIUS                        # footprint_from_control() 对圆返回 (r,r)
    P5 = [(cx, cy)]
    for a, b in ((fl, hw), (fl, -hw), (-rl, hw), (-rl, -hw)):
        P5.append((cx + fwx*a + lx*b, cy + fwy*a + ly*b))
    s = min(look(px, py).min() for px, py in P5)  # 旧：中心 + 四角
    return c, s

def full_min(v, om):
    """整条 3 s 轨迹的中心最小 ESDF —— 碰撞判据用的就是这个。"""
    x, y, th = rx, ry, yaw0
    xs, ys = [x], [y]
    for _ in range(int(DUR/DT)):
        x += v*DT*math.cos(th); y += v*DT*math.sin(th); th -= om*DT
        xs.append(x); ys.append(y)
    return look(xs, ys).min()

vxs = np.linspace(0.0, VX_MAX, 7)
oms = np.linspace(-OM_MAX, OM_MAX, 15)
ok_old = ok_new = tot = 0
for v in vxs:
    if v <= 0: continue
    for om in oms:
        c, s = gates(traj(v, om)); tot += 1
        ok_old += s >= MARGIN
        ok_new += c >= HARD
print(f"前进轨迹 {tot} 条（不含原地转）")
print(f"  现在的门限（中心+四角 >= {MARGIN:.3f}）放行 {ok_old:3d} 条  → front_blocked={'是' if ok_old==0 else '否'}")
print(f"  圆盘正确门限（只查中心 >= {HARD:.3f}）放行 {ok_new:3d} 条  → front_blocked={'是' if ok_new==0 else '否'}")
# 三道关卡的交集
g_only = c_only = both = 0
for v in vxs:
    if v <= 0: continue
    for om in oms:
        c, _ = gates(traj(v, om))
        gate_ok = c >= HARD                 # 承诺段(2s)门限
        coll_ok = full_min(v, om) >= HARD   # 整条(3s)不碰撞
        g_only += gate_ok and not coll_ok
        c_only += coll_ok and not gate_ok
        both   += gate_ok and coll_ok
print(f"\n三道关卡的交集（90 条前进轨迹）:")
print(f"  门限放行 + 整条不撞（真正可选）  : {both:3d}")
print(f"  门限放行，但 3 s 尾巴上撞了      : {g_only:3d}  ← 被硬否")
print(f"  整条不撞，但门限没放行          : {c_only:3d}")

print(f"\n直行各档（omega=0）前 2 s 的最小 ESDF:")
print("   vx    2s中心  2s五点  3s中心   旧门限|新门限|碰撞")
for v in vxs:
    if v <= 0: continue
    c, s = gates(traj(v, 0.0))
    fm = full_min(v, 0.0)
    print(f"  {v:.3f}  {c:.3f}   {s:.3f}   {fm:.3f}   {'放行' if s>=MARGIN else ' 禁 '} | "
          f"{'放行' if c>=HARD else ' 禁 '} | {'不撞' if fm>=HARD else '撞 '}")
print(f"\n等效过道净宽要求：现在 2x({RADIUS:.4f}+{MARGIN:.2f})={2*(RADIUS+MARGIN):.3f} m"
      f" | 正确 2x{HARD:.3f}={2*HARD:.3f} m")
rclpy.shutdown()
