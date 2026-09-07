#!/usr/bin/env python3
"""从 /planning/occupied_voxels 重建 build_obstacle_map，对比 span 门限 0.20 vs 0.10。
占据体素点云本身就是 grid3d > occ_threshold 的结果，所以这里重建的是真代码用的那份数据
（不像照 obstacle_mask 反推 ESDF —— 那是另一份，对不上）。只订阅。"""
import math, sys, time
import numpy as np, rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

RES = 0.05
Z_BOT, Z_TOP = -0.20, 0.40
HARD = 0.172
Q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

class P(Node):
    def __init__(s):
        super().__init__('span_probe')
        s.pc = s.pose = s.g = None
        s.create_subscription(PointCloud2, '/planning/occupied_voxels', lambda m: setattr(s,'pc',m), Q)
        s.create_subscription(PoseStamped, '/camera/camera/vio_image', lambda m: setattr(s,'pose',m), Q)
        s.create_subscription(OccupancyGrid, '/planning/obstacle_mask', lambda m: setattr(s,'g',m), Q)
        s.ui = s.create_publisher(Bool, '/planning/ui_active',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

rclpy.init(); n = P()
for _ in range(12):
    n.ui.publish(Bool(data=True)); rclpy.spin_once(n, timeout_sec=0.05); time.sleep(0.1)
t = time.monotonic()
while time.monotonic()-t < 25 and any(v is None for v in (n.pc, n.pose, n.g)):
    rclpy.spin_once(n, timeout_sec=0.1)
if any(v is None for v in (n.pc, n.pose, n.g)):
    print("缺:", [k for k,v in (('voxels',n.pc),('pose',n.pose),('mask',n.g)) if v is None]); sys.exit(1)

pts = np.array([[p[0],p[1],p[2]] for p in pc2.read_points(n.pc, field_names=('x','y','z'), skip_nans=True)])
ps = n.pose.pose; q = ps.orientation
fx = 2*(q.x*q.z+q.w*q.y); fy = 2*(q.y*q.z-q.w*q.x)
yaw = math.atan2(fy, fx); rx, ry, rz = ps.position.x, ps.position.y, ps.position.z
print(f"占据体素 {len(pts)} 个   车在 ({rx:+.2f},{ry:+.2f}) 朝向 {math.degrees(yaw):+.0f}°  pose_z={rz:+.3f}")

# 发布端是 origin + idx*RES（没有 +0.5），而 band 判据用的是 +0.5 的层中心 —— 补回来
z = pts[:,2] + 0.5*RES
zrel = z - rz
inband = (zrel >= Z_BOT) & (zrel <= Z_TOP)
print(f"波段 [{Z_BOT:+.2f},{Z_TOP:+.2f}] 内的体素 {int(inband.sum())} / {len(pts)}")
pb = pts[inband]; zb = z[inband]

# 按 (x,y) 格子聚合，算 z 跨度
key = np.round(pb[:,:2] / RES).astype(np.int64)
uniq, inv = np.unique(key, axis=0, return_inverse=True)
zmax = np.full(len(uniq), -1e9); zmin = np.full(len(uniq), 1e9)
np.maximum.at(zmax, inv, zb); np.minimum.at(zmin, inv, zb)
span = zmax - zmin
layers = np.round(span/RES).astype(int) + 1
print(f"\n有占据体素的 (x,y) 格子 {len(uniq)} 个。按 z 层数分布:")
for L in range(1, 9):
    c = int((layers == L).sum())
    if c: print(f"  {L} 层 (跨度 {(L-1)*RES:.2f} m): {c:4d} 格")
c = int((layers >= 9).sum())
if c: print(f"  >=9 层: {c} 格")

print(f"\n门限对比（obstacle = 有占据 且 z跨度 >= 门限）:")
for thr in (0.20, 0.15, 0.10, 0.05):
    keep = span >= thr - 1e-9
    print(f"  span>={thr:.2f} m（需要 {int(round(thr/RES))+1} 层）: {int(keep.sum()):4d} 格"
          f"  占有占据格子的 {100*keep.mean():5.1f}%")

# 正前方净宽：对每个门限各算一遍
def gapwidth(cells_xy, f):
    """车头前方 f 米处，过中线的连续空隙净宽。cells_xy 是被标为障碍的格中心集合。"""
    S = set(map(tuple, np.round(cells_xy/RES).astype(np.int64)))
    def blocked(l):
        px = rx + f*math.cos(yaw) - l*math.sin(yaw)
        py = ry + f*math.sin(yaw) + l*math.cos(yaw)
        return (int(round(px/RES)), int(round(py/RES))) in S
    if blocked(0.0): return None
    a = b = 0.0
    while a < 1.5 and not blocked(a+RES): a += RES
    while b < 1.5 and not blocked(-(b+RES)): b += RES
    return a + b
print(f"\n正前方净宽（判据：>= 2x{HARD:.3f} = {2*HARD:.3f} m 才过得去）:")
print("  前方     span>=0.20   span>=0.10")
for k in range(3, 12):
    f = k*0.1
    g2 = gapwidth(uniq[span >= 0.20-1e-9]*RES, f)
    g1 = gapwidth(uniq[span >= 0.10-1e-9]*RES, f)
    fmt = lambda g: '中线挡' if g is None else f'{g:.2f} m'
    tag = lambda g: '' if g is None else ('✅' if g >= 2*HARD else '🔴')
    print(f"  {f:.1f} m   {fmt(g2):>8s} {tag(g2)}   {fmt(g1):>8s} {tag(g1)}")

i_ = n.g.info
mask = np.array(n.g.data, dtype=np.int16).reshape(i_.height, i_.width) > 0
print(f"\n对照：真实 obstacle_mask {int(mask.sum())} 格"
      f"（含矮障碍层，所以会比上面 span>=0.20 的 {int((span>=0.20-1e-9).sum())} 格多）")
rclpy.shutdown()
