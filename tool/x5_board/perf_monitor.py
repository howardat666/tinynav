#!/usr/bin/env python3
"""一次采样把「VIO 健不健康」和「我们的环快不快」同时量出来，用于方案 A/B。
只订阅，不发布任何控制指令。用法: perf_monitor.py <秒数> <标签>"""
import os, re, sys, time, subprocess, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
LABEL = sys.argv[2] if len(sys.argv) > 2 else 'baseline'
HZ = os.sysconf('SC_CLK_TCK')
WATCH = ('insight_full', 'cam-service', 'looper_bridge', 'map_node',
         'planning_node', 'cmd_vel_control', 'diffcar_control', 'uvicorn')

def proc_snap():
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                p = f.read().rsplit(')', 1)[1].split()
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cl = f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
            for w in WATCH:
                if w in cl:
                    out[w] = out.get(w, 0) + int(p[11]) + int(p[12])
                    break
        except Exception:
            pass
    return out

def jcount(pat):
    try:
        r = subprocess.run(['journalctl', '--no-pager', '-b'], capture_output=True,
                           text=True, timeout=25)
        return r.stdout.count(pat)
    except Exception:
        return -1

BE = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
rec = collections.defaultdict(list)     # 话题 -> [(收到的墙钟, 消息时间戳)]
status = []

class M(Node):
    def __init__(s):
        super().__init__('perf_monitor')
        def mk(name):
            def cb(m):
                st = m.header.stamp
                rec[name].append((time.monotonic(), st.sec + st.nanosec * 1e-9))
            return cb
        s.create_subscription(PoseStamped, '/camera/camera/vio_image', mk('vio_image'), BE)
        s.create_subscription(PoseStamped, '/camera/camera/vio_100hz', mk('vio_100hz'), BE)
        s.create_subscription(Imu, '/camera/camera/imu', mk('imu'), BE)
        s.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', mk('infra1'), BE)
        s.create_subscription(Image, '/camera/camera/depth/image_rect_raw', mk('depth'), BE)
        s.create_subscription(String, '/camera/camera/vio_status',
                              lambda m: status.append((time.monotonic(), m.data)), BE)

rclpy.init(); n = M()
c0 = proc_snap()
lost0, rst0 = jcount('Image data is lost for a long time'), jcount('VIO restarted successfully')
sysload, temps, bpus = [], [], []
def bpu_ratio():
    try:
        return int(open('/sys/devices/system/bpu/bpu0/ratio').read().strip())
    except Exception:
        return -1
t0 = time.monotonic(); nxt = t0 + 1.0
while time.monotonic() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.02)
    if time.monotonic() >= nxt:
        nxt += 1.0
        sysload.append(float(open('/proc/loadavg').read().split()[0]))
        temps.append(int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000.0)
        bpus.append(bpu_ratio())
elapsed = time.monotonic() - t0
c1 = proc_snap()
lost1, rst1 = jcount('Image data is lost for a long time'), jcount('VIO restarted successfully')

def q(v, p):
    v = sorted(v)
    return v[min(int(len(v) * p), len(v) - 1)] if v else float('nan')

print(f"\n{'='*74}\n方案【{LABEL}】  采样 {elapsed:.0f} s\n{'='*74}")
print(f"load p50={q(sysload,.5):.2f} max={max(sysload):.2f}   "
      f"温度 p50={q(temps,.5):.1f}C max={max(temps):.1f}C   "
      f"BPU p50={q(bpus,.5):.0f}% p90={q(bpus,.9):.0f}% max={max(bpus)}%")
print(f"\n固件事件（本次开机累计，括号是本段增量）:")
print(f"  Image data is lost : {lost1} ({lost1-lost0:+d})")
print(f"  VIO restarted      : {rst1} ({rst1-rst0:+d})")
if status:
    print(f"  vio_status 取值    : {dict(collections.Counter(s[1] for s in status))}")

print(f"\n各话题（消息时间戳间隔 = 发布端的真实节奏；墙钟间隔 = 含我们被调度的延迟）:")
print(f"  {'话题':<11s} {'Hz':>6s} | {'戳p50':>7s} {'戳p90':>7s} {'戳max':>7s} | "
      f"{'钟p50':>7s} {'钟p90':>7s} {'钟max':>7s}")
for name in ('vio_100hz', 'vio_image', 'imu', 'infra1', 'depth'):
    v = rec.get(name, [])
    if len(v) < 3:
        print(f"  {name:<11s} {'(无数据)':>6s}"); continue
    st = sorted(x[1] for x in v)
    si = [st[i]-st[i-1] for i in range(1, len(st))]
    wi = [v[i][0]-v[i-1][0] for i in range(1, len(v))]
    print(f"  {name:<11s} {len(v)/elapsed:6.1f} | {q(si,.5)*1000:6.0f}m {q(si,.9)*1000:6.0f}m "
          f"{max(si)*1000:6.0f}m | {q(wi,.5)*1000:6.0f}m {q(wi,.9)*1000:6.0f}m {max(wi)*1000:6.0f}m")

print(f"\n各进程瞬时 CPU（%，100 = 一个核）:")
tot = 0.0
for w in WATCH:
    if w in c1 and w in c0:
        d = (c1[w]-c0[w]) / HZ / elapsed * 100
        tot += d
        print(f"  {w:<18s} {d:6.1f}%")
print(f"  {'合计':<18s} {tot:6.1f}% / {os.cpu_count()*100}%")
rclpy.shutdown()
