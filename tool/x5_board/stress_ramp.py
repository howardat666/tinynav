#!/usr/bin/env python3
"""阶梯加压，找 VIO 崩掉的门限。压力进程的 nice 跟我们导航节点一样(-5)，
这样它模拟的就是"我们的节点多干了一些活"，而不是凭空的负载。
只加压 + 只订阅，不发任何控制指令。用法: stress_ramp.py <每级秒数> <最多几线程> <nice>"""
import os, sys, time, subprocess, signal, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String

STEP = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
MAXN = int(sys.argv[2]) if len(sys.argv) > 2 else 6
NICE = int(sys.argv[3]) if len(sys.argv) > 3 else -5

BE = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
rec = collections.defaultdict(list)
status = []

class M(Node):
    def __init__(s):
        super().__init__('stress_ramp')
        def mk(name):
            def cb(m):
                st = m.header.stamp
                rec[name].append((time.monotonic(), st.sec + st.nanosec*1e-9))
            return cb
        s.create_subscription(PoseStamped, '/camera/camera/vio_image', mk('vio'), BE)
        s.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', mk('img'), BE)
        s.create_subscription(String, '/camera/camera/vio_status',
                              lambda m: status.append((time.monotonic(), m.data)), BE)

def jlost():
    try:
        r = subprocess.run(['journalctl', '--no-pager', '-b'], capture_output=True,
                           text=True, timeout=25)
        return (r.stdout.count('Image data is lost for a long time'),
                r.stdout.count('VIO restarted successfully'))
    except Exception:
        return (-1, -1)

BURN = ("import time\n"
        "t=time.monotonic()\n"
        "x=0.0\n"
        "while time.monotonic()-t < %f:\n"
        "    x += 1.7\n" % (STEP*(MAXN+2)+30))
procs = []
rclpy.init(); n = M()
print(f"每级 {STEP:.0f}s，压力进程 nice={NICE}，最多 {MAXN} 个\n")
print(f"{'压力':>4s} {'load':>6s} {'温度':>6s} | {'vio Hz':>7s} {'戳p90':>7s} {'戳max':>7s} |"
      f" {'img Hz':>7s} {'戳p90':>7s} {'戳max':>7s} | {'丢图':>5s} {'重启':>5s} | vio_status")
try:
    for k in range(MAXN + 1):
        while len(procs) < k:
            procs.append(subprocess.Popen(
                ['nice', '-n', str(NICE), 'python3', '-c', BURN],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        time.sleep(2.0)                      # 让负载稳下来
        for v in rec.values():
            v.clear()
        status.clear()
        l0, r0 = jlost()
        loads, temps = [], []
        t0 = time.monotonic(); nxt = t0 + 1.0
        while time.monotonic() - t0 < STEP:
            rclpy.spin_once(n, timeout_sec=0.02)
            if time.monotonic() >= nxt:
                nxt += 1.0
                loads.append(float(open('/proc/loadavg').read().split()[0]))
                temps.append(int(open('/sys/class/thermal/thermal_zone0/temp').read())/1000.0)
        el = time.monotonic() - t0
        l1, r1 = jlost()
        def stat(name):
            v = rec.get(name, [])
            if len(v) < 3:
                return (0.0, float('nan'), float('nan'))
            st = sorted(x[1] for x in v)
            iv = sorted(st[i]-st[i-1] for i in range(1, len(st)))
            return (len(v)/el, iv[int(len(iv)*0.9)]*1000, iv[-1]*1000)
        vh, v9, vm = stat('vio'); ih, i9, im = stat('img')
        sts = "/".join(sorted(set(s[1] for s in status))) or "-"
        print(f"{k:>4d} {max(loads):6.2f} {max(temps):6.1f} | {vh:7.1f} {v9:6.0f}m {vm:6.0f}m |"
              f" {ih:7.1f} {i9:6.0f}m {im:6.0f}m | {l1-l0:5d} {r1-r0:5d} | {sts}")
finally:
    for p in procs:
        try:
            p.kill(); p.wait(timeout=3)
        except Exception:
            pass
    print("\n压力进程已全部清理:", subprocess.run(['pgrep','-c','-f','x += 1.7'],
          capture_output=True, text=True).stdout.strip() or '0', "个残留")
    rclpy.shutdown()
