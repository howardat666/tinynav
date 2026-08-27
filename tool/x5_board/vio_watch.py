#!/usr/bin/env python3
"""盯 Looper 的 VIO 位姿，抓"原点重置"。

判据是 **z**，不是速度 —— 实测那次 z 从 −0.76 跳到**恰好 +0.00**，而原地快转时
线速度本来就小，用速度门限看不出来（见 memory vio-origin-reset-poisons-map-odom）。
重置会毒化 map→odom：约束跨了两个坐标系，导致 yaw 阶跃 94°。

用法（板上，app 已在跑）：
  source /userdata/x5/env.sh
  nohup setsid python3 /userdata/x5/vio_watch.py >/dev/null 2>&1 &
日志：/userdata/x5/logs/vio_watch.log
"""
import collections
import importlib.util
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

TOPIC = os.environ.get("VIO_TOPIC", "/camera/camera/vio_image")
LOG = os.environ.get("VIO_WATCH_LOG", "/userdata/x5/logs/vio_watch.log")
DZ_M = float(os.environ.get("VIO_WATCH_DZ_M", "0.15"))
HEARTBEAT_S = float(os.environ.get("VIO_WATCH_HEARTBEAT", "5"))


def _load_bh():
    """只加载一次。每次心跳都 exec_module 一遍是 13% 一个核的主要来源 ——
    这块板子 CPU 本来就是瓶颈，监视器不能自己成为扰动源。"""
    try:
        spec = importlib.util.spec_from_file_location("bh", "/usr/local/sbin/board_health.py")
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m
    except Exception:
        return None


_BH = _load_bh()


def insight():
    """哪个 insight_full、活了多久 —— 用来分开"固件进程重起"和"VIO 内部重初始化"。
    比 /proc/PID/comm 不比 cmdline：后者会命中执行诊断命令的 shell 自己。"""
    if _BH is None:
        return "?"
    try:
        return _BH.insight_state()
    except Exception:
        return "?"


def yaw_deg(q):
    # 只要绕重力轴那一个角，够用且不必引 scipy
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(s, c))


class Watch(Node):
    def __init__(self):
        super().__init__("vio_watch")
        self.f = open(LOG, "a", buffering=1)
        self.prev = None
        self.n = 0
        self.hits = 0
        self.t_hb = 0.0
        self.t0 = time.time()
        self.t_last_msg = 0.0
        # 固件的 rotation_prior_max_interval=0.15：帧间隔超过它就不用 IMU 旋转先验，
        # 而快速转头最依赖那个先验。所以帧间隔的尾部本身就是"会不会丢跟踪"的判据。
        self.gaps = collections.deque(maxlen=600)   # 无上界的 list 会一直长
        self.over_prior = 0
        self.create_subscription(
            PoseStamped, TOPIC, self.cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # 心跳必须时钟驱动。挂在回调上的话，流一停日志也跟着静音 ——
        # 而"流停了"正是另一个已知故障（丢跟踪把 insight_full 卡死），不能和"没在看"混。
        self.create_timer(HEARTBEAT_S, self.heartbeat)
        self.say("开始盯 %s  dz 门限 %.2f m  insight=%s" % (TOPIC, DZ_M, insight()))

    def say(self, msg):
        self.f.write("%s %s\n" % (time.strftime("%F %T"), msg))

    def cb(self, msg):
        self.n += 1
        p = msg.pose.position
        t_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        cur = (t_ns, p.x, p.y, p.z, yaw_deg(msg.pose.orientation))
        if self.prev is not None:
            dt = max(1e-3, (cur[0] - self.prev[0]) / 1e9)
            self.gaps.append(dt)
            if dt > 0.15:
                self.over_prior += 1
            dz = cur[3] - self.prev[3]
            dxy = math.hypot(cur[1] - self.prev[1], cur[2] - self.prev[2])
            dyaw = (cur[4] - self.prev[4] + 180) % 360 - 180
            # z 恰好为 0 是实测到的重置签名；单独列出来，因为小 dz 的重置会漏掉
            exact_zero = abs(cur[3]) < 1e-9 and abs(self.prev[3]) > 0.05
            if abs(dz) > DZ_M or exact_zero:
                self.hits += 1
                self.say("🔴 疑似 VIO 重置 #%d  dt=%.3fs  z %+.3f -> %+.3f (dz=%+.3f)  "
                         "xy %+.2f,%+.2f -> %+.2f,%+.2f (d=%.3f)  yaw %+.1f -> %+.1f (d=%+.1f)  "
                         "%s  insight=%s"
                         % (self.hits, dt, self.prev[3], cur[3], dz,
                            self.prev[1], self.prev[2], cur[1], cur[2], dxy,
                            self.prev[4], cur[4], dyaw,
                            "z 恰好归零" if exact_zero else "dz 超门限", insight()))
        self.prev = cur
        self.t_last_msg = time.time()

    def heartbeat(self):
        now = time.time()
        gap = now - self.t_last_msg if self.t_last_msg else -1.0
        if self.prev is None:
            self.say("还没收到任何帧（%.0fs） insight=%s" % (now - self.t0, insight()))
            return
        flag = ""
        if gap > 2.0:
            # Publisher count 归零是 insight_full 被丢跟踪卡死的判据，和原点重置是两回事
            flag = "  🔴 已 %.1fs 没有新帧（VIO 可能停发了）" % gap
        g = sorted(self.gaps)
        def q(f):
            return g[min(len(g) - 1, int(f * (len(g) - 1)))] * 1000 if g else -1
        self.say("pos=[%+.2f,%+.2f,%+.2f] yaw=%+.1f  %.1f Hz  帧间隔 p50=%.0f p95=%.0f "
                 "max=%.0f ms  >150ms=%d  帧=%d 命中=%d insight=%s%s"
                 % (self.prev[1], self.prev[2], self.prev[3], self.prev[4],
                    self.n / max(1e-3, now - self.t0), q(0.5), q(0.95), q(1.0),
                    self.over_prior, self.n, self.hits, insight(), flag))


def main():
    rclpy.init()
    w = Watch()
    try:
        rclpy.spin(w)
    except KeyboardInterrupt:
        pass
    finally:
        w.say("停止，共 %d 帧，%d 次命中" % (w.n, w.hits))
        w.destroy_node()
        try:                       # 被 SIGTERM 杀时信号处理器已经 shutdown 过，别再抛一次
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
