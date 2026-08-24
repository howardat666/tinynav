#!/usr/bin/env python3
"""差速车键盘遥控，同屏对比纯轮速里程计和 VIO。在板上跑，走 ROS 话题不碰串口。

  python3 diffcar_teleop.py [--speed 0.20] [--yaw 0.5]

为什么走 /cmd_vel 而不是串口：`/root/car/teleop.py` 直接开 /dev/ttyS3，而 app 里的
diffcar_control 也占着那个口 —— 两边一起动会互相吃字节(2026-08-21 因此毁掉一次 OTA)。
VIO 话题是相机固件 insight_full 自己发的，和 app 无关，所以这条路两个里程计都拿得到。

两个里程计都换算成同一套量：前进 / 左偏 / 转角，左偏和转角都是**左为正**，方便直接和
卷尺量的数对比。轮速侧用相对位姿变换，VIO 侧的前进方向靠第一段直行自己定(见 _latch)。
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

HOLD_S = 0.4           # 点动模式松手多久归零。不靠固件那 2 秒失联保护，太迟钝
RATE_HZ = 20.0
LATCH_M = 0.25         # 轮速侧直行这么多米后，才用 VIO 位移方向定它的"前进"
SAG_WARN_V = 10.0      # 2026-08-24 塌到 9.85 V 时 USB 网卡掉了，整机掉线

HELP = """
  w/s   前进 / 后退 (点动，松手停)      W/S   前进 / 后退 (定速，锁住)
  a/d   左转 / 右转 (点动)              A/D   左转 / 右转 (定速)
  空格  停车并退出定速                  z     两个里程计一起清零
  p     打印一份快照(留在屏幕上好抄)    +/-   调速度上限
  h     重印帮助                        q     退出

  左偏和转角都是左为正。量卷尺时建议固定量"起点 -> 车体左后角"，
  别量"后缘"——那是一条 0.35 m 宽的线，车一歪就读不准(踩过一次，差了 3.7%)。
"""


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def rotvec(q0, q1):
    rel = qmul(q1, (-q0[0], -q0[1], -q0[2], q0[3]))
    v = np.array(rel[:3], float)
    n = float(np.linalg.norm(v))
    return np.zeros(3) if n < 1e-12 else v / n * 2.0 * np.arctan2(n, rel[3])


def getkeys(timeout):
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return ""
    # 必须读裸 fd：sys.stdin.read(1) 会把一整块吸进 Python 缓冲区，之后 select 查
    # 操作系统 fd 只会说"没数据"，积压的按键就只能一轮吐一个
    return os.read(sys.stdin.fileno(), 64).decode("utf-8", "replace")


class Teleop(Node):
    def __init__(self, args):
        super().__init__("diffcar_teleop")
        self.v_max, self.w_max = args.speed, args.yaw
        self.cmd = self.create_publisher(Twist, args.topic, 10)
        self.odom = self.vio = None
        self.batt = None
        self.batt_min = 99.0
        self.create_subscription(Odometry, "/wheel/odometry", self._on_odom, 10)
        self.create_subscription(PoseStamped, "/camera/camera/vio_100hz", self._on_vio, 20)
        self.create_subscription(Float32, "/battery_voltage", self._on_batt, 5)
        self.ref_odom = self.ref_vio = None
        self._u = None              # VIO 的"前进"单位向量，第一段直行后锁定
        self.path_o = self.path_v = 0.0
        self._last_o = self._last_v = None

    def _on_batt(self, m):
        self.batt = float(m.data)
        self.batt_min = min(self.batt_min, self.batt)

    def _on_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        cur = (p.x, p.y, 2.0 * np.arctan2(o.z, o.w))
        if self._last_o is not None:
            self.path_o += float(np.hypot(cur[0] - self._last_o[0], cur[1] - self._last_o[1]))
        self._last_o = cur
        self.odom = cur
        if self.ref_odom is None:
            self.ref_odom = cur

    def _on_vio(self, m):
        p, o = m.pose.position, m.pose.orientation
        cur = (np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w))
        if self._last_v is not None:
            self.path_v += float(np.linalg.norm(cur[0] - self._last_v))
        self._last_v = cur[0].copy()
        self.vio = cur
        if self.ref_vio is None:
            self.ref_vio = (cur[0].copy(), cur[1])

    def zero(self):
        self.ref_odom = self.odom
        self.ref_vio = (self.vio[0].copy(), self.vio[1]) if self.vio else None
        self._u = None
        self.path_o = self.path_v = 0.0
        self.batt_min = self.batt or 99.0

    def wheel_rel(self):
        """轮速里程计的相对位姿 -> (前进, 左偏, 转角 rad)。"""
        if self.odom is None or self.ref_odom is None:
            return None
        x0, y0, t0 = self.ref_odom
        dx, dy = self.odom[0] - x0, self.odom[1] - y0
        dt = self.odom[2] - t0
        while dt > np.pi:
            dt -= 2 * np.pi
        while dt < -np.pi:
            dt += 2 * np.pi
        return (dx * np.cos(t0) + dy * np.sin(t0), -dx * np.sin(t0) + dy * np.cos(t0), dt)

    def _latch(self, w):
        """用第一段"往前的直行"定 VIO 的前进方向。z 已确认是竖直向上(2026-08-24 实测)，
        所以水平面就是 xy，左方向 = 前进方向绕 z 转 +90 度。"""
        if self._u is not None or w is None or self.vio is None:
            return
        if w[0] < LATCH_M or abs(w[1]) > 0.06:
            return
        d = self.vio[0] - self.ref_vio[0]
        n = float(np.hypot(d[0], d[1]))
        if n > 0.5 * LATCH_M:
            self._u = np.array([d[0], d[1]]) / n

    def vio_rel(self):
        if self.vio is None or self.ref_vio is None:
            return None
        d = self.vio[0] - self.ref_vio[0]
        yaw = float(rotvec(self.ref_vio[1], self.vio[1])[2])
        if self._u is None:
            return (None, None, yaw)
        u = self._u
        return (float(d[0] * u[0] + d[1] * u[1]), float(-d[0] * u[1] + d[1] * u[0]), yaw)


def brief(t):
    """实时那一行只放三个数，行程留给快照 —— 一行塞不下就会被截断。"""
    if t is None:
        return "%-30s" % "等数据"
    f, l, y = t
    if f is None:
        return "%-30s" % ("待定向 转角%+6.2f" % np.degrees(y))
    return "%+7.3f / %+7.3f / %+6.2f" % (f, l, np.degrees(y))


def fmt(t, path):
    if t is None:
        return "%-42s" % "等数据"
    f, l, y = t
    if f is None:
        return "%-42s" % ("待定向(先往前直走 %.2f m)  转角 %+.2f 度" % (LATCH_M, np.degrees(y)))
    return "前进 %+8.4f  左偏 %+8.4f  转角 %+7.2f  行程 %7.4f" % (f, l, np.degrees(y), path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.20)
    ap.add_argument("--yaw", type=float, default=0.5)
    ap.add_argument("--topic", default="/cmd_vel")
    args = ap.parse_args()

    rclpy.init()
    node = Teleop(args)
    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is not None and node.vio is not None:
            break
    miss = [n for n, v in (("/wheel/odometry (app 起了吗?)", node.odom),
                           ("/camera/camera/vio_100hz", node.vio)) if v is None]
    if miss:
        sys.exit("拿不到: %s" % ", ".join(miss))

    print(HELP)
    print("速度上限 %.2f m/s / %.2f rad/s，发到 %s" % (args.speed, args.yaw, args.topic))
    print("⚠️ 转向指令低于 0.40 rad/s 在地毯上只兑现 55~81%，慢转会转不到位\n")
    fd = sys.stdin.fileno()
    try:
        saved = termios.tcgetattr(fd)
    except termios.error:
        sys.exit("要在交互式终端里跑(ssh 加 -t)，键盘遥控需要 tty")
    v = w = 0.0
    latched_v = latched_w = 0.0
    last_key = time.time()
    last_pub = last_draw = 0.0
    warned = False
    try:
        tty.setcbreak(fd)
        while True:
            rclpy.spin_once(node, timeout_sec=0.002)
            keys = getkeys(0.01)
            now = time.time()
            for k in keys:
                if k in "qQ\x03":
                    raise KeyboardInterrupt
                if k == " ":
                    latched_v = latched_w = v = w = 0.0
                elif k == "z":
                    node.zero()
                    print("\n里程计已清零(软件基准，固件计数没动)")
                elif k == "h":
                    print(HELP)
                elif k in "+=":
                    node.v_max = min(0.30, node.v_max + 0.05)
                    print("\n速度上限 %.2f m/s" % node.v_max)
                elif k == "-":
                    node.v_max = max(0.05, node.v_max - 0.05)
                    print("\n速度上限 %.2f m/s" % node.v_max)
                elif k == "p":
                    wr, vr = node.wheel_rel(), node.vio_rel()
                    print("\n------- %s 快照 -------" % time.strftime("%H:%M:%S"))
                    print("  轮速里程计   " + fmt(wr, node.path_o))
                    print("  VIO          " + fmt(vr, node.path_v))
                    if wr and vr and vr[0] is not None:
                        print("  差(轮-VIO)   前进 %+8.4f  左偏 %+8.4f  转角 %+7.2f"
                              % (wr[0] - vr[0], wr[1] - vr[1], np.degrees(wr[2] - vr[2])))
                        if abs(vr[0]) > 0.05:
                            print("  轮/VIO 前进比 = %.4f   转角比 = %s"
                                  % (wr[0] / vr[0],
                                     "%.4f" % (wr[2] / vr[2]) if abs(vr[2]) > 1e-3 else "转角太小"))
                    print("  电池 %.2f V (最低 %.2f V)" % (node.batt or 0, node.batt_min))
                    print("  卷尺量：起点 -> 车体左后角")
                elif k in "wsad":
                    v = node.v_max if k == "w" else (-node.v_max if k == "s" else v)
                    w = node.w_max if k == "a" else (-node.w_max if k == "d" else w)
                    if k in "ws":
                        w = latched_w
                    else:
                        v = latched_v
                    last_key = now
                elif k in "WSAD":
                    if k in "WS":
                        latched_v = node.v_max if k == "W" else -node.v_max
                    else:
                        latched_w = node.w_max if k == "A" else -node.w_max
                    v, w = latched_v, latched_w
            if not keys and now - last_key > HOLD_S:
                v, w = latched_v, latched_w        # 点动松手回到定速值(通常是 0)

            if now - last_pub > 1.0 / RATE_HZ:
                last_pub = now
                m = Twist()
                m.linear.x, m.angular.z = float(v), float(w)
                node.cmd.publish(m)
            if node.batt and node.batt < SAG_WARN_V and not warned:
                warned = True
                print("\n⚠️ 电池塌到 %.2f V —— 2026-08-24 塌到 9.85 V 时 USB 网卡掉了整机掉线，"
                      "先充电" % node.batt)
            node._latch(node.wheel_rel())
            if now - last_draw > 0.25:
                last_draw = now
                wr, vr = node.wheel_rel(), node.vio_rel()
                sys.stdout.write("\r前进/左偏/转角   轮 %s | VIO %s | %.2f V | 指令 %+.2f %+.2f  "
                                 % (brief(wr), brief(vr), node.batt or 0, v, w))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        m = Twist()
        for _ in range(8):
            node.cmd.publish(m)
            rclpy.spin_once(node, timeout_sec=0.01)
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print("\n已停车")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
