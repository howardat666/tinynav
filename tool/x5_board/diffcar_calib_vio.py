#!/usr/bin/env python3
"""用 Looper 的 VIO 当真值标定差速车的两个机械常量。在板上跑。

  python3 diffcar_calib_vio.py dist [米]    直线 -> 反推每轮 ppr
  python3 diffcar_calib_vio.py spin [圈]    原地转 -> 反推 WHEEL_BASE

替代 calib.py 的 dist/spin：那两个要人拿卷尺量距离、用眼睛数圈数。VIO 的米制标度来自
出厂标定的 0.1002 m 立体基线，是独立于轮速的参考，而且不需要人在现场。

⚠️ VIO 只在有纹理、有光的场景里可信。跟踪丢了这里会直接中止而不是给一个错数 ——
`/camera/camera/vio_status` 和位姿跳变都在看。
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, '/root/car')
import carlib  # noqa: E402

VIO_TOPIC = '/camera/camera/vio_100hz'
V_ABORT = 9.9          # 低压保护是 9.60，留 0.3V 余量：掉到这儿就停，别拟合出垃圾
JUMP_ABORT_M = 0.15    # 单帧位移上限，VIO 重定位跳变会超过它


def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotvec_small(R):
    """小角度旋转向量。逐帧累加用，100Hz 下每帧约 0.008 rad，这个近似足够。
    整段直接取首末四元数不行：转超过 180 度就绕回去了。"""
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) * 0.5


class Vio(Node):
    def __init__(self):
        super().__init__('vio_calib')
        self.p = None          # 最新位置
        self.R = None          # 最新姿态
        self.p0 = None
        self.rot = np.zeros(3)  # 累加的旋转向量
        self.path = 0.0         # 累加的路径长度
        self.n = 0
        self.jump = None
        self.status = None
        self.create_subscription(PoseStamped, VIO_TOPIC, self._on_pose, 50)
        self.create_subscription(String, '/camera/camera/vio_status', self._on_status, 5)

    def _on_status(self, m):
        self.status = m.data

    def _on_pose(self, m):
        q = m.pose.orientation
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        R = quat_to_R((q.x, q.y, q.z, q.w))
        if self.p is not None:
            d = float(np.linalg.norm(p - self.p))
            if d > JUMP_ABORT_M:
                self.jump = d
            self.path += d
            self.rot += rotvec_small(self.R.T @ R)
        else:
            self.p0 = p.copy()
        self.p, self.R, self.n = p, R, self.n + 1

    def reset(self):
        self.p0 = self.p.copy() if self.p is not None else None
        self.rot = np.zeros(3)
        self.path = 0.0
        self.jump = None

    @property
    def straight_m(self):
        """起点到终点的直线距离。直线行驶时它就是距离；弦长略小于弧长，
        而 2 m 上零点几度的偏航带来的差是微米级。"""
        if self.p0 is None or self.p is None:
            return 0.0
        return float(np.linalg.norm(self.p - self.p0))

    @property
    def yaw_rad(self):
        return float(np.linalg.norm(self.rot))

    @property
    def axis(self):
        n = np.linalg.norm(self.rot)
        return self.rot / n if n > 1e-6 else self.rot


def firmware(car):
    """轮周长/轮距/两轮 ppr 从固件现读，绝不写死。"""
    import re
    txt = '\n'.join(car.ask('?', 1.5))
    circ = re.search(r'轮周长=([\d.]+)m', txt)
    base = re.search(r'轮距=([\d.]+)m', txt)
    ppr = re.findall(r'脉冲/圈=([\d.]+)', txt)
    if not (circ and base and len(ppr) >= 2):
        sys.exit('读不到固件参数，`?` 回的是:\n' + txt)
    return float(circ.group(1)), float(base.group(1)), float(ppr[0]), float(ppr[1])


def vbat(car):
    import re
    for line in car.ask('e', 0.6):
        m = re.search(r'当前=([\d.]+)V', line)
        if m:
            return float(m.group(1))
    return None


def drive(node, car, v, w, stop_when, limit_s):
    """20Hz 重发（固件 2s 失联保护），每次重发之间抽干 VIO 回调。
    返回中止原因，None = 正常达成条件。"""
    t0 = last_v = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.01)
        now = time.time()
        if now - t0 > limit_s:
            return 'timeout %.0fs' % limit_s
        if node.jump is not None:
            return 'VIO 位姿跳变 %.3f m，跟踪可能重定位了' % node.jump
        if stop_when(now - t0):
            return None
        if now - last_v > 0.05:
            car.twist(v, w)
            last_v = now


def stop(node, car, settle=1.5):
    car.twist(0.0, 0.0)
    t0 = time.time()
    while time.time() - t0 < settle:
        rclpy.spin_once(node, timeout_sec=0.01)
    car.send('s')
    t0 = time.time()
    while time.time() - t0 < 0.5:
        rclpy.spin_once(node, timeout_sec=0.01)


def wait_vio(node, need=30, limit=10.0):
    t0 = time.time()
    while node.n < need and time.time() - t0 < limit:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.n < need:
        sys.exit('VIO 没在发（%d 帧 / %.0fs）。相机固件跑着吗？' % (node.n, limit))
    print('VIO 就绪: %d 帧, status=%s' % (node.n, node.status))


def do_dist(node, car, target):
    circ, base, ppr_l, ppr_r = firmware(car)
    v0 = vbat(car)
    print('固件: 轮周长=%.4fm 轮距=%.3fm ppr=%.1f/%.1f   电压=%.2fV' %
          (circ, base, ppr_l, ppr_r, v0))
    if v0 < V_ABORT:
        sys.exit('电压 %.2fV 已经低于 %.2fV 的中止线，先充电' % (v0, V_ABORT))

    car.send('q')
    car.send('z')
    time.sleep(0.4)
    car.drain()
    wait_vio(node)
    node.reset()

    speed = 0.20                       # 比 calib.py 的 0.3 慢：撞上东西时后果小
    print('前进 %.2f m @ %.2f m/s …' % (target, speed))
    hit = drive(node, car, speed, 0.0,
                lambda t: t * speed >= target, target / speed + 6.0)
    stop(node, car)
    if hit:
        sys.exit('中止: ' + hit)

    pose, counts = car.pose(), car.counts()
    fw = pose[0]
    vio = node.straight_m
    print('\n固件里程计 x = %.4f m   (计数 左%d 右%d, theta=%.2f度)'
          % (fw, counts[0], counts[1], pose[2]))
    print('VIO 直线位移  = %.4f m   (路径长 %.4f m, %d 帧)'
          % (vio, node.path, node.n))
    if abs(fw) < 0.2 or vio < 0.2:
        sys.exit('位移太小，没走起来')
    k = vio / fw
    print('\n修正系数 k = VIO / 固件 = %.4f  (固件%s %.1f%%)'
          % (k, '少报' if k > 1 else '多报', abs(k - 1) * 100))
    # ppr 和轮周长在里程计里只以 ppr/circ 的比值出现，所以调 ppr 等价于调轮周长，
    # 但调 ppr 不用重烧固件。固件少报 -> 实际走得多 -> 每个计数代表的距离要变大 -> ppr 变小。
    print('新 ppr = 旧 / k:  左 %.1f  右 %.1f' % (ppr_l / k, ppr_r / k))
    print('\n照这个敲:\n  n %.1f %.1f\n  w' % (ppr_l / k, ppr_r / k))
    print('等价的轮周长 = %.4f m (轮径 %.1f mm)，仅供对照'
          % (circ * k, circ * k / math.pi * 1000))
    return {'fw_m': fw, 'vio_m': vio, 'k': k,
            'ppr_new': [ppr_l / k, ppr_r / k], 'counts': list(counts)}


def do_spin(node, car, turns):
    circ, base, ppr_l, ppr_r = firmware(car)
    v0 = vbat(car)
    print('固件: 轮周长=%.4fm 轮距=%.3fm   电压=%.2fV' % (circ, base, v0))
    if v0 < V_ABORT:
        sys.exit('电压 %.2fV 已经低于 %.2fV 的中止线，先充电' % (v0, V_ABORT))

    car.send('q')
    car.send('z')
    time.sleep(0.4)
    car.drain()
    wait_vio(node)
    node.reset()

    # 0.8 rad/s = 导航配置的 max_yaw。有效轮距要吸收原地转的打滑，而打滑跟转速有关，
    # 所以必须在实际会用到的转速下标。
    w = 0.8
    total = turns * 2 * math.pi
    print('原地左转 %.1f 圈 (%.1f rad) @ %.2f rad/s …' % (turns, total, w))
    hit = drive(node, car, 0.0, w, lambda t: t * w >= total, total / w + 8.0)
    stop(node, car)
    if hit:
        sys.exit('中止: ' + hit)

    pose, counts = car.pose(), car.counts()
    fw_rad = math.radians(pose[2])
    # 固件的 theta 会绕回 ±180，转多圈时不能直接用；用两轮计数差重算。
    ds = (counts[1] - counts[0]) / 2.0 / ((ppr_l + ppr_r) / 2.0) * circ
    fw_rad_counts = ds * 2 / base
    vio_rad = node.yaw_rad
    print('\n固件(计数反推) = %.3f rad = %.2f 圈   (报的 theta=%.1f度，多圈会绕回)'
          % (fw_rad_counts, fw_rad_counts / (2 * math.pi), math.degrees(fw_rad)))
    print('VIO 累加转角    = %.3f rad = %.2f 圈   转轴 %s'
          % (vio_rad, vio_rad / (2 * math.pi), np.round(node.axis, 3)))
    print('VIO 平移        = %.3f m  (原地转应该很小，大了说明车在跑偏)' % node.straight_m)
    if vio_rad < 0.5 or fw_rad_counts < 0.5:
        sys.exit('转角太小，没转起来')
    # 固件按 base_assumed 算 theta，实际按 base_true 转：theta_fw/theta_vio = base_true/base_assumed
    base_true = base * fw_rad_counts / vio_rad
    print('\n实际轮距 = %.3f * %.3f/%.3f = %.4f m  (固件当前 %.3f, 差 %+.1f%%)'
          % (base, fw_rad_counts, vio_rad, base_true, base, (base_true / base - 1) * 100))
    print('\n改 diffcar_esp32/src/main.cpp:\n'
          '  constexpr float WHEEL_BASE = %.4ff;\n'
          '然后 ./ota.sh —— ⚠️ 别动 CFG_REV，一动 NVS 里标好的 ppr/kff/符号全丢'
          % base_true)
    return {'fw_rad': fw_rad_counts, 'vio_rad': vio_rad,
            'base_true': base_true, 'counts': list(counts)}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('dist', 'spin'):
        sys.exit(__doc__)
    arg = float(sys.argv[2]) if len(sys.argv) > 2 else (2.0 if sys.argv[1] == 'dist' else 5.0)
    rclpy.init()
    node = Vio()
    car = carlib.Car()
    time.sleep(0.6)
    car.drain()
    try:
        out = do_dist(node, car, arg) if sys.argv[1] == 'dist' else do_spin(node, car, arg)
        path = '/userdata/x5/calib_%s.json' % sys.argv[1]
        with open(path, 'w') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('\n(原始数据 %s)' % path)
    finally:
        car.twist(0.0, 0.0)
        time.sleep(0.2)
        car.kill()


if __name__ == '__main__':
    main()
