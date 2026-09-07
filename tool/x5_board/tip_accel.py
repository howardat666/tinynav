#!/usr/bin/env python3
"""自定义加速度的倾倒测试。用 VIO 100 Hz 的俯仰角，不需要秤。

    python3 tool/x5_board/tip_accel.py 1.5                # 单个加速度
    python3 tool/x5_board/tip_accel.py 0.5 1.0 1.5 2.0    # 依次扫几个
    python3 tool/x5_board/tip_accel.py 1.5 --vmax 0.45    # 阶跃的目标速度
    python3 tool/x5_board/tip_accel.py 1.5 --abort-deg 4  # 抬头超过 4° 就立刻停

只装前脚轮时**前进加速**才是风险相位：惯性力向后 -> 绕驱动轴抬头 -> 前脚轮卸载 ->
后方没有支撑。刹车和倒车加速是把脚轮压紧，安全方向。所以每一档都测「前进阶跃」和
「从倒车刹停」两个方向。

为什么用 100 Hz 那一路：加速段只有 v/a 秒（0.45 m/s 在 1.5 m/s^2 下是 0.3 s），
20 Hz 只能采到 6 点，抬头的尖峰会被漏掉 —— 2026-09-01 第一版就是这么低报的。

⚠️ 要独占串口才能改固件的加速度上限，所以本脚本会先停 app、跑完再起回来。
   VIO 由相机固件 insight_full 自己发布，停 app 不影响。
"""
from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, "/root/car")

_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
_VL = re.compile(r"vl=(-?[\d.]+)\s+vr=(-?[\d.]+)")
_APP = "/userdata/x5/tinynav/tool/x5_board/app_start.sh"


def pitch_of(q) -> float:
    """相机光学约定在 z 朝上世界系里的俯仰，抬头为正。前向向量的世界 z 分量就是 sin(俯仰)，
    这样取和 yaw 无关，车朝哪边都能直接比。"""
    fz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.asin(max(-1.0, min(1.0, fz)))


class Pitch(Node):
    def __init__(self, topic):
        super().__init__("tip_accel")
        self.rows: list[tuple[float, float]] = []
        self.t0 = time.monotonic()
        self.create_subscription(PoseStamped, topic, self._cb, _QOS)

    def _cb(self, m: PoseStamped) -> None:
        self.rows.append((time.monotonic() - self.t0, pitch_of(m.pose.orientation)))

    def now(self) -> float:
        return self.rows[-1][1] if self.rows else float("nan")

    def hz(self) -> float:
        if len(self.rows) < 20:
            return 0.0
        return (len(self.rows) - 1) / (self.rows[-1][0] - self.rows[0][0])


def spin(node, car, secs, cmd, out, base, abort_rad):
    """维持 cmd 速度 secs 秒。同时 20 Hz 复发指令（固件失联保护 2 s）、收 vl/vr、
    转 ROS。抬头超限就返回 False 让上层立刻收手。"""
    t_end = time.monotonic() + secs
    nxt = 0.0
    while time.monotonic() < t_end:
        if time.monotonic() >= nxt:
            nxt = time.monotonic() + 0.05
            car.send("u %.3f 0" % cmd)
            car.send("p")
        for ln in car.drain():
            m = _VL.search(ln)
            if m:
                out.append((time.monotonic(), (float(m.group(1)) + float(m.group(2))) / 2.0))
        rclpy.spin_once(node, timeout_sec=0.002)
        p = node.now()
        if not math.isnan(p) and abs(p - base) > abort_rad:
            car.send("u 0 0")
            return False
        time.sleep(0.002)
    return True


def profile(node, t_a, t_b, base):
    """[t_a, t_b) 区间的俯仰，每 0.1 s 一格，返回 (峰值度数, ASCII 形状)"""
    r = [(t, p) for t, p in node.rows if t_a <= t < t_b]
    if not r:
        return float("nan"), ""
    peak = max((p - base for _, p in r), key=abs)
    cells = []
    t = t_a
    while t < t_b:
        seg = [p - base for tt, p in r if t <= tt < t + 0.1]
        cells.append(math.degrees(max(seg, key=abs)) if seg else 0.0)
        t += 0.1
    lut = " .:-=+*#%@"
    shape = "".join(lut[min(int(abs(c) / 0.3), 9)] for c in cells)
    return math.degrees(peak), shape


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("accels", nargs="+", type=float, help="要测的加速度上限 m/s^2")
    ap.add_argument("--vmax", type=float, default=0.45, help="阶跃的目标速度 m/s")
    ap.add_argument("--hold", type=float, default=0.8, help="每个阶跃维持多久 s")
    ap.add_argument("--abort-deg", type=float, default=6.0, help="抬头超过这个角度立刻停")
    ap.add_argument("--topic", default="/camera/camera/vio_100hz")
    ap.add_argument("--no-app-restart", action="store_true")
    a = ap.parse_args()

    print(f"停 app 以独占串口 …")
    subprocess.run(["bash", _APP, "stop"], capture_output=True)
    subprocess.run(["pkill", "-f", "[d]iffcar_control.py"], capture_output=True)
    time.sleep(2.0)
    from carlib import Car                      # noqa: E402  停完 app 再开串口
    car = Car()
    time.sleep(0.3)

    rclpy.init()
    node = Pitch(a.topic)
    t_wait = time.monotonic() + 5.0
    while time.monotonic() < t_wait and len(node.rows) < 50:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(node.rows) < 50:
        print(f"❌ {a.topic} 收不到姿态，先确认相机固件在跑（ros2 topic hz）")
        return
    print(f"姿态 {node.hz():.0f} Hz")

    car.send("c 0")
    time.sleep(0.3)
    car.drain()
    base = float(np.median([p for _, p in node.rows[-100:]]))
    print(f"静止基线俯仰 {math.degrees(base):+.2f}°（抬头为正）")
    print(f"超限门限 ±{a.abort_deg:.1f}°；每格 0.1 s，字符越深抬头越多（一格 0.3°）\n")
    print(" 加速度 | 方向 | 实测加速度 | 俯仰峰值 |  俯仰形状（阶跃开始 -> 之后 1.5 s）")
    aborted = False
    try:
        for acc in a.accels:
            car.ask("a %.2f" % acc, 0.4)
            for sign, tag in ((+1.0, "前进"), (-1.0, "倒车刹停")):
                spin(node, car, 1.5, 0.0, [], base, math.radians(90))   # 先静置
                t_a = node.rows[-1][0]
                sp: list[tuple[float, float]] = []
                ok = spin(node, car, a.hold, sign * a.vmax, sp, base, math.radians(a.abort_deg))
                car.send("u 0 0")
                ok2 = spin(node, car, 1.5, 0.0, sp, base, math.radians(a.abort_deg))
                t_b = node.rows[-1][0]
                # 实测加速度：对**爬升段**做最小二乘，不能取单点 dv/dt —— 20 Hz 采样下
                # 一个样本跳 0.07 m/s 就等于 1.4 m/s^2，噪声完全盖过真值（设 0.8 报出 1.47）。
                acc_m = float("nan")
                if len(sp) > 5:
                    tt = np.array([x[0] for x in sp]); vv = np.array([x[1] for x in sp])
                    tt -= tt[0]
                    tgt = a.vmax
                    av = np.abs(vv)
                    # 只取**爬升段**：掩码同时选中加速和减速时两段斜率互相抵消，实测被算成 0.01
                    top = int(np.argmax(av))
                    tt, av = tt[:top + 1], av[:top + 1]
                    m = (av > 0.10 * tgt) & (av < 0.90 * tgt)
                    if m.sum() >= 3 and float(tt[m].max() - tt[m].min()) > 0.05:
                        acc_m = abs(float(np.polyfit(tt[m], av[m], 1)[0]))
                peak, shape = profile(node, t_a, t_b, base)
                flag = "" if (ok and ok2) else "   🛑 超限已急停"
                print(f"  {acc:5.2f} | {tag:<8s} | {acc_m:8.2f}   | {peak:+7.2f}° | {shape}{flag}")
                if not (ok and ok2):
                    aborted = True
                    break
            if aborted:
                break
    finally:
        car.send("u 0 0")
        time.sleep(0.2)
        car.ask("a 1.50", 0.4)      # 恢复默认，**不敲 w**，不写进 flash
        car.send("s")
        node.destroy_node()
        rclpy.shutdown()
        print("\n固件加速度已恢复 1.50（没写 flash）")
        if not a.no_app_restart:
            print("起回 app …")
            subprocess.Popen(["bash", _APP, "start", "a"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True)
            print("app 正在起，约 30 s 后网页可用")


if __name__ == "__main__":
    main()
