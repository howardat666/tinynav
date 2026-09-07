#!/usr/bin/env python3
"""指令 / 编码器实测 / VIO 三路速度的同步采集与分析。

四个环节里 A（发出的指令）、C（编码器实测）、D（VIO 算的）都是话题上现成的量，
而 B（车对地的真实速度）没有任何传感器直接看得到 —— 这个脚本的全部意义是把三条
可观测的曲线对齐到同一时间轴上，从中反推出 A→B 链路的三个数：

    滞后    A 和 C 的互相关峰值（减去固件低通自带的约 50 ms）
    增益    C = k·A + b 的最小二乘拟合，k≠1 就是稳态执行误差
    死区    小幅阶梯输入下 |C| 仍为零的那一档

只订阅、不发布（`--drive none`），所以跑导航时可以全程后台开着。
`--drive step` 会自己发一段阶跃序列做离线自测：路径 < 1.5 m 且首尾回到原点。
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# BEST_EFFORT 的订阅端和 RELIABLE / BEST_EFFORT 的发布端都兼容，反过来不成立。
# 这里三条话题的发布端 QoS 各不相同，统一放宽是唯一不会静默收不到的写法。
_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)

# 固件的 vMeas 是 α=0.5 / 20 Hz 的一阶低通，群延迟约一个控制周期。互相关量到的滞后
# 里含这一份，报告时要减掉，否则会把滤波器的滞后算成系统的。
FW_FILTER_LAG_S = 0.050


def yaw_of(q) -> float:
    """相机光学位姿的航向。机体 +z 是前向，世界系 z 朝上 —— 与 cmd_vel_control 的
    `atan2(R[1,2], R[0,2])` 是同一个式子，换个写法会和控制器悄悄差个符号。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    fx = 2.0 * (x * z + y * w)
    fy = 2.0 * (y * z - x * w)
    return math.atan2(fy, fx)


class Probe(Node):
    # 边收边写，不攒在内存里等退出时 dump：一次导航可能跑十几分钟，进程被 kill、板子重启、
    # 网断，攒着的数据就全丢了 —— 而这些恰好正是最想事后分析的情况。
    _STREAMS = {
        "cmd": ["t", "v_cmd", "w_cmd"],
        "teleop": ["t", "v_cmd", "w_cmd"],
        "odom": ["t", "v_meas", "w_meas", "x", "y", "theta"],
        "vio": ["t", "x", "y", "yaw"],
    }

    def __init__(self, args, prefix: str):
        super().__init__("abcd_probe")
        self._files, self._wr = {}, {}
        for name, head in self._STREAMS.items():
            f = open(f"{prefix}_{name}.csv", "w", newline="")
            w = csv.writer(f)
            w.writerow(head)
            self._files[name], self._wr[name] = f, w
        self._n = dict.fromkeys(self._STREAMS, 0)
        self._flush_at = 0.0
        self.t0 = time.monotonic()
        self.create_subscription(Twist, args.cmd_topic, self._on_cmd, _QOS)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, _QOS)
        self.create_subscription(PoseStamped, args.vio_topic, self._on_vio, _QOS)
        if args.teleop_topic:
            self.create_subscription(Twist, args.teleop_topic, self._on_teleop, _QOS)
        self.pub = None
        if args.drive != "none":
            self.pub = self.create_publisher(Twist, args.cmd_topic, 10)
            self.plan = build_plan(args.drive)
            self.create_timer(0.05, self._drive)
            self.get_logger().info(
                f"drive: {len(self.plan)} segments, {sum(d for d, _, _ in self.plan):.1f} s"
            )
        else:
            self.get_logger().info(f"listening only -> {prefix}_*.csv")

    def _t(self) -> float:
        return time.monotonic() - self.t0

    def _row(self, name: str, row) -> None:
        self._wr[name].writerow(row)
        self._n[name] += 1
        t = self._t()
        if t > self._flush_at:
            self._flush_at = t + 2.0
            for f in self._files.values():
                f.flush()

    def _on_cmd(self, m: Twist) -> None:
        self._row("cmd", (self._t(), m.linear.x, m.angular.z))

    def _on_teleop(self, m: Twist) -> None:
        self._row("teleop", (self._t(), m.linear.x, m.angular.z))

    def _on_odom(self, m: Odometry) -> None:
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self._row("odom", (self._t(), m.twist.twist.linear.x, m.twist.twist.angular.z,
                           p.x, p.y, 2.0 * math.atan2(o.z, o.w)))

    def _on_vio(self, m: PoseStamped) -> None:
        p = m.pose.position
        self._row("vio", (self._t(), p.x, p.y, yaw_of(m.pose.orientation)))

    def _drive(self) -> None:
        t, acc = self._t(), 0.0
        for dur, v, w in self.plan:
            if t < acc + dur:
                msg = Twist()
                msg.linear.x, msg.angular.z = v, w
                self.pub.publish(msg)
                return
            acc += dur
        self.pub.publish(Twist())          # 计划走完，保持零速直到主循环退出

    def close(self, prefix: str) -> None:
        for name, f in self._files.items():
            f.flush()
            f.close()
            print(f"{prefix}_{name}.csv  {self._n[name]} rows")


def build_plan(kind: str) -> list[tuple[float, float, float]]:
    """(时长, v, w)。每个动作都配一个等量反向的伙伴，所以整段结束回到原点。
    阶跃和阶梯拆成两次跑，是为了让**每一次**的总路径都留在 1.5 m 以内：
    合在一起是 1.7 m，而车是在 3x3 m 空地正中心起步的。"""
    plan: list[tuple[float, float, float]] = [(3.0, 0.0, 0.0)]
    if kind == "step":                               # 滞后和增益：要干净的阶跃
        for v, dur in ((0.20, 1.5), (0.30, 1.2)):    # 直行 0.30+0.36 m，往返共 1.32 m
            plan += [(dur, v, 0.0), (2.0, 0.0, 0.0), (dur, -v, 0.0), (2.0, 0.0, 0.0)]
        for w, dur in ((0.60, 1.5), (0.30, 1.5)):    # 原地转，不产生位移
            plan += [(dur, 0.0, w), (2.0, 0.0, 0.0), (dur, 0.0, -w), (2.0, 0.0, 0.0)]
    elif kind == "spin":
        # 纯原地转，位移理论为零 —— 1x1 m 空地只够这么测。用来让 VIO 独立确认
        # "静摩擦击穿之后小角速度真的转了"，编码器自己说了不算。
        for w in (0.10, 0.15, 0.30, 0.60):
            plan += [(2.0, 0.0, w), (1.5, 0.0, 0.0), (2.0, 0.0, -w), (1.5, 0.0, 0.0)]
    elif kind == "stick":
        # 静摩擦 vs 死区的判别：先用 0.60 把车转起来，**不回零**直接降到小角速度。
        # 若小角速度此时能跟上，说明门限在"从静止起步"这一步，不在角速度本身。
        for sign in (1.0, -1.0):
            plan += [(1.5, 0.0, sign * 0.60), (2.5, 0.0, sign * 0.15),
                     (2.0, 0.0, sign * 0.30), (2.0, 0.0, 0.0)]
        # 对照：同样的 0.15 / 0.30，但每次都从静止起步
        for sign in (1.0, -1.0):
            plan += [(2.5, 0.0, sign * 0.15), (1.5, 0.0, 0.0),
                     (2.0, 0.0, sign * 0.30), (1.5, 0.0, 0.0)]
        # 带前进的小角速度：轮子在滚，静摩擦本来就已经被打破
        for sign in (1.0, -1.0):
            plan += [(2.0, 0.10, sign * 0.15), (1.5, 0.0, 0.0)]
    else:                                            # 死区：每档必须保持够久才到稳态
        for i, w in enumerate((0.05, 0.10, 0.15, 0.20, 0.30)):
            s = 1.0 if i % 2 == 0 else -1.0
            plan += [(1.6, 0.0, s * w), (1.0, 0.0, 0.0)]
        for i, v in enumerate((0.02, 0.04, 0.06, 0.10, 0.15)):
            s = 1.0 if i % 2 == 0 else -1.0
            plan += [(1.4, s * v, 0.0), (1.0, 0.0, 0.0)]   # 合计 0.52 m，正负相消
    plan += [(2.0, 0.0, 0.0)]
    return plan


# ---------------------------------------------------------------- 分析

def grid(t: np.ndarray, y: np.ndarray, tg: np.ndarray) -> np.ndarray:
    return np.interp(tg, t, y)


def best_lag(a: np.ndarray, b: np.ndarray, dt: float, lo=-0.2, hi=1.5) -> tuple[float, float]:
    """b 相对 a 的滞后。零均值化后归一化互相关，峰值附近抛物线插值取亚采样精度。"""
    a = a - a.mean()
    b = b - b.mean()
    lags = np.arange(int(lo / dt), int(hi / dt) + 1)
    best, out = -2.0, 0.0
    scores = []
    for k in lags:
        aa = a[:-k] if k > 0 else (a[-k:] if k < 0 else a)
        bb = b[k:] if k > 0 else (b[:len(b) + k] if k < 0 else b)
        n = min(len(aa), len(bb))
        if n < 20:
            scores.append(-2.0)
            continue
        d = np.linalg.norm(aa[:n]) * np.linalg.norm(bb[:n])
        scores.append(float(aa[:n] @ bb[:n] / d) if d > 0 else -2.0)
    scores = np.asarray(scores)
    i = int(np.argmax(scores))
    best, out = scores[i], lags[i] * dt
    if 0 < i < len(scores) - 1:                    # 抛物线顶点，分辨率不再被 dt 卡住
        y0, y1, y2 = scores[i - 1], scores[i], scores[i + 1]
        den = y0 - 2 * y1 + y2
        if den != 0:
            out += 0.5 * (y0 - y2) / den * dt
    return out, float(best)


def steady_mask(a: np.ndarray, dt: float, settle: float, pre: float = 0.15) -> np.ndarray:
    """指令保持不变已超过 settle 秒、且距下一次变化还有 pre 秒的样本。

    两侧都要挡。只挡上升段的话，阶跃的**结尾**会被 VIO 差分那个 0.2 s 滑窗把减速段
    平均进来 —— 实测把 C/D 直行从 1.006 拉到 1.034，会被误读成 3.4% 的打滑。"""
    change = np.flatnonzero(np.abs(np.diff(a, prepend=a[0])) > 1e-6)
    m = np.ones(len(a), dtype=bool)
    n, q = int(settle / dt), int(pre / dt)
    for i in change:
        m[max(0, i - q):i + n] = False
    return m


def segments(a: np.ndarray, dt: float, settle: float, pre: float = 0.15):
    """逐个恒定指令段。比一条回归线可信得多：回归会把不同幅度、不同方向的样本数
    不均衡揉进斜率，而每一段的稳态均值就是那一档的增益本身。"""
    change = [0] + list(np.flatnonzero(np.abs(np.diff(a, prepend=a[0])) > 1e-6)) + [len(a)]
    for i in range(len(change) - 1):
        s, e = change[i] + int(settle / dt), change[i + 1] - int(pre / dt)
        if e - s >= 10:
            yield s, e, float(a[change[i]])


def fit(a: np.ndarray, b: np.ndarray, extra=None) -> tuple[float, float, float]:
    """b = k·a + c 的最小二乘，只用 |a| 明显非零的样本 —— 零速段占了大半时间，
    带进去会把斜率往 1 拽（两边都是 0 的点完美落在任何过原点的线上）。"""
    m = np.abs(a) > 0.02
    if extra is not None:
        m &= extra
    if m.sum() < 20:
        return float("nan"), float("nan"), 0.0
    k, c = np.polyfit(a[m], b[m], 1)
    r = np.corrcoef(a[m], b[m])[0, 1]
    return float(k), float(c), float(r)


def load(path: str):
    """空文件（只有表头）在 genfromtxt 里会变成 0 维，统一成 None。"""
    # 空文件、只有表头、一两行 —— genfromtxt 对这三种各有一种不同的炸法，一律当没有
    try:
        a = np.genfromtxt(path, delimiter=",", names=True)
    except Exception:
        return None
    return a if a is not None and a.ndim == 1 and len(a) > 2 else None


def effective_cmd(cmd, teleop, tg: np.ndarray, prio: float = 0.4):
    """执行器实际执行的指令。遥控在 prio 秒窗口内完全接管导航（见 diffcar_control
    的 teleop_priority_s），只看 /cmd_vel 会把人推摇杆那段错算成"导航指令没被执行"。"""
    v = zoh(cmd["t"], cmd["v_cmd"], tg)
    w = zoh(cmd["t"], cmd["w_cmd"], tg)
    if teleop is None:
        return v, w
    fresh = (tg - zoh(teleop["t"], teleop["t"], tg)) <= prio
    v = np.where(fresh, zoh(teleop["t"], teleop["v_cmd"], tg), v)
    w = np.where(fresh, zoh(teleop["t"], teleop["w_cmd"], tg), w)
    print(f"  遥控接管了 {100.0 * fresh.mean():.1f}% 的时间（已按仲裁规则合并）")
    return v, w


def zoh(t: np.ndarray, y: np.ndarray, tg: np.ndarray) -> np.ndarray:
    """零阶保持。指令是阶梯量，线性插值会在阶跃边界插出不存在的中间值，段检测会被骗。"""
    return y[np.clip(np.searchsorted(t, tg, side="right") - 1, 0, len(y) - 1)]


def health(A: np.ndarray, C: np.ndarray, D: np.ndarray, dt: float, tag: str,
           zero: float, stuck_at: float) -> None:
    """导航工况没有恒定段，逐段表用不上。这里问的是另外三个问题：
    指令有多少时间是零、非零的时候车动没动、动起来之后 C 和 D 差多少。"""
    nz = np.abs(A) > zero
    print(f"  {tag}: 非零指令占时 {100.0 * nz.mean():5.1f}%", end="")
    if nz.sum() < 10:
        print("   —— 样本太少，不判")
        return
    stuck = nz & (np.abs(C) < stuck_at)
    # 最长连续卡住时长：比总占比更能说明问题，零星几拍是噪声，连续 1 s 是真事
    longest, cur = 0, 0
    for f in stuck:
        cur = cur + 1 if f else 0
        longest = max(longest, cur)
    longest *= dt
    print(f"，其中「指令非零但没动」{100.0 * stuck.sum() / nz.sum():5.1f}%"
          f"（最长连续 {longest:.2f} s）")
    # 打滑要看**累计行程**的比值，不能看瞬时比值：顿-滑工况下 C 和 D 都在零附近抖，
    # 两个近零数相除会炸出 p10=0.23 / p90=2.73 这种毫无意义的分布（实测栽过）。
    if nz.sum() >= 20:
        ic, id_ = np.abs(C[nz]).sum() * dt, np.abs(D[nz]).sum() * dt
        print(f"        累计行程 C={ic:.3f} D={id_:.3f}  C/D={ic / id_ if id_ > 1e-6 else float('nan'):.3f}"
              f"   （标定期望 1.006±0.018，明显偏高＝打滑）")
        # 2 s 滑窗，找"这一段确实在滑"的时刻，而不是整程平均把它稀释掉
        n = int(2.0 / dt)
        if nz.sum() > n:
            k = np.ones(n)
            wc, wd = np.convolve(np.abs(C), k, "same"), np.convolve(np.abs(D), k, "same")
            ok = (wd > 0.2 / dt * 0.01) & nz
            if ok.sum() >= n:
                q = np.percentile(wc[ok] / wd[ok], [50, 90, 99])
                print(f"        2s 滑窗 C/D  p50={q[0]:.3f} p90={q[1]:.3f} p99={q[2]:.3f}")


def analyze(prefix: str) -> None:
    cmd = load(f"{prefix}_cmd.csv")
    odo = load(f"{prefix}_odom.csv")
    vio = load(f"{prefix}_vio.csv")
    teleop = load(f"{prefix}_teleop.csv")
    if cmd is None or odo is None or vio is None:
        print("cmd / odom / vio 至少一条是空的，没法分析")
        return
    print(f"样本: cmd {len(cmd)} ({len(cmd)/(cmd['t'][-1]-cmd['t'][0]):.1f} Hz)  "
          f"odom {len(odo)} ({len(odo)/(odo['t'][-1]-odo['t'][0]):.1f} Hz)  "
          f"vio {len(vio)} ({len(vio)/(vio['t'][-1]-vio['t'][0]):.1f} Hz)")

    dt = 0.01
    t_lo = max(cmd["t"][0], odo["t"][0], vio["t"][0])
    t_hi = min(cmd["t"][-1], odo["t"][-1], vio["t"][-1])
    tg = np.arange(t_lo, t_hi, dt)

    A_v, A_w = effective_cmd(cmd, teleop, tg)
    C_v, C_w = grid(odo["t"], odo["v_meas"], tg), grid(odo["t"], odo["w_meas"], tg)

    # VIO 只有位姿，速度靠差分。0.2 s 的中心差分：短了全是抖动，长了把阶跃抹平。
    win = 0.2
    xv, yv = grid(vio["t"], vio["x"], tg), grid(vio["t"], vio["y"], tg)
    # unwrap 必须在插值之前。VIO 的 yaw 实测跨越 ±π（−3.13~+3.13），先插值会把那个
    # 跳变插成一条穿过 0 的斜坡，偏航那一路的相关系数直接掉到 −0.003。
    psi = grid(vio["t"], np.unwrap(vio["yaw"]), tg)
    n = int(win / dt)
    D_w = np.gradient(psi, dt)
    dx, dy = np.gradient(xv, dt), np.gradient(yv, dt)
    D_v = dx * np.cos(psi) + dy * np.sin(psi)
    ker = np.ones(n) / n
    D_v, D_w = np.convolve(D_v, ker, "same"), np.convolve(D_w, ker, "same")

    print(f"\n=== 执行健康度（时长 {t_hi - t_lo:.0f} s） ===")
    health(A_v, C_v, D_v, dt, "直行", 0.02, 0.01)
    health(A_w, C_w, D_w, dt, "偏航", 0.05, 0.02)

    print("\n=== 滞后（互相关峰，已减固件低通 50 ms） ===")
    for tag, a, b in (("A->C 直行", A_v, C_v), ("A->C 偏航", A_w, C_w),
                      ("A->D 直行", A_v, D_v), ("A->D 偏航", A_w, D_w)):
        if a.std() < 1e-4:                 # 这一路整程没动过，互相关只会输出噪声峰
            print(f"  {tag}: 指令全程恒定，跳过")
            continue
        lag, r = best_lag(a, b, dt)
        extra = FW_FILTER_LAG_S if "C" in tag else 0.0
        flag = "" if r > 0.5 else "   ⚠️ 相关太弱，这个数不可信"
        print(f"  {tag}: {lag - extra:+.3f} s   (峰值相关 {r:.3f}, 原始 {lag:+.3f}){flag}")

    settle = 0.9
    print(f"\n=== 增益（对齐滞后、只取指令恒定 {settle} s 之后的稳态样本） ===")
    for tag, a, b in (("C/A 直行", A_v, C_v), ("C/A 偏航", A_w, C_w),
                      ("D/A 直行", A_v, D_v), ("D/A 偏航", A_w, D_w),
                      ("C/D 直行", D_v, C_v), ("C/D 偏航", D_w, C_w)):
        lag, _ = best_lag(a, b, dt)
        k = int(round(lag / dt))
        aa, bb = (a[:-k], b[k:]) if k > 0 else (a, b)
        # 掩码按「命令」那一路算，再跟着同样的位移切；C/D 两路都不是命令，用 A 的
        src = A_v if "直行" in tag else A_w
        sm = steady_mask(src, dt, settle)
        sm = sm[:-k] if k > 0 else sm
        kk, c, r = fit(np.asarray(aa), np.asarray(bb), sm[:len(aa)])
        print(f"  {tag}: k={kk:.4f}  截距={c:+.4f}  r={r:.4f}")

    # 「增益」这一列才是主读数：它是每个幅度档各自的稳态实测/指令，比单条回归线更硬 ——
    # 回归会把死区（低档位输出为零）和不同档位的样本数不均衡都揉进斜率里。
    print(f"\n=== 逐段稳态（每段跳过头 {settle} s 和尾 0.15 s） ===")
    for tag, a, cc, dd in (("直行 m/s", A_v, C_v, D_v), ("偏航 rad/s", A_w, C_w, D_w)):
        print(f"  {tag}:  指令      C(编码器)  D(VIO)    C/A     D/A     C/D")
        for s, e, val in segments(a, dt, settle):
            if abs(val) < 1e-6:
                continue
            c_, d_ = cc[s:e].mean(), dd[s:e].mean()
            print(f"    {'':12s}{val:+.3f}   {c_:+.4f}   {d_:+.4f}   "
                  f"{c_/val:.4f}  {d_/val:.4f}  {c_/d_ if abs(d_) > 1e-6 else float('nan'):.4f}"
                  f"   n={e-s}")

    print("\n=== 分档稳态：死区看低档位是否为零，增益看各档是否一致 ===")
    for tag, a, b in (("直行 m/s", A_v, C_v), ("偏航 rad/s", A_w, C_w)):
        print(f"  {tag}:")
        sm = steady_mask(a, dt, settle)
        mag = np.where(sm, np.abs(a), np.nan)
        for lo, hi in ((0.015, 0.03), (0.03, 0.05), (0.05, 0.08), (0.08, 0.12),
                       (0.12, 0.18), (0.18, 0.25), (0.25, 0.35), (0.35, 0.7)):
            m = (mag >= lo) & (mag < hi) & sm
            if m.sum() < 10:
                continue
            print(f"    指令 {lo:.3f}~{hi:.3f}: 实测 {np.abs(b[m]).mean():.4f}  "
                  f"增益 {np.abs(b[m]).mean()/mag[m].mean():.3f}  n={int(m.sum())}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--analyze", metavar="PREFIX")
    ap.add_argument("--drive", choices=["none", "step", "stair", "stick", "spin"], default="none")
    ap.add_argument("--duration", type=float, default=0.0, help="0 = 直到 Ctrl-C 或计划走完")
    ap.add_argument("--out", default="/root/car/abcd")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    # 遥控走自己的话题并在执行器那一层优先，所以只录 /cmd_vel 会把人推摇杆的那段
    # 错算成"导航发的指令没被执行"。空字符串＝不录。
    ap.add_argument("--teleop-topic", default="/teleop/cmd_vel")
    ap.add_argument("--odom-topic", default="/wheel/odometry")
    # 关掉固件 VIO 后这个话题不存在（vio_enabled=false 时故意不 advertise），
    # 订阅它只会安静产出空文件。后端按 TINYNAV_ODOM_SOURCE 传真实话题进来。
    ap.add_argument("--vio-topic", default="/camera/camera/vio_image")
    args = ap.parse_args()

    if args.analyze:
        analyze(args.analyze)
        return

    prefix = f"{args.out}_{time.strftime('%m%d_%H%M%S')}"
    os.makedirs(os.path.dirname(prefix) or ".", exist_ok=True)
    rclpy.init()
    node = Probe(args, prefix)
    limit = args.duration
    if args.drive != "none" and limit <= 0:
        limit = sum(d for d, _, _ in node.plan) + 1.0
    # SIGTERM 也要走 finally：跑导航时这个进程是被 kill 收掉的，默认行为会跳过收尾，
    # 驱动模式下就等于把车留在最后那条速度指令上。
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.1)
            if limit > 0 and node._t() > limit:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node.pub is not None:                   # 退出前必须亲手把速度清零
            for _ in range(5):
                node.pub.publish(Twist())
                time.sleep(0.02)
        node.close(prefix)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
