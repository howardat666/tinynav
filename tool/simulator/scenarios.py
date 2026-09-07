#!/usr/bin/env python3
"""无头跑 planning lab 的固定场景，打印判据。

用途是把「改一版 → 推板子 → 跑一趟 → 读日志」这条 20 分钟的循环换成几十秒。
只用标准库，通过仿真器的 HTTP 接口驱动它（默认 127.0.0.1:8766），所以在宿主机上跑、
仿真器在 docker 里跑也行。

    python3 tool/simulator/scenarios.py                # 全部场景
    python3 tool/simulator/scenarios.py wall_ahead     # 单个
"""
from __future__ import annotations

import argparse
import json
import os
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tool.simulator import x5_presets  # noqa: E402

# 端口跟着 TINYNAV_SIM_PORT 走：上游那份仿真也占 8766，两边同时开时我们的会挪到 8767，
# 写死会静默打到上游那份上去（它没有 /api/load-scene，报 404）。
BASE = os.environ.get("TINYNAV_SIM_BASE",
                      f'http://127.0.0.1:{os.environ.get("TINYNAV_SIM_PORT", "8766")}')
ACTUATOR = "perfect"
# (物理外壳半径, 圆心在控制点前多少米)。启动时从 /api/robot-presets 取，
# 取不到就退回发布出来的 footprint（旧行为）。
BODY = None


def api(path, payload=None, timeout=20):
    url = f"{BASE}{path}"
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(
        url, data=data, headers={"Content-Type": "application/json"} if data else {})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


SCENES = x5_presets.SCENES


def footprint_points(corners, n=5):
    """把车体四角双线性铺成 n x n 个点。和规划器碰撞检查的口径一致（铺满，不是只取角）。"""
    if len(corners) < 4:
        return []
    (ax, ay), (bx, by), (cx, cy), (dx, dy) = [tuple(p) for p in corners[:4]]
    pts = []
    for i in range(n):
        u = i / (n - 1)
        for j in range(n):
            v = j / (n - 1)
            # 沿 a->b 和 d->c 插值，再在两者之间插值
            px = (1 - v) * ((1 - u) * ax + u * bx) + v * ((1 - u) * dx + u * cx)
            py = (1 - v) * ((1 - u) * ay + u * by) + v * ((1 - u) * dy + u * cy)
            pts.append((px, py))
    return pts


def clearance_to_objects(corners, objects):
    """车体到最近障碍的距离，负值表示已经压进去了。仿真不做碰撞响应，所以"撞没撞"必须
    在这里自己判 —— 少了它，「指令和实际有差距会不会撞」这个问题根本量不出来。"""
    pts = footprint_points(corners)
    if not pts:
        return float("inf")
    worst = float("inf")
    for obj in objects:
        cx, cy, _ = obj["center"]
        sx, sy, _ = obj["size"]
        hx, hy = sx / 2.0, sy / 2.0
        for px, py in pts:
            dx = abs(px - cx) - hx
            dy = abs(py - cy) - hy
            if dx <= 0 and dy <= 0:
                d = max(dx, dy)              # 在盒子里，负的
            else:
                d = math.hypot(max(dx, 0.0), max(dy, 0.0))
            if d < worst:
                worst = d
    return worst


def body_circle_xy(frame, body):
    """按物理外壳画一圈点。body = (半径, 圆心在控制点前多少米)；None 表示没配，退回 footprint。"""
    if body is None:
        return None
    r, off = body
    x, y = frame.get("robot_xy") or (None, None)
    if x is None:
        return None
    yaw = math.radians(float(frame.get("robot_yaw_deg", 0.0)))
    cx, cy = x + math.cos(yaw) * off, y + math.sin(yaw) * off
    n = 24
    return [[cx + r * math.cos(2 * math.pi * i / n),
             cy + r * math.sin(2 * math.pi * i / n)] for i in range(n)]


def run(name, sc, poll_hz=5.0):
    # 场景由服务端按 x5_presets 那一份定义生成，且 load-scene 会把 planning 和 control
    # 都换新的。少了这个隔离，planning 带着上一个场景的障碍图、control 带着累积的路径
    # 参考，下一个场景会「单独跑过、连着跑不动」—— 看着像规划器的 bug，实际不是。
    cfg = api("/api/load-scene", {"name": name, "freeze": True,
                                  "actuator": ACTUATOR})["config"]

    # 等仿真环真的转起来再开始计时：planning_node 重启要付一次 numba 编译，把那段算进
    # 预算就会把「起得慢」误判成「走不动」。判据是它发出了非退化的轨迹。
    ready_deadline = time.monotonic() + 60.0
    ready = False
    while time.monotonic() < ready_deadline:
        try:
            f = api("/api/sim-state")["frame"]
        except (urllib.error.URLError, TimeoutError):
            time.sleep(0.5)
            continue
        if len(f.get("selected_trajectory_xy") or []) > 1:
            time.sleep(2.0)     # 再让它稳一拍：第一条轨迹往往是障碍图还空着时发的
            api("/api/freeze", {"frozen": False})   # 热完了再放行，用时和路程才可比
            ready = True
            break
        time.sleep(0.5)
    if not ready:
        return dict(name=name, ok=False, note="等了 60 s 规划器还没发出轨迹")

    tgt = cfg["target"]
    t0 = time.monotonic()
    samples = []
    reached_at = None
    while time.monotonic() - t0 < sc["budget_s"]:
        try:
            f = api("/api/sim-state")["frame"]
        except (urllib.error.URLError, TimeoutError):
            time.sleep(1.0 / poll_hz)
            continue
        x, y = f["robot_xy"]
        vx, wz = f["selected_param"]
        d = math.hypot(tgt[0] - x, tgt[1] - y)
        # 🔴 不用发布出来的 footprint —— 那画的是【扫掠圆】（半径 collision_radius、
        # 圆心在控制点）。驱动轴不过圆心时它比车身大，拿它判碰撞除正前方外处处高报。
        body = body_circle_xy(f, BODY)
        clr = clearance_to_objects(body or f.get("robot_footprint_xy") or [],
                                   cfg.get("objects") or [])
        samples.append((time.monotonic() - t0, x, y, f["robot_yaw_deg"], vx, wz, d, clr))
        if d < 0.4 and reached_at is None:
            reached_at = time.monotonic() - t0
            break
        time.sleep(1.0 / poll_hz)

    if not samples:
        return dict(name=name, ok=False, note="仿真器没有回应")
    # 变向：wz 的符号翻转（忽略接近零的）
    signs = [math.copysign(1, s[5]) for s in samples if abs(s[5]) > 0.05]
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    stalled = sum(1 for s in samples if abs(s[4]) < 0.01 and abs(s[5]) < 0.05)
    # rev=OFF 时跟踪律反馈仍可能给出 -0.05 以内的小负值（v_cap=|v_ref|+slack），不算倒车
    revs = [s[4] for s in samples if s[4] < -0.06]
    path_len = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:]))
    clears = [s[7] for s in samples if math.isfinite(s[7])]
    min_clear = min(clears) if clears else float("nan")
    hits = sum(1 for c in clears if c < 0.0)
    ok = (reached_at is not None) if sc["need_reach"] else True
    # 变向次数是摆头的判据，每个场景给的上限不同（见 x5_presets.SCENES）。
    ok = ok and flips <= int(sc.get("max_flips", 6))
    # 倒车开着时它是脱困手段而不是缺陷，判据换成「有没有压进障碍」（下一行）。
    # 关着时任何真正的倒车指令都是问题（-0.06 以内是跟踪律的反馈余量，不算）。
    if os.environ.get("TINYNAV_ALLOW_REVERSE", "0") != "1":
        ok = ok and len(revs) == 0
    ok = ok and hits == 0            # 压进障碍就是失败，不管到没到
    return dict(name=name, ok=ok, reached_at=reached_at, flips=flips,
                stalled_frac=stalled / len(samples), n=len(samples),
                path_len=path_len, reverse_cmds=len(revs),
                final_d=samples[-1][6], min_clear=min_clear, hits=hits)


def main():
    global BASE, ACTUATOR
    ap = argparse.ArgumentParser()
    ap.add_argument("only", nargs="*", help="只跑这些场景")
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--actuator", default="perfect",
                    help="执行误差模型：perfect / gain_10_20 / gain_20_40 / deadband / "
                         "latency_400ms / latency_800ms / realistic")
    a = ap.parse_args()
    BASE = a.base
    ACTUATOR = a.actuator
    global BODY
    try:
        pres = api("/api/robot-presets")["presets"]
        r = next(p["robot"] for p in pres if p["name"] == "diffcar")
        if r.get("body_radius"):
            BODY = (float(r["body_radius"]), float(r.get("body_offset_x", 0.0)))
    except Exception as exc:
        print(f"⚠️ 取不到物理外壳，碰撞判据退回发布的 footprint（会按扫掠圆高报）: {exc}")
    all_sc = SCENES
    todo = a.only or list(all_sc)
    bad = 0
    print(f"执行误差模型: {ACTUATOR}")
    print(f"碰撞判据: {'物理外壳 r=%.3f 圆心在控制点前 %.3f m' % BODY if BODY else '发布的 footprint(扫掠圆)'}")
    print(f"倒车: {'开' if os.environ.get('TINYNAV_ALLOW_REVERSE','0')=='1' else '关'}"
          f"（判据随之变：开着时倒车不算缺陷，只看有没有压进障碍）")
    print(f"{'场景':<12} {'到达':>7} {'变向':>4} {'倒车':>4} {'最小净空':>8} {'压进障碍':>8} {'路程':>6} {'末距':>6}  结果")
    for name in todo:
        if name not in all_sc:
            print(f"  未知场景 {name}（有：{', '.join(all_sc)}）")
            bad += 1
            continue
        print(f"  跑 {name} ...", flush=True)
        r = run(name, all_sc[name])
        if not r.get("n"):
            print(f"{name:<12}  {r.get('note')}", flush=True)
            bad += 1
            continue
        bad += not r["ok"]
        reach = f"{r['reached_at']:.1f}s" if r["reached_at"] is not None else "未到"
        print(f"{name:<12} {reach:>7} {r['flips']:>4} {r['reverse_cmds']:>4} {r['min_clear']:>7.3f}m "
              f"{r['hits']:>8} {r['path_len']:>5.2f}m {r['final_d']:>5.2f}m  "
              f"{'OK' if r['ok'] else '**FAIL**'}", flush=True)
    print("全部通过" if not bad else f"{bad} 个场景不通过", flush=True)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
