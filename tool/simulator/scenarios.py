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
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tool.simulator import x5_presets  # noqa: E402

BASE = "http://127.0.0.1:8766"


def api(path, payload=None, timeout=20):
    url = f"{BASE}{path}"
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(
        url, data=data, headers={"Content-Type": "application/json"} if data else {})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


SCENES = x5_presets.SCENES


def run(name, sc, poll_hz=5.0):
    # 场景由服务端按 x5_presets 那一份定义生成，且 load-scene 会把 planning 和 control
    # 都换新的。少了这个隔离，planning 带着上一个场景的障碍图、control 带着累积的路径
    # 参考，下一个场景会「单独跑过、连着跑不动」—— 看着像规划器的 bug，实际不是。
    cfg = api("/api/load-scene", {"name": name, "freeze": True})["config"]

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
        samples.append((time.monotonic() - t0, x, y, f["robot_yaw_deg"], vx, wz, d))
        if d < 0.4 and reached_at is None:
            reached_at = time.monotonic() - t0
            break
        time.sleep(1.0 / poll_hz)

    if not samples:
        return dict(name=name, ok=False, note="仿真器没有回应")
    # 变向：wz 的符号翻转（忽略接近零的）
    signs = [math.copysign(1, w) for _, _, _, _, _, w, _ in samples if abs(w) > 0.05]
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    stalled = sum(1 for s in samples if abs(s[4]) < 0.01 and abs(s[5]) < 0.05)
    # rev=OFF 时跟踪律反馈仍可能给出 -0.05 以内的小负值（v_cap=|v_ref|+slack），不算倒车
    revs = [s[4] for s in samples if s[4] < -0.06]
    path_len = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:]))
    ok = (reached_at is not None) if sc["need_reach"] else True
    # 变向次数是摆头的判据，每个场景给的上限不同（见 x5_presets.SCENES）。
    ok = ok and flips <= int(sc.get("max_flips", 6))
    # 倒车默认关，所以任何真正的倒车指令都是问题（-0.06 以内是跟踪律的反馈余量，不算）
    ok = ok and len(revs) == 0
    return dict(name=name, ok=ok, reached_at=reached_at, flips=flips,
                stalled_frac=stalled / len(samples), n=len(samples),
                path_len=path_len, reverse_cmds=len(revs),
                final_d=samples[-1][6])


def main():
    global BASE
    ap = argparse.ArgumentParser()
    ap.add_argument("only", nargs="*", help="只跑这些场景")
    ap.add_argument("--base", default=BASE)
    a = ap.parse_args()
    BASE = a.base
    all_sc = SCENES
    todo = a.only or list(all_sc)
    bad = 0
    print(f"{'场景':<12} {'到达':>7} {'变向':>4} {'静止占比':>8} {'路程':>6} {'倒车指令':>8} {'末距':>6}  结果")
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
        print(f"{name:<12} {reach:>7} {r['flips']:>4} {r['stalled_frac']*100:>7.0f}% "
              f"{r['path_len']:>5.2f}m {r['reverse_cmds']:>8} {r['final_d']:>5.2f}m  "
              f"{'OK' if r['ok'] else '**FAIL**'}")
    print("全部通过" if not bad else f"{bad} 个场景不通过", flush=True)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
