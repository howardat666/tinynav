#!/usr/bin/env python3
"""扫固件的静摩擦击穿 PWM（`F`），换地面后重跑这个就能重新定值。

纯原地转，位移理论为零，1x1 m 空地够用；直行那一段每档正反各一次，位移相消。
⚠️ 串口只有一个主人，跑之前必须 `app_start.sh stop` + `pkill -f "[d]iffcar_control.py"`，
否则 app 的 19 Hz 轮询会和这里抢串口，量出一堆看着像真的假数（栽过三次）。

2026-09-01 地毯实测：40 → 增益 0.85，**45 → 0.96**，50 → 1.08，59 → 超发到 2.09。
地砖上击穿门限只要 24~28，地毯要 45 —— 换地面必须重扫。
"""
import re, sys, time, statistics
sys.path.insert(0, '/root/car')
from carlib import Car

VL = re.compile(r"vl=(-?[\d.]+)\s+vr=(-?[\d.]+)")
PO = re.compile(r"x=(-?[\d.]+)\s+y=(-?[\d.]+)\s+theta=(-?[\d.]+)")
BASE = 0.2035


def leg(car, v, w, dur):
    out, t0, nxt = [], time.monotonic(), 0.0
    while True:
        t = time.monotonic() - t0
        if t > dur:
            break
        if t >= nxt:
            nxt += 0.05
            car.send("u %.3f %.3f" % (v, w))
            car.send("p")
        for ln in car.drain():
            m, p = VL.search(ln), PO.search(ln)
            if m and p:
                vl, vr = float(m.group(1)), float(m.group(2))
                out.append((t, (vl + vr) / 2.0, (vr - vl) / BASE))
        time.sleep(0.005)
    return out


def stats(rows, idx, cmd, tail=0.6):
    if not rows:
        return None, None
    tm = next((t for t, *r in rows if abs(r[idx]) > abs(cmd) * 0.15), None)
    end = rows[-1][0]
    vals = [r[idx] for t, *r in rows if t > end - tail]
    return tm, (statistics.fmean(vals) / cmd if vals else None)


def run(car, kind, cmd, dur=2.0):
    v, w = (0.0, cmd) if kind == "w" else (cmd, 0.0)
    rows = leg(car, v, w, dur)
    leg(car, 0.0, 0.0, 1.0)
    return stats(rows, 1 if kind == "w" else 0, cmd)


def main():
    car = Car()
    time.sleep(0.3)
    car.send("c 0")
    time.sleep(0.3)
    car.drain()

    print("=== 原地转（两次重复，正反向各一次） ===")
    print("击穿  w指令 |  起转    增益   起转    增益  | 增益几何均值")
    best = {}
    for brk in (40, 45, 50):
        gs = []
        for w in (0.10, 0.15, 0.30):
            r = [run(car, "w", s * w) for s in (1.0, -1.0)]
            f = lambda x: "  --- " if x[0] is None else "%.2fs" % x[0]
            g = lambda x: " --- " if x[1] is None else "%5.2f" % x[1]
            gs += [x[1] for x in r if x[1] and x[1] > 0.01]
            print("%4d  %5.2f | %s  %s  %s  %s" % (brk, w, f(r[0]), g(r[0]), f(r[1]), g(r[1])), end="")
            print()
        gm = 2.718281828 ** statistics.fmean([__import__('math').log(x) for x in gs]) if gs else 0
        best[brk] = gm
        print("     -> brk=%d 几何均值增益 %.2f  (n=%d)" % (brk, gm, len(gs)))
        car.ask("F %d" % brk, 0.3) if False else None
        if brk != 50:
            car.ask("F %d" % (brk + 5), 0.4)

    print("\n=== 直行低速（确认击穿没把它搞坏，每档正反各一次，位移相消） ===")
    car.ask("F 45", 0.4)
    print("v指令 |  起转    增益   起转    增益")
    for v in (0.02, 0.06, 0.10):
        r = [run(car, "v", s * v, 1.5) for s in (1.0, -1.0)]
        f = lambda x: "  --- " if x[0] is None else "%.2fs" % x[0]
        g = lambda x: " --- " if x[1] is None else "%5.2f" % x[1]
        print("%5.2f | %s  %s  %s  %s" % (v, f(r[0]), g(r[0]), f(r[1]), g(r[1])))

    car.send("u 0 0"); time.sleep(0.2); car.send("s")
    print("\nbrk 几何均值汇总:", {k: round(v, 2) for k, v in best.items()})
    for ln in car.ask("p", 0.5):
        if PO.search(ln):
            print("最终里程计:", ln)


if __name__ == "__main__":
    main()
