#!/usr/bin/env python3
"""把各节点的 LAT 行拼成一张全链延迟表。

能相减的前提（不成立就全错）：depth 的 header.stamp 从相机一路原样传到
/planning/trajectory_path（bridge 的 relabel_depth 和 planning 的 path.header.stamp
都只是转抄），而四个节点跑在同一块板、同一个时钟上。所以每个 `now − 那个戳`
都是「从相机采集到此刻」，相邻两个相减就是那一跳。

位姿是另一条链：diffcar_control 在【发布时】打 now，所以 plan 的 p_in 不含
任何传感器延迟；编码器读数的真实年龄要看 diffcar 的 serial_rt。

只用 stdlib，板上和 PC 上都能跑。
"""
import argparse
import os
import re
import sys
import unicodedata

_TS = re.compile(r"^\[[A-Z]+\] \[(\d+\.\d+)\]")
_RE = re.compile(r"LAT (\w+) (.+)$")
_ENV = re.compile(r"LATENV (.+)$")
# skew 会是负数，所以每个数字都要允许前导 -
_RUN = re.compile(r"^\d{4}(_\d{2}){5}$")
_KV = re.compile(r"(\w+)=(-?[\d.]+)/(-?[\d.]+)/(-?[\d.]+)\((\d+)\)")


def collect(paths):
    """返回 [(t, tag, key, p50, p90, max, n)]，t 是 ROS 日志时间戳。"""
    out = []
    for path in paths:
        try:
            with open(path, errors="replace") as fh:
                for line in fh:
                    m = _RE.search(line)
                    if not m:
                        continue
                    ts = _TS.match(line)
                    t = float(ts.group(1)) if ts else None
                    for k, p50, p90, mx, n in _KV.findall(m.group(2)):
                        out.append((t, m.group(1), k, float(p50), float(p90),
                                    float(mx), int(n)))
        except OSError as exc:
            print(f"跳过 {path}: {exc}", file=sys.stderr)
    return out


def collect_env(paths):
    """LATENV 行：这一趟的工况（浏览器开着什么、负载）。"""
    out = []
    for path in paths:
        try:
            with open(path, errors="replace") as fh:
                for line in fh:
                    m = _ENV.search(line)
                    if not m:
                        continue
                    ts = _TS.match(line)
                    out.append((float(ts.group(1)) if ts else None, m.group(1).strip()))
        except OSError:
            pass
    return out


def window(rows):
    """取所有 tag 都有数据的交集时间窗。

    🔴 不做这一步会给出 10 倍量级的错数：常驻节点的日志从 board-app 启动就开始，
    而 cmd_vel_control 只在 enable nav 之后才有 —— 中位数把空闲窗口和导航窗口
    混在一起，相减出来的"某一跳"其实是两个不同工况的差。
    """
    span = {}
    for t, tag, *_ in rows:
        if t is None:
            continue
        lo, hi = span.get(tag, (t, t))
        span[tag] = (min(lo, t), max(hi, t))
    if not span:
        return rows, None
    lo = max(v[0] for v in span.values())
    hi = min(v[1] for v in span.values())
    if lo > hi:
        return rows, None
    return [r for r in rows if r[0] is None or lo - 1e-6 <= r[0] <= hi + 1e-6], (lo, hi, span)


def fold(rows):
    """折成 {tag: {key: (p50 列表, p90 列表, max, n 合计)}}。"""
    acc = {}
    for _t, tag, k, p50, p90, mx, n in rows:
        e = acc.setdefault(tag, {}).setdefault(k, [[], [], 0.0, 0])
        e[0].append(p50)
        e[1].append(p90)
        e[2] = max(e[2], mx)
        e[3] += n
    return acc


def pad(t, w):
    """按显示宽度补空格：中日韩字符占两列，ljust 会算成一列。"""
    n = sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in t)
    return t + " " * max(0, w - n)


def med(v):
    if not v:
        return None
    v = sorted(v)
    return v[len(v) // 2]


def get(acc, tag, key, idx=0):
    e = acc.get(tag, {}).get(key)
    return None if e is None else med(e[idx])


def fmt(v, unit="ms"):
    if v is None:
        return "  --  "
    return f"{v * 1000:6.0f}" if unit == "ms" else f"{v:6.3f}"


def chain(acc):
    """返回 (表格行, 到 /cmd_vel 的累计, diffcar 等待)。"""
    rows_tbl, c_use, d_wait = chain(acc)
    rt = get(acc, "diffcar", "serial_rt")
    rows = rows_tbl
    print(pad('跳', 34) + f"{'本跳':>6}{'累计':>8}   来源")
    print("-" * 78)
    for name, one, cum, src in rows:
        print(pad(name, 34) + f"{fmt(one)}{fmt(cum)}   {src}")

    print("-" * 78)
    print(pad('↑ 到 /cmd_vel 发出为止', 34) + f"{'':>6}{fmt(c_use)}   ctrl used")
    # Twist 没有 header，戳链断在这里 —— 只能把本地等待时长【加】上去，不能相减
    print(pad('/cmd_vel 躺在 diffcar 里（相加）', 34)
          + f"{fmt(d_wait)}{fmt((c_use or 0) + (d_wait or 0))}   diffcar cmd_wait")
    print()

    print("旁路：位姿这条链（planning 的 pose_age 量的是它，不是深度）")
    rxw = get(acc, "diffcar", "rx_wait")
    print("  " + pad("串口往返 serial_rt", 30) + f"{fmt(rt)}   （减 50ms vMeas 低通群延迟才是纯往返）")
    print("  " + pad("回复在缓冲里躺 rx_wait", 30) + f"{fmt(rxw)}   （下一拍才 drain 到）")
    print("  " + pad("→ 位姿内容的真实年龄", 30)
          + f"{fmt(None if rt is None or rxw is None else rt + rxw)}"
          + "   戳打的是 now，所以这段延迟对下游完全不可见")
    print(f"  位姿到 planning 的传输 plan p_in  {fmt(get(acc, 'plan', 'p_in'))}"
          f"   （戳是发布时打的 now，所以这是纯 ROS 侧延迟）")
    print(f"  深度比位姿老多少      plan skew  {fmt(get(acc, 'plan', 'skew'))}")
    print()

    print("其他")
    for tag, key, label in (("ctrl", "drop", "被 5Hz 限速丢掉的轨迹条数"),
                            ("ctrl", "loop", "控制环周期"),
                            ("diffcar", "tick", "diffcar tick 周期")):
        e = acc.get(tag, {}).get(key)
        if e is None:
            continue
        if key == "drop":
            print("  " + pad(label, 34) + f"{e[3]:>6} 条")
        else:
            print("  " + pad(label, 34)
                  + f"{fmt(med(e[0]))}  (p90 {fmt(med(e[1]))} max {fmt(e[2])})")

    missing = [n for n, v in (("bridge d_in", b_in), ("plan d_in", p_in),
                              ("ctrl p_in", c_in), ("diffcar cmd_wait", d_wait))
               if v is None]
    if missing:
        print(f"\n⚠️ 缺 {', '.join(missing)}，对应的跳算不出来。"
              "ctrl 只在 enable nav 之后才有节点；diffcar 的 cmd_wait 要有 /cmd_vel 才有数。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
