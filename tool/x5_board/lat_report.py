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
    """算出链路各跳。返回 (表格行, 到 /cmd_vel 的累计, diffcar 里的等待)。"""
    b_in, b_out = get(acc, "bridge", "d_in"), get(acc, "bridge", "d_out")
    p_in, p_out = get(acc, "plan", "d_in"), get(acc, "plan", "d_out")
    c_in, c_use = get(acc, "ctrl", "p_in"), get(acc, "ctrl", "used")

    def sub(hi, lo):
        return None if (hi is None or lo is None) else hi - lo

    # 累计列是各节点直接量到的「距相机采集多久」；单跳列是相邻两个相减
    return ([
        ("相机双目计算 + DDS 到 bridge", b_in, b_in, "bridge d_in"),
        ("bridge 转发", sub(b_out, b_in), b_out, "bridge d_out − d_in"),
        ("DDS 到 planning + 同步器/排队", sub(p_in, b_out), p_in, "plan d_in − bridge d_out"),
        ("planning 计算", sub(p_out, p_in), p_out, "plan d_out − d_in"),
        ("DDS 到 cmd_vel_control", sub(c_in, p_out), c_in, "ctrl p_in − plan d_out"),
        ("5Hz 限速 + 控制环等待", sub(c_use, c_in), c_use, "ctrl used − p_in"),
    ], c_use, get(acc, "diffcar", "cmd_wait"))


def phases(env):
    """按 LATENV 的 prev=（在预览哪些图像话题）把一趟切成若干段。

    分两趟比预览开销会把路线/光照/温度一起差进来；同一趟里切预览是唯一能控住
    它们的做法。返回 [(prev 值, 起, 止)]。
    """
    seq = []
    for t, txt in env:
        if t is None:
            continue
        m = re.search(r"prev=(\S+)", txt)
        if not m:
            continue
        v = m.group(1)
        if not seq or seq[-1][0] != v:
            seq.append((v, t, t))
        else:
            seq[-1] = (v, seq[-1][1], t)
    return seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="*", help="日志文件；留空则扫 --dir")
    ap.add_argument("--dir", default=os.environ.get("TINYNAV_APP_LOG_DIR", "/userdata/x5/logs"))
    ap.add_argument("--run", help='只看这一趟，如 2026_09_07_10_38_03；默认最新一趟')
    ap.add_argument("--all-runs", action="store_true", help="把目录里所有趟混在一起（一般不要）")
    ap.add_argument("--no-window", action="store_true",
                    help="不取交集时间窗（会把空闲和导航混在一起，一般不要）")
    ap.add_argument("--by-prev", action="store_true",
                    help="按 LATENV 的 prev=（在预览什么）把一趟切成若干段分别出表")
    ap.add_argument("--stale-h", type=float, default=2.0,
                    help="比最新日志旧这么多小时的就跳过（默认 2）")
    a = ap.parse_args()

    paths = a.paths
    if not paths and os.path.isdir(a.dir):
        names = [f for f in os.listdir(a.dir) if f.endswith(".log") or f.endswith(".txt")]
        # 🔴 不能按文件名前缀分组取"最新一趟"：常驻节点(app_start.sh)的时间戳是
        # board-app 启动时刻，而 cmd_vel_control 是 enable nav 那一刻才建文件
        # (node_manager._make_log)，两者必然不同 —— 那样筛只会剩 cmd_vel_control。
        # 改成按节点各取最新一份。
        if a.run:
            names = [f for f in names if f.startswith(a.run)]
        if not a.all_runs:
            newest = {}
            for f in names:
                node = f[20:] if _RUN.match(f[:19]) else f
                if node not in newest or f > newest[node]:
                    newest[node] = f
            names = sorted(newest.values())
            # 旧节点的日志会一直留着（09-04 的 cmd_vel_control 就还在），只要它里面
            # 恰好有 LAT 行就会混进中位数。按最新文件的 mtime 划一条线。
            mt = {f: os.path.getmtime(os.path.join(a.dir, f)) for f in names}
            cutoff = max(mt.values()) - a.stale_h * 3600
            print("按节点各取最新一份（--all-runs 可看全部）：")
            keep = []
            for f in names:
                if mt[f] < cutoff:
                    print(f"  跳过 {f}（比最新的旧 {(max(mt.values()) - mt[f]) / 3600:.1f} 小时）")
                else:
                    print(f"  {f}")
                    keep.append(f)
            names = keep
        paths = [os.path.join(a.dir, f) for f in sorted(names)]
        # 后端的 LATENV 落在 logs/app.log（app_start.sh 把 uvicorn 的 stdout 重定向到
        # 那里），不在 logs/nodes/ 里 —— 不带上它这一趟的工况就丢了。
        app_log = os.path.join(os.path.dirname(a.dir.rstrip("/")), "app.log")
        if os.path.exists(app_log):
            paths.append(app_log)
            print(f"  {os.path.basename(app_log)}（后端 LATENV）")
    if not paths:
        print(f"没有日志可读（--dir {a.dir}）", file=sys.stderr)
        return 1

    rows = collect(paths)
    rows, win = window(rows) if not a.no_window else (rows, None)
    if win:
        lo, hi, span = win
        print(f"交集时间窗 {hi - lo:.0f}s（各 tag 覆盖："
              + " ".join(f"{t}={v[1] - v[0]:.0f}s" for t, v in sorted(span.items())) + "）")
    elif not a.no_window:
        print("⚠️ 各 tag 的时间范围没有交集，下面的相减【不可信】")
    env = collect_env(paths)
    if win:
        lo, hi, _ = win
        env = [e for e in env if e[0] is None or lo - 15 <= e[0] <= hi + 15]
    if env:
        print("这一趟的工况（LATENV，来自后端）：")
        seen = set()
        for _t, txt in env:
            key = txt.split(" ui_age=")[0]      # 只在「开着什么」变化时打一行
            if key in seen:
                continue
            seen.add(key)
            print(f"  {txt}")
        print("  ⚠️ ws/prev 不同的两趟【不可比】—— 观看本身就要花 CPU，直接吃延迟预算")
        print()
    if a.by_prev:
        segs = phases(env)
        if len(segs) < 2:
            print("⚠️ 这一趟 prev= 从头到尾没变过，切不出段（要在跑的过程中切换预览）\n")
        else:
            t_all = [r[0] for r in rows if r[0] is not None]
            hi_end = win[1] if win else (max(t_all) if t_all else 0)
            print(f"按 prev= 切出 {len(segs)} 段：\n")
            for i, (prev, t0, _t1) in enumerate(segs):
                t_end = segs[i + 1][1] if i + 1 < len(segs) else hi_end
                sub = fold([r for r in rows if r[0] is not None and t0 <= r[0] <= t_end])
                n = len(next(iter(next(iter(sub.values())).values()))[0]) if sub else 0
                print(f"── 第 {i + 1} 段  prev={prev}  {t_end - t0:.0f}s"
                      f"  ({len(sub)} 个 tag, {n} 个窗口)")
                if n < 3:
                    print("   窗口太少，跳过 —— LAT 行按 TINYNAV_LAT_LOG_S 打，"
                          "每段至少留 30 s\n")
                    continue
                tbl, c_use_s, d_wait_s = chain(sub)
                for name, one, _cum, _src in tbl:
                    print("   " + pad(name, 32) + fmt(one))
                print("   " + pad("→ 到 /cmd_vel", 32) + fmt(c_use_s))
                print("   " + pad("→ 到串口写（+diffcar）", 32)
                      + fmt(None if c_use_s is None else c_use_s + (d_wait_s or 0)))
                print()
            print("各段只差 prev=，路线/光照/温度是同一趟，可以直接横向比。\n")

    acc = fold(rows)
    if not acc:
        print("日志里没有 LAT 行 —— 是不是 TINYNAV_LAT_LOG_S=0，或者这趟没跑到导航？",
              file=sys.stderr)
        return 1

    print(f"读了 {len(paths)} 个文件；各 tag 的窗口数："
          + " ".join(f"{t}={len(next(iter(v.values()))[0])}" for t, v in acc.items()))
    print()

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
