#!/usr/bin/env python3
"""Find the keyframe where relocalization jumped, and print that attempt's funnel.

Aliasing between similar workstations produces a *successful* relocalization, so there
is no failure line to grep for. What it does produce is a discontinuity in the reported
map position between consecutive keyframes -- larger than the robot could have moved --
so that is the detector. For every jump found, the matching "Relocalization stage
timing" line is printed with every stage of the funnel, which is what says whether the
retrieval brought back the wrong place or the solve went wrong on the right one.

Usage: reloc_forensics.py map_node.log [--jump-m 1.0]
"""
import argparse
import re
import sys

# [INFO] [1787741152.730708385] [map_node]: ...
LINE = re.compile(r"^\[(\w+)\]\s+\[(\d+\.\d+)\]\s+\[([^\]]+)\]:\s*(.*)$")
ROBOT_MAP = re.compile(r"robot_map=\[?\s*([-\d.]+)[,\s]+([-\d.]+)")
RELOC = re.compile(r"Relocalization stage timing ms:\s*timestamp=(\d+),.*?success=(\w+)(.*)$")
CAND = re.compile(r"(\d+):sim=([\d.]+),matches=(\d+)(?:,valid_depth=(\d+))?,jump=([\d.naN]+)")


def parse(path):
    """(wall_s, node, text) per line, joining nothing -- these logs are one line each."""
    out = []
    with open(path, errors="replace") as fh:
        for raw in fh:
            m = LINE.match(raw.rstrip("\n"))
            if m:
                out.append((float(m.group(2)), m.group(3), m.group(4)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--jump-m", type=float, default=1.0,
                    help="报告超过这个距离的位置跳变")
    ap.add_argument("--context-s", type=float, default=3.0,
                    help="跳变前后多少秒内的重定位记录算作相关")
    ap.add_argument("--window", action="store_true",
                    help="打印 map->odom 约束窗口的成分随时间的变化")
    a = ap.parse_args()

    rows = parse(a.log)
    if not rows:
        print("没有解析到任何 ROS 日志行 —— 确认这是 map_node 的日志", file=sys.stderr)
        return 1

    positions, relocs = [], []
    for wall, _node, text in rows:
        m = ROBOT_MAP.search(text)
        if m:
            positions.append((wall, float(m.group(1)), float(m.group(2))))
        m = RELOC.search(text)
        if m:
            relocs.append((wall, int(m.group(1)), m.group(2) == "True", m.group(3)))

    ok = sum(1 for r in relocs if r[2])
    print(f"解析到 {len(positions)} 条位置、{len(relocs)} 次重定位（成功 {ok}，"
          f"失败 {len(relocs)-ok}）\n")

    # compute_transform_from_map_to_odom 只取最近 100 个约束，没有鲁棒核。所以「这 100 条
    # 里有几条是干净的」才是位姿跳变的直接解释 —— 单次重定位再健康，10/100 也拉不动解。
    clean = []
    for wall, _ts, succ, extra in relocs:
        if not succ:
            continue
        cs = CAND.findall(extra)
        spread = ((max(int(c[0]) for c in cs) - min(int(c[0]) for c in cs)) / 1e9
                  if len(cs) > 1 else 0.0)
        i = re.search(r"inliers=(\d+)/(\d+)", extra)
        ratio = int(i.group(1)) / max(1, int(i.group(2))) if i else 0.0
        clean.append((wall, spread <= 2.0 and ratio >= 0.70))
    if clean:
        good_now = sum(1 for c in clean[-100:] if c[1])
        print(f"约束窗口（最近 100 条成功重定位）现在有 {good_now}/100 条是干净的"
              f"（候选跨度 ≤2 s 且内点率 ≥70%）")
        if good_now < 30:
            print(f"   ⚠ 干净约束不足 30% —— map->odom 的最小二乘由陈旧/低质约束主导，"
                  f"位姿会在两个解之间跳。鲁棒核救不了这种情况（离群点是多数）。")
        print()
        if a.window:
            print("=== 约束窗口成分随时间 ===")
            print("  墙钟        干净/100")
            step = max(1, len(clean) // 25)
            for k in range(99, len(clean), step):
                g = sum(1 for c in clean[max(0, k - 99):k + 1] if c[1])
                bar = "#" * (g // 2)
                print(f"  {clean[k][0]:.1f}  {g:3d}   {bar}")
            print()

    jumps = []
    for i in range(1, len(positions)):
        (t0, x0, y0), (t1, x1, y1) = positions[i - 1], positions[i]
        d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        dt = t1 - t0
        # 0.5 m/s 是底盘上限，留 3 倍余量再当成"跳变"，避免把日志节流造成的
        # 长间隔当成瞬移。
        if d >= a.jump_m and d > 1.5 * max(dt, 1e-3):
            jumps.append((t1, d, dt, (x0, y0), (x1, y1)))

    if not jumps:
        print(f"没有超过 {a.jump_m} m 的位置跳变。"
              f"可以把 --jump-m 调小，或者确认这份日志覆盖了出问题的那一段。")
        return 0

    print(f"=== {len(jumps)} 处疑似重定位跳变 ===\n")
    for wall, d, dt, p0, p1 in jumps:
        print(f"[{wall:.3f}] 跳了 {d:.2f} m / {dt:.2f} s   "
              f"({p0[0]:+.2f},{p0[1]:+.2f}) -> ({p1[0]:+.2f},{p1[1]:+.2f})")
        near = [r for r in relocs if abs(r[0] - wall) <= a.context_s]
        if not near:
            print("   这段时间内没有重定位记录 —— 跳变来自别处（位姿图/里程计）\n")
            continue
        for rwall, ts, succ, extra in near:
            cands = CAND.findall(extra)
            print(f"   [{rwall:.3f}] ts={ts} success={succ}")
            inl = re.search(r"inliers=(\d+)/(\d+)", extra)
            lm = re.search(r"landmarks=(\d+)", extra)
            if inl:
                ratio = int(inl.group(1)) / max(1, int(inl.group(2)))
                print(f"      漏斗: 路标={lm.group(1) if lm else '?'} "
                      f"内点={inl.group(1)}/{inl.group(2)} ({ratio:.0%}, 门限 12 且 25%)")
            if cands:
                ts_list = [int(c[0]) for c in cands]
                spread = (max(ts_list) - min(ts_list)) / 1e9
                print(f"      候选跨度={spread:.1f}s  "
                      f"{'<- 候选散在地图各处，检索没找到真匹配' if spread > 5 else ''}")
                for c in cands:
                    print(f"        ts={c[0]} sim={c[1]} matches={c[2]} "
                          f"valid_depth={c[3] or '-'} jump={c[4]}m")
        print()
    print("怎么读：候选跨度大 + jump 大 = 检索认错了地方（工位混淆）；\n"
          "候选跨度小但内点率低 = 检索对了、解算差，那是另一类问题。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
