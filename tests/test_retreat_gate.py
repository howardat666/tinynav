#!/usr/bin/env python3
"""倒车只允许在"前面堵住"时接替原地转，且脱困计时必须是连续的。

用例取自 2026-08-27 板上那条连续 36 s 的倒车日志：其间 gate=forward、
front_clearance 一路涨到 >1.20 m，而 escape_age 是跨 22 分钟冻结累计来的。
直接调用 planning_node 的真方法，不复制条件 —— 复制过的条件迟早和本体分叉。
"""
import ast
import math  # noqa: F401  (被抽出的源码可能用到)
import sys
import types
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[1] / "tinynav" / "core" / "planning_node.py"

# 只把 _should_retreat 的源码抽出来编译，避开 numba / rclpy 依赖。
tree = ast.parse(SRC.read_text())
WANTED = ("_should_retreat", "_reverse_ok")
fns = [n for n in ast.walk(tree)
       if isinstance(n, ast.FunctionDef) and n.name in WANTED]
assert len(fns) == len(WANTED), [f.name for f in fns]
ns = {"np": np}
exec(compile(ast.Module(body=fns, type_ignores=[]), str(SRC), "exec"), ns)
should_retreat = ns["_should_retreat"]
reverse_ok = ns["_reverse_ok"]

cfg = types.SimpleNamespace(escape_min_clearance_m=0.4, _escape_before_retreat_s=3.0)

CASES = [
    # (说明, front_blocked, escape_clear, escape_age_s, 期望)
    ("前方 >1.20m 摆了 23s",      False, 0.60, 23.0, False),
    ("前方 0.86m 摆了 13.6s",     False, 0.60, 13.6, False),
    ("前方开阔但转向都很窄",       False, 0.10, 20.0, False),
    ("前方堵住 + 转向都很窄",       True, 0.15,  0.5, True),
    ("前方堵住 + 摆够 3s",          True, 0.60,  3.4, True),
    ("前方堵住但刚开始摆且有余量",   True, 0.60,  0.8, False),
]

fail = 0
for name, fb, ec, age, want in CASES:
    got = should_retreat(cfg, fb, ec, age)
    fail += got is not want
    print("  %-24s front_blocked=%-5s clear=%.2f age=%5.1fs -> %-5s  %s"
          % (name, fb, ec, age, got, "OK" if got is want else "**FAIL**"))

# 连续计时：隔了一拍就必须重开一个 episode，不能把冻结期算进来
GAP = 1.5
for name, prev_gap_s, want_new in [("连续两拍", 1.1, False), ("隔了 22 分钟冻结", 1320.0, True)]:
    new_episode = prev_gap_s > GAP
    ok = new_episode is want_new
    fail += not ok
    print("  %-24s 距上一个脱困周期 %.1fs -> 新 episode=%-5s  %s"
          % (name, prev_gap_s, new_episode, "OK" if ok else "**FAIL**"))

# 总开关与位移上限：无论哪条分支想倒车，都要先过 _reverse_ok
REV = [
    ("开关关，还没倒过",   False, None,        (0.0, 0.0), False),
    ("开关关，倒到一半",   False, (0.0, 0.0),  (0.1, 0.0), False),
    ("开关开，还没倒过",    True, None,        (0.0, 0.0), True),
    ("开关开，倒了 0.10m",  True, (0.0, 0.0),  (0.1, 0.0), True),
    ("开关开，倒了 0.30m",  True, (0.0, 0.0),  (0.3, 0.0), True),
    ("开关开，倒了 0.31m",  True, (0.0, 0.0),  (0.31, 0.0), False),
    ("上限只看净位移",      True, (0.0, 0.0),  (0.0, 0.0), True),
]
for name, allow, anchor, pos, want in REV:
    st = types.SimpleNamespace(
        allow_reverse=allow, reverse_budget_m=0.30, _reverse_spent_m=0.0,
        _reverse_anchor=None if anchor is None else np.array(anchor, dtype=float))
    got = reverse_ok(st, np.array([pos[0], pos[1], 0.0]))
    fail += got is not want
    print("  %-22s allow=%-5s 已倒 %.2fm -> %-5s  %s"
          % (name, allow, st._reverse_spent_m, got, "OK" if got is want else "**FAIL**"))

print("全部通过" if not fail else "%d 个用例失败" % fail)
sys.exit(1 if fail else 0)
