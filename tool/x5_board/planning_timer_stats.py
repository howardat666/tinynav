#!/usr/bin/env python3
"""Aggregate planning_node's codetiming output into a per-stage latency budget.

planning_node already instruments its whole loop with codetiming (see the Timer
decorators around `Planning Loop`, `preprocess`, `raycasting`, `obstacle map`, `vis`,
`traj gen`, `traj score`, `pub`), but the emitted form is one line per stage per cycle
-- about 40 lines/s -- which is unreadable by eye and hides the distribution. Running
it through this turns the log into the thing the question actually needs: where the
cycle time goes, and how the tail behaves.

    TINYNAV_VERBOSE_TIMER=1 bash tool/x5_board/app_start.sh start a   # on the board
    # let it run, then
    python3 tool/x5_board/planning_timer_stats.py <planning log>

WHY THE TAIL MATTERS MORE THAN THE MEAN
---------------------------------------
cmd_vel_control rejects a trajectory whose first pose is already behind the latest
odometry, and logs `received stale /planning/trajectory_path`. What causes that is not
the average cycle but the slow ones, so p95 and max are the numbers to optimise
against. A stage with a small mean and a large max is still a problem.

Resolution note: the format string is `{milliseconds:.0f}`, so every stage is rounded
to whole milliseconds and anything under ~0.5 ms reads as 0. Good enough to find the
expensive stages, useless for confirming a stage is free -- treat a 0 as "below the
resolution", not as "costs nothing".
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
from collections import defaultdict

# "[preprocess] Elapsed time: 12 ms"
LINE = re.compile(r"\[([^\]]+)\] Elapsed time: ([0-9.]+) ms")

WHOLE_LOOP = "Planning Loop"


def pct(values: list[float], q: int) -> float:
    if not values:
        return float("nan")
    if len(values) < 2:
        return values[0]
    return statistics.quantiles(values, n=100)[q - 1]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("log", help="planning_node log written with TINYNAV_VERBOSE_TIMER=1")
    parser.add_argument("--tail", type=int, default=0,
                        help="only consider the last N matching lines (0 = all)")
    args = parser.parse_args()

    samples: dict[str, list[float]] = defaultdict(list)
    # errors='replace': these logs interleave binary-ish node output on the board.
    with open(args.log, errors="replace") as fh:
        matched = [(m.group(1), float(m.group(2))) for line in fh for m in [LINE.search(line)] if m]
    if args.tail:
        matched = matched[-args.tail:]
    for name, ms in matched:
        samples[name].append(ms)

    if not samples:
        print("no codetiming lines found -- was TINYNAV_VERBOSE_TIMER=1 set?", file=sys.stderr)
        return 1

    loop = samples.get(WHOLE_LOOP, [])
    print("=" * 78)
    print(f"planning_node stage budget from {args.log}")
    print(f"{len(loop)} complete cycles" if loop else "no whole-loop samples")
    print("=" * 78)
    print(f"  {'stage':<16} {'n':>6} {'mean':>8} {'p50':>7} {'p95':>7} {'max':>7}  {'% of loop':>9}")
    print("-" * 78)

    loop_mean = statistics.fmean(loop) if loop else float("nan")
    # Whole loop first, then the stages inside it, biggest mean first.
    order = ([WHOLE_LOOP] if loop else []) + sorted(
        (k for k in samples if k != WHOLE_LOOP),
        key=lambda k: statistics.fmean(samples[k]),
        reverse=True,
    )
    stage_mean_total = 0.0
    for name in order:
        v = sorted(samples[name])
        mean = statistics.fmean(v)
        # A name with a colon is a sub-timer nested inside its parent stage
        # ("vis:mask" inside "vis"). Adding it to the total would count the same
        # microseconds twice -- which it did, and produced a nonsensical "stages sum to
        # 119.55 ms of a 95.37 ms loop".
        if name != WHOLE_LOOP and ":" not in name:
            stage_mean_total += mean
        share = 100.0 * mean / loop_mean if loop_mean == loop_mean and loop_mean > 0 else float("nan")
        tag = "" if name != WHOLE_LOOP else "  <- whole cycle"
        print(f"  {name:<16} {len(v):>6} {mean:>7.2f}m {pct(v, 50):>6.1f} {pct(v, 95):>6.1f} "
              f"{v[-1]:>6.1f}  {share:>8.1f}%{tag}")

    if loop:
        print("-" * 78)
        unaccounted = loop_mean - stage_mean_total
        print(f"  stages sum to {stage_mean_total:.2f} ms of a {loop_mean:.2f} ms loop; "
              f"{unaccounted:.2f} ms ({100.0 * unaccounted / loop_mean:.1f}%) is outside any stage")
        print(f"  loop p95 = {pct(sorted(loop), 95):.1f} ms, max = {max(loop):.1f} ms")
        print("  A loop p95 above ~200 ms is what produces the 0.2 s stale trajectories that")
        print("  make cmd_vel_control stop the robot at every local endpoint.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
