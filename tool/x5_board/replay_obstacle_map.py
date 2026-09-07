#!/usr/bin/env python3
"""离线复现障碍图，扫 (z 波段下界 x z 跨度门限) 这两个参数。

数据来自 `capture_depth_scene.py`。**光栅化和 build_obstacle_map 都调仓库里的真函数**
—— 重写一份就会和板上分叉，那个坑踩过（replay_escape_decision 自己抄了一份判据，
一致到分叉那天为止）。

    docker run --rm --entrypoint bash -v $PWD:/tinynav -w /tinynav uniflexai/tinynav:latest \
      -lc 'source /opt/ros/humble/setup.bash && python3 tool/x5_board/replay_obstacle_map.py \
             data/obstacle_scenes/*.npz'

判据（每个场景一行）：
  近处格子   离车 < 0.6 m 的障碍格子数。空地场景里这应该是 0。
  最近       最近障碍格到【控制中心=驱动轴】的距离。< 0.1875（扫掠半径 0.1375 + 安全 0.05）就是 blocked。
  远处格子   >= 0.6 m 的，真墙/真障碍应该留在这里。
"""
from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tinynav.core.planning_kernels import run_raycasting_loopy       # noqa: E402
from tinynav.core.planning_node import (build_obstacle_map,           # noqa: E402
                                        roll_occupancy_grid)
from tinynav.core.robot_config import robot_config                    # noqa: E402

GRID_SHAPE = (80, 80, 14)
RES = 0.05
GRID_OFFSET = np.array([0.0, 0.0, 0.15])
DECAY_REF_DT = 0.23
NEAR_M = 0.6


def build_grid(npz, step, zphase=0.0):
    """跑完整段序列，返回最后一帧的障碍图 + 控制中心。"""
    K = npz["K"]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    poses, depths, stamps = npz["poses"], npz["depth_mm"], npz["stamps"]
    # 🔴 原点必须和板上同一个格。planning 的初值【不含位姿】：
    #     self.origin = shape*res/-2 + grid_offset      -> [-2.0, -2.0, -0.20]
    # 之后 roll_occupancy_grid 返回的是 old_origin + 整数格*res，所以 origin 永远停在
    # 这个格上。实机 origin[2] = -0.10（不是 -0.076），于是**层边界正好压在地面 z=0**，
    # 地面自己劈成两层、往下的噪声再加一层 = 3 层 = span 0.10，刚好过门限。
    # 我第一版把位姿加进初值，层中心落在地面上，地面只占 2 层 —— 离线看着干净、
    # 板上却在报，一模一样的配置差出 70 个格子。原点差半格，结论就反了。
    origin = np.array(GRID_SHAPE) * RES / -2.0 + GRID_OFFSET
    origin = origin.astype(np.float64)
    origin[2] += zphase
    grid = np.zeros(GRID_SHAPE)
    last = None
    for i in range(len(stamps)):
        T = poses[i]
        # 和 planning 逐字一致的重定心
        center = origin + np.array(GRID_SHAPE) * RES / 2 - GRID_OFFSET
        if np.linalg.norm(T[:3, 3] - center) > 0.1:
            new_origin = (T[:3, 3] - np.array(GRID_SHAPE) * RES / 2 + GRID_OFFSET)
            grid, origin = roll_occupancy_grid(grid, origin, new_origin, RES)
        d = depths[i].astype(np.float32) / 1000.0
        occ = run_raycasting_loopy(d, T, GRID_SHAPE, fx, fy, cx, cy, origin, step, RES)
        dt = DECAY_REF_DT if last is None else min(
            max(stamps[i] - last, DECAY_REF_DT), 8.0 * DECAY_REF_DT)
        last = stamps[i]
        grid *= 0.99 ** (dt / DECAY_REF_DT)
        grid += occ
        np.clip(grid, -0.2, 0.2, out=grid)
    return grid, origin, poses[-1]


def make_mask(grid, origin, T, cfg, cam_offset):
    mask = build_obstacle_map(grid, origin, RES, robot_z=T[2, 3], config=cfg)
    return mask, T[:3, 3] - T[:3, :3] @ cam_offset


def score(mask, origin, centre):
    idx = np.argwhere(mask)
    if not len(idx):
        return 0, float('inf'), 0
    xy = origin[:2] + (idx + 0.5) * RES
    dist = np.linalg.norm(xy - centre[:2], axis=1)
    return int((dist < NEAR_M).sum()), float(dist.min()), int((dist >= NEAR_M).sum())


def height_split(grid, mask, origin, centre, cfg, robot_z, win=1.2, low_top=0.15):
    """把检出的障碍格子按【它自己最高那一层的高度】分「矮」和「高」。

    回答的是「span 门限设高了还能不能看见椅脚」——桌面/坐垫那种高东西的 z 跨度天然很大，
    门限抬多高都拦不住它；真正被门限挡掉的是**只有矮结构**的格子（星形底盘的辐条、脚轮，
    离地 4~8 cm）。只看总格数会把这两类混在一起，我第一版就混了。
    低/高的界：格内最高占据层的【离地高度】<= low_top 算矮。"""
    zd = grid.shape[2]
    zc = origin[2] + (np.arange(zd) + 0.5) * RES
    zrel = zc - robot_z
    inb = (zrel >= cfg.robot_z_bottom) & (zrel <= cfg.robot_z_top)
    floor_z = robot_z - 0.124                     # 相机高度，和 planning 同口径
    idx = np.argwhere(mask)
    low = high = 0
    lowd = highd = float('inf')
    for i, j in idx:
        d = float(np.hypot(origin[0] + (i + 0.5) * RES - centre[0],
                           origin[1] + (j + 0.5) * RES - centre[1]))
        if d > win:
            continue
        occ = (grid[i, j] > cfg.occ_threshold) & inb
        if not occ.any():
            continue
        top_h = float(zc[np.where(occ)[0].max()] - floor_z)
        if top_h <= low_top:
            low += 1; lowd = min(lowd, d)
        else:
            high += 1; highd = min(highd, d)
    return low, lowd, high, highd


def score_sides(mask, origin, centre, T, win=1.5, lat_dead=0.10):
    """按车头方向把障碍格子分左右。一次采集里「左边有椅子、右边空地」就能同时给两个判据：
    椅子侧要检出，空地侧要 0。方向取法和 _front_obstacle_dist / publish_footprint 一致。"""
    fwd = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    lft = T[:3, :3] @ np.array([1.0, 0.0, 0.0])
    f = fwd[:2] / max(np.linalg.norm(fwd[:2]), 1e-9)
    l = lft[:2] / max(np.linalg.norm(lft[:2]), 1e-9)
    idx = np.argwhere(mask)
    out = {}
    for name, sign in (("A", +1.0), ("B", -1.0)):
        out[name] = (0, float('inf'))
    if not len(idx):
        return out
    v = origin[:2] + (idx + 0.5) * RES - centre[:2]
    dist = np.linalg.norm(v, axis=1)
    along = v @ f
    lat = v @ l
    for name, sign in (("A", +1.0), ("B", -1.0)):
        m = (dist < win) & (along > -0.1) & (sign * lat > lat_dead)
        out[name] = (int(m.sum()), float(dist[m].min()) if m.any() else float('inf'))
    return out


def debug_near(grid, origin, centre, cfg, robot):
    """车周 0.6 m 内的栅格值和层结构。板上说这里有障碍、离线说没有时，靠这个分辨
    是【几何对不上】（这里一个占据格都没有）还是【投票强度对不上】（有值但没过阈值）。"""
    h, w, zd = grid.shape
    ii, jj = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
    xy = origin[:2] + (np.stack([ii, jj], -1) + 0.5) * RES
    d = np.linalg.norm(xy - centre[:2], axis=-1)
    near = d < NEAR_M
    sub = grid[near]                       # (N, zd)
    zc = origin[2] + (np.arange(zd) + 0.5) * RES
    zrel = zc - (centre[2] + 0.0)
    print(f"    [debug] 近处 {near.sum()} 个 xy 格, z 层中心(世界) "
          f"{zc[0]:+.3f}..{zc[-1]:+.3f}, occ_threshold={cfg.occ_threshold}")
    print(f"    [debug] 近处栅格值 max={sub.max():+.3f} min={sub.min():+.3f} "
          f"过阈值的(格,层)对 = {(sub > cfg.occ_threshold).sum()}")
    for k in range(zd):
        n = int((sub[:, k] > cfg.occ_threshold).sum())
        if n:
            print(f"      层{k:2d} z={zc[k]:+.3f}  过阈值 {n:4d} 格  "
                  f"该层最大值 {sub[:, k].max():+.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", nargs="+")
    ap.add_argument("--robot", default="diffcar")
    ap.add_argument("--step", type=int, default=3)
    ap.add_argument("--zbot", default="-0.20,-0.14,-0.11,-0.09,-0.07")
    ap.add_argument("--span", default="0.05,0.10,0.15,0.20")
    ap.add_argument("--zphase", default="0", help="逗号分隔，给 origin[2] 加的偏移（米）")
    ap.add_argument("--heights", action="store_true",
                    help="把检出格子按「自己最高那层的离地高度」分矮(<=0.15m)/高，1.2m 窗口内")
    ap.add_argument("--sides", action="store_true",
                    help="按车头方向分左右报（A/B 两侧），1.5 m 窗口内")
    ap.add_argument("--debug-near", action="store_true",
                    help="打印车周 0.6 m 内每层的占据情况，用来对齐板上读数")
    args = ap.parse_args()

    robot = robot_config(args.robot)
    hard = robot.hard_clearance
    print(f"车体 {robot.hull_radius:.3f} + 安全 {robot.safety_radius:.2f} "
          f"= hard {hard:.3f} m；近处定义 < {NEAR_M} m；raycast step={args.step}\n")

    scenes = []
    for p in args.scenes:
        z = np.load(p, allow_pickle=True)
        scenes.append((Path(p).stem, z))
        print(f"{Path(p).stem}: {len(z['stamps'])} 帧, {z['note']}")
    print()

    zbots = [float(v) for v in args.zbot.split(",")]
    spans = [float(v) for v in args.span.split(",")]
    head = "  ".join(f"{n[:14]:>26}" for n, _ in scenes)
    print(f"{'zbot(离地)':>16} {'span':>6}  {head}")
    lbl = "A侧格数/最近 B侧格数/最近" if args.sides else "近处/最近/远处"
    print(f"{'':>16} {'':>6}  " + "  ".join(f"{lbl:>30}" for _ in scenes))
    phases = [float(v) for v in args.zphase.split(",")]
    # 光栅化 + 累积和门限无关，所以每个 (场景, 相位) 只积一次，之后扫门限几乎免费。
    grids = {}
    for zph in phases:
        for name, z in scenes:
            print(f"  积栅格 {name} 相位{zph:+.3f} ...", flush=True)
            grids[(name, zph)] = build_grid(z, args.step, zph)
    print()
    for zph in phases:
      if len(phases) > 1:
        print(f"---- z 相位偏移 {zph:+.3f} m ----")
      for zb in zbots:
        for sp in spans:
            cells = []
            hcells = []
            for name, z in scenes:
                g, o, T = grids[(name, zph)]
                cfg = dataclasses.replace(robot.obstacle, robot_z_bottom=zb,
                                          min_wall_span_m=sp)
                m, c = make_mask(g, o, T, cfg, robot.cam_offset_3d)
                if args.debug_near:
                    debug_near(g, o, c, cfg, robot)
                if args.heights:
                    lo, lod, hi, hid = height_split(g, m, o, c, cfg, T[2, 3])
                    hcells.append(f"矮{lo:3d}/{lod:5.2f} 高{hi:3d}/{hid:5.2f}".rjust(28))
                near, mn, far = score(m, o, c)
                flag = "🔴" if mn < hard else "  "
                if args.sides:
                    sd = score_sides(m, o, c, T)
                    a, b = sd["A"], sd["B"]
                    cells.append(f"A侧{a[0]:3d}/{a[1]:5.2f} B侧{b[0]:3d}/{b[1]:5.2f}"
                                 f"{flag}".rjust(30))
                else:
                    cells.append(f"{near:4d}/{mn:6.2f}/{far:4d}{flag}".rjust(26))
            hgt = zb + 0.124
            print(f"{zb:+.2f}({hgt:+.3f}) {sp:6.2f}  " + "  ".join(cells))
            if args.heights:
                print(f"{'':>16} {'':>6}  " + "  ".join(hcells))


if __name__ == "__main__":
    main()
