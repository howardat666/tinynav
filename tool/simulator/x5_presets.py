#!/usr/bin/env python3
"""板上那套配置 + 预设场景，仿真器和无头脚本共用这一份定义。

改这里就同时改了 web UI 的下拉框和 `scenarios.py` 的回归套件 —— 两份定义迟早分叉。
"""
from __future__ import annotations

import copy
import math
from typing import Any

# ---------------------------------------------------------------------------
# 板上 app_start.sh 里与规划有关的环境变量（tool/x5_board/app_start.sh:169-178）。
# 子进程是 os.environ.copy() 起的，所以在仿真进程里 setdefault 就能传下去。
# ---------------------------------------------------------------------------
BOARD_ENV: dict[str, str] = {
    # 跟着平台配置：0.10，配 robot_z_bottom=-0.16（对 zbot 的容差宽一倍半，见 robot_config）。
    # 🔴 2026-09-03：cam_h 从 0.18 改成板上实测的 0.124（mount_height 一起改，两者必须
    # 相等）。0.18 时地面落在 z 波段【下界之外】，最小可见障碍高度是 130 mm 而板上是
    # 74 mm —— 相差 56 mm，仿真结构上复现不了压上矮椅脚。所有场景的历史基线因此作废。
    "TINYNAV_MIN_WALL_SPAN_M": "0.10",
    "TINYNAV_DILATION_CELLS": "0",
    "TINYNAV_LOW_OBS": "0",
    "TINYNAV_LOW_OBS_H_LO": "0.05",
    "TINYNAV_LOW_OBS_RANGE_M": "1.5",
    "TINYNAV_LOW_OBS_MIN_PTS": "2",
    "TINYNAV_CAMERA_HEIGHT_M": "0.124",
    # z 相位旋钮。0.15 = 板上当前（层边界正压在地面上，检出高度在 50/100 mm 之间抖）；
    # 0.1625 = 真实椅子数据上矮障碍格 8 -> 27、最近 0.68 -> 0.53 m，空地误报仍 0。
    "TINYNAV_GRID_OFFSET_Z": "0.15",
    "TINYNAV_ALLOW_REVERSE": "0",
    "TINYNAV_PUBLISH_PLANNING_OVERLAYS": "1",
    # TINYNAV_RAYCAST_STEP 不写在这里：它必须跟着下面的相机缩放一起变，见 CAMERAS。
}

# ---------------------------------------------------------------------------
# 相机。实机深度是 544x640、fx=fy=309.5（板上 CameraInfo 实测），板上 raycast step=5,
# 也就是每 5/309.5 = 0.01615 rad 取一条射线 —— **角分辨率**才是决定障碍图有多少洞的量，
# 不是像素数（见 memory dilation-is-patching-raycast-holes）。
#
# 全分辨率渲染实测 263 ms/帧，仿真的 tick 是 8 Hz（125 ms），撑不住。所以默认用 0.4 倍
# 缩放配 step=2：2/123.8 = 0.01616 rad，和板上那 0.01615 逐位对得上，成本 25 ms/帧。
# ---------------------------------------------------------------------------
_REAL_W, _REAL_H, _REAL_F = 544, 640, 309.5
_REAL_STEP = 5


def _camera(scale: float, step: int) -> dict[str, Any]:
    return {
        "width": int(round(_REAL_W * scale)),
        "image_height": int(round(_REAL_H * scale)),
        "fx": _REAL_F * scale,
        "fy": _REAL_F * scale,
        "max_range": 15.0,
        "mount_height": 0.124,     # 板上实测光心离地，必须和 TINYNAV_CAMERA_HEIGHT_M 相等
        "_raycast_step": step,
    }


CAMERAS: dict[str, dict[str, Any]] = {
    # 默认。视场和角分辨率都和实机一致，渲染 25 ms/帧。
    "match": _camera(0.4, 2),
    # 逐像素和实机一致。渲染 263 ms/帧 -> 仿真掉到约 3.8 Hz，只在需要复核时用。
    "full": _camera(1.0, _REAL_STEP),
    # 上游默认，快但视场和角分辨率都不是我们的，只适合看流程通不通。
    "fast": {"width": 160, "image_height": 100, "fx": 80.0, "fy": 50.0,
             "max_range": 15.0, "mount_height": 0.124, "_raycast_step": 5},
}
DEFAULT_CAMERA = "match"


def camera_and_step(name: str = DEFAULT_CAMERA) -> tuple[dict[str, Any], int]:
    cam = copy.deepcopy(CAMERAS.get(name, CAMERAS[DEFAULT_CAMERA]))
    step = int(cam.pop("_raycast_step"))
    return cam, step


def angular_pitch_deg(cam: dict[str, Any], step: int) -> float:
    return math.degrees(step / float(cam["fx"]))


# ---------------------------------------------------------------------------
# 场景。墙用长方体拼；z 中心 0.3 / 高 0.6 是一堵到腰的墙，矮障碍单独给高度。
# `need_reach=False` 的场景是**故意**到不了的，判据在别处（见 scenarios.py）。
# ---------------------------------------------------------------------------
_WZ, _WH = 0.3, 0.6


def _box(name, center, size):
    return {"name": name, "kind": "box", "center": list(center), "size": list(size)}


# `route` 是这个场景里 map_node 会给出的全局路线（世界系折线）。仿真里没有 map_node，
# 所以按"在地图 SDF 走廊里搜出来的、绕开静态障碍的一条路"手写。规划器订的是
# /mapping/global_plan_odom。
SCENES: dict[str, dict[str, Any]] = {
    "open_run": dict(
        title="空地直行",
        note="3 m 空地。基线，任何改动都不许把它弄坏。",
        route=[[0.0, 0.0], [3.0, 0.0]],
        start=[0.0, 0.0], yaw=0.0, target=[3.0, 0.0], objects=[],
        budget_s=45, need_reach=True, max_flips=6),
    "corridor": dict(
        title="0.8 m 走廊",
        note="净宽 0.8 m，直着穿过去。车宽 0.35，两边各 0.22 m 余量。",
        route=[[0.0, 0.0], [3.0, 0.0]],
        start=[0.0, 0.0], yaw=0.0, target=[3.0, 0.0],
        objects=[_box("left", [1.5, 0.6, _WZ], [3.0, 0.2, _WH]),
                 _box("right", [1.5, -0.6, _WZ], [3.0, 0.2, _WH])],
        budget_s=60, need_reach=True, max_flips=6),
    "narrow_gap": dict(
        title="0.9 m 门洞",
        note="一堵墙中间留 0.9 m。代码注释里旧直线探针在这里「转 102 次、超时」。",
        route=[[0.0, 0.0], [1.5, 0.0], [3.0, 0.0]],
        start=[0.0, 0.0], yaw=0.0, target=[3.0, 0.0],
        objects=[_box("wall_left", [1.5, 1.15, _WZ], [0.2, 1.7, _WH]),
                 _box("wall_right", [1.5, -1.15, _WZ], [0.2, 1.7, _WH])],
        budget_s=75, need_reach=True, max_flips=8),
    "wall_ahead": dict(
        title="贴墙 + 目标在后（摆头那一幕）",
        note="正前方 0.2 m 一堵宽墙、两侧开阔、目标在右后方。2026-08-27 板上摆头的复刻，"
             "判据是变向次数不是到不到。",
        route=[[0.0, 0.0], [0.0, -1.0], [-0.5, -1.8]],
        start=[0.0, 0.0], yaw=0.0, target=[-0.5, -1.8],
        objects=[_box("wall", [0.5, 0.0, _WZ], [0.2, 3.0, _WH])],
        budget_s=75, need_reach=True, max_flips=2),
    "dead_end": dict(
        title="死角（三面围住）",
        note="倒车默认关，所以正确行为是停住报无解 —— 既不许倒、也不许在里面摆头。",
        route=[[0.0, 0.0], [3.0, 0.0]],
        start=[0.0, 0.0], yaw=0.0, target=[3.0, 0.0],
        objects=[_box("front", [0.55, 0.0, _WZ], [0.2, 1.4, _WH]),
                 _box("left", [0.0, 0.7, _WZ], [1.4, 0.2, _WH]),
                 _box("right", [0.0, -0.7, _WZ], [1.4, 0.2, _WH])],
        budget_s=40, need_reach=False, max_flips=4),
    "chair_legs": dict(
        title="办公椅腿（矮障碍）",
        note="五条 5 cm 的腿，离地 0~0.35 m。专门压 low_obs 那条通路（0.05~0.25 m、"
             "1.5 m 内、>=2 点）—— z 跨度那条判不出来，靠离地高度判。",
        route=[[0.0, 0.0], [0.8, 0.45], [1.6, 0.45], [2.5, 0.0]],
        start=[0.0, 0.0], yaw=0.0, target=[2.5, 0.0],
        objects=[_box("leg_c", [1.2, 0.0, 0.17], [0.05, 0.05, 0.35]),
                 _box("leg_a", [1.35, 0.22, 0.17], [0.05, 0.05, 0.35]),
                 _box("leg_b", [1.35, -0.22, 0.17], [0.05, 0.05, 0.35]),
                 _box("leg_d", [1.05, 0.22, 0.17], [0.05, 0.05, 0.35]),
                 _box("leg_e", [1.05, -0.22, 0.17], [0.05, 0.05, 0.35])],
        budget_s=75, need_reach=True, max_flips=8),
    # 2026-09-02：chair_legs 的路线本来就绕开腿，所以它分辨不出「腿有没有被看见」——
    # low_obs 开/关四组结果一模一样。这个变体把路线画成【正对着腿】，不看见就必然压上去。
    "chair_leg_head_on": dict(
        title="正对一条椅腿（判据是压不压上去）",
        note="路线笔直穿过 1.2 m 处那条 5 cm 的腿。腿被看见 -> 自己绕开且不压障碍；"
             "没被看见 -> 直接压过去。用来判 0.05 m 栅格下 z 跨度单独够不够。",
        route=[[0.0, 0.0], [2.5, 0.0]],
        start=[0.0, 0.0], yaw=0.0, target=[2.5, 0.0],
        objects=[_box("leg_c", [1.2, 0.0, 0.17], [0.05, 0.05, 0.35]),
                 _box("far", [4.0, 0.0, _WZ], [0.2, 6.0, _WH])],
        # 路线是刻意画穿腿的，所以「到不到」和「摆几次」在这里没意义 —— 路线代价和
        # 避障代价本来就在互相拉。判据只有一条：压不压上去。
        budget_s=45, need_reach=False, max_flips=10 ** 6),
    # 上面那条腿是 0.35 m 高的，远在 z 跨度门限之上，所以它【永远看得见】。实测椅脚只有
    # 6~7 cm，这个场景把腿降到 0.065 m。
    # 🔴 2026-09-04 实测：**它不能用来调 TINYNAV_GRID_OFFSET_Z**，仿真在这件事上没有
    # 区分力，而且给出相反的结论（0.15 绕开 / 0.1625 压上去，真实数据是反过来的）。
    # 原因：仿真的深度是几何精确的，**一点噪声都没有**（planning_scene / map_volume 里
    # 搜 noise/random 零命中），而 z 相位要解决的正是"地面回波落在层边界的哪一侧"——
    # 没有噪声就没有这个现象。z 相位只能用 data/obstacle_scenes/*.npz 那种真实深度扫。
    # 这个场景仍然有用：判"规划器会不会直接开过一个矮盒子"。
    "chair_leg_low_head_on": dict(
        title="正对一条 6.5 cm 的矮椅腿",
        note="腿 6.5 cm 高，刚好卡在 z 跨度门限下面几毫米。判据只有一条：压不压上去。",
        route=[[0.0, 0.0], [2.5, 0.0]],
        start=[0.0, 0.0], yaw=0.0, target=[2.5, 0.0],
        objects=[_box("leg_low", [1.2, 0.0, 0.0325], [0.05, 0.05, 0.065]),
                 _box("far", [4.0, 0.0, _WZ], [0.2, 6.0, _WH])],
        budget_s=45, need_reach=False, max_flips=10 ** 6),
    "doorway_turn": dict(
        title="过门后要右转",
        note="0.9 m 门洞，出去之后目标在右侧 —— 贪心的终点距离在门里就想往右切。",
        route=[[0.0, 0.0], [1.2, 0.0], [2.2, -0.6], [2.2, -1.8]],
        start=[0.0, 0.0], yaw=0.0, target=[2.2, -1.8],
        objects=[_box("wall_left", [1.2, 1.15, _WZ], [0.2, 1.7, _WH]),
                 _box("wall_right", [1.2, -1.15, _WZ], [0.2, 1.7, _WH]),
                 _box("far", [3.2, 0.0, _WZ], [0.2, 4.0, _WH])],
        budget_s=90, need_reach=True, max_flips=10),
    # 2026-09-02 板上复刻：前方全开、目标偏 100°。100° 刚好压在 force_turn_heading_rad
    # (120°) 下面，于是走的是 no-progress 那条脱困分支而不是 heading。
    "target_aside_open": dict(
        title="前方全开 + 目标偏 100 度",
        note="没有任何障碍，目标在左后 100°。判据是到不到 —— 板上这一幕站了 35 s 不动，"
             "其间 fwd_ok 高达 88/90。",
        route=[[0.0, 0.0], [-0.35, 1.97]],
        start=[0.0, 0.0], yaw=0.0, target=[-0.35, 1.97],
        objects=[_box("far", [6.0, 0.0, _WZ], [0.2, 8.0, _WH])],
        budget_s=60, need_reach=True, max_flips=10),

    # ==== 2026-09-03 新增：全局路线【自己穿过障碍】的那一类 ====
    # 上面所有场景的 route 都是"正确的"（已经绕开障碍）。可是板上实测 46% 的决策里
    # route_clear < hard_clearance —— 全局搜索只认建图那一刻的占据图（map_node 一次性
    # np.load，全程零写入），建图之后搬进来的东西它永远看不见。那一趟 3 次真碰撞和
    # 17 次全速贴着走【全部】落在这一类里，而旧场景一个都没覆盖到。
    # 起点也刻意给偏 + 给转角：正对着能一条直线开过去的起点测不出转向期间的判据。
    # ---- 从上游搬来的三个场景（#240 的 web SCENARIOS + xiaole/planning-cost-and-s-bend
    # 放宽后的 S 弯）。上游那套只在 web 里存在、没有全局路线，这里补上 route 才能进我们
    # 的无头判据。走廊净宽都按上游原值，1.4 m / 1.9 m，比现场的 0.4 m 宽得多。
    "l_turn": dict(
        title="L 形走廊 + 内角（上游 #240）",
        note="1.4 m 宽的走廊往东走到底再左转往北。内角有个 0.3 m 方块。上游正在另一条分支"
             "(xiaole/smooth-global-plan)攻这个形状的 lookahead，我们这边也没有 L 角场景。",
        route=[[0.0, 0.0], [1.5, 0.0], [3.0, 0.0], [3.95, 0.0],
               [3.95, 1.2], [3.95, 2.6], [3.95, 4.3]],
        start=[0.0, 0.0], yaw=0.0, target=[3.9, 4.3],
        objects=[_box("lower_h_wall", [1.8, -0.85, 0.65], [5.6, 0.3, 1.3]),
                 _box("upper_h_wall_pre", [1.15, 0.85, 0.65], [4.3, 0.3, 1.3]),
                 _box("inside_corner", [3.45, 0.85, 0.65], [0.3, 0.3, 1.3]),
                 _box("left_v_wall", [3.15, 2.8, 0.65], [0.3, 3.6, 1.3]),
                 _box("right_v_wall", [4.85, 2.55, 0.65], [0.3, 5.1, 1.3]),
                 _box("entry_left", [-1.05, 0.85, 0.65], [0.8, 0.3, 1.3]),
                 _box("entry_right", [-1.05, -0.85, 0.65], [0.8, 0.3, 1.3]),
                 _box("far_cap", [4.0, 5.25, 0.65], [2.0, 0.3, 1.3])],
        budget_s=120, need_reach=True, max_flips=14),
    "s_bend": dict(
        title="S 弯（上游放宽版）",
        note="两个错开的挡板逼出一个 S：先抬到 y≈+0.2 过第一个，再压到 y≈-0.3 过第二个。"
             "上游原版「too tight」，这里用的是 xiaole 分支放宽后的尺寸。",
        route=[[0.0, -0.8], [1.2, -0.7], [2.0, -0.4], [2.65, 0.15], [3.4, 0.2],
               [4.15, -0.3], [5.0, -0.15], [5.8, 0.4]],
        start=[0.0, -0.8], yaw=0.0, target=[5.8, 0.4],
        objects=[_box("lower_entry", [1.4, -1.95, 0.65], [3.4, 0.22, 1.3]),
                 _box("upper_entry", [1.85, 1.06, 0.65], [4.3, 0.22, 1.3]),
                 _box("lower_exit", [4.7, -1.06, 0.65], [3.4, 0.22, 1.3]),
                 _box("upper_exit", [4.7, 1.95, 0.65], [3.4, 0.22, 1.3]),
                 _box("left_deflector", [2.65, -1.145, 0.65], [0.22, 1.39, 1.3]),
                 _box("right_deflector", [4.15, 1.145, 0.65], [0.22, 1.39, 1.3])],
        budget_s=120, need_reach=True, max_flips=16),
    "back_target": dict(
        title="目标在正后方，前方有墙（上游 #240）",
        note="车朝 +x，0.95 m 处一整面墙，目标在身后 1.35 m。必须原地掉头 —— 专门压"
             "「站着不动是代价最小值」和原地转方向那两条。",
        route=[[0.0, 0.0], [-0.5, 0.0], [-1.0, 0.0], [-1.35, 0.0]],
        start=[0.0, 0.0], yaw=0.0, target=[-1.35, 0.0],
        objects=[_box("front_block", [0.95, 0.0, 0.65], [0.25, 2.0, 1.3]),
                 _box("left_bound", [-0.35, 1.05, 0.65], [2.6, 0.22, 1.3]),
                 _box("right_bound", [-0.35, -1.05, 0.65], [2.6, 0.22, 1.3])],
        budget_s=75, need_reach=True, max_flips=12),
    "route_thru_wall": dict(
        title="路线穿墙 + 起点偏斜",
        note="建图时路中间没东西，之后放了一堵 0.6x0.5 的墙，而全局路线还是直穿它。"
             "起点偏 (-0.4,-0.5)、偏航 +40°，要先转过来。判据是到不到、以及最小净空。",
        route=[[0.0, 0.0], [3.0, 0.0]],
        start=[-0.4, -0.5], yaw=40.0, target=[3.0, 0.0],
        objects=[_box("new_wall", [1.5, 0.0, _WZ], [0.6, 0.5, _WH]),
                 _box("side_l", [1.5, 1.3, _WZ], [3.0, 0.2, _WH]),
                 _box("side_r", [1.5, -1.3, _WZ], [3.0, 0.2, _WH]),
                 _box("far", [3.8, 0.0, _WZ], [0.2, 3.0, _WH])],
        budget_s=90, need_reach=True, max_flips=12),
    "route_grazes_leg": dict(
        title="路线擦着椅子腿 + 起点偏斜",
        note="路线沿 y=0 直行，一条腿在 (1.4,+0.12) —— route_clear 约 0.10，即路线本身"
             "低于碰撞门限，但只要往右让 5~10 cm 就能过。板上占比最大的一档（253/710），"
             "也是「贴着障碍走」的原型。起点 (-0.5,+0.7) 偏航 -50°。",
        route=[[0.0, 0.0], [3.0, 0.0]],
        start=[-0.5, 0.7], yaw=-50.0, target=[2.6, 0.0],
        objects=[_box("leg", [1.4, 0.12, 0.17], [0.05, 0.05, 0.35]),
                 _box("far", [3.4, 0.0, _WZ], [0.2, 3.0, _WH])],
        budget_s=75, need_reach=True, max_flips=10),
    # ⚠️ 预期难：解钉只清"贴合"项，w_route_progress 仍然按剩余弧长拉向那扇【被堵的】门。
    # 留着当上限探针 —— 它不过说明"要绕到另一条通路"需要的是重规划，不是解钉。
    "route_thru_blocked_door": dict(
        title="路线指向已被堵死的门（预期难）",
        note="路线走左门，左门后来被封了，真正能过的是右门（偏 1.4 m）。测解钉的上限："
             "只清贴合项救不了「进展项还指着堵住的那条路」。",
        route=[[0.0, 0.0], [1.2, 0.7], [2.4, 0.7]],
        start=[-0.3, -0.2], yaw=25.0, target=[2.6, 0.0],
        objects=[_box("door_block", [1.2, 0.7, _WZ], [0.25, 0.7, _WH]),
                 _box("pier_mid", [1.2, -0.05, _WZ], [0.25, 0.6, _WH]),
                 _box("pier_top", [1.2, 1.6, _WZ], [0.25, 1.2, _WH]),
                 _box("pier_bot", [1.2, -1.5, _WZ], [0.25, 1.4, _WH]),
                 _box("far", [3.2, 0.0, _WZ], [0.2, 4.0, _WH])],
        budget_s=90, need_reach=False, max_flips=40),
}
DEFAULT_SCENE = "wall_ahead"


def scene_catalog() -> list[dict[str, Any]]:
    """给 web 下拉框用的清单。"""
    return [{"name": k, "title": v.get("title", k), "note": v["note"],
             "need_reach": bool(v["need_reach"])} for k, v in SCENES.items()]


# 执行误差模型。规划器打分用的是它自己发出去的弧，默认假设完美执行；真机有增益、死区、
# 滞后。这几档是用来回答"多大的偏差会撞"的。
ACTUATORS: dict[str, dict[str, Any]] = {
    "perfect": {"vx_gain": 1.0, "wz_gain": 1.0,
                "vx_deadband": 0.0, "wz_deadband": 0.0, "latency_s": 0.0},
    # 你举的那个例子：vx 0.2->0.18，w 0.1->0.08
    "gain_10_20": {"vx_gain": 0.90, "wz_gain": 0.80,
                   "vx_deadband": 0.0, "wz_deadband": 0.0, "latency_s": 0.0},
    "gain_20_40": {"vx_gain": 0.80, "wz_gain": 0.60,
                   "vx_deadband": 0.0, "wz_deadband": 0.0, "latency_s": 0.0},
    # 低速转不动：静摩擦让小指令完全没输出，脱困时的原地转最容易踩到
    "deadband": {"vx_gain": 1.0, "wz_gain": 1.0,
                 "vx_deadband": 0.06, "wz_deadband": 0.20, "latency_s": 0.0},
    # 指令排队。板上实测控制滞后 p90~0.8 s
    "latency_400ms": {"vx_gain": 1.0, "wz_gain": 1.0,
                      "vx_deadband": 0.0, "wz_deadband": 0.0, "latency_s": 0.4},
    "latency_800ms": {"vx_gain": 1.0, "wz_gain": 1.0,
                      "vx_deadband": 0.0, "wz_deadband": 0.0, "latency_s": 0.8},
    # 全都有
    "realistic": {"vx_gain": 0.85, "wz_gain": 0.75,
                  "vx_deadband": 0.04, "wz_deadband": 0.12, "latency_s": 0.3},
}


def apply_scene(cfg: dict[str, Any], name: str,
                camera: str = DEFAULT_CAMERA,
                actuator: str = "perfect") -> dict[str, Any]:
    """把场景和我们的相机套到一份 default-config 上，返回新的 config。"""
    sc = SCENES.get(name)
    if sc is None:
        raise KeyError(name)
    out = copy.deepcopy(cfg)
    cam, _step = camera_and_step(camera)
    out["camera"] = {**out.get("camera", {}), **cam}
    out["start"] = {"xy": list(sc["start"]), "yaw_deg": float(sc["yaw"])}
    out["target"] = [float(sc["target"][0]), float(sc["target"][1]), 0.0]
    out["objects"] = copy.deepcopy(sc["objects"])
    out["map_name"] = None
    out["map_path"] = None
    out["scene_name"] = name
    # map_node 会给的那条"已经绕开静态障碍"的全局路线。仿真里没有 map_node，所以手写。
    out["route"] = copy.deepcopy(sc.get("route") or [])
    out["actuator"] = copy.deepcopy(ACTUATORS.get(actuator, ACTUATORS["perfect"]))
    out["actuator_name"] = actuator
    return out
