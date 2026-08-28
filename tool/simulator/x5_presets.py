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
    "TINYNAV_MIN_WALL_SPAN_M": "0.2",
    "TINYNAV_DILATION_CELLS": "0",
    "TINYNAV_LOW_OBS": "1",
    "TINYNAV_LOW_OBS_H_LO": "0.05",
    "TINYNAV_LOW_OBS_RANGE_M": "1.5",
    "TINYNAV_LOW_OBS_MIN_PTS": "2",
    "TINYNAV_CAMERA_HEIGHT_M": "0.18",
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
        "mount_height": 0.18,      # Looper 实装高度
        "_raycast_step": step,
    }


CAMERAS: dict[str, dict[str, Any]] = {
    # 默认。视场和角分辨率都和实机一致，渲染 25 ms/帧。
    "match": _camera(0.4, 2),
    # 逐像素和实机一致。渲染 263 ms/帧 -> 仿真掉到约 3.8 Hz，只在需要复核时用。
    "full": _camera(1.0, _REAL_STEP),
    # 上游默认，快但视场和角分辨率都不是我们的，只适合看流程通不通。
    "fast": {"width": 160, "image_height": 100, "fx": 80.0, "fy": 50.0,
             "max_range": 15.0, "mount_height": 0.18, "_raycast_step": 5},
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
    "doorway_turn": dict(
        title="过门后要右转",
        note="0.9 m 门洞，出去之后目标在右侧 —— 贪心的终点距离在门里就想往右切。",
        route=[[0.0, 0.0], [1.2, 0.0], [2.2, -0.6], [2.2, -1.8]],
        start=[0.0, 0.0], yaw=0.0, target=[2.2, -1.8],
        objects=[_box("wall_left", [1.2, 1.15, _WZ], [0.2, 1.7, _WH]),
                 _box("wall_right", [1.2, -1.15, _WZ], [0.2, 1.7, _WH]),
                 _box("far", [3.2, 0.0, _WZ], [0.2, 4.0, _WH])],
        budget_s=90, need_reach=True, max_flips=10),
}
DEFAULT_SCENE = "wall_ahead"


def scene_catalog() -> list[dict[str, Any]]:
    """给 web 下拉框用的清单。"""
    return [{"name": k, "title": v["title"], "note": v["note"],
             "need_reach": bool(v["need_reach"])} for k, v in SCENES.items()]


def apply_scene(cfg: dict[str, Any], name: str,
                camera: str = DEFAULT_CAMERA) -> dict[str, Any]:
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
    return out
