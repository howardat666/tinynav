"""
WebSocket endpoints:
  WS /ws/status      — pushes device status every ~1 s
  WS /ws/pose        — pushes pose whenever a new Odometry arrives
  WS /ws/map-update  — pushes a notification when map files change
  WS /ws/preview     — streams JPEG frames for a given image topic
  WS /ws/planning    — polls planning snapshot at 5 fps
  WS /ws/teleop      — receives cmd_vel commands from the client
"""
from __future__ import annotations

import asyncio
import base64
import json
import os
import time

from starlette.websockets import WebSocketState
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Query

from .manager_client import get_manager_json, is_display_role
from .state import runner

router = APIRouter(tags=['ws'])

_MANAGER_STATUS_CACHE_TTL_S = 0.5
_manager_status_cache: dict | None = None
_manager_status_cache_time = 0.0


def _safe_put(queue: asyncio.Queue, item):
    """Put item onto queue, dropping the oldest entry if full."""
    if queue.full():
        try:
            queue.get_nowait()
        except asyncio.QueueEmpty:
            pass
    queue.put_nowait(item)


def _connected(ws: WebSocket) -> bool:
    return (
        ws.client_state == WebSocketState.CONNECTED
        and ws.application_state == WebSocketState.CONNECTED
    )


def _get_cached_manager_status() -> dict | None:
    global _manager_status_cache
    global _manager_status_cache_time

    now = time.monotonic()
    if now - _manager_status_cache_time <= _MANAGER_STATUS_CACHE_TTL_S:
        return _manager_status_cache

    _manager_status_cache = get_manager_json('/device/status')
    _manager_status_cache_time = now
    return _manager_status_cache


def _clear_stopped_nav_snapshot(snapshot: dict) -> dict:
    if not is_display_role():
        return snapshot

    status = _get_cached_manager_status()
    if status is None or status.get('navNodesRunning', False):
        return snapshot

    # The display backend owns ROS telemetry subscriptions and may still hold
    # the last localization/path frame after the manager stops nav nodes.
    snapshot = dict(snapshot)
    snapshot.update({
        'localized': False,
        'odom_pose_at_kf': None,
        'map_pose': None,
        'esdf_image': None,
        'obstacle_image': None,
        'trajectory': [],
        'global_path': [],
        'map_global_path': [],
        'grid_info': None,
        'nav_target_pose': None,
        'footprint': [],
        'voxel_points': [],
    })
    return snapshot


# --------------------------------------------------------------------------- #
# /ws/status  — polls node state every 1 s and broadcasts                     #
# --------------------------------------------------------------------------- #

@router.websocket('/ws/status')
async def ws_status(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            if is_display_role():
                status = get_manager_json('/device/status')
                payload = json.dumps(status if status is not None else {'online': False})
            else:
                node = runner.node
                if node is not None:
                    node.client_seen('status')   # 真正的 UI 通道；HTTP 的 /device/status 不算
                    payload = json.dumps({'online': True, **node.get_status()})
                else:
                    payload = json.dumps({'online': False})
            await ws.send_text(payload)
            await asyncio.sleep(1.0)
    except WebSocketDisconnect:
        pass


# --------------------------------------------------------------------------- #
# /ws/pose  — pushed on every new odometry message                            #
# --------------------------------------------------------------------------- #

@router.websocket('/ws/pose')
async def ws_pose(ws: WebSocket):
    await ws.accept()
    queue: asyncio.Queue = asyncio.Queue(maxsize=10)

    loop = asyncio.get_event_loop()

    def _on_pose(pose: dict):
        # Called from rclpy spin thread — schedule onto event loop.
        loop.call_soon_threadsafe(lambda: _safe_put(queue, pose))

    node = runner.node
    if node is None:
        await ws.close(code=1013)
        return

    node.pose_callbacks.append(_on_pose)
    try:
        while True:
            # 位姿订阅按客户端在场动态建销，所以这里必须续期 —— 否则一个只开
            # /ws/pose 的客户端会把自己的数据源饿死
            node.client_seen('pose')
            try:
                pose = await asyncio.wait_for(queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                if not _connected(ws):
                    break
                continue
            await ws.send_text(json.dumps(pose))
    except WebSocketDisconnect:
        pass
    finally:
        try:
            node.pose_callbacks.remove(_on_pose)
        except ValueError:
            pass


# --------------------------------------------------------------------------- #
# /ws/nav-progress  — pushed on every /mapping/nav_progress message           #
# --------------------------------------------------------------------------- #

@router.websocket('/ws/nav-progress')
async def ws_nav_progress(ws: WebSocket):
    await ws.accept()
    queue: asyncio.Queue = asyncio.Queue(maxsize=10)

    loop = asyncio.get_event_loop()

    def _on_progress(data: dict):
        # Called from the rclpy spin thread — schedule onto the event loop.
        loop.call_soon_threadsafe(lambda: _safe_put(queue, data))

    node = runner.node
    if node is None:
        await ws.close(code=1013)
        return

    node.nav_progress_callbacks.append(_on_progress)
    try:
        while True:
            # Bounded wait, not a bare queue.get(): progress only flows while a POI is
            # active, so an idle robot would leave this coroutine parked forever and
            # the callback registered long after the browser tab went away.
            try:
                data = await asyncio.wait_for(queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                if not _connected(ws):
                    break
                continue
            await ws.send_text(json.dumps(data))
    except WebSocketDisconnect:
        pass
    finally:
        try:
            node.nav_progress_callbacks.remove(_on_progress)
        except ValueError:
            pass


# --------------------------------------------------------------------------- #
# /ws/map-update  — polls for occupancy_grid.npy mtime changes               #
# --------------------------------------------------------------------------- #

@router.websocket('/ws/map-update')
async def ws_map_update(ws: WebSocket):
    await ws.accept()
    node = runner.node
    if node is None:
        await ws.close(code=1013)
        return

    grid_file = os.path.join(node.map_path, 'occupancy_grid.npy')
    last_mtime: float = 0.0

    try:
        while True:
            try:
                mtime = os.path.getmtime(grid_file)
            except OSError:
                mtime = 0.0

            if mtime != last_mtime and mtime != 0.0:
                last_mtime = mtime
                await ws.send_text(json.dumps({
                    'event': 'map_updated',
                    'timestamp': mtime,
                }))
            await asyncio.sleep(2.0)
    except WebSocketDisconnect:
        pass


# --------------------------------------------------------------------------- #
# /ws/planning  — polls planning snapshot at 5 fps                            #
# --------------------------------------------------------------------------- #

async def _planning_view_mode_reader(ws: WebSocket, node):
    """Read {"voxels": bool} from the client for as long as it keeps talking.

    The socket used to be send-only. Reading on it is what lets the local view's 2D/3D
    button reach the publisher without reconnecting -- a reconnect would blank the view
    for the length of the retry, on a control page, to save bandwidth.
    """
    try:
        while True:
            msg = json.loads(await ws.receive_text())
            if isinstance(msg, dict) and 'voxels' in msg:
                node.set_want_voxels(bool(msg['voxels']))
    except (WebSocketDisconnect, json.JSONDecodeError, KeyError, RuntimeError):
        pass


@router.websocket('/ws/planning')
async def ws_planning(ws: WebSocket):
    await ws.accept()
    node = runner.node
    if node is None:
        await ws.close(code=1013)
        return
    # Counted, not just flagged: planning_node stops producing the local-view layers
    # while this is zero, so a second browser must not switch them off for the first.
    node.ui_client_attach()
    reader = asyncio.create_task(_planning_view_mode_reader(ws, node))
    try:
        while True:
            # 每轮查连接 + 续期心跳。光靠 finally 里的 detach 不够：连接被黑洞化时
            # send_text 会挂住，detach 永远不执行，planning 就一直为没人看的画面渲染。
            if not _connected(ws):
                break
            node.ui_client_seen()
            snapshot = _clear_stopped_nav_snapshot(node.get_planning_snapshot())
            payload = json.dumps(snapshot)
            await ws.send_text(payload)
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass
    finally:
        reader.cancel()
        node.ui_client_detach()


# --------------------------------------------------------------------------- #
# /ws/preview  — streams JPEG frames for a given image topic                  #
# --------------------------------------------------------------------------- #

@router.websocket('/ws/preview')
async def ws_preview(ws: WebSocket, topic: str = Query(...)):
    await ws.accept()

    node = runner.node
    if node is None or topic not in node.preview_callbacks:
        await ws.close(code=1013)
        return

    # Depth 1, not 4: on WiFi the uplink is the bottleneck (~2.5 Mbit/s measured, see
    # docs/x5/board_bringup.md 2.5), so a deeper queue only converts bandwidth shortage
    # into latency -- the viewer ends up watching frames that are already stale.
    queue: asyncio.Queue = asyncio.Queue(maxsize=1)
    loop = asyncio.get_event_loop()

    def _on_frame(frame: bytes):
        # Drop oldest frame if full — always keep the latest.
        loop.call_soon_threadsafe(lambda: _safe_put(queue, frame))

    registered = False
    try:
        if not node.add_preview_callback(topic, _on_frame):
            await ws.close(code=1013)
            return
        registered = True
    except Exception as e:
        print(f'Preview subscription failed for {topic}: {e}', flush=True)
        await ws.close(code=1013)
        return
    try:
        while True:
            # 每轮都查连接、每轮都续期心跳。原来只在【超时分支】查 _connected，
            # 而帧一直来时 queue.get() 永不超时，死连接上的 send_bytes 又能无限缓冲，
            # 于是这个循环永不退出、finally 永不执行、订阅永久泄漏。
            if not _connected(ws):
                break
            node.preview_seen(topic)
            try:
                frame = await asyncio.wait_for(queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            # Binary, not base64 text: base64 costs a flat 33% on a link that is
            # already the bottleneck. The frontend's decodeFrame has always accepted
            # both, so this needs no coordinated release.
            await ws.send_bytes(frame)
    except WebSocketDisconnect:
        pass
    finally:
        if registered:
            node.remove_preview_callback(topic, _on_frame)


# --------------------------------------------------------------------------- #
# /ws/teleop  — receives velocity commands and publishes to /cmd_vel          #
# --------------------------------------------------------------------------- #

# How often the last command is re-published while the socket is open, and the
# watchdog it exists to satisfy. wheel_odometry_node zeroes the wheels when no
# /cmd_vel has arrived for cmd_timeout_s (0.5 s by default), which is correct for
# a robot whose controller may die mid-drive.
#
# The browser cannot satisfy that on its own: the joystick sends only on *change*,
# so holding it steady sends nothing, and the robot drives for half a second and
# stops until the stick is jiggled. Repeating here rather than in the frontend
# keeps the watchdog contract where it belongs -- the browser should not have to
# know a ROS timeout -- and it also covers network stalls, which a client-side
# timer would not.
_TELEOP_REPEAT_HZ = 20.0
_TELEOP_REPEAT_PERIOD = 1.0 / _TELEOP_REPEAT_HZ
# 重发的有效期。客户端非零时按 10 Hz 发心跳（operate_tab.dart 的 _teleopHeartbeat），
# 所以 0.5 s 等于连丢 5 条才刹车。没有这个上限时，松手那一条被链路延迟卡住的整段时间里
# 重发线程会一直发上一条非零指令 —— 实测链路 ping 会跳到 2 s，那就是 2 s 的"后摇"。
# WebSocket 走 TCP，消息不会丢只会迟，所以这里判的是"操作者还在不在说话"，不是丢包。
_TELEOP_STALE_S = 0.5


@router.websocket('/ws/teleop')
async def ws_teleop(ws: WebSocket):
    await ws.accept()
    node = runner.node
    if node is None:
        await ws.close(code=1013)
        return

    last = (0.0, 0.0, 0.0)
    last_at = 0.0
    stop = asyncio.Event()

    async def repeat():
        """Re-publish the latest command until it goes stale or the socket closes."""
        nonlocal last
        while not stop.is_set():
            # Zero is the resting state, which the watchdog already produces;
            # republishing it would only race a real command arriving.
            if last != (0.0, 0.0, 0.0):
                if time.monotonic() - last_at > _TELEOP_STALE_S:
                    last = (0.0, 0.0, 0.0)
                    try:
                        node.publish_cmd_vel(0.0, 0.0, 0.0)
                    except Exception:
                        pass
                else:
                    try:
                        node.publish_cmd_vel(*last)
                    except Exception:
                        pass
            await asyncio.sleep(_TELEOP_REPEAT_PERIOD)

    repeater = asyncio.create_task(repeat())
    try:
        while True:
            data = await ws.receive_text()
            msg = json.loads(data)
            last = (
                float(msg.get('linear_x', 0.0)),
                float(msg.get('linear_y', 0.0)),
                float(msg.get('angular_z', 0.0)),
            )
            last_at = time.monotonic()
            node.publish_cmd_vel(*last)
    except WebSocketDisconnect:
        pass
    finally:
        stop.set()
        repeater.cancel()
        try:
            # Explicit zero on the way out: closing the tab must stop the robot,
            # and it must not have to wait out the watchdog to do it.
            node.publish_cmd_vel(0.0, 0.0, 0.0)
        except Exception:
            pass
