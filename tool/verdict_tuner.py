#!/usr/bin/env python3
"""判决图手动调参：拖滑条改参数，图立刻重画。

为什么要有这个：障碍参数（z 跨度门限、栅格 z 相位、波段下界…）在板上改一轮要重启
planning + 等 30 s，六组就是四五分钟；而这些参数的好坏最终要靠人看图判断 ——
"那个椅子腿到底被认了没有"没法只靠数字回答。

跑的是**仓库里真的 kernel**（run_raycasting_loopy / run_raycasting_split /
build_obstacle_map / classify_verdict），数据是板上真录的深度序列，所以调出来的参数
直接就能填到 app_start.sh 里。

    scripts/run_verdict_tuner.sh          # 然后开 http://127.0.0.1:8768

参数分两级：
  · 【即时】改完 <100 ms 重画：z 跨度门限 / 波段上下界 / 占据门限 / 膨胀 / 离地界 / 底图
  · 【重算】要重跑射线累积，几秒：场景 / 帧数 / 栅格 z 偏置 / hit step / carve step
"""
from __future__ import annotations

import base64
import dataclasses
import json
import os
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tinynav.core.planning_node import (  # noqa: E402
    _VERDICT_LUT, _VERDICT_NAMES, build_obstacle_map, classify_verdict,
    min_visible_height_m, roll_occupancy_grid, tint_verdict)
from tinynav.core.planning_kernels import run_raycasting_loopy  # noqa: E402
from tinynav.core.raycast_split import run_raycasting_split  # noqa: E402
from tinynav.core.robot_config import robot_config  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(ROOT, 'data', 'obstacle_scenes')
PORT = int(os.environ.get('TINYNAV_TUNER_PORT', '8768'))
PLATFORM = os.environ.get('TINYNAV_ROBOT_TYPE', 'diffcar')

# 和 planning_node 对齐，别在这里"顺手改好一点"—— 这些是板上写死的。
GRID_SHAPE = (80, 80, 14)
RESOLUTION = 0.05
DECAY_REF_DT = 0.23
RECENTER_M = 0.1
STRIDE = 2

_ROBOT = robot_config(PLATFORM)
_CACHE: dict = {}          # 累积好的栅格，按【重算级】参数缓存
_CACHE_MAX = 8             # 一档约 8 MB(栅格) + 渲染中间量，留 8 档让来回切参数是瞬时的
_NPZ: dict = {}            # 解压后的原始序列
_NPZ_MAX = 2               # 一段 250x640x544 uint16 = 174 MB，PC 上留两段够换着比
_LOCK = threading.Lock()
_PREFETCH: set = set()     # 后台正在预算的 key，避免同一档算两遍


def _scenes():
    if not os.path.isdir(SCENE_DIR):
        return []
    return sorted(f[:-4] for f in os.listdir(SCENE_DIR) if f.endswith('.npz'))


def _camera_height():
    return float(os.environ.get('TINYNAV_CAMERA_HEIGHT_M', '0.124'))


def load_scene(scene):
    """解压后的序列单独缓存：np.load 一段 59 MB 的 npz 实测 0.93 s（解压到 174 MB），
    换个 grid_offset_z 不该重付这笔钱。真正的大头是累积本身，250 帧 3~5 s。"""
    with _LOCK:
        if scene in _NPZ:
            return _NPZ[scene]
    z = np.load(os.path.join(SCENE_DIR, f'{scene}.npz'))
    out = {k: z[k] for k in ('depth_mm', 'poses', 'K', 'stamps', 'infra1', 'note')}
    with _LOCK:
        while len(_NPZ) >= _NPZ_MAX:
            _NPZ.pop(next(iter(_NPZ)))
        _NPZ[scene] = out
    return out


def accumulate(scene, frames, goff_z, hit_step, carve_step):
    """把整段序列的占据栅格攒出来，逐行照抄 planning_node._plan_once 的顺序。"""
    key = (scene, frames, round(goff_z, 4), hit_step, carve_step)
    with _LOCK:
        if key in _CACHE:
            return _CACHE[key]
    z = load_scene(scene)
    depth_mm, poses, K = z['depth_mm'], z['poses'], z['K']
    stamps = z['stamps']
    frames = int(min(frames, len(poses)))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    grid_offset = np.array([0.0, 0.0, goff_z])
    origin = np.array(GRID_SHAPE) * RESOLUTION / -2.0 + grid_offset
    grid = np.zeros(GRID_SHAPE)
    last_stamp = None
    t0 = time.perf_counter()
    for n in range(frames):
        T = poses[n]
        # 相机位姿就是控制点（这个场景是静止采的，camera_to_robot_center 的偏置不影响
        # 重定心判据，因为它整段都是同一个常量）。
        robot_pos = T[:3, 3]
        center = origin + np.array(GRID_SHAPE) * RESOLUTION / 2.0 - grid_offset
        if np.linalg.norm(robot_pos - center) > RECENTER_M:
            new_origin = (robot_pos - np.array(GRID_SHAPE) * RESOLUTION / 2.0 + grid_offset)
            grid, origin = roll_occupancy_grid(grid, origin, new_origin, RESOLUTION)
        d = depth_mm[n].astype(np.float32) / 1000.0
        if hit_step == carve_step:
            new_occ = run_raycasting_loopy(d, T, GRID_SHAPE, fx, fy, cx, cy,
                                           origin, hit_step, RESOLUTION)
        else:
            new_occ = run_raycasting_split(d, T, GRID_SHAPE, fx, fy, cx, cy,
                                           origin, hit_step, carve_step, RESOLUTION)
        st = float(stamps[n])
        dt = (DECAY_REF_DT if last_stamp is None
              else min(max(st - last_stamp, DECAY_REF_DT), 8.0 * DECAY_REF_DT))
        last_stamp = st
        grid *= 0.99 ** (dt / DECAY_REF_DT)
        grid += new_occ
        np.clip(grid, -0.2, 0.2, out=grid)
    ms = 1000.0 * (time.perf_counter() - t0)

    # 渲染用最后一帧
    T = poses[frames - 1]
    d = depth_mm[frames - 1].astype(np.float32)[::STRIDE, ::STRIDE] / 1000.0
    v, u = np.mgrid[0:depth_mm.shape[1]:STRIDE, 0:depth_mm.shape[2]:STRIDE]
    gu = (u.astype(np.float32) - cx) / fx
    gv = (v.astype(np.float32) - cy) / fy
    R = T[:3, :3]
    pw_x = d * (R[0, 0] * gu + R[0, 1] * gv + R[0, 2]) + T[0, 3]
    pw_y = d * (R[1, 0] * gu + R[1, 1] * gv + R[1, 2]) + T[1, 3]
    pw_z = d * (R[2, 0] * gu + R[2, 1] * gv + R[2, 2]) + T[2, 3]
    infra = z['infra1']
    infra = infra[::STRIDE, ::STRIDE] if infra.shape[:2] == depth_mm.shape[1:] else None
    out = dict(grid=grid, origin=origin, T=T, d=d, pw=(pw_x, pw_y, pw_z), infra=infra,
               note=str(z['note']), frames=frames, ms=ms, total=len(poses))
    with _LOCK:
        while len(_CACHE) >= _CACHE_MAX:
            _CACHE.pop(next(iter(_CACHE)))      # dict 保序 = 先进先出
        _CACHE[key] = out
    return out


def prefetch(scene, frames, goff_z, hit_step, carve_step):
    """后台把 grid_offset_z 的相邻两档先算出来 —— 这个滑条一定是来回拖着比的。"""
    for g in (round(goff_z - 0.0125, 4), round(goff_z + 0.0125, 4)):
        if not (0.10 <= g <= 0.22):
            continue
        key = (scene, frames, g, hit_step, carve_step)
        with _LOCK:
            if key in _CACHE or key in _PREFETCH:
                continue
            _PREFETCH.add(key)
        try:
            accumulate(scene, frames, g, hit_step, carve_step)
        except Exception:
            pass
        finally:
            with _LOCK:
                _PREFETCH.discard(key)


def render(prep, cfg_over, low_h, base_mode, focus):
    grid, origin, T = prep['grid'], prep['origin'], prep['T']
    d = prep['d']
    pw_x, pw_y, pw_z = prep['pw']
    cfg = dataclasses.replace(_ROBOT.obstacle, **cfg_over)
    cam_h = _camera_height()
    mask = build_obstacle_map(grid, origin, RESOLUTION, robot_z=T[2, 3], config=cfg)
    verdict, st = classify_verdict(grid, origin, RESOLUTION, T[2, 3], cam_h,
                                   mask, d, pw_x, pw_y, pw_z, low_h, cfg)
    if base_mode == 'infra' and prep['infra'] is not None:
        base = cv2.cvtColor(prep['infra'], cv2.COLOR_GRAY2BGR)
    elif base_mode == 'flat':
        base = np.zeros(d.shape + (3,), np.uint8)
    else:
        vv = np.clip((3.0 - d) / 2.8, 0.0, 1.0) * 0.70 + 0.30
        vv[d <= 0] = 0.22
        base = cv2.cvtColor((vv * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    if focus:
        # 只留"障碍"和"矮物被否"两类的颜色，其余按底图原样出 —— 专门用来回答
        # "这个东西被认了没有"，别的类别全是干扰。
        keep = (verdict == 2) | (verdict == 6)
        v2 = np.where(keep, verdict, 1).astype(np.uint8)
        img = tint_verdict(base, v2)
        img[:base.shape[0]][~keep] = base[~keep]
    else:
        img = tint_verdict(base, verdict)
    ok, buf = cv2.imencode('.png', img)
    tot = verdict.size
    cnt = np.bincount(verdict.ravel(), minlength=len(_VERDICT_NAMES))
    mvh = min_visible_height_m(origin[2], T[2, 3], GRID_SHAPE[2], RESOLUTION, cam_h, cfg)
    return dict(
        png=base64.b64encode(buf.tobytes()).decode() if ok else '',
        pct=[round(100.0 * c / tot, 1) for c in cnt],
        names=list(_VERDICT_NAMES),
        lut=[[int(x) for x in _VERDICT_LUT[k]] for k in range(len(_VERDICT_LUT))],
        cells=st,
        min_visible_mm=None if not np.isfinite(mvh) else round(mvh * 1000),
        origin_z=round(float(origin[2]), 4), robot_z=round(float(T[2, 3]), 4),
        floor_z=round(float(T[2, 3] - cam_h), 4), cam_h=cam_h,
        band_floor_lo=round(cfg.robot_z_bottom + cam_h, 4),
        band_floor_hi=round(cfg.robot_z_top + cam_h, 4),
        note=prep['note'], frames=prep['frames'], total=prep['total'],
        accum_ms=round(prep['ms']),
        has_infra=prep['infra'] is not None,
    )


HTML = r"""<!doctype html><meta charset=utf-8><title>判决图调参</title>
<style>
body{background:#14161a;color:#dfe3e8;font:13px/1.5 system-ui,sans-serif;margin:0;display:flex}
#side{width:330px;padding:14px;background:#1b1e24;overflow:auto;height:100vh;box-sizing:border-box}
#main{flex:1;padding:14px;overflow:auto;height:100vh;box-sizing:border-box}
h3{margin:14px 0 6px;font-size:12px;letter-spacing:.08em;color:#8b94a0;text-transform:uppercase}
h3:first-child{margin-top:0}
.row{margin:7px 0}
.row label{display:flex;justify-content:space-between;font-size:12px;color:#aeb6c2}
.row b{color:#f0f3f7;font-variant-numeric:tabular-nums}
input[type=range]{width:100%;margin:2px 0 0}
select,button{background:#252a32;color:#dfe3e8;border:1px solid #363d47;border-radius:4px;padding:5px 8px;font:inherit;width:100%}
button{cursor:pointer;margin-top:8px}
button.go{background:#2d6cdf;border-color:#2d6cdf;font-weight:600}
img{width:min(100%,760px);image-rendering:pixelated;border:1px solid #2a2f37;background:#000}
table{border-collapse:collapse;font-variant-numeric:tabular-nums;margin-top:10px}
td,th{padding:3px 9px;border-bottom:1px solid #262b33;text-align:left}
.sw{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:6px;vertical-align:-1px}
.meta{color:#8b94a0;font-size:12px;margin-top:10px;line-height:1.7}
.warn{color:#ffb347}
.tag{display:inline-block;background:#252a32;border-radius:3px;padding:1px 6px;margin-right:4px;font-size:11px}
label.ck{display:flex;gap:7px;align-items:center;color:#aeb6c2;font-size:12px;margin:6px 0}
</style>
<div id=side>
  <h3>重算级 <span class=warn>(改完点重算)</span></h3>
  <div class=row><label>场景</label><select id=scene></select></div>
  <div class=row><label>帧数 <b id=vframes></b></label><input type=range id=frames min=20 max=250 step=10 value=250></div>
  <div class=row><label>栅格 z 偏置 grid_offset_z <b id=vgoff></b></label><input type=range id=goff min=0.10 max=0.22 step=0.0125 value=0.1625></div>
  <div class=row><label>hit step (标命中,密) <b id=vhit></b></label><input type=range id=hit min=1 max=6 step=1 value=3></div>
  <div class=row><label>carve step (刻空,疏) <b id=vcarve></b></label><input type=range id=carve min=1 max=12 step=1 value=3></div>
  <button class=go id=go>重算</button>

  <h3>即时级</h3>
  <div class=row><label>z 跨度门限 min_wall_span_m <b id=vspan></b></label><input type=range id=span min=0.05 max=0.40 step=0.05 value=0.10></div>
  <div class=row><label>波段下界 robot_z_bottom <b id=vzbot></b></label><input type=range id=zbot min=-0.40 max=0.00 step=0.01 value=-0.16></div>
  <div class=row><label>波段上界 robot_z_top <b id=vztop></b></label><input type=range id=ztop min=0.10 max=0.60 step=0.02 value=0.40></div>
  <div class=row><label>占据门限 occ_threshold <b id=vocc></b></label><input type=range id=occ min=0.02 max=0.20 step=0.01 value=0.10></div>
  <div class=row><label>膨胀 dilation_cells <b id=vdil></b></label><input type=range id=dil min=0 max=3 step=1 value=0></div>
  <div class=row><label>离地界 verdict_low_h <b id=vlow></b></label><input type=range id=low min=0.00 max=0.15 step=0.01 value=0.03></div>
  <div class=row><label>底图</label><select id=base><option value=infra>红外图</option><option value=depth>深度亮度</option><option value=flat>纯平涂</option></select></div>
  <label class=ck><input type=checkbox id=focus> 只显示「障碍」和「矮物被否」</label>
</div>
<div id=main>
  <img id=img>
  <div id=stats></div>
  <div class=meta id=meta></div>
</div>
<script>
const E=id=>document.getElementById(id);
const CHEAP=['span','zbot','ztop','occ','dil','low','base','focus'];
const EXP=['scene','frames','goff','hit','carve'];
function show(){
  E('vframes').textContent=E('frames').value + (E('frames').value<200?' ⚠ 短于稳态':'');
  E('vgoff').textContent=(+E('goff').value).toFixed(4);
  E('vhit').textContent=E('hit').value;
  E('vcarve').textContent=E('carve').value;
  E('vspan').textContent=(+E('span').value).toFixed(2)+' m';
  E('vzbot').textContent=(+E('zbot').value).toFixed(2);
  E('vztop').textContent=(+E('ztop').value).toFixed(2);
  E('vocc').textContent=(+E('occ').value).toFixed(2);
  E('vdil').textContent=E('dil').value;
  E('vlow').textContent=(+E('low').value).toFixed(2)+' m';
}
function q(extra){
  const p=new URLSearchParams();
  for(const k of EXP.concat(CHEAP)){
    const el=E(k); p.set(k, el.type==='checkbox'?(el.checked?1:0):el.value);
  }
  return p.toString();
}
let busy=false, pending=false;
async function go(prepare){
  if(busy){pending=true;return;}
  busy=true;
  E('go').textContent = prepare? '重算中…' : '重算';
  try{
    const r=await fetch('/api/'+(prepare?'prepare':'render')+'?'+q());
    const j=await r.json();
    if(j.error){E('meta').innerHTML='<span class=warn>'+j.error+'</span>';return;}
    E('img').src='data:image/png;base64,'+j.png;
    let h='<table><tr><th>类别</th><th>像素%</th></tr>';
    j.names.forEach((n,i)=>{const c=j.lut[i];
      h+=`<tr><td><span class=sw style="background:rgb(${c[2]},${c[1]},${c[0]})"></span>${n}</td><td>${j.pct[i]}%</td></tr>`;});
    h+='</table>';
    E('stats').innerHTML=h;
    const cc=j.cells;
    E('meta').innerHTML=
      `<span class=tag>格子</span>障碍 <b>${cc.obstacle_cells}</b> · 矮物被否 <b>${cc.low_rejected_cells}</b>`
      +` · 只有地面 ${cc.ground_only_cells} · 空列 ${cc.empty_cols}<br>`
      +`<span class=tag>最小可见高度</span><b>${j.min_visible_mm===null?'>1000':j.min_visible_mm} mm</b> 离地`
      +` — 比这更矮的东西 z 跨度不够，恒不是障碍<br>`
      +`<span class=tag>波段</span>离地 ${j.band_floor_lo} ~ ${j.band_floor_hi} m`
      +` · ${cc.band_layers} 层 · 世界 z ${cc.band_z_lo.toFixed(3)}~${cc.band_z_hi.toFixed(3)}<br>`
      +`<span class=tag>origin_z</span>${j.origin_z} · floor_z ${j.floor_z} · cam_h ${j.cam_h}<br>`
      +`<span class=tag>场景</span>${j.note||'(无备注)'} · ${j.frames}/${j.total} 帧`
      +` · 累积 ${j.accum_ms} ms${j.has_infra?'':' · <span class=warn>无红外底图</span>'}`;
  }finally{
    busy=false; E('go').textContent='重算';
    if(pending){pending=false; go(false);}
  }
}
let t=null;
CHEAP.forEach(k=>E(k).addEventListener('input',()=>{show();clearTimeout(t);t=setTimeout(()=>go(false),60);}));
EXP.forEach(k=>E(k).addEventListener('input',show));
E('go').addEventListener('click',()=>go(true));
fetch('/api/scenes').then(r=>r.json()).then(j=>{
  E('scene').innerHTML=j.scenes.map(s=>`<option>${s}</option>`).join('');
  E('goff').value=j.grid_offset_z; E('span').value=j.min_wall_span_m;
  E('zbot').value=j.robot_z_bottom; E('ztop').value=j.robot_z_top;
  E('occ').value=j.occ_threshold; E('dil').value=j.dilation_cells;
  E('hit').value=j.step; E('carve').value=j.step;
  show(); go(true);
});
</script>"""


class H(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def _send(self, body, ctype='application/json'):
        b = body if isinstance(body, bytes) else body.encode()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        Q = {k: v[0] for k, v in urllib.parse.parse_qs(u.query).items()}
        if u.path == '/':
            return self._send(HTML, 'text/html; charset=utf-8')
        if u.path == '/api/scenes':
            o = _ROBOT.obstacle
            return self._send(json.dumps(dict(
                scenes=_scenes(),
                grid_offset_z=float(os.environ.get('TINYNAV_GRID_OFFSET_Z', '0.1625')),
                step=int(os.environ.get('TINYNAV_RAYCAST_STEP', '3')),
                min_wall_span_m=o.min_wall_span_m, robot_z_bottom=o.robot_z_bottom,
                robot_z_top=o.robot_z_top, occ_threshold=o.occ_threshold,
                dilation_cells=o.dilation_cells)))
        if u.path in ('/api/prepare', '/api/render'):
            try:
                prep = accumulate(Q['scene'], int(Q['frames']), float(Q['goff']),
                                  int(Q['hit']), int(Q['carve']))
                out = render(prep, dict(
                    min_wall_span_m=float(Q['span']),
                    robot_z_bottom=float(Q['zbot']), robot_z_top=float(Q['ztop']),
                    occ_threshold=float(Q['occ']), dilation_cells=int(Q['dil'])),
                    float(Q['low']), Q['base'], Q['focus'] == '1')
                if u.path == '/api/prepare':
                    threading.Thread(target=prefetch, daemon=True, args=(
                        Q['scene'], int(Q['frames']), float(Q['goff']),
                        int(Q['hit']), int(Q['carve']))).start()
                return self._send(json.dumps(out))
            except Exception as e:                      # 别让一次参数越界把服务打死
                import traceback
                traceback.print_exc()
                return self._send(json.dumps(dict(error=f'{type(e).__name__}: {e}')))
        self.send_error(404)


def _warmup():
    """numba 首次调用要编译（约 3 s）。不预热的话这笔账挂在你第一次点"重算"上。"""
    d = np.zeros((64, 64), np.float32) + 1.0
    T = np.eye(4)
    o = np.array(GRID_SHAPE) * RESOLUTION / -2.0
    for hs, cs in ((3, 3), (1, 6)):
        if hs == cs:
            run_raycasting_loopy(d, T, GRID_SHAPE, 30., 30., 32., 32., o, hs, RESOLUTION)
        else:
            run_raycasting_split(d, T, GRID_SHAPE, 30., 30., 32., 32., o, hs, cs, RESOLUTION)


if __name__ == '__main__':
    sc = _scenes()
    t = time.perf_counter(); _warmup()
    print(f"kernel 预热 {1000 * (time.perf_counter() - t):.0f} ms")
    print(f"平台 {PLATFORM} · 场景 {len(sc)} 个: {', '.join(sc) or '(空)'}")
    if not sc:
        print(f"⚠️ {SCENE_DIR} 里没有 npz。先在板上录一段：\n"
              f"   python3 tool/x5_board/capture_depth_scene.py --out /userdata/x5/scene_x.npz "
              f"--frames 250 --note '左边有椅子'")
    print(f"打开 http://127.0.0.1:{PORT}")
    ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever()
