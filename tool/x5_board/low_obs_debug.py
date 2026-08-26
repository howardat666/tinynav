#!/usr/bin/env python3
"""Render one captured frame the way the planner sees it, to eyeball the low-obstacle path.

Consumes the npz files written on the board by grab_depth.py (depth + pose + K) and,
optionally, grab_scene.py (the accumulated occupancy cloud + the live obstacle mask).
Writes a PNG: camera-view height-above-floor on the left, top-down cells on the right.

Deliberately re-implements the kernel in numpy rather than importing it -- the point is
to check the maths independently, and numba is not installed on a dev box anyway. The
thresholds must stay in sync with planning_node's TINYNAV_LOW_OBS_* defaults.
"""
import argparse
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# PIL 自带的位图字体是 latin-1，中文标签会直接抛 UnicodeEncodeError。
_FONT_CANDIDATES = [
    '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    '/usr/share/fonts/truetype/arphic/uming.ttc',
]


def _font(size=13):
    for path in _FONT_CANDIDATES:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


FONT = _font(13)
FONT_S = _font(11)

H_LO, H_HI, RANGE_M, MIN_PTS = 0.05, 0.25, 1.5, 5
CAM_H, RES, STEP = 0.18, 0.1, 5

# 离地高度的固定配色。固定是关键 —— 前端现在按每帧的 5%~95% 分位自适应，同一个高度
# 每帧颜色都不同，这是「看不清」的头号原因。
BANDS = [
    (-9.0, 0.02, (90, 96, 105), "地面 <2cm"),
    (0.02, H_LO, (40, 80, 150), f"2~{H_LO*100:.0f}cm 噪声带"),
    (H_LO, 0.15, (235, 60, 60), f"{H_LO*100:.0f}~15cm 矮障碍"),
    (0.15, H_HI, (245, 140, 40), f"15~{H_HI*100:.0f}cm"),
    (H_HI, 0.60, (240, 210, 60), f"{H_HI*100:.0f}~60cm"),
    (0.60, 9.0, (200, 205, 215), ">60cm"),
]


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def load_depth(path):
    d = np.load(path)
    dep = d['depth'].astype(np.float32)
    if str(d['enc'][0]) == 'mono16':
        dep = dep / 1000.0
    return dep, d['pose'], d['K'].reshape(3, 3)


def project(dep, pose, K, step=1):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    v, u = np.mgrid[0:dep.shape[0]:step, 0:dep.shape[1]:step]
    dd = dep[v, u]
    R, t = quat_to_matrix(pose[3:]), pose[:3]
    P = np.stack([(u - cx) * dd / fx, (v - cy) * dd / fy, dd])
    W = np.einsum('ij,jhw->ihw', R, P) + t[:, None, None]
    return W, dd > 0, (v, u)


def body_frame(W, pose):
    """(forward, left, height-above-camera) in the car's frame."""
    R, t = quat_to_matrix(pose[3:]), pose[:3]
    fwd = R @ np.array([0.0, 0.0, 1.0])
    yaw = np.arctan2(fwd[1], fwd[0])
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = W[0] - t[0], W[1] - t[1]
    return dx * c + dy * s, -dx * s + dy * c, W[2]


def band_color(h):
    out = np.zeros(h.shape + (3,), dtype=np.uint8)
    for lo, hi, col, _ in BANDS:
        out[(h >= lo) & (h < hi)] = col
    return out


def draw_legend(dr, x, y, entries, title):
    dr.text((x, y), title, fill=(230, 235, 245), font=FONT)
    y += 16
    for col, label in entries:
        dr.rectangle([x, y + 3, x + 11, y + 12], fill=col, outline=(80, 86, 96))
        dr.text((x + 17, y), label, fill=(198, 205, 216), font=FONT_S)
        y += 16
    return y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('depth_npz')
    ap.add_argument('--scene-npz', default=None,
                    help='grab_scene.py output, for the accumulated occupancy overlay')
    ap.add_argument('-o', '--out', default='low_obs_debug.png')
    ap.add_argument('--camera-height', type=float, default=CAM_H)
    ap.add_argument('--h-lo', type=float, default=H_LO)
    ap.add_argument('--range', type=float, default=RANGE_M)
    ap.add_argument('--min-pts', type=int, default=MIN_PTS)
    a = ap.parse_args()

    dep, pose, K = load_depth(a.depth_npz)
    W, valid, _ = project(dep, pose, K)
    F, L, Z = body_frame(W, pose)

    # 地面高度：先按标称的相机高度，再用近处点的低分位纠一次，两者差得太多就报出来。
    nominal = pose[2] - a.camera_height
    near = valid & (F > 0.3) & (F < 1.0) & (np.abs(L) < 0.8)
    measured = float(np.percentile(Z[near], 5)) if near.sum() > 500 else nominal
    print(f"地面 z: 标称 {nominal:.4f} / 实测 {measured:.4f}  (差 {abs(measured-nominal)*100:.1f} cm)")
    print(f"相机离地实测 {pose[2]-measured:.4f} m")
    hgt = Z - nominal

    # --- 左：相机视角的离地高度 ---
    cam = band_color(hgt)
    cam[~valid] = (16, 18, 22)
    cam_img = Image.fromarray(cam)   # Looper 深度本身就是竖的 544x640，别转

    # --- 右：俯视图 ---
    TD, MPP = 560, 0.006          # 像素 / 米每像素
    top = np.full((TD, TD, 3), 22, dtype=np.uint8)
    ox_pix, oy_pix = TD // 2, int(TD * 0.86)     # 车在下方中央

    def to_pix(f, l):
        return (ox_pix - l / MPP).astype(int), (oy_pix - f / MPP).astype(int)

    sub = np.zeros(dep.shape, dtype=bool)
    sub[::STEP, ::STEP] = True          # 和板上一样的 step 取样

    # 累加占据（来自 scene npz）：先铺底，代表「栅格里有东西」
    span_cells = set()
    if a.scene_npz:
        sc = np.load(a.scene_npz)
        mask, info, spose = sc['mask'], sc['info'], sc['pose']
        sox, soy = float(info[1]), float(info[2])
        ii, jj = np.nonzero(mask > 0)
        R = quat_to_matrix(spose[3:]); fw = R @ np.array([0., 0., 1.])
        yaw = np.arctan2(fw[1], fw[0]); c, s = np.cos(yaw), np.sin(yaw)
        ri = int(round((spose[0] - sox) / RES)); rj = int(round((spose[1] - soy) / RES))
        dx, dy = (ii - ri) * RES, (jj - rj) * RES
        sf, sl = dx * c + dy * s, -dx * s + dy * c
        span_cells = set(zip(np.floor(sf / RES).astype(int), np.floor(sl / RES).astype(int)))

    # 矮障碍：按内核的语义在 numpy 里重算
    m = valid & (dep <= a.range) & (hgt > a.h_lo) & (hgt < H_HI) & sub
    ci = np.floor(F[m] / RES).astype(int)
    cj = np.floor(L[m] / RES).astype(int)
    cnt = {}
    for k in zip(ci, cj):
        cnt[k] = cnt.get(k, 0) + 1
    low_cells = {k for k, n in cnt.items() if n >= a.min_pts}

    def cell_rect(k):
        f0, l0 = k[0] * RES, k[1] * RES
        x1, y1 = to_pix(np.array([f0]), np.array([l0 + RES]))
        x2, y2 = to_pix(np.array([f0 + RES]), np.array([l0]))
        return [int(x1[0]), int(y2[0]), int(x2[0]), int(y1[0])]

    img = Image.fromarray(top)
    dr = ImageDraw.Draw(img)
    for r in range(1, 5):                       # 距离环
        rad = r * 0.5 / MPP
        dr.ellipse([ox_pix - rad, oy_pix - rad, ox_pix + rad, oy_pix + rad],
                   outline=(52, 58, 68))
        dr.text((ox_pix + 4, oy_pix - rad - 13), f"{r*0.5:.1f}m", fill=(96, 104, 116), font=FONT_S)
    # 本帧射线真正落到的格子（不分高低），用来看「哪里是确认空的、哪里根本没看到」。
    seen_f = np.floor(F[valid & sub & (dep <= 4.0)] / RES).astype(int)
    seen_l = np.floor(L[valid & sub & (dep <= 4.0)] / RES).astype(int)
    for k in set(zip(seen_f, seen_l)):
        dr.rectangle(cell_rect(k), fill=(34, 42, 54))
    for k in sorted(span_cells):
        dr.rectangle(cell_rect(k), fill=(225, 230, 240))
    for k in sorted(low_cells):
        dr.rectangle(cell_rect(k), fill=(235, 60, 60))
    # 车体：0.28 x 0.35，控制中心在驱动轴上（前 0.10 / 后 0.18 / 半宽 0.175）
    x1, y1 = to_pix(np.array([-0.18]), np.array([0.175]))
    x2, y2 = to_pix(np.array([0.10]), np.array([-0.175]))
    dr.rectangle([int(x1[0]), int(y2[0]), int(x2[0]), int(y1[0])],
                 outline=(64, 200, 240), width=2)
    dr.line([ox_pix, oy_pix, ox_pix, oy_pix - int(0.25 / MPP)], fill=(64, 200, 240), width=2)
    # 检测量程边界
    rad = a.range / MPP
    dr.ellipse([ox_pix - rad, oy_pix - rad, ox_pix + rad, oy_pix + rad],
               outline=(235, 60, 60))
    dr.text((ox_pix - rad + 4, oy_pix - rad + 2), f"矮障碍量程 {a.range}m", fill=(180, 70, 70), font=FONT_S)

    # --- 拼版 ---
    ch = 420
    cam_img = cam_img.resize((int(cam_img.width * ch / cam_img.height), ch))
    pad, lgw = 14, 210
    canvas = Image.new('RGB', (cam_img.width + TD + lgw + pad * 4,
                               max(cam_img.height, TD) + 96), (14, 16, 20))
    d2 = ImageDraw.Draw(canvas)
    d2.text((pad, 10), f"离地高度（相机视角） | 地面 z={nominal:.3f} 实测 {measured:.3f}",
            fill=(230, 235, 245), font=FONT)
    canvas.paste(cam_img, (pad, 34))
    d2.text((pad * 2 + cam_img.width, 10), "俯视：白=z跨度障碍(累加)  红=矮障碍(本帧)",
            fill=(230, 235, 245), font=FONT)
    canvas.paste(img, (pad * 2 + cam_img.width, 34))
    lx = pad * 3 + cam_img.width + TD
    y = draw_legend(d2, lx, 34, [(c, lab) for _, _, c, lab in BANDS], "离地高度配色（固定）")
    y = draw_legend(d2, lx, y + 12,
                    [((34, 42, 54), "本帧看到、判定为空"),
                     ((225, 230, 240), "z 跨度障碍格"), ((235, 60, 60), "矮障碍格"),
                     ((64, 200, 240), "车体 0.28x0.35")], "俯视图")
    d2.text((lx, y + 12), f"门限 {a.h_lo:.2f}~{H_HI:.2f} m", fill=(198, 205, 216), font=FONT_S)
    d2.text((lx, y + 28), f"量程 {a.range:.1f} m / 最少 {a.min_pts} 点", fill=(198, 205, 216), font=FONT_S)
    d2.text((lx, y + 44), f"矮障碍格 {len(low_cells)}", fill=(235, 120, 120), font=FONT_S)
    d2.text((lx, y + 60), f"跨度障碍格 {len(span_cells)}", fill=(225, 230, 240), font=FONT_S)
    canvas.save(a.out)
    print(f"矮障碍格 {len(low_cells)}  跨度障碍格 {len(span_cells)}  -> {a.out}")


if __name__ == '__main__':
    main()
