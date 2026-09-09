#!/usr/bin/env python3
"""比较同一个 bag 建出来的两张地图的关键帧轨迹。

同一个 bag ⇒ 关键帧时间戳应当逐个对上，所以可以做逐帧比较，而不只是看统计量。
两者在各自的里程计坐标系里，所以先做刚体对齐（看形状差多少），
再做带尺度的相似变换对齐（尺度比就是"轮速尺度对不对"的答案）。
"""
import argparse
import numpy as np


def load(path):
    d = np.load(path, allow_pickle=True).item()
    ts = np.array(sorted(d.keys()), dtype=np.int64)
    T = np.stack([d[t] for t in ts])
    return ts, T


def yaw_of(R):
    """绕世界 Z 轴的偏航。poses.npy 存的是 T_world_camera：世界 z 向上（R 第 2 列恒为
    (0,0,-1)），相机前向是 R 第 3 列，所以取它在世界 XY 面上的方向。"""
    return np.arctan2(R[:, 1, 2], R[:, 0, 2])


def path_len(P):
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())


def align(A, B, with_scale):
    """把 B 对齐到 A（Umeyama）。返回 (rmse, scale)。"""
    ca, cb = A.mean(0), B.mean(0)
    A0, B0 = A - ca, B - cb
    H = B0.T @ A0
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    s = (S * np.array([1, 1, d])).sum() / (B0 ** 2).sum() if with_scale else 1.0
    resid = A0 - s * (R @ B0.T).T
    return float(np.sqrt((resid ** 2).sum(1).mean())), float(s)


def report(name, ts, T):
    P = T[:, :3, 3]
    y = np.degrees(np.unwrap(yaw_of(T)))
    step = np.linalg.norm(np.diff(P, axis=0), axis=1)
    print(f"\n== {name} ==")
    print(f"  关键帧            {len(ts)}")
    print(f"  时间跨度          {(ts[-1] - ts[0]) / 1e9:.1f} s")
    print(f"  轨迹总里程        {path_len(P):.2f} m")
    print(f"  首尾直线距离      {np.linalg.norm(P[-1] - P[0]):.3f} m   ← 走闭环时这就是闭环误差")
    print(f"  包围盒 (x,y,z)    {np.ptp(P, axis=0).round(2).tolist()} m")
    print(f"  单步位移 p50/p99  {np.percentile(step, 50):.4f} / {np.percentile(step, 99):.4f} m")
    print(f"  最大单步跳变      {step.max():.4f} m  (第 {int(step.argmax())} 帧)")
    print(f"  累计转角          {abs(y[-1] - y[0]):.1f} deg   净变化")
    print(f"  偏航总变化量      {np.abs(np.diff(y)).sum():.1f} deg   绝对值累加")
    return P, y, step


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--name-a", default="A")
    ap.add_argument("--name-b", default="B")
    args = ap.parse_args()

    ta, Ta = load(args.a)
    tb, Tb = load(args.b)
    Pa, ya, _ = report(args.name_a, ta, Ta)
    Pb, yb, _ = report(args.name_b, tb, Tb)

    # 两边关键帧的戳来自各自的位姿话题，精确交集恒为 0；间隔都是 200 ms(深度帧率)，
    # 所以用最近邻配对，容差取半个周期。
    tol_ns = int(0.1 * 1e9)
    ib = np.searchsorted(tb, ta)
    ib = np.clip(ib, 1, len(tb) - 1)
    pick = np.where(np.abs(tb[ib - 1] - ta) <= np.abs(tb[ib] - ta), ib - 1, ib)
    ok = np.abs(tb[pick] - ta) <= tol_ns
    ia, ib = np.nonzero(ok)[0], pick[ok]
    common = ia
    print(f"  配对帧数          {len(ia)} / {len(ta)} vs {len(tb)}  (最近邻, 容差 100 ms)")
    A, B = Pa[ia], Pb[ib]
    rmse_rigid, _ = align(A, B, with_scale=False)
    rmse_sim, s = align(A, B, with_scale=True)
    print(f"  刚体对齐 RMSE     {rmse_rigid:.3f} m   ← 形状差异（含尺度误差）")
    print(f"  相似对齐 RMSE     {rmse_sim:.3f} m   ← 扣掉尺度后的形状差异")
    print(f"  尺度比 b/a        {s:.4f}   ({(s - 1) * 100:+.2f}%)")
    print(f"  两条轨迹里程       {path_len(A):.2f} m  vs  {path_len(B):.2f} m"
          f"   ({(path_len(B) / path_len(A) - 1) * 100:+.2f}%)")
    dy = np.degrees(np.unwrap(yaw_of(Ta[ia]))) - np.degrees(np.unwrap(yaw_of(Tb[ib])))
    dy -= dy[0]
    print(f"  偏航差 末端/最大   {dy[-1]:+.1f} / {np.abs(dy).max():.1f} deg")


if __name__ == "__main__":
    main()
