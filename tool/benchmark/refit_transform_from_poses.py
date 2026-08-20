#!/usr/bin/env python3
"""Re-fit the query->reference SE(2) transform from PnP-solved poses, and report the
offset against whatever transform the run was scored with.

Why this exists: the transform every cross-session number is scored against was fitted
from one retrieval backend's own Top-1 matches (see map_retrieval_self_consistency.py's
own note). Scoring a backend against a transform derived from retrieval is circular, and
a small rotation error is invisible at the 0.5 m threshold while dominating the 0.1 m one.

PnP poses come from 3D-2D geometry, so re-fitting from them is an independent check: if
the two transforms agree, the ground truth is sound; if they do not, every tight-threshold
number is suspect.
"""
import argparse
import csv
import json
import math

import numpy as np


def fit_se2(src_xy, dst_xy):
    """Least-squares SE(2) (Umeyama without scale)."""
    src_c, dst_c = src_xy.mean(0), dst_xy.mean(0)
    H = (src_xy - src_c).T @ (dst_xy - dst_c)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1
        R = Vt.T @ U.T
    return R, dst_c - R @ src_c


def ransac_se2(src_xy, dst_xy, threshold_m, iters, rng):
    best_inl = np.zeros(len(src_xy), dtype=bool)
    for _ in range(iters):
        idx = rng.choice(len(src_xy), size=2, replace=False)
        try:
            R, t = fit_se2(src_xy[idx], dst_xy[idx])
        except np.linalg.LinAlgError:
            continue
        res = np.linalg.norm((src_xy @ R.T + t) - dst_xy, axis=1)
        inl = res <= threshold_m
        if inl.sum() > best_inl.sum():
            best_inl = inl
    if best_inl.sum() >= 2:
        R, t = fit_se2(src_xy[best_inl], dst_xy[best_inl])
    else:
        R, t = fit_se2(src_xy, dst_xy)
    return R, t, np.linalg.norm((src_xy @ R.T + t) - dst_xy, axis=1), best_inl


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", required=True, help="reloc_offline_eval CSV with solved_x/query_x columns")
    p.add_argument("--transform-json", required=True, help="the transform the run was scored with")
    p.add_argument("--max-err-m", type=float, default=0.5,
                   help="Only fit on rows the run already called a success; a failed PnP "
                        "pose carries no information about the transform")
    p.add_argument("--ransac-threshold-m", type=float, default=0.3)
    p.add_argument("--iters", type=int, default=2000)
    p.add_argument("--out-json", default=None)
    args = p.parse_args()

    src, dst = [], []
    with open(args.csv) as f:
        for row in csv.DictReader(f):
            if row.get("reloc_true") != "1" or not row.get("solved_x"):
                continue
            if float(row["xy_err_m"]) > args.max_err_m:
                continue
            src.append([float(row["query_x"]), float(row["query_y"])])
            dst.append([float(row["solved_x"]), float(row["solved_y"])])
    src, dst = np.asarray(src), np.asarray(dst)
    print(f"[info] fitting on {len(src)} successful relocalizations", flush=True)
    if len(src) < 10:
        raise SystemExit("not enough successful poses to fit a transform")

    R, t, res, inl = ransac_se2(src, dst, args.ransac_threshold_m, args.iters,
                                np.random.default_rng(0))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    blk = json.load(open(args.transform_json))
    blk = blk.get("T_map_a_map_b_se2", blk)
    R0 = np.asarray(blk["R_xy"], dtype=np.float64)
    t0 = np.asarray(blk["t_xy"], dtype=np.float64)
    yaw0 = math.degrees(math.atan2(R0[1, 0], R0[0, 0]))

    print(f"\n  scored-with transform : yaw={yaw0:+7.3f}°  t=({t0[0]:+.4f}, {t0[1]:+.4f})")
    print(f"  refit from PnP poses  : yaw={yaw:+7.3f}°  t=({t[0]:+.4f}, {t[1]:+.4f})")
    print(f"  DELTA                 : yaw={yaw-yaw0:+7.3f}°  t=({t[0]-t0[0]:+.4f}, {t[1]-t0[1]:+.4f})")
    print(f"  refit inliers         : {int(inl.sum())}/{len(src)}   "
          f"residual p50={np.median(res):.4f} m  p90={np.percentile(res,90):.4f} m")

    # What the disagreement is worth in metres, at the radii the map actually spans.
    span = np.linalg.norm(dst - dst.mean(0), axis=1)
    d_yaw = math.radians(abs(yaw - yaw0))
    print(f"\n  map radius from centroid: p50={np.median(span):.1f} m  p90={np.percentile(span,90):.1f} m")
    print(f"  rotation disagreement is worth {d_yaw*np.median(span):.3f} m at p50 radius, "
          f"{d_yaw*np.percentile(span,90):.3f} m at p90")
    print(f"  translation disagreement   : {np.linalg.norm(t-t0):.3f} m")

    # Re-score the same successes against the refit transform.
    err_old = np.linalg.norm((src @ R0.T + t0) - dst, axis=1)
    err_new = np.linalg.norm((src @ R.T + t) - dst, axis=1)
    print(f"\n  {'threshold':>10}  {'scored-with':>12}  {'refit':>10}")
    for th in (0.1, 0.3, 0.5):
        print(f"  {th:>9.1f}m  {np.mean(err_old<=th)*100:>11.1f}%  {np.mean(err_new<=th)*100:>9.1f}%")
    print("  (denominator is the already-successful subset, so these are not run success rates)")

    if args.out_json:
        T = np.eye(4)
        T[:2, :2] = R
        T[:2, 3] = t
        json.dump({"T_map_a_map_b_se2": {"T": T.tolist(), "R_xy": R.tolist(),
                                         "t_xy": t.tolist(), "yaw_deg": yaw},
                   "source": "refit from PnP-solved poses", "csv": args.csv,
                   "n_poses": int(len(src)), "n_inliers": int(inl.sum())},
                  open(args.out_json, "w"), indent=1)


if __name__ == "__main__":
    main()
