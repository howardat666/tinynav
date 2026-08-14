#!/usr/bin/env python3
"""Offline end-to-end relocalization benchmark for the CPU-only (ORB + DBoW3) path.

Why this exists
---------------
`tinynav/core/map_node.py` only relocalizes from a ROS TimeSynchronizer callback
(`/slam/keyframe_image` + `/slam/keyframe_odom` + `/slam/keyframe_depth`), which
means you normally need a bag, a VIO node and an IMU propagator just to measure
one relocalization. On the X5 that is not practical.

This script instantiates the *real* `MapNode` (so the measured code path is
byte-for-byte the shipped one) but never spins it. Query images are pulled out of
a map's own `infra1_images_db` video and fed straight into
`MapNode.keyframe_relocalization()`.

Ground truth
------------
  * self-consistency (`--query-map` == `--map`): GT is `poses.npy` of the map.
  * cross-session (`--query-map` != `--map`): the two maps are independent VIO
    sessions, so a shared SE(2) transform (`--transform-json`, as produced by
    tool/benchmark/map_retrieval_self_consistency.py --save-transform) is
    required to bring query GT poses into the reference map frame. Position error
    is therefore reported in the XY plane only (the fitted transform has no z
    component).

Success = relocalize returned True AND xy error <= --pos-threshold-m AND
rotation geodesic error <= --rot-threshold-deg.
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import resource
import sys
import time
from pathlib import Path

import numpy as np


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--map", required=True, help="Reference map directory (the map we relocalize INTO)")
    p.add_argument("--query-map", default=None, help="Map whose infra1 frames are used as queries (default: --map)")
    p.add_argument("--vocab", required=True, help="DBoW3 vocabulary (.dbow3 binary or ORBvoc.txt)")
    p.add_argument("--transform-json", default=None, help="Shared SE(2) transform map_a<-map_b, required for cross-session")
    p.add_argument("--db-path", default="/jobtmp/reloc_db", help="Scratch dir for MapNode's nav_temp DB")
    p.add_argument("--out-prefix", required=True, help="Output prefix; writes <prefix>.json and <prefix>.csv")
    p.add_argument("--every-n", type=int, default=1)
    p.add_argument("--max-queries", type=int, default=0, help="0 = all")
    p.add_argument("--top-k", type=int, default=0, help="Override MapNode.relocalization_loop_top_k (0 = keep default 3)")
    p.add_argument("--nfeatures", type=int, default=0, help="Override ORB nfeatures (0 = keep default 1024)")
    p.add_argument("--matcher", choices=("flann", "bf"), default="flann",
                   help="flann = LSH + Lowe ratio (shipping default); bf = BFMatcher Hamming + crossCheck")
    p.add_argument("--ransac", action="store_true",
                   help="Filter matches by findFundamentalMat before PnP (off in the shipping default)")
    p.add_argument("--pos-threshold-m", type=float, default=0.5)
    p.add_argument("--rot-threshold-deg", type=float, default=10.0)
    # One pass, several thresholds: 0.5 m is the retrieval literature's convention but the
    # planning grid is 0.1 m, so a run that only reports 0.5 m cannot distinguish "good
    # enough to navigate on" from "barely inside the loosest bar anyone uses".
    p.add_argument("--pos-thresholds-m", type=str, default="0.1,0.3,0.5",
                   help="Comma-separated extra position thresholds reported alongside the main one")
    p.add_argument("--retrieval-threshold-m", type=float, default=0.5,
                   help="A candidate counts as a retrieval hit when its map pose is this close to GT")
    p.add_argument("--warmup-queries", type=int, default=2, help="Queries run before timing starts (discarded)")
    return p.parse_args()


def _parse_float_list(value: str) -> list[float]:
    return sorted({float(v) for v in value.split(",") if v.strip()})


def _load_poses(map_dir: Path) -> dict[int, np.ndarray]:
    raw = np.load(map_dir / "poses.npy", allow_pickle=True).item()
    return {int(k): np.asarray(v, dtype=np.float64) for k, v in raw.items()}


def _load_se2(path: Path | None) -> np.ndarray:
    if path is None:
        return np.eye(4)
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    block = payload.get("T_map_a_map_b_se2", payload)
    T = np.eye(4)
    T[:2, :2] = np.asarray(block["R_xy"], dtype=np.float64)
    T[:2, 3] = np.asarray(block["t_xy"], dtype=np.float64)
    return T


def _rot_geodesic_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    R = R_a.T @ R_b
    cos = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def _percentiles(values: list[float]) -> dict[str, float]:
    if not values:
        return {}
    a = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(a.mean()),
        "p50": float(np.percentile(a, 50)),
        "p90": float(np.percentile(a, 90)),
        "max": float(a.max()),
    }


def main() -> int:
    args = _parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(filename)s:%(lineno)s %(message)s")

    map_dir = Path(args.map)
    query_dir = Path(args.query_map) if args.query_map else map_dir
    cross_session = query_dir.resolve() != map_dir.resolve()
    if cross_session and args.transform_json is None:
        print("ERROR: cross-session evaluation needs --transform-json", file=sys.stderr)
        return 2

    os.makedirs(args.db_path, exist_ok=True)

    # --- import + node construction cost -------------------------------------
    t_import0 = time.perf_counter()
    import rclpy
    from builtin_interfaces.msg import Time as RosTime

    from tinynav.core.map_node import MapNode, DummyEmbeddingEngine
    from tinynav.core.models_trt import ORBFeatureTRTCompatible, ORBMatcher
    from tinynav.core.build_map_node import TinyNavDB
    import_s = time.perf_counter() - t_import0

    rclpy.init(args=None)
    extractor = ORBFeatureTRTCompatible(**({"nfeatures": args.nfeatures} if args.nfeatures else {}))
    matcher = ORBMatcher(mode=args.matcher, use_ransac=args.ransac)

    t_node0 = time.perf_counter()
    node = MapNode(
        tinynav_db_path=args.db_path,
        tinynav_map_path=str(map_dir),
        extractor=extractor,
        matcher=matcher,
        embedding_extractor=DummyEmbeddingEngine(),
        loop_closure_mode="bow",
        loop_closure_use_bow=True,
        dbow3_vocabulary_path=args.vocab,
        verbose_timer=False,
    )
    node_build_s = time.perf_counter() - t_node0
    rss_after_load_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0

    # keyframe_relocalization() only checks `self.K is not None`; PnP itself uses
    # self.map_K. In the live node K comes from /camera/camera/infra2/camera_info.
    node.K = node.map_K
    if args.top_k:
        node.relocalization_loop_top_k = int(args.top_k)

    # --- queries -------------------------------------------------------------
    ref_poses = node.map_poses
    query_poses = _load_poses(query_dir)
    query_db = node.db if not cross_session else TinyNavDB(str(query_dir), is_scratch=False)

    query_ts = sorted(query_poses)
    query_ts = query_ts[:: max(1, args.every_n)]
    if args.max_queries > 0:
        query_ts = query_ts[: args.max_queries]

    T_se2 = _load_se2(Path(args.transform_json) if args.transform_json else None)

    rows: list[dict] = []
    stage_names = [
        "feature_extract", "embedding", "candidate_search",
        "db_load", "match", "depth3d", "pnp", "publish", "unaccounted",
    ]
    stage_acc: dict[str, list[float]] = {n: [] for n in stage_names}
    totals: list[float] = []
    n_reloc_true = 0
    n_success = 0
    n_decode_fail = 0
    xy_errors: list[float] = []
    rot_errors: list[float] = []
    # The whole point of this run: where does the pipeline lose the query? A single
    # success rate cannot tell "retrieval never found the place" from "retrieval found it
    # and the matcher threw it away", and those two have opposite fixes.
    extra_thresholds = _parse_float_list(args.pos_thresholds_m)
    n_success_at: dict[float, int] = {t: 0 for t in extra_thresholds}
    n_retrieval_hit = 0
    fail_codes: dict[str, int] = {}
    counters: dict[str, list[float]] = {
        k: [] for k in ("query_kpts", "top_sim", "matches_best", "valid_depth_best",
                        "landmarks", "pnp_inliers", "retrieval_best_m")
    }

    total_queries = len(query_ts)
    print(f"[info] reference map={map_dir} keyframes={len(ref_poses)}")
    print(f"[info] query map={query_dir} queries={total_queries} cross_session={cross_session}")
    print(f"[info] import_s={import_s:.2f} node_build_s={node_build_s:.2f} rss_after_load_mb={rss_after_load_mb:.0f}")
    print(f"[info] vocab={args.vocab} top_k={node.relocalization_loop_top_k} orb_nfeatures={extractor.nfeatures}")

    for i, ts in enumerate(query_ts):
        image = query_db.infra1_video_db.read(int(ts))
        if image is None:
            n_decode_fail += 1
            continue
        if image.ndim == 3:
            image = image[..., 0]

        stamp = RosTime(sec=int(ts // 1_000_000_000), nanosec=int(ts % 1_000_000_000))
        t0 = time.perf_counter()
        ok, pose_in_world = node.keyframe_relocalization(stamp, image)
        wall_ms = (time.perf_counter() - t0) * 1000.0
        timings = dict(node.last_relocalization_timing)

        stats = dict(node.last_relocalization_stats)

        gt_in_ref = T_se2 @ query_poses[int(ts)]
        xy_err = float("nan")
        rot_err = float("nan")
        if ok:
            xy_err = float(np.linalg.norm(pose_in_world[:2, 3] - gt_in_ref[:2, 3]))
            rot_err = _rot_geodesic_deg(gt_in_ref[:3, :3], pose_in_world[:3, :3])

        # Retrieval is scored against the reference map's own poses rather than against
        # what PnP later returned, so it stays measurable even when everything downstream
        # of it fails -- which is exactly the case worth diagnosing.
        retrieval_best_m = float("inf")
        for cand_ts in stats.get("cand_ts", []):
            cand_pose = ref_poses.get(int(cand_ts))
            if cand_pose is not None:
                retrieval_best_m = min(
                    retrieval_best_m,
                    float(np.linalg.norm(cand_pose[:2, 3] - gt_in_ref[:2, 3])),
                )
        retrieval_hit = retrieval_best_m <= args.retrieval_threshold_m

        warm = i >= args.warmup_queries
        if warm:
            if ok:
                n_reloc_true += 1
                xy_errors.append(xy_err)
                rot_errors.append(rot_err)
                if xy_err <= args.pos_threshold_m and rot_err <= args.rot_threshold_deg:
                    n_success += 1
                for t in extra_thresholds:
                    if xy_err <= t and rot_err <= args.rot_threshold_deg:
                        n_success_at[t] += 1
            if retrieval_hit:
                n_retrieval_hit += 1
            code = stats.get("fail_code", "") if not ok else "success"
            fail_codes[code] = fail_codes.get(code, 0) + 1
            counters["query_kpts"].append(float(stats.get("query_kpts", 0)))
            counters["top_sim"].append(float(stats.get("top_sim", 0.0)))
            counters["matches_best"].append(float(max(stats.get("cand_matches") or [0])))
            counters["valid_depth_best"].append(float(max(stats.get("cand_valid_depth") or [0])))
            counters["landmarks"].append(float(stats.get("landmarks", 0)))
            counters["pnp_inliers"].append(float(stats.get("pnp_inliers", 0)))
            if np.isfinite(retrieval_best_m):
                counters["retrieval_best_m"].append(retrieval_best_m)
            totals.append(wall_ms)
            for name in stage_names:
                stage_acc[name].append(float(timings.get(name, 0.0)))

        rows.append({
            "query_ts": int(ts),
            "warm": int(warm),
            "reloc_true": int(ok),
            "wall_ms": round(wall_ms, 2),
            "xy_err_m": None if not ok else round(xy_err, 4),
            "rot_err_deg": None if not ok else round(rot_err, 3),
            "fail_code": "" if ok else stats.get("fail_code", ""),
            "retrieval_hit": int(retrieval_hit),
            "retrieval_best_m": None if not np.isfinite(retrieval_best_m) else round(retrieval_best_m, 4),
            "query_kpts": stats.get("query_kpts", 0),
            "top_sim": round(float(stats.get("top_sim", 0.0)), 4),
            "matches_best": max(stats.get("cand_matches") or [0]),
            "valid_depth_best": max(stats.get("cand_valid_depth") or [0]),
            "landmarks": stats.get("landmarks", 0),
            "pnp_inliers": stats.get("pnp_inliers", 0),
            "fail_reason": "" if ok else node.last_relocalization_failure_reason,
            **{f"ms_{n}": round(float(timings.get(n, 0.0)), 2) for n in stage_names},
        })

        if (i + 1) % 25 == 0 or i + 1 == total_queries:
            print(f"[prog] {i+1}/{total_queries} reloc_true={n_reloc_true} success={n_success}", flush=True)

    n_eval = len(totals)
    summary = {
        "reference_map": str(map_dir),
        "query_map": str(query_dir),
        "cross_session": cross_session,
        "vocabulary": args.vocab,
        "orb_nfeatures": int(extractor.nfeatures),
        "matcher": args.matcher,
        "ransac": bool(args.ransac),
        "top_k": int(node.relocalization_loop_top_k),
        "reference_keyframes": len(ref_poses),
        "queries_attempted": total_queries,
        "queries_decode_failed": n_decode_fail,
        "queries_timed": n_eval,
        "warmup_queries_discarded": min(args.warmup_queries, total_queries),
        "import_s": round(import_s, 3),
        "node_build_s": round(node_build_s, 3),
        "rss_after_map_load_mb": round(rss_after_load_mb, 1),
        "rss_peak_mb": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, 1),
        "pos_threshold_m": args.pos_threshold_m,
        "rot_threshold_deg": args.rot_threshold_deg,
        "relocalize_returned_true": n_reloc_true,
        "relocalize_returned_true_rate": (n_reloc_true / n_eval) if n_eval else 0.0,
        "success_within_threshold": n_success,
        "success_rate": (n_success / n_eval) if n_eval else 0.0,
        "retrieval_threshold_m": args.retrieval_threshold_m,
        "retrieval_hit": n_retrieval_hit,
        "retrieval_hit_rate": (n_retrieval_hit / n_eval) if n_eval else 0.0,
        "success_rate_at_m": {
            str(t): (n_success_at[t] / n_eval) if n_eval else 0.0 for t in extra_thresholds
        },
        "outcome_counts": dict(sorted(fail_codes.items(), key=lambda kv: -kv[1])),
        "counters": {k: _percentiles(v) for k, v in counters.items()},
        "wall_ms": _percentiles(totals),
        "stage_ms": {n: _percentiles(v) for n, v in stage_acc.items()},
        "xy_err_m_when_true": _percentiles(xy_errors),
        "rot_err_deg_when_true": _percentiles(rot_errors),
    }

    out_json = Path(f"{args.out_prefix}.json")
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    import csv
    with Path(f"{args.out_prefix}.csv").open("w", newline="", encoding="utf-8") as f:
        if rows:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    print("\n===== SUMMARY =====")
    print(json.dumps({k: v for k, v in summary.items()
                      if k not in ("stage_ms", "counters", "outcome_counts")}, indent=2))

    print("\n--- where the queries went ---")
    for code, count in summary["outcome_counts"].items():
        label = code or "(none)"
        print(f"  {label:<16} {count:5d}  {count / n_eval * 100 if n_eval else 0:5.1f}%")
    print(f"  {'retrieval hit':<16} {n_retrieval_hit:5d}  "
          f"{n_retrieval_hit / n_eval * 100 if n_eval else 0:5.1f}%  "
          f"(a candidate within {args.retrieval_threshold_m} m of GT)")
    for t in extra_thresholds:
        rate = n_success_at[t] / n_eval * 100 if n_eval else 0
        print(f"  {'success <=' + str(t) + 'm':<16} {n_success_at[t]:5d}  {rate:5.1f}%")

    print("\n--- per-stage counters (mean/p50/p90) ---")
    for name, s in summary["counters"].items():
        if s:
            print(f"  {name:<18} mean={s['mean']:8.1f}  p50={s['p50']:8.1f}  p90={s['p90']:8.1f}")

    print("\n--- stage_ms (mean/p50/p90) ---")
    for name in stage_names:
        s = summary["stage_ms"][name]
        if s:
            print(f"  {name:<18} mean={s['mean']:7.1f}  p50={s['p50']:7.1f}  p90={s['p90']:7.1f}  max={s['max']:7.1f}")
    print(f"wrote {out_json}")

    if cross_session:
        query_db.close()
    try:
        node.db.close()
        node.nav_temp_db.close()
    except Exception:
        pass
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
