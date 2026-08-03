#!/usr/bin/env python3
"""Cross-map place-retrieval evaluation with pluggable descriptor backends.

Map B keyframes query Map A. The metric is R@K under a distance threshold, so a
metric SE(2) transform between the two independent VIO sessions is required.

Two ways to get that transform:

  * ``--fit-transform`` (default) fits it by RANSAC over the *evaluated
    backend's* own Top-1 matches. Self-consistent, but circular: a backend that
    is wrong in a consistent way still scores well. Fine for a single map pair,
    misleading when comparing backends.
  * ``--transform-json FILE`` reuses a transform fitted once and shared by every
    backend. This is what makes a cross-backend comparison meaningful. Run the
    strongest available backend first with ``--save-transform FILE``, then point
    all remaining backends at that file.

Backends (``--descriptor``):

  ``embedding``     global vector stored in the map (DINOv2 CLS / VLAD), cosine
  ``dbow3``         DBoW3 over the map's ORB descriptors (needs pyDBoW3 + ORBvoc)
  ``dbow3-trained`` DBoW3 over a small k**L vocabulary trained on map A itself,
                    instead of the ~1M-word pretrained ORBvoc
  ``bow-cv2``       cv2.kmeans vocabulary + TF-IDF over the map's descriptors,
                    pure OpenCV, no pyDBoW3 build required
  ``bf-l2``         brute-force descriptor matching, scored by inlier count.
                    Accurate but O(N*M) -- use ``--every-n`` to subsample.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import resource
import time
from pathlib import Path
from typing import Any

import numpy as np

from tinynav.core.build_map_node import TinyNavDB


def _parse_float_list(value: str) -> list[float]:
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def _parse_int_list(value: str) -> list[int]:
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def _load_poses(map_path: Path) -> dict[int, np.ndarray]:
    poses_path = map_path / "poses.npy"
    if not poses_path.exists():
        raise FileNotFoundError(f"Missing poses file: {poses_path}")
    raw_poses = np.load(poses_path, allow_pickle=True).item()
    return {int(timestamp): np.asarray(pose, dtype=np.float64) for timestamp, pose in raw_poses.items()}


# --------------------------------------------------------------------------- #
# descriptor loading
# --------------------------------------------------------------------------- #

def _normalize_rows(rows: list[np.ndarray], map_path: Path) -> np.ndarray:
    if not rows:
        raise RuntimeError(f"No descriptors loaded from {map_path}")
    x = np.stack([np.asarray(row, dtype=np.float32).reshape(-1) for row in rows], axis=0)
    norms = np.linalg.norm(x, axis=1, keepdims=True)
    if np.any(norms <= 1e-8):
        raise ValueError(f"{map_path} contains a near-zero descriptor -- was the map built in bow mode?")
    return x / np.maximum(norms, 1e-8)


def _load_global_embeddings(map_path: Path, timestamps: list[int]) -> np.ndarray:
    """Read the per-keyframe global descriptor written during mapping."""
    db = TinyNavDB(str(map_path), is_scratch=False)
    rows: list[np.ndarray] = []
    try:
        # Newer maps expose a dedicated VLAD shelf; older ones only have `embeddings`.
        store = getattr(db, "vlad_descriptors", None)
        if store is None:
            store = db.embeddings
        for timestamp in timestamps:
            rows.append(np.asarray(store[int(timestamp)], dtype=np.float32))
    finally:
        db.close()
    return _normalize_rows(rows, map_path)


def _to_uint8_desc(desc: np.ndarray) -> np.ndarray:
    """ORB descriptors are stored as float32 in [0,1]; bring them back to bytes."""
    desc = np.asarray(desc)
    if desc.ndim == 3:
        desc = desc[0]
    if desc.dtype == np.uint8:
        return np.ascontiguousarray(desc)
    return np.ascontiguousarray(np.clip(np.rint(desc * 255.0), 0, 255).astype(np.uint8))


def _load_local_descriptors(map_path: Path, timestamps: list[int]) -> list[np.ndarray]:
    """Read per-keyframe local descriptors (N, D) in timestamp order."""
    db = TinyNavDB(str(map_path), is_scratch=False)
    out: list[np.ndarray] = []
    try:
        for timestamp in timestamps:
            features = db.features[int(timestamp)]
            desc = np.asarray(features["descps"])
            if desc.ndim == 3:
                desc = desc[0]
            out.append(np.ascontiguousarray(desc))
    finally:
        db.close()
    return out


# --------------------------------------------------------------------------- #
# similarity backends -- each returns an (len(A), len(B)) score matrix
# --------------------------------------------------------------------------- #

def _similarity_embedding(map_a, ts_a, map_b, ts_b, args) -> np.ndarray:
    return _load_global_embeddings(map_a, ts_a) @ _load_global_embeddings(map_b, ts_b).T


def _similarity_dbow3(map_a, ts_a, map_b, ts_b, args) -> np.ndarray:
    try:
        import pyDBoW3 as bow
    except ImportError:
        import pydbow3 as bow  # type: ignore

    voc = bow.Vocabulary()
    if voc.load(args.dbow3_vocabulary_path) is False:
        raise RuntimeError(f"Failed to load DBoW3 vocabulary: {args.dbow3_vocabulary_path}")
    database = bow.Database()
    database.setVocabulary(voc)

    desc_a = _load_local_descriptors(map_a, ts_a)
    # DBoW3 indexes entries by insertion order, which we keep aligned with ts_a.
    for desc in desc_a:
        database.add(_to_uint8_desc(desc))

    desc_b = _load_local_descriptors(map_b, ts_b)
    similarities = np.zeros((len(ts_a), len(ts_b)), dtype=np.float32)
    for query_index, desc in enumerate(desc_b):
        try:
            results = database.query(_to_uint8_desc(desc), len(ts_a))
        except TypeError:
            results = database.query(_to_uint8_desc(desc))
        for r in results:
            entry = int(getattr(r, "Id", getattr(r, "id", -1)))
            score = float(getattr(r, "Score", getattr(r, "score", 0.0)))
            if 0 <= entry < len(ts_a):
                similarities[entry, query_index] = score
    return similarities


def _similarity_dbow3_trained(map_a, ts_a, map_b, ts_b, args) -> np.ndarray:
    """DBoW3 over a k^L hierarchical vocabulary trained on map A itself.

    The stock ORBvoc is k=10, L=6 (~1M words, ~475 MB resident) which does not
    fit comfortably beside the camera firmware on the X5. A vocabulary trained
    on the target map is orders of magnitude smaller and domain-matched.
    """
    try:
        import pyDBoW3 as bow
    except ImportError:
        import pydbow3 as bow  # type: ignore

    desc_a = _load_local_descriptors(map_a, ts_a)
    training = [_to_uint8_desc(d) for d in desc_a if d.shape[0] > 0]

    voc = bow.Vocabulary(args.dbow3_k, args.dbow3_levels)
    voc.create(training)

    database = bow.Database()
    database.setVocabulary(voc)
    del voc  # the Database keeps its own copy; drop ours to halve resident size

    for desc in desc_a:
        database.add(_to_uint8_desc(desc))

    desc_b = _load_local_descriptors(map_b, ts_b)
    similarities = np.zeros((len(ts_a), len(ts_b)), dtype=np.float32)
    for query_index, desc in enumerate(desc_b):
        for r in database.query(_to_uint8_desc(desc), len(ts_a)):
            entry = int(r.Id)
            if 0 <= entry < len(ts_a):
                similarities[entry, query_index] = float(r.Score)
    return similarities


def _similarity_bow_cv2(map_a, ts_a, map_b, ts_b, args) -> np.ndarray:
    """cv2.kmeans vocabulary + TF-IDF, trained on map A. No pyDBoW3 needed."""
    import cv2

    desc_a = _load_local_descriptors(map_a, ts_a)
    desc_b = _load_local_descriptors(map_b, ts_b)

    pool = np.concatenate([d for d in desc_a if d.size], axis=0).astype(np.float32)
    rng = np.random.default_rng(args.seed)
    if pool.shape[0] > args.bow_train_samples:
        pool = pool[rng.choice(pool.shape[0], args.bow_train_samples, replace=False)]

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1e-3)
    _, _, centers = cv2.kmeans(
        np.ascontiguousarray(pool), args.bow_clusters, None, criteria, 3, cv2.KMEANS_PP_CENTERS
    )

    def histograms(descriptors: list[np.ndarray]) -> np.ndarray:
        hist = np.zeros((len(descriptors), args.bow_clusters), dtype=np.float32)
        for i, desc in enumerate(descriptors):
            if desc.size == 0:
                continue
            # (N, K) squared distances via the |a-b|^2 = |a|^2 - 2ab + |b|^2 expansion.
            d = desc.astype(np.float32)
            dist = (d * d).sum(1)[:, None] - 2.0 * d @ centers.T + (centers * centers).sum(1)[None, :]
            np.add.at(hist[i], np.argmin(dist, axis=1), 1.0)
        return hist

    hist_a = histograms(desc_a)
    hist_b = histograms(desc_b)

    # IDF is defined by the database (map A) only, so queries stay independent.
    doc_freq = (hist_a > 0).sum(axis=0).astype(np.float32)
    idf = np.log(len(desc_a) / np.maximum(doc_freq, 1.0))

    def tfidf(hist: np.ndarray) -> np.ndarray:
        tf = hist / np.maximum(hist.sum(axis=1, keepdims=True), 1.0)
        v = tf * idf[None, :]
        return v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-8)

    return tfidf(hist_a) @ tfidf(hist_b).T


def _similarity_bf_l2(map_a, ts_a, map_b, ts_b, args) -> np.ndarray:
    """Brute-force matching scored by ratio-test survivor count. O(N*M) -- slow."""
    import cv2

    desc_a = _load_local_descriptors(map_a, ts_a)
    desc_b = _load_local_descriptors(map_b, ts_b)
    binary = desc_a[0].shape[1] == 32
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING if binary else cv2.NORM_L2)

    prepared_a = [_to_uint8_desc(d) if binary else d.astype(np.float32) for d in desc_a]
    similarities = np.zeros((len(ts_a), len(ts_b)), dtype=np.float32)
    for j, desc in enumerate(desc_b):
        query = _to_uint8_desc(desc) if binary else desc.astype(np.float32)
        if query.shape[0] == 0:
            continue
        for i, train in enumerate(prepared_a):
            if train.shape[0] < 2:
                continue
            good = 0
            for pair in matcher.knnMatch(query, train, k=2):
                if len(pair) == 2 and pair[0].distance < args.ratio * pair[1].distance:
                    good += 1
            similarities[i, j] = good
    return similarities


SIMILARITY_BACKENDS = {
    "embedding": _similarity_embedding,
    "dbow3": _similarity_dbow3,
    "dbow3-trained": _similarity_dbow3_trained,
    "bow-cv2": _similarity_bow_cv2,
    "bf-l2": _similarity_bf_l2,
}


# --------------------------------------------------------------------------- #
# SE(2) fitting
# --------------------------------------------------------------------------- #

def _fit_se2(src_xy: np.ndarray, dst_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    src_mean = src_xy.mean(axis=0)
    dst_mean = dst_xy.mean(axis=0)
    src_centered = src_xy - src_mean
    dst_centered = dst_xy - dst_mean
    h = src_centered.T @ dst_centered
    u, _, vt = np.linalg.svd(h)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = dst_mean - rotation @ src_mean
    return rotation, translation


def _apply_se2(xy: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return xy @ rotation.T + translation[None, :]


def _ransac_fit_se2(
    src_xy: np.ndarray,
    dst_xy: np.ndarray,
    threshold_m: float,
    iterations: int,
    seed: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    best_inliers = np.zeros(len(src_xy), dtype=bool)
    best_residuals = np.full(len(src_xy), np.inf, dtype=np.float64)
    for _ in range(iterations):
        indices = rng.choice(len(src_xy), size=2, replace=False)
        if np.linalg.norm(src_xy[indices[0]] - src_xy[indices[1]]) < 1e-6:
            continue
        rotation, translation = _fit_se2(src_xy[indices], dst_xy[indices])
        residuals = np.linalg.norm(_apply_se2(src_xy, rotation, translation) - dst_xy, axis=1)
        inliers = residuals <= threshold_m
        if inliers.sum() > best_inliers.sum() or (
            inliers.sum() == best_inliers.sum()
            and np.median(residuals[inliers]) < np.median(best_residuals[best_inliers])
        ):
            best_inliers = inliers
            best_residuals = residuals
    if best_inliers.sum() >= 2:
        rotation, translation = _fit_se2(src_xy[best_inliers], dst_xy[best_inliers])
    else:
        rotation, translation = _fit_se2(src_xy, dst_xy)
    residuals = np.linalg.norm(_apply_se2(src_xy, rotation, translation) - dst_xy, axis=1)
    return rotation, translation, residuals


def _load_transform(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open(encoding="utf-8") as f:
        payload = json.load(f)
    block = payload.get("T_map_a_map_b_se2", payload)
    rotation = np.asarray(block["R_xy"], dtype=np.float64)
    translation = np.asarray(block["t_xy"], dtype=np.float64)
    if rotation.shape != (2, 2) or translation.shape != (2,):
        raise ValueError(f"Malformed SE(2) transform in {path}")
    return rotation, translation


# --------------------------------------------------------------------------- #
# evaluation
# --------------------------------------------------------------------------- #

def _retrieve_rows(
    similarities: np.ndarray,
    map_a_timestamps: list[int],
    map_a_poses: dict[int, np.ndarray],
    map_b_timestamps: list[int],
    topk: int,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for query_index, timestamp_b in enumerate(map_b_timestamps):
        query_similarities = similarities[:, query_index]
        if topk >= len(query_similarities):
            top_indices = np.argsort(-query_similarities)
        else:
            unsorted = np.argpartition(-query_similarities, topk - 1)[:topk]
            top_indices = unsorted[np.argsort(-query_similarities[unsorted])]
        retrieved = []
        for rank, map_a_index in enumerate(top_indices, start=1):
            timestamp_a = int(map_a_timestamps[int(map_a_index)])
            retrieved.append(
                {
                    "rank": rank,
                    "timestamp_ns": timestamp_a,
                    "similarity": float(query_similarities[int(map_a_index)]),
                    "pose_xy": map_a_poses[timestamp_a][:2, 3].tolist(),
                }
            )
        rows.append({"query_timestamp_ns": int(timestamp_b), "retrieved": retrieved})
    return rows


def _positive_set(
    map_a_timestamps: list[int],
    map_a_positions: np.ndarray,
    gt_xy: np.ndarray,
    threshold_m: float,
) -> set[int]:
    distances = np.linalg.norm(map_a_positions[:, :2] - gt_xy[None, :], axis=1)
    return {int(map_a_timestamps[index]) for index in np.flatnonzero(distances <= threshold_m)}


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=True) + "\n")


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_eval(args: argparse.Namespace) -> dict[str, Any]:
    start_time = time.time()
    map_a = Path(args.map_a)
    map_b = Path(args.map_b)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    map_a_poses = _load_poses(map_a)
    map_b_poses = _load_poses(map_b)
    map_a_timestamps = sorted(map_a_poses)
    map_b_timestamps = sorted(map_b_poses)
    if args.max_queries > 0:
        map_b_timestamps = map_b_timestamps[: args.max_queries]
    if args.every_n > 1:
        map_b_timestamps = map_b_timestamps[:: args.every_n]

    topk_values = _parse_int_list(args.topk)
    thresholds = _parse_float_list(args.distance_thresholds)
    max_topk = max(topk_values)

    backend = SIMILARITY_BACKENDS[args.descriptor]
    descriptor_start = time.time()
    similarities = backend(map_a, map_a_timestamps, map_b, map_b_timestamps, args)
    descriptor_elapsed = time.time() - descriptor_start
    if similarities.shape != (len(map_a_timestamps), len(map_b_timestamps)):
        raise RuntimeError(
            f"Backend {args.descriptor} returned {similarities.shape}, "
            f"expected {(len(map_a_timestamps), len(map_b_timestamps))}"
        )

    query_rows = _retrieve_rows(similarities, map_a_timestamps, map_a_poses, map_b_timestamps, max_topk)
    src_xy = np.stack([map_b_poses[int(row["query_timestamp_ns"])][:2, 3] for row in query_rows], axis=0)

    if args.transform_json:
        rotation, translation = _load_transform(Path(args.transform_json))
        transform_source = str(args.transform_json)
    else:
        # Circular when comparing backends -- see the module docstring.
        dst_xy = np.stack(
            [map_a_poses[int(row["retrieved"][0]["timestamp_ns"])][:2, 3] for row in query_rows], axis=0
        )
        rotation, translation, _ = _ransac_fit_se2(
            src_xy, dst_xy, args.ransac_threshold_m, args.ransac_iterations, args.seed
        )
        transform_source = f"fitted from {args.descriptor} top1"

    query_gt_xy = _apply_se2(src_xy, rotation, translation)
    top1_xy = np.stack(
        [map_a_poses[int(row["retrieved"][0]["timestamp_ns"])][:2, 3] for row in query_rows], axis=0
    )
    top1_residuals = np.linalg.norm(top1_xy - query_gt_xy, axis=1)
    map_a_positions = np.stack([map_a_poses[timestamp][:3, 3] for timestamp in map_a_timestamps], axis=0)

    metrics: list[dict[str, Any]] = []
    for threshold in thresholds:
        for topk in topk_values:
            hit_count = 0
            empty_gt = 0
            precision_values = []
            recall_values = []
            iou_values = []
            for row, gt_xy in zip(query_rows, query_gt_xy):
                gt_set = _positive_set(map_a_timestamps, map_a_positions, gt_xy, threshold)
                if not gt_set:
                    empty_gt += 1
                predicted = {int(hit["timestamp_ns"]) for hit in row["retrieved"][:topk]}
                intersection = predicted & gt_set
                union = predicted | gt_set
                if intersection:
                    hit_count += 1
                precision_values.append(len(intersection) / max(1, len(predicted)))
                recall_values.append(len(intersection) / len(gt_set) if gt_set else 0.0)
                iou_values.append(len(intersection) / len(union) if union else 0.0)
            metrics.append(
                {
                    "descriptor": args.descriptor,
                    "threshold_m": threshold,
                    "topk": topk,
                    "query_count": len(query_rows),
                    "hit_count": hit_count,
                    # Queries with no map-A keyframe inside the threshold can never
                    # be hit; report them so a low R@K is not misread as a bad
                    # descriptor when it is really a coverage gap.
                    "queries_without_positive": empty_gt,
                    "recall_at_k": hit_count / max(1, len(query_rows)),
                    "mean_precision": float(np.mean(precision_values)),
                    "mean_set_recall": float(np.mean(recall_values)),
                    "mean_iou": float(np.mean(iou_values)),
                }
            )

    yaw_deg = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    t4 = np.eye(4, dtype=np.float64)
    t4[:2, :2] = rotation
    t4[:2, 3] = translation

    for row, gt_xy, residual in zip(query_rows, query_gt_xy, top1_residuals):
        row["query_gt_xy_in_map_a"] = gt_xy.tolist()
        row["top1_residual_m"] = float(residual)

    summary = {
        "type": "cross_map_retrieval",
        "descriptor": args.descriptor,
        "transform_source": transform_source,
        "note": (
            "R@K is measured against a SE(2) transform between two independent VIO "
            "sessions. With --transform-json the transform is shared across backends "
            "and the comparison is fair; without it the transform is fitted from this "
            "backend's own Top1 matches and the result is only self-consistency."
        ),
        "map_a": str(map_a),
        "map_b": str(map_b),
        "map_a_keyframes": len(map_a_timestamps),
        "map_b_queries": len(query_rows),
        "topk": topk_values,
        "distance_thresholds_m": thresholds,
        "ransac_threshold_m": args.ransac_threshold_m,
        "T_map_a_map_b_se2": {
            "T": t4.tolist(),
            "R_xy": rotation.tolist(),
            "t_xy": translation.tolist(),
            "yaw_deg": yaw_deg,
        },
        "top1_residual_m": {
            "mean": float(np.mean(top1_residuals)),
            "median": float(np.median(top1_residuals)),
            "p90": float(np.percentile(top1_residuals, 90)),
            "max": float(np.max(top1_residuals)),
        },
        "top1_inlier_ratio": {
            f"{threshold}m": float(np.mean(top1_residuals <= threshold)) for threshold in thresholds
        },
        "metrics": metrics,
        "descriptor_elapsed_s": descriptor_elapsed,
        # Peak RSS is the deciding number for on-board deployment: the X5 has
        # ~943 MB available with no swap, shared with the camera firmware.
        "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
        "elapsed_s": time.time() - start_time,
    }

    _write_jsonl(output_dir / "per_query_results.jsonl", query_rows)
    _write_csv(output_dir / "metrics.csv", metrics)
    with (output_dir / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=True, indent=2)
    if args.save_transform:
        with Path(args.save_transform).open("w", encoding="utf-8") as f:
            json.dump({"T_map_a_map_b_se2": summary["T_map_a_map_b_se2"]}, f, indent=2)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Cross-map place-retrieval evaluation with pluggable descriptor backends."
    )
    parser.add_argument("--map-a", required=True, help="Reference map directory")
    parser.add_argument("--map-b", required=True, help="Query/eval map directory")
    parser.add_argument("--output-dir", default="/tinynav/output/map_retrieval_self_consistency")
    parser.add_argument(
        "--descriptor", default="embedding", choices=sorted(SIMILARITY_BACKENDS),
        help="Retrieval backend to score",
    )
    parser.add_argument(
        "--dbow3-vocabulary-path", default="/tinynav/docs/Vocabulary/ORBvoc.txt",
        help="ORB vocabulary, used by --descriptor dbow3",
    )
    parser.add_argument("--dbow3-k", type=int, default=10, help="--descriptor dbow3-trained branching factor")
    parser.add_argument(
        "--dbow3-levels", type=int, default=4,
        help="--descriptor dbow3-trained tree depth; the vocabulary holds k**levels words",
    )
    parser.add_argument("--bow-clusters", type=int, default=512, help="--descriptor bow-cv2 vocabulary size")
    parser.add_argument(
        "--bow-train-samples", type=int, default=200000,
        help="Descriptors sampled from map A to train the bow-cv2 vocabulary",
    )
    parser.add_argument("--ratio", type=float, default=0.75, help="Lowe ratio for --descriptor bf-l2")
    parser.add_argument(
        "--transform-json",
        help="Reuse a shared SE(2) transform instead of fitting one. Required for a fair backend comparison.",
    )
    parser.add_argument("--save-transform", help="Write the SE(2) transform used, for reuse by other backends")
    parser.add_argument("--topk", default="1,3,5,10")
    parser.add_argument("--distance-thresholds", default="0.5,1.0")
    parser.add_argument("--ransac-threshold-m", type=float, default=0.5)
    parser.add_argument("--ransac-iterations", type=int, default=3000)
    parser.add_argument("--every-n", type=int, default=1)
    parser.add_argument("--max-queries", type=int, default=0)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    summary = run_eval(args)
    print(json.dumps(summary, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
