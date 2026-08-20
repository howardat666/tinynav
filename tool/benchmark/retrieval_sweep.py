#!/usr/bin/env python3
"""Sweep vocabulary size across four retrieval backends on one pair of maps.

Scoring is imported verbatim from the upstream PR #210 tool so these numbers stay
directly comparable to its report; only descriptor generation is replaced here.

Two departures from upstream, both deliberate:

1. VLAD vocabularies are trained on Map A and reused to encode Map B. Upstream reads
   each map's own stored vlad_descriptors, i.e. each map uses centres trained on itself
   -- that leaks query-set statistics and is not what deployment does (the map ships its
   vocabulary; live frames are encoded with it). --vocab-source own reproduces upstream.
2. Every backend can be scored against one shared transform (--transform-json) instead
   of one fitted from its own Top-1 matches. Self-fitted transforms make the comparison
   circular: a weak backend drags the ground truth toward its own mistakes.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from upstream.map_retrieval_self_consistency_pr210 import (  # noqa: E402
    _apply_se2,
    _load_poses,
    _normalize_rows,
    _positive_set,
    _ransac_fit_se2,
    _retrieve_rows,
)

from tinynav.core.bow_retrieval import (  # noqa: E402
    BowConfig,
    _fit_vocab,
    assign_words,
    make_bow_vector,
    valid_superpoint_descriptors,
)
from tinynav.core.build_map_node import TinyNavDB  # noqa: E402
from tinynav.core.vlad import compute_vlad, train_vocabulary_streaming  # noqa: E402


_DESC_OVERRIDE: dict[str, dict] = {}


def _iter_local_descriptors(map_path: Path, timestamps: list[int], source: str):
    """Yield the per-keyframe local descriptor matrix a VLAD/BoW backend aggregates.

    An override lets a quantization study feed descriptors extracted outside the map,
    falling back per-keyframe so a frame missing from the npz still scores.
    """
    override = _DESC_OVERRIDE.get(str(Path(map_path).resolve())) if source == "superpoint" else None
    db = TinyNavDB(str(map_path), is_scratch=False)
    try:
        for timestamp in timestamps:
            if override is not None and str(int(timestamp)) in override:
                yield np.asarray(override[str(int(timestamp))], dtype=np.float32)
            elif source == "superpoint":
                yield valid_superpoint_descriptors(db.features[int(timestamp)])
            else:
                yield np.asarray(db.patch_tokens[int(timestamp)], dtype=np.float32)
    finally:
        db.close()


def _l2(x: np.ndarray) -> np.ndarray:
    return x / np.maximum(np.linalg.norm(x, axis=1, keepdims=True), 1e-8)


def _sampled_frames(map_paths, timestamps_list, source, per_frame, seed):
    rng = np.random.default_rng(seed)
    if not isinstance(map_paths, list):
        map_paths, timestamps_list = [map_paths], [timestamps_list]
    for mp, ts in zip(map_paths, timestamps_list):
        yield from _sample_one(mp, ts, source, per_frame, rng)


def _sample_one(map_path, timestamps, source, per_frame, rng):
    for descriptors in _iter_local_descriptors(map_path, timestamps, source):
        if len(descriptors) > per_frame:
            descriptors = descriptors[rng.choice(len(descriptors), size=per_frame, replace=False)]
        yield np.asarray(descriptors, dtype=np.float32)


def _train_centres(map_path, timestamps, source, vocab_size, per_frame, seed, trainer="batch",
                   attempts=None, max_iter=40):
    if trainer == "streaming":
        # Online k-means: one incremental pass per epoch instead of 40 iterations x 3 restarts.
        # This is what vlad.py already uses, and it is why VLAD trains in seconds.
        # Centres are seeded from the first batch, so it must hold at least vocab_size
        # vectors -- vlad.py's 1024 default only ever saw K=32.
        return train_vocabulary_streaming(
            lambda: _sampled_frames(map_path, timestamps, source, per_frame, seed),
            vocab_size=vocab_size, seed=seed, batch_size=max(1024, vocab_size * 2))
    samples = list(_sampled_frames(map_path, timestamps, source, per_frame, seed))
    # patch tokens arrive unnormalized; SuperPoint is already unit-norm, so this is a no-op there.
    pool = _l2(np.concatenate(samples, axis=0).astype(np.float32))
    if attempts is None:
        return _fit_vocab(pool, BowConfig(vocab_size=vocab_size, random_seed=seed))
    # bow_retrieval hard-codes attempts=3; cost is linear in attempts x max_iter.
    import cv2
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iter, 0.02)
    _c, _l, centres = cv2.kmeans(pool, vocab_size, None, crit, attempts, cv2.KMEANS_PP_CENTERS)
    return _l2(centres.astype(np.float32))


def _encode_vlad(map_path, timestamps, source, centres):
    rows = [compute_vlad(d, centres) for d in _iter_local_descriptors(map_path, timestamps, source)]
    return _normalize_rows(rows, map_path)


def _encode_bow(map_path, timestamps, centres, idf):
    rows = [
        make_bow_vector(d, centres, idf)
        for d in _iter_local_descriptors(map_path, timestamps, "superpoint")
    ]
    return _normalize_rows(rows, map_path)


def _bow_idf(map_path, timestamps, centres):
    df = np.zeros(len(centres), dtype=np.float32)
    for descriptors in _iter_local_descriptors(map_path, timestamps, "superpoint"):
        words = assign_words(descriptors, centres)
        df += np.bincount(words, minlength=len(centres)) > 0
    return (np.log((1 + len(timestamps)) / (1 + df)) + 1.0).astype(np.float32)


def _load_stored(map_path: Path, timestamps: list[int], field: str) -> np.ndarray:
    db = TinyNavDB(str(map_path), is_scratch=False)
    try:
        rows = [np.asarray(getattr(db, field)[int(t)], dtype=np.float32).reshape(-1) for t in timestamps]
    finally:
        db.close()
    return _normalize_rows(rows, map_path)


def build_embeddings(args, map_a, map_b, ts_a, ts_b, vocab_size):
    """Return (map_a_embeddings, map_b_embeddings) for one backend/vocab-size point."""
    backend = args.backend
    if backend == "dinov2_global":
        return _load_stored(map_a, ts_a, "embeddings"), _load_stored(map_b, ts_b, "embeddings")

    if backend == "superpoint_bow":
        centres = _train_centres(map_a, ts_a, "superpoint", vocab_size,
                                 args.train_desc_per_frame_bow, args.seed, args.vocab_trainer,
                                 args.kmeans_attempts, args.kmeans_max_iter)
        idf = _bow_idf(map_a, ts_a, centres)
        return _encode_bow(map_a, ts_a, centres, idf), _encode_bow(map_b, ts_b, centres, idf)

    source = "superpoint" if backend == "sp_vlad" else "dinov2"
    if args.vocab_source == "own":
        if backend != "dinov2_vlad":
            raise SystemExit("--vocab-source own only exists for dinov2_vlad (the stored K=32 index)")
        return _load_stored(map_a, ts_a, "vlad_descriptors"), _load_stored(map_b, ts_b, "vlad_descriptors")

    # A frozen vocabulary trained elsewhere is what makes VLAD usable during mapping:
    # loop closure needs a descriptor before the map's own vocabulary could exist.
    if args.vocab_map:
        vocab_map = [Path(m) for m in args.vocab_map.split(",")]
        vocab_ts = [sorted(_load_poses(m)) for m in vocab_map]
    else:
        vocab_map, vocab_ts = map_a, ts_a
    centres = _train_centres(vocab_map, vocab_ts, source, vocab_size,
                             args.train_desc_per_frame_vlad, args.seed, args.vocab_trainer)
    return _encode_vlad(map_a, ts_a, source, centres), _encode_vlad(map_b, ts_b, source, centres)


def retrieve_dbow3(map_a, map_b, ts_a, ts_b, poses_a, vocab_size, branching, topk, train_every_n):
    """DBoW3 ranks internally and never exposes a vector, so it skips the embedding
    matrix entirely and emits _retrieve_rows' output shape directly.

    Its vocabulary is a tree: `branching`-way k-means repeated `levels` deep gives
    branching**levels words, which is why 10k words cost minutes here and hours for
    the flat cv2 k-means the other backends use.
    """
    import pydbow3

    levels = max(1, round(math.log(vocab_size) / math.log(branching)))
    desc_a = list(_iter_local_descriptors(map_a, ts_a, "superpoint"))
    voc = pydbow3.Vocabulary(branching, levels)
    voc.create([d for d in desc_a[::train_every_n] if len(d)])
    db = pydbow3.Database()
    db.setVocabulary(voc)

    row_to_ts = []
    for timestamp, descriptors in zip(ts_a, desc_a):
        if len(descriptors):
            db.add(np.ascontiguousarray(descriptors, dtype=np.float32))
            row_to_ts.append(int(timestamp))

    rows = []
    for timestamp, descriptors in zip(ts_b, _iter_local_descriptors(map_b, ts_b, "superpoint")):
        retrieved = []
        if len(descriptors):
            for rank, r in enumerate(db.query(np.ascontiguousarray(descriptors, dtype=np.float32),
                                              topk), start=1):
                rid = int(getattr(r, "Id", getattr(r, "id", -1)))
                if 0 <= rid < len(row_to_ts):
                    ts_hit = row_to_ts[rid]
                    retrieved.append({"rank": rank, "timestamp_ns": ts_hit,
                                      "similarity": float(getattr(r, "Score", 0.0)),
                                      "pose_xy": poses_a[ts_hit][:2, 3].tolist()})
        if retrieved:
            rows.append({"query_timestamp_ns": int(timestamp), "retrieved": retrieved})

    # A tf-idf histogram is one float per word per keyframe, same accounting as the other backends.
    return rows, int(voc.size()), int(voc.size()) * len(row_to_ts) * 4


def _quantize(x, dtype):
    """Round-trip through a narrower dtype to model a quantized resident index.

    Scoring still runs in float32: what we are measuring is the information lost by
    storing the index narrower, not the arithmetic of a narrow matmul.
    """
    if dtype == "float16":
        return x.astype(np.float16).astype(np.float32)
    scale = np.abs(x).max(axis=1, keepdims=True) / 127.0
    return (np.round(x / np.maximum(scale, 1e-12)).clip(-127, 127) * scale).astype(np.float32)


def score(query_rows, query_gt_xy, ts_a, positions_a, topk_values, thresholds):
    out = []
    for threshold in thresholds:
        for topk in topk_values:
            hits = 0
            for row, gt_xy in zip(query_rows, query_gt_xy):
                gt_set = _positive_set(ts_a, positions_a, gt_xy, threshold)
                if {int(h["timestamp_ns"]) for h in row["retrieved"][:topk]} & gt_set:
                    hits += 1
            out.append({"threshold_m": threshold, "topk": topk,
                        "recall_at_k": hits / max(1, len(query_rows))})
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--map-a", required=True, help="Reference map (holds the vocabulary)")
    p.add_argument("--map-b", required=True, help="Query map")
    p.add_argument("--backend", required=True,
                   choices=["dinov2_global", "dinov2_vlad", "superpoint_bow", "sp_vlad", "sp_dbow3"])
    p.add_argument("--vocab-sizes", default="16,32,64,128,256")
    p.add_argument("--vocab-source", choices=["shared", "own"], default="shared")
    p.add_argument("--transform-json", default=None,
                   help="Score against this transform instead of one fitted from Top-1")
    p.add_argument("--out", required=True)
    p.add_argument("--topk", default="1,3,5,10")
    p.add_argument("--distance-thresholds", default="0.5,1.0")
    p.add_argument("--train-desc-per-frame-vlad", type=int, default=64)
    p.add_argument("--train-desc-per-frame-bow", type=int, default=40)
    p.add_argument("--ransac-threshold-m", type=float, default=0.5)
    p.add_argument("--ransac-iterations", type=int, default=3000)
    p.add_argument("--dbow3-branching", type=int, default=10,
                   help="Tree branching factor; words = branching**levels")
    p.add_argument("--dbow3-train-every-n", type=int, default=4,
                   help="Subsample keyframes for tree training (matches the old SP run)")
    p.add_argument("--vocab-trainer", choices=["batch", "streaming"], default="batch",
                   help="batch = cv2.kmeans; streaming = vlad.py online k-means")
    p.add_argument("--kmeans-attempts", type=int, default=None,
                   help="Override bow_retrieval's hard-coded 3 restarts")
    p.add_argument("--kmeans-max-iter", type=int, default=40)
    p.add_argument("--index-dtype", choices=["float32", "float16", "int8"], default="float32",
                   help="Model a quantized resident index; scoring stays float32")
    p.add_argument("--vocab-map", default=None,
                   help="Comma-separated maps to train the vocabulary on instead of map-a")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--map-a-stride", type=int, default=1,
                   help="Keep every Nth reference keyframe: the map is built at ~10 Hz / 6 cm "
                        "spacing, far denser than the 0.5 m the metric asks for")
    p.add_argument("--descriptor-npz", default=None,
                   help="Comma-separated map_dir=file.npz overriding stored SuperPoint descriptors")
    args = p.parse_args()

    for spec in filter(None, (args.descriptor_npz or "").split(",")):
        map_dir, npz = spec.split("=", 1)
        _DESC_OVERRIDE[str(Path(map_dir).resolve())] = np.load(npz)
        print(f"[override] {map_dir} <- {npz}", file=sys.stderr)

    map_a, map_b = Path(args.map_a), Path(args.map_b)
    poses_a, poses_b = _load_poses(map_a), _load_poses(map_b)
    ts_a, ts_b = sorted(poses_a)[:: max(1, args.map_a_stride)], sorted(poses_b)
    positions_a = np.stack([poses_a[t][:3, 3] for t in ts_a], axis=0)
    topk_values = [int(v) for v in args.topk.split(",")]
    thresholds = [float(v) for v in args.distance_thresholds.split(",")]

    fixed = None
    if args.transform_json:
        blk = json.load(open(args.transform_json))
        blk = blk.get("T_map_a_map_b_se2", blk)
        fixed = (np.asarray(blk["R_xy"], np.float64), np.asarray(blk["t_xy"], np.float64))

    vocab_sizes = [0] if args.backend == "dinov2_global" else \
        [int(v) for v in args.vocab_sizes.split(",")]

    results = []
    for vocab_size in vocab_sizes:
        t0 = time.time()
        if args.backend == "sp_dbow3":
            rows, dim, index_bytes = retrieve_dbow3(map_a, map_b, ts_a, ts_b, poses_a,
                                                   vocab_size, args.dbow3_branching,
                                                   max(topk_values), args.dbow3_train_every_n)
            emb_a = emb_b = None
        else:
            emb_a, emb_b = build_embeddings(args, map_a, map_b, ts_a, ts_b, vocab_size)
            dim, index_bytes = int(emb_a.shape[1]), emb_a.nbytes
            if args.index_dtype != "float32":
                emb_a = _quantize(emb_a, args.index_dtype)
                emb_b = _quantize(emb_b, args.index_dtype)
                index_bytes = index_bytes // (2 if args.index_dtype == "float16" else 4)
            rows = _retrieve_rows(emb_a @ emb_b.T, ts_a, poses_a, ts_b, max(topk_values))
        encode_s = time.time() - t0

        src_xy = np.stack([poses_b[int(r["query_timestamp_ns"])][:2, 3] for r in rows])
        dst_xy = np.stack([poses_a[int(r["retrieved"][0]["timestamp_ns"])][:2, 3] for r in rows])
        self_R, self_t, _ = _ransac_fit_se2(src_xy, dst_xy, args.ransac_threshold_m,
                                            args.ransac_iterations, args.seed)

        entry = {
            "backend": args.backend, "vocab_size": vocab_size,
            "vocab_source": args.vocab_source,
            "descriptor_dim": dim,
            "index_mb": round(index_bytes / 1e6, 1),
            "encode_seconds": round(encode_s, 1),
            "query_count": len(rows),
            "self_fitted_yaw_deg": round(math.degrees(math.atan2(self_R[1, 0], self_R[0, 0])), 3),
            "self_fitted_R_xy": self_R.tolist(),
            "self_fitted_t_xy": self_t.tolist(),
            "self_fitted": score(rows, _apply_se2(src_xy, self_R, self_t),
                                 ts_a, positions_a, topk_values, thresholds),
        }
        if fixed is not None:
            entry["shared_transform"] = score(rows, _apply_se2(src_xy, *fixed),
                                              ts_a, positions_a, topk_values, thresholds)
        results.append(entry)
        del emb_a, emb_b

        r1 = next(m["recall_at_k"] for m in entry["self_fitted"]
                  if m["topk"] == 1 and m["threshold_m"] == 0.5)
        line = f"[{args.backend}] K={vocab_size:<4} dim={entry['descriptor_dim']:<6} " \
               f"R@1={r1*100:5.1f}%  yaw={entry['self_fitted_yaw_deg']:+7.3f}°  " \
               f"{entry['index_mb']:>6.1f} MB  {encode_s:5.1f}s"
        if fixed is not None:
            r1s = next(m["recall_at_k"] for m in entry["shared_transform"]
                       if m["topk"] == 1 and m["threshold_m"] == 0.5)
            line += f"   [shared T] R@1={r1s*100:5.1f}%"
        print(line, flush=True)

        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        json.dump({"map_a": str(map_a), "map_b": str(map_b), "results": results},
                  open(args.out, "w"), indent=1)


if __name__ == "__main__":
    main()
