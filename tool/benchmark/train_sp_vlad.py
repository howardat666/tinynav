#!/usr/bin/env python3
"""Train a VLAD vocabulary on SuperPoint descriptors and score its retrieval.

Upstream pairs VLAD with DINOv2 patch tokens (PR #198) and BoW with SuperPoint
(PR #194/#210); nobody has tried SuperPoint + VLAD, which is the combination that
would keep VLAD's night advantage without putting a ViT on the X5. This measures
whether that advantage survives the descriptor swap.

Reports R@K against the same ground truth as train_sp_vocabulary.py so the two
retrieval backends are directly comparable.
"""
import argparse
import json
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/tinynav")
from tinynav.core.vlad import compute_vlad  # noqa: E402


def load_map(map_dir, tag):
    z = np.load(f"{map_dir}/sp_features_{tag}.npz")
    stamps = sorted(int(k[: -len("_kpts")]) for k in z.files if k.endswith("_kpts"))
    return stamps, {ts: np.ascontiguousarray(z[f"{ts}_descps"], dtype=np.float32) for ts in stamps}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ref-map", required=True)
    p.add_argument("--query-map", required=True)
    p.add_argument("--tag", default="th5e-05")
    p.add_argument("--vocab-size", type=int, default=64)
    p.add_argument("--train-every-n", type=int, default=4)
    p.add_argument("--top-k", type=int, default=10)
    p.add_argument("--transform-json", default=None)
    p.add_argument("--save-centres", default=None)
    p.add_argument("--out-json", default=None)
    args = p.parse_args()

    ref_stamps, ref_desc = load_map(args.ref_map, args.tag)
    q_stamps, q_desc = load_map(args.query_map, args.tag)
    print(f"[info] reference {len(ref_stamps)} kf, query {len(q_stamps)} kf", flush=True)

    train = np.concatenate(
        [ref_desc[ts] for ts in ref_stamps[:: args.train_every_n] if ref_desc[ts].shape[0]], axis=0
    )
    print(f"[info] training on {train.shape[0]} descriptors, dim={train.shape[1]}", flush=True)

    t0 = time.perf_counter()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.02)
    _, _, centres = cv2.kmeans(train, args.vocab_size, None, criteria, 3, cv2.KMEANS_PP_CENTERS)
    centres = centres.astype(np.float32)
    centres /= np.maximum(np.linalg.norm(centres, axis=1, keepdims=True), 1e-8)
    train_s = time.perf_counter() - t0
    dim = centres.shape[0] * centres.shape[1]
    print(f"[ok] {args.vocab_size} centres in {train_s:.0f}s, descriptor dim={dim}", flush=True)
    if args.save_centres:
        np.savez_compressed(args.save_centres, centres=centres)

    t0 = time.perf_counter()
    ref_rows = [ts for ts in ref_stamps if ref_desc[ts].shape[0]]
    ref_vlad = np.stack([compute_vlad(ref_desc[ts], centres) for ts in ref_rows])
    encode_ms = (time.perf_counter() - t0) / len(ref_rows) * 1000
    index_mb = ref_vlad.nbytes / 1e6
    print(f"[ok] index {ref_vlad.shape} = {index_mb:.1f} MB, encode {encode_ms:.1f} ms/frame", flush=True)

    ref_poses = np.load(f"{args.ref_map}/poses.npy", allow_pickle=True).item()
    q_poses = np.load(f"{args.query_map}/poses.npy", allow_pickle=True).item()
    T = np.eye(4)
    if args.transform_json:
        blk = json.load(open(args.transform_json))
        blk = blk.get("T_map_a_map_b_se2", blk)
        T[:2, :2] = np.asarray(blk["R_xy"])
        T[:2, 3] = np.asarray(blk["t_xy"])
    ref_xy = np.stack([ref_poses[ts][:2, 3] for ts in ref_rows])

    hits = {1: 0, 3: 0, 5: 0, 10: 0}
    n = 0
    t0 = time.perf_counter()
    for ts in q_stamps:
        d = q_desc[ts]
        if d.shape[0] == 0:
            continue
        gt = (T @ q_poses[int(ts)])[:2, 3]
        sims = ref_vlad @ compute_vlad(d, centres)
        order = np.argsort(-sims)[: args.top_k]
        dists = np.linalg.norm(ref_xy[order] - gt, axis=1)
        n += 1
        for K in hits:
            if dists[:K].size and dists[:K].min() <= 0.5:
                hits[K] += 1
    q_ms = (time.perf_counter() - t0) / max(n, 1) * 1000
    print(f"\nR@1={hits[1]/n*100:.1f}%  R@3={hits[3]/n*100:.1f}%  "
          f"R@5={hits[5]/n*100:.1f}%  R@10={hits[10]/n*100:.1f}%   "
          f"(n={n}, query {q_ms:.1f} ms/frame)")

    if args.out_json:
        json.dump({"vocab_size": args.vocab_size, "descriptor_dim": dim,
                   "train_s": train_s, "index_mb": index_mb,
                   "encode_ms_per_frame": encode_ms, "query_ms_per_frame": q_ms,
                   "n_queries": n, "recall": {str(k): hits[k] / n for k in hits}},
                  open(args.out_json, "w"), indent=1)


if __name__ == "__main__":
    main()
