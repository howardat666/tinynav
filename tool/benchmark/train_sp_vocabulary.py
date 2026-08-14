#!/usr/bin/env python3
"""Train a DBoW3 vocabulary on SuperPoint descriptors and score its night retrieval.

The whole "one map for day and night" question reduces to the retrieval layer: matching
and PnP already convert essentially every retrieval hit into a correct pose, so night
success equals night retrieval hit rate. ORB retrieval caps that at 32%. This measures
what SuperPoint retrieval reaches on the same maps.

DBoW3 dispatches on cv::Mat type -- Hamming for CV_8U, L2 for CV_32F -- and the shim
already forwards float32 arrays as CV_32F, so no C++ change should be needed. Verify
that before trusting any number out of it.
"""
import argparse
import sys
import time

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2]))
import pydbow3  # noqa: E402


def load_map(map_dir, tag):
    z = np.load(f"{map_dir}/sp_features_{tag}.npz")
    stamps = sorted(int(k[: -len("_kpts")]) for k in z.files if k.endswith("_kpts"))
    return stamps, {ts: np.ascontiguousarray(z[f"{ts}_descps"], dtype=np.float32) for ts in stamps}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ref-map", required=True)
    p.add_argument("--query-map", required=True)
    p.add_argument("--tag", default="th5e-05")
    p.add_argument("--k", type=int, default=10)
    p.add_argument("--levels", type=int, default=3)
    p.add_argument("--train-every-n", type=int, default=4, help="Subsample frames for k-means")
    p.add_argument("--top-k", type=int, default=10)
    p.add_argument("--transform-json", default=None)
    p.add_argument("--save-vocab", default=None)
    args = p.parse_args()

    ref_stamps, ref_desc = load_map(args.ref_map, args.tag)
    q_stamps, q_desc = load_map(args.query_map, args.tag)
    print(f"[info] reference {len(ref_stamps)} kf, query {len(q_stamps)} kf", flush=True)

    train = [ref_desc[ts] for ts in ref_stamps[:: args.train_every_n] if ref_desc[ts].shape[0]]
    print(f"[info] training on {len(train)} frames, "
          f"{sum(t.shape[0] for t in train)} descriptors, dtype={train[0].dtype}", flush=True)
    t0 = time.perf_counter()
    voc = pydbow3.Vocabulary(args.k, args.levels)
    voc.create(train)
    print(f"[ok] vocabulary trained: {voc.size()} words in {time.perf_counter()-t0:.0f}s", flush=True)
    if args.save_vocab:
        voc.save(args.save_vocab)

    db = pydbow3.Database()
    db.setVocabulary(voc)
    row_to_ts = []
    for ts in ref_stamps:
        if ref_desc[ts].shape[0]:
            db.add(ref_desc[ts])
            row_to_ts.append(ts)
    print(f"[ok] database holds {db.size()} keyframes", flush=True)

    ref_poses = np.load(f"{args.ref_map}/poses.npy", allow_pickle=True).item()
    q_poses = np.load(f"{args.query_map}/poses.npy", allow_pickle=True).item()
    T = np.eye(4)
    if args.transform_json:
        import json
        blk = json.load(open(args.transform_json))
        blk = blk.get("T_map_a_map_b_se2", blk)
        T[:2, :2] = np.asarray(blk["R_xy"]); T[:2, 3] = np.asarray(blk["t_xy"])

    hits = {1: 0, 3: 0, 5: 0, 10: 0}
    n = 0
    t0 = time.perf_counter()
    for ts in q_stamps:
        d = q_desc[ts]
        if d.shape[0] == 0:
            continue
        gt = (T @ q_poses[int(ts)])[:2, 3]
        res = db.query(d, args.top_k)
        dists = []
        for r in res:
            rid = int(getattr(r, "Id", getattr(r, "id", -1)))
            if 0 <= rid < len(row_to_ts):
                dists.append(float(np.linalg.norm(ref_poses[row_to_ts[rid]][:2, 3] - gt)))
        n += 1
        for K in hits:
            if dists[:K] and min(dists[:K]) <= 0.5:
                hits[K] += 1
    q_ms = (time.perf_counter() - t0) / max(n, 1) * 1000
    print(f"\nR@1={hits[1]/n*100:.1f}%  R@3={hits[3]/n*100:.1f}%  "
          f"R@5={hits[5]/n*100:.1f}%  R@10={hits[10]/n*100:.1f}%   "
          f"(n={n}, query {q_ms:.1f} ms/frame)")


if __name__ == "__main__":
    main()
