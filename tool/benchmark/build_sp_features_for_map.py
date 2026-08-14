#!/usr/bin/env python3
"""Add SuperPoint descriptors to a map that was built with ORB, without rebuilding it.

Rebuilding a map to change its feature layer takes an hour per map and produces a
different keyframe set and different poses, so an ORB-vs-SuperPoint comparison across
two rebuilds would be measuring three changes at once. Reading the map's own keyframe
images back out and running SuperPoint over them keeps the keyframes, the poses and
the stored depth byte-identical, which leaves the descriptor as the only difference.

Writes <map>/sp_features_<tag>.npz, loadable by reloc_offline_eval.py --sp-features.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tinynav.core.build_map_node import TinyNavDB  # noqa: E402
from tinynav.core.models_trt import SuperPointORT  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--map", required=True, help="Map directory to augment")
    p.add_argument("--model", required=True, help="superpoint_fp16_dynamic.onnx")
    p.add_argument("--threshold", type=float, default=0.0005)
    p.add_argument("--net-hw", type=str, default="320,272", help="Network input H,W")
    p.add_argument("--top-k-keypoints", type=int, default=0,
                   help="Keep only the highest-scoring K keypoints (0 = keep all). Use to "
                        "control for count when comparing against ORB's fixed 1024")
    p.add_argument("--tag", default=None, help="Output suffix (default derived from threshold)")
    p.add_argument("--num-threads", type=int, default=0)
    args = p.parse_args()

    net_h, net_w = (int(v) for v in args.net_hw.split(","))
    tag = args.tag or (f"th{args.threshold:g}" + (f"_top{args.top_k_keypoints}" if args.top_k_keypoints else ""))
    out_path = Path(args.map) / f"sp_features_{tag}.npz"

    sp = SuperPointORT(args.model, net_hw=(net_h, net_w), threshold=args.threshold,
                       num_threads=args.num_threads)
    db = TinyNavDB(args.map, is_scratch=False)
    poses = np.load(Path(args.map) / "poses.npy", allow_pickle=True).item()
    stamps = sorted(int(k) for k in poses)

    import asyncio
    payload: dict[str, np.ndarray] = {}
    counts, t0 = [], time.perf_counter()
    for i, ts in enumerate(stamps):
        image = db.infra1_video_db.read(int(ts))
        if image is None:
            continue
        if image.ndim == 3:
            image = image[..., 0]
        f = asyncio.run(sp.infer(image))
        keep = np.asarray(f["mask"])[0, :, 0] > 0
        kp = np.asarray(f["kpts"])[0][keep]
        desc = np.asarray(f["descps"])[0][keep]
        score = np.asarray(f["scores"])[0][keep]
        if args.top_k_keypoints and kp.shape[0] > args.top_k_keypoints:
            order = np.argsort(-score)[: args.top_k_keypoints]
            kp, desc, score = kp[order], desc[order], score[order]
        counts.append(kp.shape[0])
        payload[f"{ts}_kpts"] = kp.astype(np.float32)
        payload[f"{ts}_descps"] = desc.astype(np.float32)
        if (i + 1) % 200 == 0:
            print(f"  [{i+1}/{len(stamps)}] mean_kpts={np.mean(counts):.0f}", flush=True)

    np.savez(out_path, **payload)
    db.close()
    a = np.asarray(counts)
    print(f"wrote {out_path}  frames={len(counts)}  "
          f"kpts mean={a.mean():.1f} p50={np.percentile(a,50):.0f} p10={np.percentile(a,10):.0f} "
          f"min={a.min()} zero={int((a==0).sum())}  elapsed={time.perf_counter()-t0:.0f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
