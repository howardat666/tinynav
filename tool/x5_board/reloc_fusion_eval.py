#!/usr/bin/env python3
"""跨会话评测候选融合策略：地图 B 的关键帧当 query 去地图 A 里重定位。

为什么不用图像：地图里每个关键帧的 SuperPoint 特征和 VLAD 描述子都存着，直接拿来当
query 就跳过了提特征这一步 —— 不需要图像、不需要 onnxruntime，而且用的就是板子当时
真正算出来的那份描述子。

真值：B 的关键帧位姿经过一个 A<-B 的 SE(2) 变换。这个变换本身有误差，所以它是
【共同参照系】不是绝对真值 —— 但四种策略共用同一个参照系，误差共模会抵消，做 A/B 够用。
默认用 pool 那一臂的解 RANSAC 拟合出来，再拿去给所有臂打分。
"""
import argparse, json, math, os, sys, time
import numpy as np


def se2_apply(T, xy):
    c, s, tx, ty = T
    return np.stack([c * xy[:, 0] - s * xy[:, 1] + tx,
                     s * xy[:, 0] + c * xy[:, 1] + ty], axis=1)


def fit_se2(src, dst, iters=2000, thresh=0.5, seed=7):
    """RANSAC 拟合 dst ≈ R(theta)·src + t。内点最多的那个解。"""
    rng = np.random.default_rng(seed)
    n = len(src)
    best, best_in = None, -1
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        v1, v2 = src[j] - src[i], dst[j] - dst[i]
        if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
            continue
        th = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
        c, s = math.cos(th), math.sin(th)
        t = dst[i] - np.array([c * src[i][0] - s * src[i][1], s * src[i][0] + c * src[i][1]])
        T = (c, s, t[0], t[1])
        nin = int((np.linalg.norm(se2_apply(T, src) - dst, axis=1) < thresh).sum())
        if nin > best_in:
            best, best_in = T, nin
    return best, best_in


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--map-a", required=True, help="参考地图（往里定位）")
    p.add_argument("--map-b", required=True, help="查询地图（拿它的关键帧当 query）")
    p.add_argument("--fusion", default="pool,best_inliers,best_inliers_then_pool,top1")
    p.add_argument("--every-n", type=int, default=5)
    p.add_argument("--max-queries", type=int, default=0)
    p.add_argument("--db-path", default="/tmp/reloc_fusion_scratch")
    p.add_argument("--vlad-centres", default=None, help="不给就用仓库自带的 k256")
    p.add_argument("--transform-json", default=None, help="共用参照系；不给就用第一臂拟合")
    p.add_argument("--save-transform", default=None)
    p.add_argument("--out-json", default=None)
    a = p.parse_args()

    os.makedirs(a.db_path, exist_ok=True)
    import rclpy
    from tinynav.core.map_node import MapNode, DummyEmbeddingEngine
    from tinynav.core.build_map_node import TinyNavDB, LoopClosure, DEFAULT_VLAD_CENTRES, load_vlad_centres
    from tinynav.core.models_trt import SuperPointMatcher
    from tinynav.core.math_utils import se3_inv

    rclpy.init(args=None)
    centres_path = a.vlad_centres or DEFAULT_VLAD_CENTRES
    t0 = time.perf_counter()
    node = MapNode(tinynav_db_path=a.db_path, tinynav_map_path=a.map_a,
                   extractor=None, matcher=SuperPointMatcher(),
                   embedding_extractor=DummyEmbeddingEngine(),
                   loop_closure_mode="vlad", vlad_centres_path=centres_path,
                   verbose_timer=False)
    node.K = node.map_K
    print(f"[info] 参考地图 {a.map_a}: {len(node.map_poses)} 关键帧, 载入 {time.perf_counter()-t0:.1f}s")

    centres = load_vlad_centres(centres_path)
    db_b = TinyNavDB(a.map_b, is_scratch=False)
    poses_b = np.load(os.path.join(a.map_b, "poses.npy"), allow_pickle=True)
    poses_b = poses_b.item() if poses_b.dtype == object and poses_b.shape == () else poses_b
    ts_b = list(poses_b.keys())
    lc_b = LoopClosure(db=db_b, timestamps=ts_b, mode="vlad", vlad_centres=centres)
    if lc_b.embeddings.shape[0] != len(ts_b):
        print(f"ERROR: 查询地图的 VLAD 索引对不上 ({lc_b.embeddings.shape} vs {len(ts_b)})")
        return 2
    print(f"[info] 查询地图 {a.map_b}: {len(ts_b)} 关键帧, VLAD {lc_b.embeddings.shape}")

    idx = list(range(0, len(ts_b), max(1, a.every_n)))
    if a.max_queries:
        idx = idx[: a.max_queries]

    shared_T = None
    if a.transform_json:
        shared_T = tuple(json.load(open(a.transform_json))["se2"])

    results = {}
    for mode in a.fusion.split(","):
        node.reloc_fusion = mode
        solved, truth, nfail = [], [], 0
        t_start = time.perf_counter()
        for i in idx:
            ts = ts_b[i]
            try:
                feats = db_b.get_features(ts)
            except Exception:
                nfail += 1
                continue
            emb = lc_b.embeddings[i]
            node.get_embeddings = lambda _img, e=emb: e
            try:
                ok, pose_cam, _w = node.relocalize_with_depth(None, feats, node.map_K, {})
            except Exception as exc:
                print(f"  ⚠️ {ts}: {type(exc).__name__} {exc}")
                nfail += 1
                continue
            if not ok:
                nfail += 1
                continue
            solved.append(se3_inv(pose_cam)[:2, 3])           # 解出来的相机在 A 里的位置
            truth.append(np.asarray(poses_b[ts])[:2, 3])      # 它在 B 里的位置（待变换）
        el = time.perf_counter() - t_start
        if len(solved) < 10:
            print(f"{mode:24s} 成功 {len(solved)}/{len(idx)} —— 太少，跳过")
            continue
        solved, truth = np.asarray(solved), np.asarray(truth)
        if shared_T is None:
            shared_T, nin = fit_se2(truth, solved)
            print(f"[info] 用 {mode} 拟合共同参照系: 内点 {nin}/{len(solved)}, "
                  f"yaw={math.degrees(math.atan2(shared_T[1], shared_T[0])):+.2f}°")
            if a.save_transform:
                json.dump({"se2": list(shared_T)}, open(a.save_transform, "w"))
        err = np.linalg.norm(se2_apply(shared_T, truth) - solved, axis=1)
        e = np.sort(err)
        # 多个门限一起报：只看 <0.5m 会和 consensus 的 0.5m 判据耦合，看着像"拐点"其实是自证。
        # p50/p90/p95 不含门限，是更干净的判据。分母用【总查询数】而不是成功数 ——
        # 否则拒得越多分数越好，等于奖励不作为。
        N = len(idx)
        results[mode] = dict(n=N, ok=len(solved), fail=nfail,
                             p50=float(e[len(e)//2]), p90=float(e[int(len(e)*.9)]),
                             p95=float(e[int(len(e)*.95)]),
                             **{f"le{int(t*100):02d}": float((err < t).sum()) / N
                                for t in (0.1, 0.2, 0.3, 0.5, 1.0)},
                             ms=1000*el/max(N, 1))
        r = results[mode]
        print(f"{mode:22s} 出解{r['ok']:4d}/{r['n']:3d} p50={r['p50']:.3f} p90={r['p90']:.3f} "
              f"p95={r['p95']:.3f} | 占全部查询: <0.1m {100*r['le10']:.1f}% <0.2m {100*r['le20']:.1f}% "
              f"<0.3m {100*r['le30']:.1f}% <0.5m {100*r['le50']:.1f}% <1m {100*r['le100']:.1f}%")
    if a.out_json:
        json.dump(results, open(a.out_json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
