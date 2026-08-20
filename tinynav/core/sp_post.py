"""Pure-numpy SuperPoint post-processing, to run on CPU behind a BPU backbone.

Two NMS implementations: `exact` mirrors the ONNX graph node for node and exists to prove
the rest of the pipeline is right; `fast` is what the board would actually run, since the
exact one needs five 9x9 max-pools over the full 640x544 grid.
"""
from __future__ import annotations

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

CELL = 8
NMS_RADIUS = 4
TOP_K = 512


def _maxpool_same(x: np.ndarray, k: int) -> np.ndarray:
    """kxk max-pool, stride 1, 'same' padding -- separated into two 1-D passes."""
    r = k // 2
    p = np.pad(x, r, mode="constant", constant_values=-np.inf)
    m = sliding_window_view(p, k, axis=1).max(axis=-1)
    return sliding_window_view(m, k, axis=0).max(axis=-1)


def heatmap_from_logits(logits: np.ndarray) -> np.ndarray:
    """[65, hc, wc] -> [hc*8, wc*8] keypoint probability."""
    e = np.exp(logits - logits.max(axis=0, keepdims=True))
    dense = e / e.sum(axis=0, keepdims=True)
    nodust = dense[:-1]  # last channel is the "no keypoint here" dustbin
    _, hc, wc = nodust.shape
    return nodust.reshape(CELL, CELL, hc, wc).transpose(2, 0, 3, 1).reshape(hc * CELL, wc * CELL)


def nms_exact(scores: np.ndarray, radius: int = NMS_RADIUS) -> np.ndarray:
    """SuperPoint's simple_nms: two rounds that let a suppressed point's neighbour recover."""
    k = radius * 2 + 1
    zeros = np.zeros_like(scores)
    max_mask = scores == _maxpool_same(scores, k)
    for _ in range(2):
        supp_mask = _maxpool_same(max_mask.astype(np.float32), k) > 0
        supp_scores = np.where(supp_mask, zeros, scores)
        new_max = supp_scores == _maxpool_same(supp_scores, k)
        max_mask = max_mask | (new_max & ~supp_mask)
    return np.where(max_mask, scores, zeros)


def nms_fast(scores: np.ndarray, radius: int = NMS_RADIUS, threshold: float = 0.0,
             limit: int = TOP_K) -> tuple[np.ndarray, np.ndarray]:
    """Greedy NMS over thresholded candidates only -- returns (xy, score), score-sorted.

    Avoids the five full-grid max-pools by working on the few thousand points above
    threshold, which is the whole reason this is viable on the board.
    """
    flat = scores.reshape(-1)
    hits = np.nonzero(flat > threshold)[0]
    if len(hits) == 0:
        return np.zeros((0, 2), np.float32), np.zeros(0, np.float32)
    # No score-based pre-truncation: a low-scoring but spatially isolated point still
    # survives NMS into the top-`limit` list, so capping the pool silently loses keypoints.
    vals = flat[hits]
    order = np.argsort(-vals, kind="stable")
    hits, vals = hits[order], vals[order]
    w = scores.shape[1]
    ys, xs = hits // w, hits % w

    h, w = scores.shape
    taken = np.zeros((h, w), dtype=bool)
    keep = []
    for i in range(len(ys)):
        y, x = ys[i], xs[i]
        if taken[y, x]:
            continue
        keep.append(i)
        taken[max(0, y - radius):y + radius + 1, max(0, x - radius):x + radius + 1] = True
        if len(keep) >= limit:
            break
    keep = np.asarray(keep, dtype=np.int64)
    return np.stack([xs[keep], ys[keep]], axis=1).astype(np.float32), vals[keep]


def top_keypoints(nms_scores: np.ndarray, top_k: int = TOP_K) -> tuple[np.ndarray, np.ndarray]:
    """Flat top-k over an NMS'd heatmap, as (xy, score)."""
    flat = nms_scores.reshape(-1)
    k = min(top_k, flat.size)
    idx = np.argpartition(-flat, k - 1)[:k]
    idx = idx[np.argsort(-flat[idx], kind="stable")]
    w = nms_scores.shape[1]
    return np.stack([idx % w, idx // w], axis=1).astype(np.float32), flat[idx]


def _unit_rows(x: np.ndarray) -> np.ndarray:
    return x / np.maximum(np.linalg.norm(x, axis=1, keepdims=True), 1e-12)


def sample_descriptors(desc_map: np.ndarray, kpts: np.ndarray, layout: str = "chw") -> np.ndarray:
    """Bilinear sample at keypoints (x, y), matching grid_sample(align_corners=True).

    Pass layout="hwc" for the BPU's native NHWC output to skip the transpose entirely.
    Only the 4 gathered corners per keypoint get L2-normalized (the ONNX graph's ReduceL2
    over the whole desc_map is wasted work when 512 points touch <40% of the cells).
    """
    if layout == "hwc":
        gh, gw, c = desc_map.shape
        flat = desc_map.reshape(gh * gw, c)
    else:
        c, gh, gw = desc_map.shape
        flat = np.ascontiguousarray(desc_map.reshape(c, gh * gw).T)
    if len(kpts) == 0:
        return np.zeros((0, c), np.float32)

    gx = (kpts[:, 0] - CELL / 2 + 0.5) / (gw * CELL - CELL / 2 - 0.5) * (gw - 1)
    gy = (kpts[:, 1] - CELL / 2 + 0.5) / (gh * CELL - CELL / 2 - 0.5) * (gh - 1)

    x0 = np.clip(np.floor(gx).astype(np.intp), 0, gw - 1)
    y0 = np.clip(np.floor(gy).astype(np.intp), 0, gh - 1)
    x1 = np.clip(x0 + 1, 0, gw - 1)
    y1 = np.clip(y0 + 1, 0, gh - 1)
    wx, wy = (gx - x0)[:, None], (gy - y0)[:, None]

    row, row1 = y0 * gw, y1 * gw
    d = (_unit_rows(flat[row + x0]) * ((1 - wx) * (1 - wy))
         + _unit_rows(flat[row + x1]) * (wx * (1 - wy))
         + _unit_rows(flat[row1 + x0]) * ((1 - wx) * wy)
         + _unit_rows(flat[row1 + x1]) * (wx * wy))
    return _unit_rows(d).astype(np.float32)


def postprocess(logits: np.ndarray, desc_map: np.ndarray, threshold: float = 5e-4,
                top_k: int = TOP_K, mode: str = "fast"):
    """-> (kpts [N,2] xy, scores [N], descriptors [N,256]), score-sorted."""
    prob = heatmap_from_logits(logits)
    if mode == "exact":
        kpts, scores = top_keypoints(nms_exact(prob), top_k)
    else:
        kpts, scores = nms_fast(prob, threshold=threshold, limit=top_k)
    keep = scores > threshold
    kpts, scores = kpts[keep], scores[keep]
    return kpts, scores, sample_descriptors(desc_map, kpts)
