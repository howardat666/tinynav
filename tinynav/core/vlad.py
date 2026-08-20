"""DINOv2 Patch VLAD (Vector of Locally Aggregated Descriptors).

Replaces the CLS-token cosine-similarity retrieval in tinynav with a patch-level
place-recognition descriptor inspired by AnyLoc.

Pipeline:
  1. Extract DINOv2 patch tokens (N_patch, C) per image — already computed by TRT.
  2. Train a K-means vocabulary on all patch tokens from the map.
  3. For each image, compute a VLAD descriptor by assigning patches to clusters
     and aggregating residuals.
  4. Use the shared descriptor retrieval helper to rank top-k candidates.

All routines are pure numpy so they run on x64, Jetson aarch64 and the X5 board, whose
Python ships no scipy.
"""

from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)


def _l2_normalize_rows(x: np.ndarray) -> np.ndarray:
    """L2-normalise each row of ``x``, guarding against near-zero norms."""
    norms = np.linalg.norm(x, axis=1, keepdims=True)
    return x / np.maximum(norms, 1e-8)


def _nearest_centre(tokens: np.ndarray, centres: np.ndarray) -> np.ndarray:
    """Index of the nearest centre per row, via ||t-c||^2 = ||t||^2 - 2t.c + ||c||^2.

    The ||t||^2 term is constant within a row and dropped. Measured 4-6x faster than the
    cKDTree this replaces, with bit-identical labels: these are small dense problems.
    """
    return np.argmin(np.square(centres).sum(axis=1) - 2.0 * (tokens @ centres.T), axis=1)


def train_vocabulary_streaming(
    batch_iterator_factory,
    vocab_size: int = 32,
    epochs: int = 5,
    batch_size: int = 1024,
    seed: int = 42,
) -> np.ndarray:
    """Train a K-means vocabulary via disk-streamed online updates.

    Never materialises the full patch-token pool in memory: ``batch_iterator_factory()``
    is called once per epoch and must return a fresh iterator over per-keyframe
    (N_i, C) patch-token arrays (e.g. reading sequentially off a disk-backed store, in
    a freshly shuffled keyframe order each call). Arrays are concatenated into
    ``batch_size`` chunks on the fly; each chunk is assigned to its nearest centre
    (against a frozen snapshot of the centres) and then folded in point-by-point via a
    decaying running mean (Robbins-Monro / sequential k-means update), the same update
    rule as classic online k-means and as
    https://gist.github.com/yjzhang/aaf460849a4398422785c0e85932688d — this
    implementation batches the assignment step for throughput instead of updating one
    point at a time.

    Args:
        batch_iterator_factory: zero-arg callable returning a fresh iterator of
            (N_i, C) patch-token arrays; called once per epoch.
        vocab_size: number of cluster centres (K).
        epochs: number of streamed passes over the data.
        batch_size: size of each online-update chunk.
        seed: RNG seed for reproducibility.

    Returns:
        centres: (vocab_size, C) L2-normalised cluster centres.
    """
    rng = np.random.default_rng(seed)
    centres = None
    counts = None

    def normalise(tokens: np.ndarray) -> np.ndarray:
        return _l2_normalize_rows(tokens.astype(np.float32, copy=False))

    def apply_batch(batch: np.ndarray) -> None:
        nonlocal centres, counts
        if centres is None:
            if len(batch) < vocab_size:
                raise ValueError(f"Need at least {vocab_size} tokens in the first batch, got {len(batch)}")
            centres = batch[rng.choice(len(batch), size=vocab_size, replace=False)].copy()
            counts = np.zeros(vocab_size, dtype=np.int64)
        labels = _nearest_centre(batch, centres)
        for label, vec in zip(labels, batch):
            counts[label] += 1
            eta = 1.0 / counts[label]
            centres[label] = (1.0 - eta) * centres[label] + eta * vec

    for epoch in range(epochs):
        pending: list[np.ndarray] = []
        pending_count = 0
        for frame_tokens in batch_iterator_factory():
            pending.append(normalise(frame_tokens))
            pending_count += len(frame_tokens)
            while pending_count >= batch_size:
                buf = np.concatenate(pending, axis=0)
                batch, rest = buf[:batch_size], buf[batch_size:]
                pending = [rest] if len(rest) else []
                pending_count = len(rest)
                apply_batch(batch)
        if pending_count > 0:
            apply_batch(np.concatenate(pending, axis=0))
        logger.info(f"VLAD streaming online k-means epoch {epoch + 1}/{epochs}")

    return _l2_normalize_rows(centres).astype(np.float32)


def compute_vlad(patch_tokens: np.ndarray, centres: np.ndarray) -> np.ndarray:
    """Compute a single-image VLAD descriptor.

    Args:
        patch_tokens: (N, C) patch tokens for one image (L2-normalised).
        centres: (K, C) vocabulary centres (L2-normalised).

    Returns:
        descriptor: (K * C,) L2-normalised VLAD descriptor.
    """
    K, C = centres.shape
    if len(patch_tokens) == 0:
        return np.zeros(K * C, dtype=np.float32)

    tokens = _l2_normalize_rows(patch_tokens.astype(np.float32, copy=False))

    labels = _nearest_centre(tokens, centres)

    # sum(tokens in cluster k) - count_k * centre_k, grouped by sorting instead of K mask
    # passes: ~4x faster at K=256 and the gap widens with K.
    order = np.argsort(labels, kind="stable")
    used, starts = np.unique(labels[order], return_index=True)
    residuals = np.zeros_like(centres, dtype=np.float32)
    residuals[used] = np.add.reduceat(tokens[order], starts, axis=0)
    residuals -= np.bincount(labels, minlength=K).astype(np.float32)[:, None] * centres

    # Intra-normalisation.
    residuals = _l2_normalize_rows(residuals)

    descriptor = residuals.reshape(-1)
    desc_norm = np.linalg.norm(descriptor)
    if desc_norm > 1e-8:
        descriptor /= desc_norm
    return descriptor.astype(np.float32)


def compute_vlad_batch(
    patch_tokens_list: list[np.ndarray],
    centres: np.ndarray,
) -> np.ndarray:
    """Compute VLAD descriptors for a list of images.

    Args:
        patch_tokens_list: list of (N_i, C) patch token arrays.
        centres: (K, C) vocabulary centres.

    Returns:
        descriptors: (len(list), K * C) L2-normalised VLAD descriptors.
    """
    K, C = centres.shape
    descriptors = np.zeros((len(patch_tokens_list), K * C), dtype=np.float32)
    for i, tokens in enumerate(patch_tokens_list):
        descriptors[i] = compute_vlad(tokens, centres)
        if (i + 1) % 100 == 0:
            logger.info(f"VLAD encoded {i + 1}/{len(patch_tokens_list)}")
    return descriptors
