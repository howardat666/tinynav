#!/usr/bin/env python3
"""Self-check for the pydbow3 shim: train a tiny vocabulary, index it, and make
sure a query returns itself as top-1.

Run inside the aarch64 qemu container (never on the board, which has no
compiler and no OpenCV):

    docker run --rm --platform linux/arm64 --user $(id -u):$(id -g) -e HOME=/tmp \
        -v /home/dm/looper/x5_aarch64:/out:ro \
        -v /home/dm/looper/tinynav-x5/tool/x5_dbow3:/shim:ro \
        -e PYTHONPATH=/out -e LD_LIBRARY_PATH=/out/lib \
        tinynav-x5:dbow3-arm64 python3 /shim/test_pydbow3.py
"""
import os
import sys
import time

import numpy as np

import pydbow3

rng = np.random.default_rng(0)

# Vocabulary training is k-means, which is ~50x slower under qemu-aarch64 than
# native; shrink the float32 (256-D) case there via the environment.
N_IMG = int(os.environ.get("PYDBOW3_TEST_IMAGES", "50"))
N_DESC = int(os.environ.get("PYDBOW3_TEST_DESCRIPTORS", "500"))
N_IMG_F32 = int(os.environ.get("PYDBOW3_TEST_IMAGES_F32", str(N_IMG)))
N_DESC_F32 = int(os.environ.get("PYDBOW3_TEST_DESCRIPTORS_F32", str(N_DESC)))


def roundtrip(feats, label):
    t0 = time.time()
    voc = pydbow3.Vocabulary(10, 3)
    voc.create(feats)
    db = pydbow3.Database()
    db.setVocabulary(voc)
    for f in feats:
        db.add(f)
    r = db.query(feats[0], 5)
    assert len(r) > 0, f"{label}: empty query result"
    assert r[0].Id == 0, f"{label}: top-1 is {r[0].Id}, expected 0"
    print(f"{label}: {feats[0].shape[0]}x{feats[0].shape[1]} {feats[0].dtype} "
          f"x{len(feats)} -> voc.size={voc.size()} db.size={db.size()} "
          f"results={len(r)} top1=(Id={r[0].Id}, Score={r[0].Score:.4f}) "
          f"[{time.time() - t0:.1f}s]")


print("python:", sys.version.split()[0])
print("numpy:", np.__version__)
print("pydbow3:", pydbow3.__file__)

# ORB: (N, 32) uint8, Hamming / CV_8U branch of DBoW3's DescManip.
roundtrip([rng.integers(0, 256, (N_DESC, 32), dtype=np.uint8)
           for _ in range(N_IMG)], "uint8 ORB")

# SuperPoint: (N, 256) float32, L2 / CV_32F branch.
roundtrip([rng.random((N_DESC_F32, 256), dtype=np.float32)
           for _ in range(N_IMG_F32)], "float32 SuperPoint")

# Fortran-ordered input must still work (ensure() copies it).
roundtrip([np.asfortranarray(rng.integers(0, 256, (64, 32), dtype=np.uint8))
           for _ in range(12)], "uint8 non-contiguous")

# Mixing dtypes in one training batch must be refused, not silently asserted on.
try:
    pydbow3.Vocabulary(4, 2).create(
        [np.zeros((16, 32), np.uint8), np.zeros((16, 32), np.float32)])
except Exception as exc:  # noqa: BLE001
    print("mixed dtype batch rejected as expected ->", type(exc).__name__)
else:
    print("WARNING: mixed dtype training batch was accepted")

# A dtype the shim cannot map must raise, not silently cast.
try:
    pydbow3.Database().add(np.zeros((10, 32), dtype=np.float64))
except Exception as exc:  # noqa: BLE001
    print("float64 rejected as expected ->", type(exc).__name__)
else:
    print("WARNING: float64 was silently accepted (forcecast still in place?)")

print("OK")
