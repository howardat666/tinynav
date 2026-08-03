# DBoW3 bindings for the CPU-only retrieval path

The plan-1 relocalization stack (ORB features + DBoW3 retrieval + brute-force
Hamming matching) needs no GPU and no neural network, which is what makes it
viable on the X5 inside the Looper camera. Its only awkward dependency is a
Python binding for DBoW3.

## Why not upstream pyDBoW3

[`foxis/pyDBoW3`](https://github.com/foxis/pyDBoW3) does not build against
OpenCV 4.x. Its vendored numpy↔`cv::Mat` converter declares a `NumpyAllocator`
deriving from `cv::MatAllocator`, and OpenCV 4 added pure-virtual overloads to
that base class, so the type is abstract:

```
error: cannot declare variable 'g_numpyAllocator' to be of abstract type 'NumpyAllocator'
```

Its CMakeLists also asks for the Boost component `python-py310`, while Ubuntu
22.04 ships it as `python310`.

`pydbow3_shim.cpp` sidesteps both: ~70 lines of pybind11 that accept dense
`(N, 32)` uint8 ORB descriptors directly, with no general-purpose Mat
converter. The exposed API is source-compatible with the subset of pyDBoW3 that
`tinynav/core/models_trt.py::DBoW3Engine` uses, plus `Vocabulary(k, levels)` and
`Vocabulary.create()` for training a map-specific vocabulary.

Nothing here is coupled to an OpenCV version, so the same build works on
aarch64.

## Build

```bash
docker build -t tinynav-x5:dbow3 -f Dockerfile .
```

## Train a map-specific vocabulary

The pretrained ORBvoc (~1M words) costs ~431 MB resident, which does not fit
next to the camera firmware on the X5. A vocabulary trained on the target map is
both far smaller and more accurate — see `docs/x5/x5.md` § 10.2.

```python
import pydbow3
voc = pydbow3.Vocabulary(10, 5)      # k=10, L=5 -> ~100k words, ~38 MB resident
voc.create(descriptors_per_keyframe) # list of (N, 32) uint8 arrays
voc.save("voc_office.dbow3")
```

Then, at load time, drop the Python-side vocabulary once the database has taken
its copy — that alone saves ~154 MB with the stock ORBvoc:

```python
voc = pydbow3.Vocabulary(); voc.load("voc_office.dbow3")
db = pydbow3.Database(); db.setVocabulary(voc)
del voc
```
