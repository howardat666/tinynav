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

`pydbow3_shim.cpp` sidesteps both: ~120 lines of pybind11 that accept dense
descriptor blocks directly, with no general-purpose Mat converter. The exposed
API is source-compatible with the subset of pyDBoW3 that
`tinynav/core/models_trt.py::DBoW3Engine` uses, plus `Vocabulary(k, levels)` and
`Vocabulary.create()` for training a map-specific vocabulary.

Nothing here is coupled to an OpenCV version, so the same build works on
aarch64.

## Descriptor dtypes

Dispatch is on the numpy dtype, matching the two branches DBoW3's `DescManip`
already implements:

| input | cv type | distance | typical producer |
| --- | --- | --- | --- |
| `(N, 32)` `uint8` | `CV_8U` | Hamming | ORB |
| `(N, 256)` `float32` | `CV_32F` | L2 | SuperPoint |

Any other dtype raises. Note there is deliberately **no** `py::array::forcecast`:
with it, a float32 array is silently truncated to uint8 (SuperPoint descriptors
in `[0, 1)` all become zeros) and DBoW3 trains on the garbage without ever
raising.

## Build

x86_64 (native):

```bash
docker build -t tinynav-x5:dbow3 -f Dockerfile .
```

aarch64 for the X5 inside the Looper camera (qemu-aarch64 binfmt, no buildx
needed) — see `Dockerfile.aarch64` and `build_aarch64.sh`; artifacts and the
deployment notes land in `/home/dm/looper/x5_aarch64/`:

```bash
DOCKER_BUILDKIT=0 docker build --platform linux/arm64 \
    -t tinynav-x5:dbow3-arm64 -f Dockerfile.aarch64 .
./build_aarch64.sh
```

## Self-check

`test_pydbow3.py` trains a tiny vocabulary, indexes it and asserts a query
returns itself as top-1, for both dtypes. The extension is baked into the
image, so rebuild it after touching the shim:

```bash
docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp \
    -v "$PWD:/shim:ro" --entrypoint /opt/venv/bin/python3 \
    tinynav-x5:dbow3 /shim/test_pydbow3.py
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

## Do not persist the Database (measured)

`Database.save()` / `Database.load()` are bound (the saved file embeds the
vocabulary, so `load()` needs no prior `setVocabulary()`), but **they are useless
as a startup optimisation and must not be put on the board's boot path.**

Measured on this PC (x86_64) over `maps/map_gt` — 1123 keyframes, 1.03 M ORB
descriptors, 0.29 s to read them all out of the feature shelf. "rebuild" is the
status quo: binary `voc.load()` plus 1123 × `db.add()`.

| vocabulary | words | rebuild | save | file | load | load / rebuild |
| --- | --- | --- | --- | --- | --- | --- |
| `voc_office_k10L4` | 10 000 | 1.54 s | 0.79 s | 52.6 MB | **6.4 s** | 4× |
| `voc_office_k10L5` | 99 023 | 1.54 s | 0.96 s | 84.6 MB | **595.5 s** | **387×** |
| `voc_office_k10L5` (`.yml.gz`) | 99 023 | 1.66 s | 7.27 s | 20.5 MB | **851.1 s** | **512×** |
| `ORBvoc` | 971 814 | 5.29 s | 5.01 s | 352.5 MB | not attempted, ~16 h | ~10 000× |

Every round trip is exact — identical top-10 ids and `max|ΔScore| = 0` — so this
is a performance verdict, not a correctness one. Gzipping shrinks the file 4×
and makes loading *slower*: the cost is parsing, not I/O.

The cause is not "YAML is slow", it is a complexity bug in DBoW3. Both loops in
the load path index a FileStorage **sequence** by integer, and
`cv::FileNode::operator[](int)` is a linear walk, so both are **O(n²)**:
`Vocabulary::load(fs)` does `fn[i]["nodeId"]` per node, and `Database::load`
does `fn[wid]` per word. A 9.9× larger vocabulary cost 93× the load time
(9.9² = 98) while the file only grew 1.6×.

Load time is therefore set by the *vocabulary*, not by the map. With `k10L5`
fixed, cutting the keyframe count changes nothing:

| entries | rebuild | file | load |
| --- | --- | --- | --- |
| 100 | 0.25 s | 35.0 MB | 691.6 s |
| 300 | 0.48 s | 44.3 MB | 786.9 s |
| 1123 | 1.48 s | 84.6 MB | 595.5 s |

Note that a standalone `.dbow3` file is DBoW3's own **binary** format, which
`Vocabulary::load(const std::string&)` sniffs and reads in 0.06 s.
`Database::save` throws that away and re-encodes the same vocabulary as YAML, so
most of the 595 s is spent re-parsing something we already had a fast reader for.
Any real fix has to bypass `Database::save`/`load` and serialise `m_ifile`
directly.

For reference, rebuilding costs ~16 s of a 34 s `MapNode.__init__` on the X5,
which is ~9× slower than this PC. The PC load is already 387× the PC rebuild,
and a PC number is a *lower bound* on the board: extrapolating gives ~90 min to
load what the board rebuilds in 16 s. `.yml` at 84.6 MB also has to be parsed
into an OpenCV node tree on a board with 908 MB available and no swap.

```python
voc = pydbow3.Vocabulary(); voc.load("voc_office.dbow3")
db = pydbow3.Database(); db.setVocabulary(voc)
del voc
```
