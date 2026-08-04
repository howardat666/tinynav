"""A PyAV-backed stand-in for the parts of `decord` that tinynav uses.

WHY
---
`decord` publishes no aarch64 wheel (neither `decord` nor `eva-decord`), and no
sdist either, so there is nothing to install on the X5 and building it would
mean bringing a full ffmpeg + CMake toolchain onto the board.  Meanwhile
`tool/video_db.py` imports it at module scope and `TinyNavDB.__init__`
constructs `VideoDB` unconditionally, so *opening any map at all* fails on the
board with `TypeError: 'NoneType' object is not callable`.

`decord`'s surface used by tinynav is two operations -- `len(reader)` and
`reader[i].asnumpy()` returning an RGB frame -- and PyAV, which *does* ship an
aarch64 wheel, can serve both.  So this is a working replacement, not a stub
that raises: relocalization does not read map imagery (the loaders in
`get_depth_embedding_features_images` are lazy closures), but tooling and
visualisation do, and those keep working.

Put this on PYTHONPATH ahead of site-packages on the board only.  On any machine
where the real decord is installed, do not use it -- decord's own CPU/GPU
decoding is considerably faster.

DIFFERENCES FROM REAL DECORD
----------------------------
* Only `VideoReader` is provided, with `__len__`, `__getitem__` (single integer
  index) and `get_batch`.  `bridge`, GPU contexts, audio and `VideoLoader` are
  absent -- an attribute error here means something needs more than this shim.
* Random access seeks to the preceding keyframe and decodes forward, so a
  backwards jump costs a seek.  Sequential access is fast; the one-frame cache
  makes the common "read frame i, then i+1" pattern cheap.
* Frame count comes from the container metadata when available and otherwise
  from a full decode pass, which is slow but happens at most once.
"""

from __future__ import annotations

import contextlib

import numpy as np

try:
    import av
except ImportError as exc:  # pragma: no cover - the board always has PyAV
    raise ImportError(
        "the decord shim needs PyAV: pip install av (an aarch64 wheel exists)"
    ) from exc

__all__ = ["VideoReader", "_Frame"]
__version__ = "0.0.0+pyav-shim"


class _Frame:
    """Mimics decord's frame handle: the caller only ever calls .asnumpy()."""

    __slots__ = ("_rgb",)

    def __init__(self, rgb: np.ndarray):
        self._rgb = rgb

    def asnumpy(self) -> np.ndarray:
        return self._rgb


class VideoReader:
    """Index-addressable video frames, decord-compatible for tinynav's use."""

    def __init__(self, uri, ctx=None, width=-1, height=-1, num_threads=0, **kwargs):
        # ctx/width/height/num_threads are accepted so existing call sites keep
        # working; resizing is not implemented because tinynav never asks for it.
        if width not in (-1, 0, None) or height not in (-1, 0, None):
            raise NotImplementedError("the decord shim does not resize frames")
        self._path = uri
        self._container = av.open(str(uri))
        self._stream = self._container.streams.video[0]
        # Let PyAV pick its own thread count; on 8 slow A55 cores frame-level
        # threading is what keeps decode off the critical path.
        self._stream.thread_type = "AUTO"
        if num_threads:
            self._stream.thread_count = int(num_threads)

        self._len = None
        self._next_index = 0      # index the sequential decoder is about to yield
        self._iter = None
        self._cache_index = None  # one-frame cache, for repeat reads of one index
        self._cache_frame = None

    # ------------------------------------------------------------------ length
    def __len__(self) -> int:
        if self._len is None:
            n = self._stream.frames
            if not n:
                duration = self._stream.duration
                rate = self._stream.average_rate
                if duration and rate and self._stream.time_base:
                    n = round(float(duration * self._stream.time_base * rate))
            if not n:
                n = self._count_by_decoding()
            self._len = int(n)
        return self._len

    def _count_by_decoding(self) -> int:
        """Last resort for containers with no frame count in the metadata."""
        count = 0
        self._container.seek(0)
        for _ in self._container.decode(video=0):
            count += 1
        self._reset_decoder()
        return count

    # ------------------------------------------------------------------ access
    def __getitem__(self, index):
        if isinstance(index, slice):
            return self.get_batch(range(*index.indices(len(self))))
        index = int(index)
        if index < 0:
            index += len(self)
        if index < 0 or index >= len(self):
            raise IndexError(f"frame index {index} out of range ({len(self)} frames)")
        if index == self._cache_index:
            return _Frame(self._cache_frame)
        rgb = self._decode_at(index)
        self._cache_index, self._cache_frame = index, rgb
        return _Frame(rgb)

    def get_batch(self, indices):
        frames = [self[int(i)].asnumpy() for i in indices]
        return _Frame(np.stack(frames)) if frames else _Frame(np.empty((0,)))

    def _reset_decoder(self):
        self._iter = self._container.decode(video=0)
        self._next_index = 0

    def _decode_at(self, index: int) -> np.ndarray:
        # Decoding forward from where we already are is much cheaper than
        # seeking, so only seek when the target is behind us or far ahead.
        SEEK_THRESHOLD = 32
        if self._iter is None or index < self._next_index or index - self._next_index > SEEK_THRESHOLD:
            landed = self._seek_to(index)
            # The seek had to decode one frame to learn where it landed; that
            # frame may already be the one asked for.
            if landed == index and self._cache_frame is not None:
                return self._cache_frame

        for frame in self._iter:
            current = self._next_index
            self._next_index += 1
            if current == index:
                return frame.to_ndarray(format="rgb24")
            if current > index:
                break
        raise IndexError(f"ran out of frames before reaching index {index}")

    def _seek_to(self, index: int):
        """Seek to the keyframe at or before `index`.

        Returns the index the decoder actually landed on, whose frame is left in
        the one-frame cache -- a keyframe seek lands on an arbitrary earlier
        frame, and the only way to learn which is to decode one and read its
        presentation timestamp.
        """
        rate = self._stream.average_rate
        time_base = self._stream.time_base
        if not rate or not time_base:
            self._reset_decoder()
            return -1
        target_pts = int(index / float(rate) / float(time_base))
        self._container.seek(target_pts, stream=self._stream, any_frame=False, backward=True)
        self._iter = self._container.decode(video=0)
        try:
            first = next(self._iter)
        except StopIteration:
            self._container.seek(0)
            self._reset_decoder()
            return -1

        pts = first.pts if first.pts is not None else 0
        landed = round(float(pts * time_base * rate))
        if landed > index:
            # Overshot (non-monotonic pts, or a container whose timestamps do
            # not map cleanly onto frame indices): start over from the top.
            self._container.seek(0)
            self._reset_decoder()
            return -1

        self._cache_index = landed
        self._cache_frame = first.to_ndarray(format="rgb24")
        self._next_index = landed + 1
        return landed

    # ------------------------------------------------------------------- misc
    def get_avg_fps(self) -> float:
        return float(self._stream.average_rate) if self._stream.average_rate else 0.0

    def close(self):
        # Closing must never raise: this also runs from __del__ during
        # interpreter shutdown, where PyAV internals may already be torn down.
        with contextlib.suppress(Exception):
            self._container.close()

    def __del__(self):
        self.close()
