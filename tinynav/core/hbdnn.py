"""Minimal ctypes binding for Horizon's hbDNN runtime (X5 / bayes-e).

The board ships /usr/lib/libdnn.so and the headers under /usr/include/dnn, but no Python
bindings and no pip to install any. Structures below mirror hb_dnn.h / hb_sys.h exactly.
"""
from __future__ import annotations

import ctypes as C

import numpy as np

_lib = C.CDLL("/usr/lib/libdnn.so")

MAX_DIMS = 8
LAYOUT = {0: "NHWC", 2: "NCHW", 255: "NONE"}
# hbDNNDataType ordinals that carry plain tensors (image types occupy 0..5).
DTYPE = {6: "s4", 7: "u4", 8: np.int8, 9: np.uint8, 10: np.float16, 11: np.int16,
         12: np.uint16, 13: np.float32, 14: np.int32, 15: np.uint32, 16: np.float64,
         17: np.int64, 18: np.uint64}
QUANTI_NONE, QUANTI_SHIFT, QUANTI_SCALE = 0, 1, 2
PRIORITY_LOWEST, PRIORITY_HIGHEST = 0, 255


class hbSysMem(C.Structure):
    _fields_ = [("phyAddr", C.c_uint64), ("virAddr", C.c_void_p), ("memSize", C.c_uint32)]


class hbDNNTensorShape(C.Structure):
    _fields_ = [("dimensionSize", C.c_int32 * MAX_DIMS), ("numDimensions", C.c_int32)]


class hbDNNQuantiShift(C.Structure):
    _fields_ = [("shiftLen", C.c_int32), ("shiftData", C.POINTER(C.c_uint8))]


class hbDNNQuantiScale(C.Structure):
    _fields_ = [("scaleLen", C.c_int32), ("scaleData", C.POINTER(C.c_float)),
                ("zeroPointLen", C.c_int32), ("zeroPointData", C.POINTER(C.c_int8))]


class hbDNNTensorProperties(C.Structure):
    _fields_ = [("validShape", hbDNNTensorShape), ("alignedShape", hbDNNTensorShape),
                ("tensorLayout", C.c_int32), ("tensorType", C.c_int32),
                ("shift", hbDNNQuantiShift), ("scale", hbDNNQuantiScale),
                ("quantiType", C.c_int32), ("quantizeAxis", C.c_int32),
                ("alignedByteSize", C.c_int32), ("stride", C.c_int32 * MAX_DIMS)]


class hbDNNTensor(C.Structure):
    _fields_ = [("sysMem", hbSysMem * 4), ("properties", hbDNNTensorProperties)]


class hbDNNInferCtrlParam(C.Structure):
    _fields_ = [("bpuCoreId", C.c_int32), ("dspCoreId", C.c_int32), ("priority", C.c_int32),
                ("more", C.c_int32), ("customId", C.c_int64),
                ("reserved1", C.c_int32), ("reserved2", C.c_int32)]


def _check(ret, what):
    if ret != 0:
        raise RuntimeError(f"{what} failed: {ret}")


def _shape(s: hbDNNTensorShape):
    return tuple(s.dimensionSize[i] for i in range(s.numDimensions))


class BPUModel:
    """One .bin model, held open with its input/output buffers preallocated."""

    def __init__(self, bin_path: str, priority: int = PRIORITY_LOWEST):
        self.priority = priority
        self._packed = C.c_void_p()
        paths = (C.c_char_p * 1)(bin_path.encode())
        _check(_lib.hbDNNInitializeFromFiles(C.byref(self._packed), paths, 1), "InitializeFromFiles")

        names, count = C.POINTER(C.c_char_p)(), C.c_int32()
        _check(_lib.hbDNNGetModelNameList(C.byref(names), C.byref(count), self._packed),
               "GetModelNameList")
        self.name = names[0]
        self._model = C.c_void_p()
        _check(_lib.hbDNNGetModelHandle(C.byref(self._model), self._packed, self.name),
               "GetModelHandle")

        self.inputs = self._alloc_tensors(is_input=True)
        self.outputs = self._alloc_tensors(is_input=False)

    def _alloc_tensors(self, is_input: bool):
        n = C.c_int32()
        get_count = _lib.hbDNNGetInputCount if is_input else _lib.hbDNNGetOutputCount
        get_props = _lib.hbDNNGetInputTensorProperties if is_input else _lib.hbDNNGetOutputTensorProperties
        get_name = _lib.hbDNNGetInputName if is_input else _lib.hbDNNGetOutputName
        _check(get_count(C.byref(n), self._model), "GetCount")

        arr = (hbDNNTensor * n.value)()
        meta = []
        for i in range(n.value):
            _check(get_props(C.byref(arr[i].properties), self._model, i), "GetTensorProperties")
            nm = C.c_char_p()
            _check(get_name(C.byref(nm), self._model, i), "GetName")
            size = arr[i].properties.alignedByteSize
            _check(_lib.hbSysAllocCachedMem(C.byref(arr[i].sysMem[0]), size), "AllocCachedMem")
            meta.append({"name": nm.value.decode(), "index": i,
                         "valid": _shape(arr[i].properties.validShape),
                         "aligned": _shape(arr[i].properties.alignedShape),
                         "layout": LAYOUT.get(arr[i].properties.tensorLayout, "?"),
                         "dtype": DTYPE.get(arr[i].properties.tensorType, "?"),
                         "quanti": arr[i].properties.quantiType,
                         "bytes": size})
        return {"array": arr, "meta": meta}

    def _np_view(self, t: hbDNNTensor, m: dict):
        dt = np.dtype(m["dtype"]) if not isinstance(m["dtype"], str) else None
        if dt is None:
            raise RuntimeError(f"unsupported tensor type for {m['name']}")
        n = t.sysMem[0].memSize // dt.itemsize
        buf = (C.c_byte * t.sysMem[0].memSize).from_address(t.sysMem[0].virAddr)
        return np.frombuffer(buf, dtype=dt, count=n)

    def _dequant(self, raw: np.ndarray, t: hbDNNTensor, m: dict):
        """Undo the model's output quantization; SCALE is per-channel on quantizeAxis."""
        p = t.properties
        aligned = m["aligned"]
        x = raw[: int(np.prod(aligned))].reshape(aligned).astype(np.float32)
        if p.quantiType == QUANTI_NONE:
            return x
        if p.quantiType == QUANTI_SCALE:
            n = p.scale.scaleLen
            sc = np.ctypeslib.as_array(p.scale.scaleData, shape=(n,)).astype(np.float32)
            axis = p.quantizeAxis if n > 1 else -1
            if n > 1:
                shape = [1] * x.ndim
                shape[axis] = n
                x = x * sc.reshape(shape)
            else:
                x = x * sc[0]
        else:
            n = p.shift.shiftLen
            sh = np.ctypeslib.as_array(p.shift.shiftData, shape=(n,)).astype(np.float32)
            shape = [1] * x.ndim
            shape[p.quantizeAxis if n > 1 else -1] = n
            x = x / (2.0 ** sh.reshape(shape))
        return x

    def infer(self, feed: np.ndarray | list[np.ndarray]):
        """Run one forward pass; returns dequantized outputs cropped to their valid shape."""
        feeds = [feed] if isinstance(feed, np.ndarray) else feed
        for i, (t, m) in enumerate(zip(self.inputs["array"], self.inputs["meta"])):
            v = self._np_view(t, m)
            src = np.ascontiguousarray(feeds[i], dtype=np.dtype(m["dtype"])).reshape(-1)
            v[: src.size] = src
            _check(_lib.hbSysFlushMem(C.byref(t.sysMem[0]), 2), "FlushMem(CLEAN)")  # 2 = CLEAN

        ctrl = hbDNNInferCtrlParam(bpuCoreId=0, dspCoreId=0, priority=self.priority,
                                   more=0, customId=0, reserved1=0, reserved2=0)
        task = C.c_void_p()
        out_ptr = C.cast(self.outputs["array"], C.POINTER(hbDNNTensor))
        _check(_lib.hbDNNInfer(C.byref(task), C.byref(out_ptr),
                               C.cast(self.inputs["array"], C.POINTER(hbDNNTensor)),
                               self._model, C.byref(ctrl)), "Infer")
        _check(_lib.hbDNNWaitTaskDone(task, 0), "WaitTaskDone")
        _lib.hbDNNReleaseTask(task)

        res = []
        for t, m in zip(self.outputs["array"], self.outputs["meta"]):
            _check(_lib.hbSysFlushMem(C.byref(t.sysMem[0]), 1), "FlushMem(INVALIDATE)")  # 1
            x = self._dequant(self._np_view(t, m), t, m)
            valid = m["valid"]
            if x.shape != valid:  # BPU pads the last axis for alignment
                x = x[tuple(slice(0, v) for v in valid)]
            res.append(x)
        return res

    def close(self):
        for group in (self.inputs, self.outputs):
            for t in group["array"]:
                if t.sysMem[0].virAddr:
                    _lib.hbSysFreeMem(C.byref(t.sysMem[0]))
                    t.sysMem[0].virAddr = None
        if self._packed:
            _lib.hbDNNRelease(self._packed)
            self._packed = C.c_void_p()
