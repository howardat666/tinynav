try:
    import tensorrt as trt
except ImportError:
    trt = None
import numpy as np
import cv2
from codetiming import Timer
import platform
import asyncio
from tinynav.core.func import alru_cache_numpy

try:
    from cuda import cudart
except ImportError:
    cudart = None
import ctypes
import einops
import logging
import os
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

numpy_to_ctypes = {
    np.dtype(np.float32): ctypes.c_float,
    np.dtype(np.float16): ctypes.c_uint16,
    np.dtype(np.int8):   ctypes.c_int8,
    np.dtype(np.uint8):  ctypes.c_uint8,
    np.dtype(np.int32):  ctypes.c_int32,
    np.dtype(np.int64):  ctypes.c_int64,
    np.dtype(np.bool_):  ctypes.c_bool
}


def disparity_to_depth(disparity: np.ndarray, baseline: float, focal_length: float) -> np.ndarray:
    disparity = np.asarray(disparity, dtype=np.float32)
    baseline = float(np.asarray(baseline).reshape(-1)[0])
    focal_length = float(np.asarray(focal_length).reshape(-1)[0])

    if baseline <= 0.0:
        raise ValueError(f"baseline must be positive, got {baseline}")
    if focal_length <= 0.0:
        raise ValueError(f"focal_length must be positive, got {focal_length}")

    depth = np.zeros_like(disparity, dtype=np.float32)
    valid = np.isfinite(disparity) & (disparity > 0.0)
    depth[valid] = (baseline * focal_length) / disparity[valid]
    return depth

class TRTBase:
    def __init__(self, engine_path):
        if trt is None or cudart is None:
            raise ImportError(
                "tensorrt and cuda-python are required for TRT models. "
                "Use BoW mode or install TensorRT Python bindings."
            )
        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers()
        with Timer(name="[capture_graph]", text="[{name}] Elapsed time: {milliseconds:.0f} ms"):
            self.graph_exec = self.capture_graph()
        logging.info(f"load {engine_path} done!")

    def _get_static_shape(self, name):
        """Return a concrete shape for a tensor, resolving dynamic dims via the profile if needed."""
        shape = tuple(self.context.get_tensor_shape(name))
        if -1 not in shape:
            return shape

        # Resolve from optimization profile (profile 0) when available.
        try:
            _, _, max_shape = self.engine.get_tensor_profile_shape(name, 0)
            return tuple(int(d) for d in max_shape)
        except Exception:
            # Fallback: replace dynamic dims with 1 to avoid crashes.
            return tuple(d if d != -1 else 1 for d in shape)

    def allocate_buffers(self):
        inputs = []
        outputs = []
        bindings = []
        _, stream = cudart.cudaStreamCreate()

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = self._get_static_shape(name)
            dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            ctype_dtype = numpy_to_ctypes[dtype]
            is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT

            size = trt.volume(shape)
            nbytes = trt.volume(shape) * dtype.itemsize

            if "aarch64" in platform.machine():
                ptr = cudart.cudaHostAlloc(nbytes, cudart.cudaHostAllocMapped)[1]
                host_mem = np.ctypeslib.as_array((ctype_dtype * size).from_address(ptr))
                host_mem = host_mem.view(dtype).reshape(shape)
                device_ptr = cudart.cudaHostGetDevicePointer(ptr, 0)[1]
            else:
                ptr = cudart.cudaMallocHost(nbytes)[1]
                host_mem = np.ctypeslib.as_array((ctype_dtype * size).from_address(ptr))
                host_mem = host_mem.view(dtype).reshape(shape)
                device_ptr = cudart.cudaMalloc(nbytes)[1]

            bindings.append(int(device_ptr))

            if is_input:
                inputs.append({"host": host_mem, "device": device_ptr, "shape": shape, "nbytes": nbytes})
            else:
                outputs.append({"host": host_mem, "device": device_ptr, "name": name, "nbytes": nbytes})

        return inputs, outputs, bindings, stream


    def capture_graph(self):
        # Ensure dynamic input shapes are specified before first execution.
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                shape = self._get_static_shape(name)
                self.context.set_input_shape(name, shape)

        cudart.cudaStreamBeginCapture(self.stream, cudart.cudaStreamCaptureMode.cudaStreamCaptureModeGlobal)

        for i in range(self.engine.num_io_tensors):
            self.context.set_tensor_address(self.engine.get_tensor_name(i), self.bindings[i])
        self.context.execute_async_v3(stream_handle=self.stream)

        _, graph = cudart.cudaStreamEndCapture(self.stream)
        _, graph_exec = cudart.cudaGraphInstantiate(graph, 0)
        cudart.cudaStreamSynchronize(self.stream)
        return graph_exec

    async def run_graph(self):
        if "aarch64" not in platform.machine():
            for inp in self.inputs:
                cudart.cudaMemcpyAsync(inp["device"], inp["host"].ctypes.data,
                                   inp["nbytes"],
                                   cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
                                   self.stream)

        cudart.cudaGraphLaunch(self.graph_exec, self.stream)

        if "aarch64" not in platform.machine():
            for out in self.outputs:
                cudart.cudaMemcpyAsync(out['host'].ctypes.data, out['device'], out['nbytes'], cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream)

        _, event = cudart.cudaEventCreate()
        cudart.cudaEventRecord(event, self.stream)
        while cudart.cudaEventQuery(event)[0] == cudart.cudaError_t.cudaErrorNotReady:
            await asyncio.sleep(0)

        results = {}
        for out in self.outputs:
            results[out["name"]] = out["host"].copy()
        return results


class SuperPointTRT(TRTBase):
    def __init__(self, engine_path=f"/tinynav/tinynav/models/superpoint_fp16_dynamic_{platform.machine()}.plan"):
        super().__init__(engine_path)
        # model input [1,1,H,W]
        self.input_shape = self.inputs[0]["shape"][2:4] # [H,W]

    # default threshold as
    # https://github.com/cvg/LightGlue/blob/746fac2c042e05d1865315b1413419f1c1e7ba55/lightglue/superpoint.py#L111
    #
    @alru_cache_numpy(maxsize=32)
    async def infer(self, input_image:np.ndarray, threshold = np.array([[0.0005]], dtype=np.float32)):
        # Resize to engine input size (may change aspect ratio for non-matching resolutions).
        h_in, w_in = input_image.shape[0], input_image.shape[1]
        h_net, w_net = self.input_shape[0], self.input_shape[1]
        image = cv2.resize(input_image, (w_net, h_net))
        image = image[None, None, :, :]

        np.copyto(self.inputs[0]["host"], image)
        np.copyto(self.inputs[1]["host"], threshold)

        results = await self.run_graph()

        # Scale keypoints from network coords (h_net, w_net) back to input image coords (h_in, w_in).
        # Use per-axis scale so Looper (640x544) and other resolutions match; img_shape is (width, height).
        scale_x = w_in / w_net
        scale_y = h_in / h_net
        k = results["kpts"][0]
        if k.shape[0] == 2:
            k[0] = (k[0] + 0.5) * scale_x - 0.5
            k[1] = (k[1] + 0.5) * scale_y - 0.5
        else:
            k[:, 0] = (k[:, 0] + 0.5) * scale_x - 0.5
            k[:, 1] = (k[:, 1] + 0.5) * scale_y - 0.5
        results["mask"] = results["mask"][:, :, None]
        return results


class SuperPointORT:
    """SuperPoint through ONNX Runtime, output-identical to SuperPointTRT.

    The X5 has no TensorRT, so the board runs SuperPoint on ONNX Runtime; evaluating
    it on the PC through the TRT class would measure a different graph execution than
    the one that ships. Same class here, same numbers there.

    Measured 2026-08-14 on map_day / map_night, keypoints per frame:

        threshold   day    night   night/day   ORB comparison
        0.00005     489.5  420.9     86.0%     (day hits the graph's 512 slot cap)
        0.0005      315.5  217.7     69.0%     the TRT default
        0.005       166.9  107.6     64.5%
        ORB 1024    913.3  320.4     35.1%     one night frame at zero

    So SuperPoint's advantage is *stability*, not count: it finds fewer points than
    ORB in daylight and cannot exceed 512, but keeps far more of them at night and
    never returns an empty frame.
    """

    def __init__(
        self,
        model_path: str,
        net_hw: tuple[int, int] = (320, 272),
        threshold: float = 0.0005,
        num_threads: int = 0,
        top_k_keypoints: int = 0,
    ):
        import onnxruntime as ort  # local: the board has it, a TRT-only host may not

        so = ort.SessionOptions()
        # Arena and mem-pattern trade memory for speed and never return it to the OS.
        # On a 1307 MB board that is the wrong trade -- measured on DINOv2 it cost
        # 312 MB for no latency change.
        so.enable_cpu_mem_arena = False
        so.enable_mem_pattern = False
        so.log_severity_level = 3
        if num_threads:
            so.intra_op_num_threads = int(num_threads)
            so.inter_op_num_threads = 1
        self.session = ort.InferenceSession(model_path, so, providers=["CPUExecutionProvider"])
        self.output_names = [o.name for o in self.session.get_outputs()]
        self.net_h, self.net_w = int(net_hw[0]), int(net_hw[1])
        self.threshold = float(threshold)
        self.top_k_keypoints = int(top_k_keypoints)

    async def infer(self, input_image: np.ndarray, threshold: np.ndarray | None = None):
        if input_image is None:
            raise ValueError("input_image is None")
        if input_image.ndim == 3:
            input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        h_in, w_in = input_image.shape[:2]
        image = cv2.resize(input_image, (self.net_w, self.net_h))
        thr = (np.array([[self.threshold]], dtype=np.float32) if threshold is None
               else np.asarray(threshold, dtype=np.float32).reshape(1, 1))

        raw = self.session.run(
            None,
            {"image": np.ascontiguousarray(image[None, None], dtype=np.uint8),
             "keypoint_threshold": thr},
        )
        res = dict(zip(self.output_names, raw))

        # Network coords -> input image coords, per axis, matching SuperPointTRT.
        scale_x = w_in / self.net_w
        scale_y = h_in / self.net_h
        k = np.array(res["kpts"], dtype=np.float32)
        k[..., 0] = (k[..., 0] + 0.5) * scale_x - 0.5
        k[..., 1] = (k[..., 1] + 0.5) * scale_y - 0.5

        # The graph pads its outputs to a fixed slot count and marks the real
        # detections in `mask`; downstream expects (1, N, 1) like the TRT path.
        mask = np.asarray(res["mask"]).astype(bool)
        desc = np.asarray(res["descps"], dtype=np.float32)
        scores = np.asarray(res["scores"], dtype=np.float32)
        if self.top_k_keypoints:
            # Keep the strongest K by masking the rest off, rather than compacting the
            # arrays: everything downstream already reads `mask` to find the real slots.
            valid = np.flatnonzero(mask[0])
            if valid.size > self.top_k_keypoints:
                drop = valid[np.argsort(-scores[0][valid])[self.top_k_keypoints:]]
                mask[0, drop] = False
        return {
            "kpts": k,
            "descps": desc,
            "scores": scores,
            "mask": mask.astype(np.float32)[:, :, None],
        }


def _bpu_model_path() -> str:
    return os.environ.get("TINYNAV_SP_BPU_MODEL") or str(
        Path(__file__).resolve().parents[1] / "models" / "sp_backbone.bin"
    )


class SuperPointBPU:
    """SuperPoint with its conv backbone on the X5 BPU, output-compatible with SuperPointTRT.

    Everything from Softmax onward has shapes that depend on how many keypoints survive,
    and the BPU needs static shapes -- so only the backbone is offloaded and the rest runs
    on the CPU as pure numpy (`sp_post`). Board-measured: 39.4 ms backbone + 78.8 ms post
    = 146.7 ms, against 3260 ms for the whole ONNX on the same CPU.
    """

    # Baked into sp_backbone.bin. The BPU input is static, so changing this needs a
    # recompile -- see x5_work/bpu/README.md. 640x544 is the Looper infra native size,
    # which means no resize and scale=1 in the deployed path.
    NET_H, NET_W = 640, 544

    def __init__(
        self,
        model_path: str | None = None,
        threshold: float = 5e-4,
        top_k: int = 512,
        nms_mode: str = "fast",
        priority: int | None = None,
    ):
        # Board-only deps: libdnn.so exists on the X5 and nowhere else, so import late
        # to keep this module importable on a PC.
        from tinynav.core import hbdnn, sp_post

        self._post = sp_post
        self.model_path = model_path or _bpu_model_path()
        kwargs = {} if priority is None else {"priority": int(priority)}
        self.model = hbdnn.BPUModel(self.model_path, **kwargs)
        self.threshold = float(threshold)
        self.top_k = int(top_k)
        self.nms_mode = nms_mode
        logger.info("SuperPointBPU: %s at %dx%d", self.model_path, self.NET_H, self.NET_W)

    # No alru_cache_numpy here on purpose: the sibling TRT class caches 32 frames of
    # results, which on a board with no swap costs more than the repeat inference saves.
    async def infer(self, input_image: np.ndarray, threshold: np.ndarray | None = None):
        if input_image is None:
            raise ValueError("input_image is None")
        if input_image.ndim == 3:
            input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        h_in, w_in = input_image.shape[:2]
        image = (input_image if (h_in, w_in) == (self.NET_H, self.NET_W)
                 else cv2.resize(input_image, (self.NET_W, self.NET_H)))
        thr = (self.threshold if threshold is None
               else float(np.asarray(threshold, dtype=np.float32).reshape(-1)[0]))

        x = np.ascontiguousarray(image, dtype=np.float32).reshape(1, 1, self.NET_H, self.NET_W)
        x /= 255.0
        outs = [np.asarray(o) for o in self.model.infer(x)]
        # Identify by channel count, not output order: 65 = heatmap logits + dust bin,
        # 256 = descriptor map. Output order is a property of the compiled .bin.
        try:
            logits = next(o for o in outs if o.shape[-3] == 65)
            desc_map = next(o for o in outs if o.shape[-3] == 256)
        except StopIteration:
            raise RuntimeError(
                f"unexpected BPU output shapes {[o.shape for o in outs]}; "
                f"expected one 65-channel and one 256-channel tensor"
            ) from None
        logits = logits.reshape(logits.shape[-3:])
        desc_map = desc_map.reshape(desc_map.shape[-3:])

        kpts, scores, descs = self._post.postprocess(
            logits, desc_map, thr, self.top_k, self.nms_mode
        )

        # Network coords -> input image coords, per axis, matching SuperPointTRT.
        k = np.asarray(kpts, dtype=np.float32).reshape(-1, 2).copy()
        if k.size:
            k[:, 0] = (k[:, 0] + 0.5) * (w_in / self.NET_W) - 0.5
            k[:, 1] = (k[:, 1] + 0.5) * (h_in / self.NET_H) - 0.5
        n = k.shape[0]
        descs = np.asarray(descs, dtype=np.float32).reshape(n, -1)
        # sp_post already dropped the sub-threshold slots, so every row is real. An
        # all-ones mask is the existing convention for variable-length features here
        # (build_map_node.py builds one the same way for map-loaded descriptors).
        return {
            "kpts": k.reshape(1, n, 2),
            "descps": descs.reshape(1, n, descs.shape[-1] if n else 256),
            "scores": np.asarray(scores, dtype=np.float32).reshape(1, n),
            "mask": np.ones((1, n, 1), dtype=np.float32),
        }


def make_sp_extractor(backend: str | None = None):
    """Pick a SuperPoint backend: `bpu` on the X5, `trt` on a CUDA host, `ort` as fallback.

    Default is auto -- BPU when both its runtime and the compiled model are present, TRT
    otherwise. An explicit request that cannot be honoured raises instead of falling back:
    a silent downgrade to the 3.3 s CPU path reads as a hang, not as an error.
    """
    want = (backend or os.environ.get("TINYNAV_SP_BACKEND") or "auto").lower()
    model_path = _bpu_model_path()
    have_model, have_runtime = os.path.exists(model_path), os.path.exists("/usr/lib/libdnn.so")
    if want == "auto":
        want = "bpu" if (have_model and have_runtime) else "trt"
    if want == "bpu":
        if not (have_model and have_runtime):
            raise RuntimeError(
                f"SuperPoint BPU backend requested but unavailable: {model_path} "
                f"exists={have_model}, /usr/lib/libdnn.so exists={have_runtime}"
            )
        return SuperPointBPU(model_path)
    if want == "trt":
        return SuperPointTRT()
    if want == "ort":
        return SuperPointORT(
            str(Path(__file__).resolve().parents[1] / "models" / "superpoint_fp16_dynamic.onnx")
        )
    raise ValueError(f"unknown SuperPoint backend {want!r}; want bpu, trt or ort")


class SuperPointMatcher:
    """Classical matching over SuperPoint's float descriptors, LightGlue-compatible out.

    LightGlue costs 2870 ms on the X5 against 38.6 ms for cv2's brute-force L2, so the
    learned matcher is not an option there; this is what replaces it. The filters are
    the same three as ORBMatcher's and reject the same three different things, but the
    distance is L2 over 256 floats rather than Hamming over 32 bytes.
    """

    def __init__(
        self,
        mode: str = "cross",
        ratio: float = 0.8,
        use_ransac: bool = True,
        ransac_reproj_threshold: float = 2.0,
    ):
        if mode not in ("cross", "ratio"):
            raise ValueError(f"SuperPointMatcher mode must be 'cross' or 'ratio', got {mode!r}")
        self.mode = mode
        self.ratio = float(ratio)
        self.use_ransac = bool(use_ransac)
        self.ransac_reproj_threshold = float(ransac_reproj_threshold)
        self.bf_cross = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        self.bf_plain = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

    async def infer(self, kpts0, kpts1, desc0, desc1, mask0, mask1,
                    img_shape0=None, img_shape1=None, match_threshold=None):
        k0 = np.asarray(kpts0[0], dtype=np.float32)
        k1 = np.asarray(kpts1[0], dtype=np.float32)
        d0 = np.asarray(desc0[0], dtype=np.float32)
        d1 = np.asarray(desc1[0], dtype=np.float32)
        n0 = int(min(k0.shape[0], d0.shape[0]))
        n1 = int(min(k1.shape[0], d1.shape[0]))
        out = {"match_indices": np.full((1, max(n0, 1)), -1, dtype=np.int32)}
        if n0 == 0 or n1 == 0 or k0.ndim != 2 or k1.ndim != 2:
            return out
        k0, k1, d0, d1 = k0[:n0, :2], k1[:n1, :2], d0[:n0], d1[:n1]

        def valid(mask, n):
            if mask is None or np.asarray(mask).size == 0:
                return np.ones((n,), dtype=bool)
            m = np.asarray(mask)[0, :, 0] > 0
            return m[:n] if m.shape[0] >= n else np.ones((n,), dtype=bool)

        idx0 = np.where(valid(mask0, n0))[0]
        idx1 = np.where(valid(mask1, n1))[0]
        if idx0.size == 0 or idx1.size < 2:
            return out
        q, t = np.ascontiguousarray(d0[idx0]), np.ascontiguousarray(d1[idx1])

        if self.mode == "cross":
            good = sorted(self.bf_cross.match(q, t), key=lambda m: m.distance)
        else:
            good = []
            for pair in self.bf_plain.knnMatch(q, t, k=2):
                if len(pair) < 2:
                    continue
                m, n = pair[:2]
                if m.distance < self.ratio * n.distance:
                    good.append(m)
        if not good:
            return out

        if self.use_ransac and len(good) >= 8:
            p0 = np.asarray([k0[idx0[m.queryIdx]] for m in good], dtype=np.float32)
            p1 = np.asarray([k1[idx1[m.trainIdx]] for m in good], dtype=np.float32)
            _, inl = cv2.findFundamentalMat(
                p0, p1, cv2.FM_RANSAC, self.ransac_reproj_threshold, 0.99
            )
            if inl is not None and inl.size == len(good):
                good = [m for m, keep in zip(good, inl.reshape(-1).astype(bool)) if keep]
            if not good:
                return out

        for m in good:
            out["match_indices"][0, int(idx0[m.queryIdx])] = int(idx1[m.trainIdx])
        return out


class ORBFeatureTRTCompatible:
    """
    ORB feature extractor with a SuperPoint-compatible output interface.

    Returns a dict with:
      - kpts:  (1, N, 2) float32
      - descps: (1, N, 32) float32
      - mask:  (1, N, 1) float32
    """

    def __init__(self, nfeatures: int = 1024):
        self.nfeatures = int(nfeatures)
        self.orb = cv2.ORB_create(nfeatures=self.nfeatures)

    async def infer(self, input_image: np.ndarray):
        if input_image is None:
            raise ValueError("input_image is None")
        if input_image.ndim == 3:
            gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        elif input_image.ndim == 2:
            gray = input_image
        else:
            raise ValueError(f"Unsupported input_image ndim={input_image.ndim}")

        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        if keypoints is None or len(keypoints) == 0:
            return {
                "kpts": np.zeros((1, 0, 2), dtype=np.float32),
                "descps": np.zeros((1, 0, 32), dtype=np.float32),
                "mask": np.zeros((1, 0, 1), dtype=np.float32),
            }

        kpts = np.array([[kp.pt[0], kp.pt[1]] for kp in keypoints], dtype=np.float32)
        if descriptors is None:
            descps = np.zeros((kpts.shape[0], 32), dtype=np.float32)
        else:
            descps = descriptors.astype(np.float32) / 255.0

        return {
            "kpts": kpts[None, :, :],
            "descps": descps[None, :, :],
            "mask": np.ones((1, kpts.shape[0], 1), dtype=np.float32),
        }


class ORBMatcher:
    """
    ORB descriptor matcher for binary descriptors, in either of two configurations.

    Output is LightGlue-compatible:
      - match_indices: (1, N0) int32, each value is matched index in keypoints1 or -1.

    `mode` selects the correspondence search and `use_ransac` the geometric check.
    They are separate because they reject different things: the ratio test and
    crossCheck are descriptor-space filters that cannot see geometry, so neither
    can reject a corridor's two identical doors -- only the epipolar RANSAC can.
    Both were present until 3381b48 (2026-06-03) replaced BF+crossCheck+RANSAC
    with FLANN LSH + ratio and dropped the RANSAC; keeping both paths selectable
    is what lets that regression be measured rather than argued about.
    """

    def __init__(
        self,
        ransac_reproj_threshold: float = 1.0,
        flann_ratio: float = 0.75,
        mode: str = "flann",
        use_ransac: bool = False,
    ):
        if mode not in ("flann", "bf"):
            raise ValueError(f"ORBMatcher mode must be 'flann' or 'bf', got {mode!r}")
        self.mode = mode
        self.use_ransac = bool(use_ransac)
        flann_index_lsh = 6
        index_params = dict(
            algorithm=flann_index_lsh,
            table_number=6,
            key_size=12,
            multi_probe_level=1,
        )
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        # crossCheck already enforces mutual nearest, which is why the bf path needs
        # no ratio test: the two are alternatives, and OpenCV refuses knnMatch(k=2)
        # when crossCheck is on.
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.ransac_reproj_threshold = float(ransac_reproj_threshold)
        self.flann_ratio = float(flann_ratio)

    async def infer(
        self,
        kpts0: np.ndarray,
        kpts1: np.ndarray,
        desc0: np.ndarray,
        desc1: np.ndarray,
        mask0: np.ndarray,
        mask1: np.ndarray,
        img_shape0=None,
        img_shape1=None,
        match_threshold=None,
    ):
        k0 = np.asarray(kpts0[0], dtype=np.float32)
        k1 = np.asarray(kpts1[0], dtype=np.float32)
        if k0.ndim != 2 or k1.ndim != 2 or k0.shape[1] < 2 or k1.shape[1] < 2:
            n0 = int(k0.shape[0]) if k0.ndim >= 1 else 0
            return {"match_indices": np.full((1, n0), -1, dtype=np.int32)}
        # keep only xy coordinates
        k0 = k0[:, :2]
        k1 = k1[:, :2]
        n0 = int(k0.shape[0])
        out = {"match_indices": np.full((1, n0), -1, dtype=np.int32)}
        if n0 == 0 or k1.shape[0] == 0:
            return out

        d0 = np.asarray(desc0[0], dtype=np.float32)
        d1 = np.asarray(desc1[0], dtype=np.float32)
        if d0.ndim != 2 or d1.ndim != 2 or d0.shape[0] == 0 or d1.shape[0] == 0:
            return out
        # Ensure descriptor rows and keypoints rows are aligned.
        n0_valid = min(k0.shape[0], d0.shape[0])
        n1_valid = min(k1.shape[0], d1.shape[0])
        if n0_valid == 0 or n1_valid == 0:
            return out
        k0 = k0[:n0_valid]
        k1 = k1[:n1_valid]
        d0 = d0[:n0_valid]
        d1 = d1[:n1_valid]
        out = {"match_indices": np.full((1, n0_valid), -1, dtype=np.int32)}

        m0 = np.asarray(mask0[0, :, 0] > 0, dtype=bool) if mask0.size else np.ones((d0.shape[0],), dtype=bool)
        m1 = np.asarray(mask1[0, :, 0] > 0, dtype=bool) if mask1.size else np.ones((d1.shape[0],), dtype=bool)
        if m0.shape[0] != d0.shape[0]:
            m0 = np.ones((d0.shape[0],), dtype=bool)
        if m1.shape[0] != d1.shape[0]:
            m1 = np.ones((d1.shape[0],), dtype=bool)

        idx0 = np.where(m0)[0]
        idx1 = np.where(m1)[0]
        if idx0.size == 0 or idx1.size == 0:
            return out

        # ORB descriptor is binary; convert normalized float [0,1] back to uint8 [0,255].
        d0_u8 = np.ascontiguousarray(np.clip(np.rint(d0[idx0] * 255.0), 0, 255).astype(np.uint8))
        d1_u8 = np.ascontiguousarray(np.clip(np.rint(d1[idx1] * 255.0), 0, 255).astype(np.uint8))
        if d0_u8.size == 0 or d1_u8.size == 0:
            return out

        # Lowe's ratio test needs a second-nearest neighbour to compare against.
        # When the train set holds a single descriptor there is none, and
        # accepting the sole neighbour unconditionally is not a degenerate corner
        # case -- it is actively harmful: every query descriptor then "matches"
        # that one train descriptor, so all correspondences collapse onto a single
        # keypoint. Downstream that yields hundreds of coincident 2D observations,
        # a PnP problem whose translation is unobservable, a pose at ~1e15 m with
        # exactly zero reprojection error, and an inlier ratio of 1.0 -- i.e. the
        # worst possible estimate carries the highest possible confidence. A frame
        # with one keypoint cannot localize anything, so report no matches.
        if d1_u8.shape[0] < 2:
            logger.warning(
                "ORBMatcher: train set has %d descriptor(s), too few to localize; "
                "no matches reported",
                d1_u8.shape[0],
            )
            return out

        if self.mode == "bf":
            good_matches = sorted(self.bf.match(d0_u8, d1_u8), key=lambda m: m.distance)
            n_raw = len(good_matches)
        else:
            raw_matches = self.flann.knnMatch(d0_u8, d1_u8, k=2)
            n_raw = len(raw_matches)
            good_matches = []
            for pair in raw_matches:
                if len(pair) < 2:
                    # LSH is approximate and can return a single neighbour even when
                    # the train set holds many. Accepting it would mean accepting a
                    # match that passed no quality test at all, and measurement says
                    # that costs more than it gains: keeping these matches drops
                    # night precision from 71.1% to 61.0% and day correct
                    # relocalizations from 1123 to 1120. Undecidable, so discard.
                    continue
                m, n = pair[:2]
                if m.distance < self.flann_ratio * n.distance:
                    good_matches.append(m)

        if len(good_matches) == 0:
            return out

        n_before_ransac = len(good_matches)
        # Eight correspondences is the fundamental matrix's minimum; below that the
        # solve is meaningless and the matches pass through unfiltered.
        if self.use_ransac and len(good_matches) >= 8:
            pts0 = np.asarray([k0[idx0[m.queryIdx]] for m in good_matches], dtype=np.float32)
            pts1 = np.asarray([k1[idx1[m.trainIdx]] for m in good_matches], dtype=np.float32)
            _, inliers = cv2.findFundamentalMat(
                pts0, pts1, cv2.FM_RANSAC, self.ransac_reproj_threshold, 0.99
            )
            if inliers is not None and inliers.size == len(good_matches):
                mask = inliers.reshape(-1).astype(bool)
                good_matches = [m for m, keep in zip(good_matches, mask) if keep]
            if len(good_matches) == 0:
                return out

        logger.info(
            "ORBMatcher %s: queries=%d train=%d raw=%d filtered=%d ransac=%s kept=%d",
            self.mode,
            d0_u8.shape[0],
            d1_u8.shape[0],
            n_raw,
            n_before_ransac,
            "on" if self.use_ransac else "disabled",
            len(good_matches),
        )

        for m in good_matches:
            gi = int(idx0[m.queryIdx])
            gj = int(idx1[m.trainIdx])
            out["match_indices"][0, gi] = gj
        return out


class DBoW3Engine:
    """
    Thin Python wrapper for PyDBoW3 database operations.

    Supports adding/querying either:
      - ORBFeatureTRTCompatible outputs (dict with "descps")
      - Raw descriptor arrays shaped (N, 32), dtype uint8/float32
    """

    def __init__(self, vocabulary_path: str | None = None, voc: Any = None):
        """
        Args:
            vocabulary_path: vocabulary to load. Ignored when `voc` is given.
            voc: an already-loaded Vocabulary to reuse, as returned by
                load_vocabulary(). Several engines may be built from one
                Vocabulary object: DBoW3's Database::setVocabulary() deep-copies
                the vocabulary it is handed (`m_voc = new Vocabulary(voc)`), so
                the databases never alias each other's word tree. Reusing it only
                avoids re-reading and re-inflating the file, which for the stock
                ORBvoc is ~48 MB of disk and ~300 MiB resident per copy (measured
                on x86_64).
        """
        self._bow = self._import_module()
        if voc is None:
            if vocabulary_path is None:
                raise ValueError("DBoW3Engine needs either vocabulary_path or voc")
            voc = self.load_vocabulary(vocabulary_path)
        self.db = self._bow.Database()
        self.db.setVocabulary(voc)
        # Database::setVocabulary() took its own copy, so holding a second
        # reference here would keep a redundant word tree resident for the life
        # of the engine (~300 MiB with ORBvoc). Callers that want to keep the
        # vocabulary around for reuse hold their own reference.
        self.voc = None

    @classmethod
    def load_vocabulary(cls, vocabulary_path: str):
        """Load a vocabulary once so it can be shared by several engines."""
        bow = cls._import_module()
        voc = bow.Vocabulary()
        load_ret = voc.load(vocabulary_path)
        # PyDBoW3 often returns None on success (instead of True).
        if load_ret is False:
            raise RuntimeError(f"Failed to load DBoW3 vocabulary: {vocabulary_path}")
        return voc

    @staticmethod
    def _import_module():
        try:
            import pydbow3 as bow  # type: ignore
            return bow
        except Exception:
            import pyDBoW3 as bow  # type: ignore
            return bow

    @staticmethod
    def _normalize_desc(features: Any) -> np.ndarray:
        if isinstance(features, dict):
            if "descps" not in features:
                raise ValueError("features dict must contain 'descps'")
            desc = np.asarray(features["descps"])
            if desc.ndim == 3:
                desc = desc[0]
        else:
            desc = np.asarray(features)

        if desc.ndim != 2:
            raise ValueError(f"Descriptor array must be 2D, got shape {desc.shape}")
        if desc.size == 0:
            return np.zeros((0, desc.shape[1] or 32), dtype=np.uint8)

        # Width picks the metric: DBoW3 dispatches on cv::Mat type, CV_8U -> Hamming,
        # CV_32F -> L2. Anything other than ORB's 32 bytes is a real-valued descriptor
        # (SuperPoint is 256) and has to reach the shim as float32 -- the uint8 rounding
        # below would quantize it to noise without raising.
        if desc.shape[1] != 32:
            return np.ascontiguousarray(desc, dtype=np.float32)

        if desc.dtype == np.uint8:
            return np.ascontiguousarray(desc)
        # ORBFeatureTRTCompatible emits float32 in [0,1], convert back to binary-bytes domain.
        if np.issubdtype(desc.dtype, np.floating):
            return np.ascontiguousarray(np.clip(np.rint(desc * 255.0), 0, 255).astype(np.uint8))
        return np.ascontiguousarray(desc.astype(np.uint8))

    def add(self, features: Any):
        desc = self._normalize_desc(features)
        return self.db.add(desc)

    def query(self, features: Any, max_results: int = 10):
        desc = self._normalize_desc(features)
        try:
            results = self.db.query(desc, int(max_results))
        except TypeError:
            # Some bindings expose query(desc) only.
            results = self.db.query(desc)
        return [
            {
                "id": int(getattr(r, "Id", getattr(r, "id", -1))),
                "score": float(getattr(r, "Score", getattr(r, "score", 0.0))),
            }
            for r in results
        ]


class LightGlueTRT(TRTBase):
    def __init__(self, engine_path=f"/tinynav/tinynav/models/lightglue_fp16_{platform.machine()}.plan"):
        super().__init__(engine_path)

    # default threshold as
    # https://github.com/cvg/LightGlue/blob/746fac2c042e05d1865315b1413419f1c1e7ba55/lightglue/lightglue.py#L333
    #
    @alru_cache_numpy(maxsize=32)
    async def infer(self, kpts0, kpts1, desc0, desc1, mask0, mask1, img_shape0, img_shape1, match_threshold = np.array([[0.1]], dtype=np.float32)):
        np.copyto(self.inputs[0]["host"], kpts0)
        np.copyto(self.inputs[1]["host"], kpts1)
        np.copyto(self.inputs[2]["host"], desc0)
        np.copyto(self.inputs[3]["host"], desc1)
        np.copyto(self.inputs[4]["host"], mask0)
        np.copyto(self.inputs[5]["host"], mask1)
        np.copyto(self.inputs[6]["host"], img_shape0)
        np.copyto(self.inputs[7]["host"], img_shape1)
        np.copyto(self.inputs[8]["host"], match_threshold)

        return await self.run_graph()

class Dinov2TRT(TRTBase):
    def __init__(self, engine_path=f"/tinynav/tinynav/models/dinov2_base_224x224_fp16_{platform.machine()}.plan"):
        super().__init__(engine_path)

    def preprocess_image(self, image, target_size=224):
        image = cv2.resize(image, (target_size, target_size), interpolation=cv2.INTER_CUBIC)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        image = einops.rearrange(image, "h w c-> 1 c h w")
        image = image.astype(np.float32) / 255.0

        mean = np.array([0.485, 0.456, 0.406]).reshape(3, 1, 1)
        std = np.array([0.229, 0.224, 0.225]).reshape(3, 1, 1)
        image = (image - mean) / std
        return image

    async def infer(self, image):
        image = self.preprocess_image(image)
        np.copyto(self.inputs[0]["host"], image)
        results = await self.run_graph()
        return results["last_hidden_state"][:, 0, :].squeeze(0)


class StereoEngineTRT(TRTBase):
    def _get_static_shape(self, name):
        """Ensure the stereo output gets a valid max shape for buffer allocation.

        Retinify is disp-only with NHWC tensors (B, H, W, C). Some TensorRT
        versions report dynamic outputs with empty/scalar shapes. Instead of
        asking output profile shape directly, derive max output shape from the
        "left" input profile because output shares the same spatial resolution.
        """
        if self.engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
            try:
                _, _, max_in_shape = self.engine.get_tensor_profile_shape("left", 0)
                # left input is NHWC -> output is NHWC with single channel.
                return (1, int(max_in_shape[1]), int(max_in_shape[2]), 1)
            except Exception:
                pass
        return super()._get_static_shape(name)

    def __init__(self, engine_path=f"/tinynav/tinynav/models/retinify_0_1_5_dynamic_{platform.machine()}.plan"):
        super().__init__(engine_path)
        if len(self.inputs) != 2:
            raise RuntimeError(f"Retinify disp-only engine must have 2 inputs, got {len(self.inputs)}")
        if len(self.outputs) != 1:
            raise RuntimeError(f"Retinify disp-only engine must have 1 output, got {len(self.outputs)}")
        self.output_name = self.outputs[0]["name"]
        self.input_dtype = self.inputs[0]["host"].dtype
        # Current shapes/byte sizes are set per infer() call, based on the
        # actually received image size (H, W), not the engine's max profile.
        self._current_input_shapes = (1, 1, 1, 1)
        self._current_input_nbytes = 0

    def capture_graph(self):
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            self.context.set_tensor_address(name, self.bindings[i])
        return None

    async def run_graph(self):
        input_shapes = self._current_input_shapes
        if "aarch64" not in platform.machine():
            cudart.cudaMemcpyAsync(self.inputs[0]["device"], self.inputs[0]["host"].ctypes.data,
                                   self._current_input_nbytes, cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream)
            cudart.cudaMemcpyAsync(self.inputs[1]["device"], self.inputs[1]["host"].ctypes.data,
                                   self._current_input_nbytes, cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream)
        self.context.set_optimization_profile_async(0, self.stream)
        self.context.set_input_shape("left", input_shapes)
        self.context.set_input_shape("right", input_shapes)
        self.context.execute_async_v3(stream_handle=self.stream)
        h_net, w_net = input_shapes[1], input_shapes[2]
        if "aarch64" not in platform.machine():
            for out in self.outputs:
                nbytes = h_net * w_net * np.float32().itemsize
                cudart.cudaMemcpyAsync(
                    out["host"].ctypes.data,
                    out["device"],
                    nbytes,
                    cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                    self.stream,
                )
        cudart.cudaStreamSynchronize(self.stream)
        results = {}
        for out in self.outputs:
            flat = np.asarray(out["host"]).reshape(-1)
            needed = h_net * w_net
            results[out["name"]] = flat[:needed].reshape(h_net, w_net).copy()
        return results

    async def infer(self, left_img, right_img, baseline, focal_length):
        h_in, w_in = left_img.shape[0], left_img.shape[1]

        self._current_input_shapes = (1, h_in, w_in, 1)
        # Retinify ONNX takes FLOAT inputs in NHWC layout.
        left_tensor = left_img.astype(self.input_dtype, copy=False)[None, :, :, None]
        right_tensor = right_img.astype(self.input_dtype, copy=False)[None, :, :, None]
        self._current_input_nbytes = left_tensor.nbytes

        # Copy only the active region into max-profile host buffers.
        np.copyto(self.inputs[0]["host"].reshape(-1)[: left_tensor.size], left_tensor.reshape(-1))
        np.copyto(self.inputs[1]["host"].reshape(-1)[: right_tensor.size], right_tensor.reshape(-1))

        results = await self.run_graph()
        disp = results[self.output_name]
        if disp.shape != (h_in, w_in):
            raise RuntimeError(
                f"StereoEngine output shape mismatch: got disp {disp.shape}, expected ({h_in}, {w_in})"
            )
        disp = disp.astype(np.float32)
        depth = disparity_to_depth(disp, baseline, focal_length)
        return disp, depth


if __name__ == "__main__":
    # Synthetic sanity test for both RealSense and Looper resolutions.
    dinov2 = Dinov2TRT()
    superpoint = SuperPointTRT()
    light_glue = LightGlueTRT()
    stereo_engine = StereoEngineTRT()

    # Each entry: (name, width, height)
    resolutions = [
        ("realsense", 848, 480),
        ("looper", 544, 640),
    ]

    match_threshold = np.array([0.1], dtype=np.float32)
    threshold = np.array([0.015], dtype=np.float32)

    for tag, width, height in resolutions:
        print(f"\n=== Testing stereo pipeline for {tag} resolution: {height}x{width} ===")
        image_shape = np.array([width, height], dtype=np.int64)

        dummy_left = np.random.randint(0, 256, (height, width), dtype=np.uint8)
        dummy_right = np.random.randint(0, 256, (height, width), dtype=np.uint8)

        with Timer(text=f"[dinov2:{tag}] Elapsed time: {{milliseconds:.0f}} ms"):
            _ = asyncio.run(dinov2.infer(dummy_left))

        with Timer(text=f"[superpoint:{tag}] Elapsed time: {{milliseconds:.0f}} ms"):
            left_extract_result = asyncio.run(superpoint.infer(dummy_left))
            right_extract_result = asyncio.run(superpoint.infer(dummy_right))

        with Timer(text=f"[lightglue:{tag}] Elapsed time: {{milliseconds:.0f}} ms"):
            _ = asyncio.run(
                light_glue.infer(
                    left_extract_result["kpts"],
                    right_extract_result["kpts"],
                    left_extract_result["descps"],
                    right_extract_result["descps"],
                    left_extract_result["mask"],
                    right_extract_result["mask"],
                    image_shape,
                    image_shape,
                    match_threshold,
                )
            )

        with Timer(text=f"[stereo:{tag}] Elapsed time: {{milliseconds:.0f}} ms"):
            baseline = np.array([[0.05]], dtype=np.float32)
            focal_length = np.array([[323.0]], dtype=np.float32)
            _disp, _depth = asyncio.run(
                stereo_engine.infer(dummy_left, dummy_right, baseline, focal_length)
            )
