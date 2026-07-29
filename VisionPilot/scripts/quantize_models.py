"""Quantise the VisionPilot fp32 ONNX models to TensorRT-compatible symmetric int8.

The int8 weights that shipped previously were asymmetric UINT8 QDQ graphs. TensorRT
refuses those outright ("TensorRT only supports symmetric quantization"), so every int8
run fell back to the CUDA provider, which has no int8 kernels and simply dequantises —
roughly half the speed of fp32. Symmetric, per-channel, INT8 QDQ is what TensorRT can
actually fuse into int8 kernels.

Calibration frames come from the bundled nuScenes clips and go through exactly the same
preprocessing as the C++ pipeline (see modules/models/src/inference.cpp):

  AutoDrive              warpPerspective(frame, C) -> ImageNet mean/std, two frames
  AutoSteer / AutoSpeed  top-crop to 2:1 -> resize 1024x512 -> [0, 1]

Usage:
    /usr/bin/python3 quantize_models.py                    # all three models
    /usr/bin/python3 quantize_models.py --models autosteer --frames 48
    /usr/bin/python3 quantize_models.py --out-dir /tmp/int8_test
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import onnx
from onnxruntime.quantization import (
    CalibrationDataReader,
    CalibrationMethod,
    QuantFormat,
    QuantType,
    quantize_static,
)
from onnxruntime.quantization.shape_inference import quant_pre_process

sys.path.insert(0, str(Path(__file__).resolve().parent))
from find_homography_C_matrix import find_homography_C_matrix, load_homography_H_matrix

NET_W, NET_H = 1024, 512
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def compute_top_crop_2_1(height: int, width: int) -> int:
    """Mirror of compute_top_crop_2_1() in modules/common/include/common/utils.hpp."""
    return max(0, int(round(height - width / 2.0)))


def chw_imagenet(bgr: np.ndarray) -> np.ndarray:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return ((rgb - MEAN) / STD).transpose(2, 0, 1)[None].astype(np.float32)


def chw_01(bgr: np.ndarray) -> np.ndarray:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return rgb.transpose(2, 0, 1)[None].astype(np.float32)


def scene_frames(video: Path, count: int):
    """Sample `count` consecutive frame pairs spread evenly across the clip."""
    cap = cv2.VideoCapture(str(video))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
    picks = np.linspace(0, max(total - 2, 0), count, dtype=int)
    for idx in picks:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(idx))
        ok_prev, prev = cap.read()
        ok_curr, curr = cap.read()
        if ok_prev and ok_curr:
            yield prev, curr
    cap.release()


def build_samples(data_dir: Path, frames_per_clip: int):
    """Preprocessed calibration tensors, keyed by the model that consumes them."""
    warped_pairs, resized = [], []

    for video in sorted(data_dir.glob("*.mp4")):
        h_yaml = video.with_name(f"{video.stem}_H.yaml")
        if not h_yaml.exists():
            print(f"  skipping {video.name} — no {h_yaml.name}")
            continue
        C = find_homography_C_matrix(load_homography_H_matrix(h_yaml))
        print(f"  {video.name}: sampling {frames_per_clip} frame pairs")

        for prev, curr in scene_frames(video, frames_per_clip):
            warp = lambda f: cv2.warpPerspective(  # noqa: E731
                f, C, (NET_W, NET_H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101
            )
            warped_pairs.append((chw_imagenet(warp(prev)), chw_imagenet(warp(curr))))

            crop_top = compute_top_crop_2_1(curr.shape[0], curr.shape[1])
            crop = curr[crop_top:, :]
            resized.append(chw_01(cv2.resize(crop, (NET_W, NET_H), interpolation=cv2.INTER_LINEAR)))

    if not warped_pairs:
        raise SystemExit(f"no usable clips in {data_dir}")
    return warped_pairs, resized


class Reader(CalibrationDataReader):
    """Feeds the calibrator one dict of named inputs per frame."""

    def __init__(self, model: Path, tensors):
        graph_inputs = [i.name for i in onnx.load(str(model)).graph.input]
        self.batches = [
            dict(zip(graph_inputs, t if isinstance(t, tuple) else (t,))) for t in tensors
        ]
        self.it = iter(self.batches)

    def get_next(self):
        return next(self.it, None)

    def rewind(self):
        self.it = iter(self.batches)


def unquantizable_convs(model: Path) -> list:
    """Conv nodes TensorRT cannot take in int8 — anything but a 4-D (2-D conv) kernel.

    These models carry a handful of Conv1d nodes (3-D kernels, 32768/8192/2048/512 output
    channels). TensorRT parses them fine in fp32, but with QuantizeLinear/DequantizeLinear
    wrapped around them its parser asserts `checkSpatialDims(kernelTensor->getDimensions())`
    and then refuses the *entire* graph, falling back to CUDA. Leaving just these nodes in
    fp32 keeps the rest of the network int8.
    """
    m = onnx.load(str(model))
    init = {i.name: i for i in m.graph.initializer}
    producer = {o: n for n in m.graph.node for o in n.output}
    excluded = []
    for node in m.graph.node:
        if node.op_type not in ("Conv", "ConvTranspose"):
            continue

        w = node.input[1]

        # Keep the prediction head in fp32. Its class-score convs have a narrow logit range
        # that int8 flattens to zero: quantising them left AutoSpeed detecting nothing at
        # all (68 boxes over threshold in fp32, 0 in int8) while the box channels still
        # looked fine. The head is a few percent of the FLOPs, so this costs almost no speed.
        if w.startswith("head."):
            excluded.append(node.name)
            continue

        if w not in init or len(init[w].dims) != 4:
            excluded.append(node.name)          # Conv1d / runtime kernel
            continue

        # Grouped (depthwise) conv fed by a Reshape: quantising it puts the Reshape and the
        # Conv in different TensorRT subgraphs, and TensorRT then reads the channel count as
        # 1 and rejects the layer ("num_groups must divide input's channel count").
        group = next((a.i for a in node.attribute if a.name == "group"), 1)
        src = producer.get(node.input[0])
        if group > 1 and src is not None and src.op_type == "Reshape":
            excluded.append(node.name)
    return excluded


def quantize(name: str, weights: Path, out_dir: Path, tensors, tmp_dir: Path,
             per_channel: bool = True, method: str = "minmax") -> Path:
    src = weights / f"{name}_fp32.onnx"
    dst = out_dir / f"{name}_int8.onnx"
    prepped = tmp_dir / f"{name}_prepped.onnx"

    print(f"\n[{name}] preprocessing {src.name}")
    quant_pre_process(str(src), str(prepped), skip_symbolic_shape=True)

    skip = unquantizable_convs(prepped)
    if skip:
        print(f"[{name}] leaving {len(skip)} node(s) in fp32 (head + Conv1d + depthwise)")

    print(f"[{name}] calibrating on {len(tensors)} frames (CPU, this takes a while)")
    quantize_static(
        model_input=str(prepped),
        model_output=str(dst),
        calibration_data_reader=Reader(prepped, tensors),
        nodes_to_exclude=skip,
        # Quantise convolutions only. Quantising the graph's tail as well is what silently
        # broke detection: the final Concat merges box coordinates (0..1021 px) with class
        # probabilities (0..1) into one tensor, so a single per-tensor scale of ~8 rounds
        # every probability to zero — AutoSpeed returned no detections at all while its
        # boxes still looked plausible. Convs are where the compute is anyway.
        op_types_to_quantize=["Conv"],
        quant_format=QuantFormat.QDQ,
        activation_type=QuantType.QInt8,
        weight_type=QuantType.QInt8,
        per_channel=per_channel,
        calibrate_method={
            "minmax": CalibrationMethod.MinMax,
            "entropy": CalibrationMethod.Entropy,
            "percentile": CalibrationMethod.Percentile,
        }[method],
        extra_options={
            # Symmetric on both sides is a hard requirement: TensorRT rejects any
            # QuantizeLinear whose zero_point is not all zeros.
            "ActivationSymmetric": True,
            "WeightSymmetric": True,
            # Leave conv biases in float. ORT would otherwise quantise them to INT32 and
            # wrap them in DequantizeLinear, which TensorRT's dequantise layer refuses
            # ("A DequantizeLayer can only run in DataType::kINT8, kFP8, kFP4") — one such
            # node is enough to make TensorRT reject the whole graph.
            "QuantizeBias": False,
        },
    )
    print(f"[{name}] wrote {dst}  ({dst.stat().st_size / 1e6:.1f} MB)")
    return dst


def verify_symmetric(model: Path) -> bool:
    """Every zero_point must be zero and int8, or TensorRT will reject the graph."""
    m = onnx.load(str(model))
    init = {i.name: i for i in m.graph.initializer}
    bad_dtype = bad_value = 0
    for node in m.graph.node:
        if node.op_type not in ("QuantizeLinear", "DequantizeLinear"):
            continue
        if len(node.input) < 3 or node.input[2] not in init:
            continue
        zp = init[node.input[2]]
        if zp.data_type == onnx.TensorProto.UINT8:
            bad_dtype += 1
        if np.any(onnx.numpy_helper.to_array(zp) != 0):
            bad_value += 1
    ok = bad_dtype == 0 and bad_value == 0
    print(f"[{model.stem}] uint8 zero_points={bad_dtype}  non-zero zero_points={bad_value}  "
          f"-> {'TensorRT-compatible' if ok else 'STILL ASYMMETRIC'}")
    return ok


def main() -> None:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights-dir", type=Path, default=here.parent / "modules/models/weights")
    ap.add_argument("--data-dir", type=Path, default=here.parent / "data/nuscenes")
    ap.add_argument("--out-dir", type=Path, default=None, help="default: --weights-dir")
    ap.add_argument("--frames", type=int, default=16, help="frame pairs per clip (default 16)")
    ap.add_argument("--models", nargs="+", default=["autodrive", "autosteer", "autospeed"])
    ap.add_argument("--calibrate", choices=["minmax", "entropy", "percentile"],
                    default="minmax",
                    help="activation range estimator. minmax is fastest but follows "
                         "outliers; entropy (what TensorRT uses internally) usually keeps "
                         "detections closer to fp32")
    ap.add_argument("--per-tensor", action="store_true",
                    help="per-tensor instead of per-channel weight scales — slightly less "
                         "accurate, but avoids the per-channel axis attributes TensorRT "
                         "rejects on some graphs")
    args = ap.parse_args()

    out_dir = args.out_dir or args.weights_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp_dir = out_dir / ".quant_tmp"
    tmp_dir.mkdir(exist_ok=True)

    print(f"Calibration frames from {args.data_dir}")
    warped_pairs, resized = build_samples(args.data_dir, args.frames)
    print(f"  {len(warped_pairs)} frame pairs total")

    all_ok = True
    for name in args.models:
        tensors = warped_pairs if name == "autodrive" else resized
        all_ok &= verify_symmetric(
            quantize(name, args.weights_dir, out_dir, tensors, tmp_dir,
                     per_channel=not args.per_tensor, method=args.calibrate))

    for leftover in tmp_dir.glob("*"):
        leftover.unlink()
    tmp_dir.rmdir()

    print("\nDone." if all_ok else "\nDone, but some graphs are still asymmetric.")
    print("Run with engine.provider = tensorrt to get real int8 kernels.")


if __name__ == "__main__":
    main()
