#!/usr/bin/env python3
"""Convert nuScenes CAM_FRONT scenes into VisionPilot open-loop test inputs.

For every selected scene this produces three files:

    <scene>.mp4        constant-frame-rate video (default 30 fps) built from the
                       CAM_FRONT samples + sweeps, resampled on wall-clock time
                       so playback speed matches reality.
    <scene>_speed.txt  one ego speed in m/s per video frame, derived from the
                       scene's ego_pose trajectory.
    <scene>_H.yaml     ground-plane homography (image px -> world X fwd / Y left,
                       metres, origin at the front bumper) derived from the
                       scene's own camera intrinsics + extrinsics.

Usage:
    python3 nuscenes_to_visionpilot.py --root <NUSCENES_ROOT> --out <OUT_DIR> \
        [--num-scenes 3 | --scenes scene-0061 scene-0757] [--fps 30]
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

# nuScenes ego frame: origin at the centre of the rear axle, on the ground.
# VisionPilot world frame: origin at the front bumper centre, on the ground.
# Renault Zoe (the nuScenes ego vehicle): 2.588 m wheelbase + ~0.83 m front
# overhang.
DEFAULT_BUMPER_OFFSET_M = 3.42


# ── nuScenes table loading ───────────────────────────────────────────────────
def load_tables(root: Path, version: str):
    meta = root / version
    if not meta.is_dir():
        sys.exit(f"error: metadata directory not found: {meta}")
    names = ["scene", "sample", "sample_data", "ego_pose", "calibrated_sensor", "sensor"]
    tables = {}
    for name in names:
        with open(meta / f"{name}.json") as f:
            tables[name] = json.load(f)
    return tables


def quaternion_to_rotation(q):
    """(w, x, y, z) -> 3x3 rotation matrix."""
    w, x, y, z = q
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class NuScenes:
    def __init__(self, root: Path, version: str):
        t = load_tables(root, version)
        # ffmpeg's concat demuxer resolves entries relative to the list file,
        # so every image path handed to it has to be absolute.
        self.root = root.resolve()
        self.scenes = t["scene"]
        self.sample = {s["token"]: s for s in t["sample"]}
        self.ego_pose = {e["token"]: e for e in t["ego_pose"]}
        self.calib = {c["token"]: c for c in t["calibrated_sensor"]}
        sensor = {s["token"]: s for s in t["sensor"]}
        self.calib_channel = {
            tok: sensor[c["sensor_token"]]["channel"] for tok, c in self.calib.items()
        }
        # sample_data grouped by sample token
        self.sd_by_sample = {}
        for sd in t["sample_data"]:
            self.sd_by_sample.setdefault(sd["sample_token"], []).append(sd)

    def scene_sample_tokens(self, scene):
        tokens, tok = [], scene["first_sample_token"]
        while tok:
            tokens.append(tok)
            tok = self.sample[tok]["next"]
        return tokens

    def scene_sample_data(self, scene, channel=None):
        """All sample_data rows of a scene (samples *and* sweeps), time sorted."""
        rows = []
        for tok in self.scene_sample_tokens(scene):
            for sd in self.sd_by_sample.get(tok, []):
                if channel is None or self.calib_channel[sd["calibrated_sensor_token"]] == channel:
                    rows.append(sd)
        rows.sort(key=lambda sd: sd["timestamp"])
        return rows

    def ego_track(self, scene):
        """(t_seconds, x, y) of every distinct ego pose in the scene."""
        seen, poses = set(), []
        for sd in self.scene_sample_data(scene):
            tok = sd["ego_pose_token"]
            if tok in seen:
                continue
            seen.add(tok)
            poses.append(self.ego_pose[tok])
        poses.sort(key=lambda p: p["timestamp"])
        t = np.array([p["timestamp"] for p in poses], dtype=np.float64) / 1e6
        xy = np.array([p["translation"][:2] for p in poses], dtype=np.float64)
        return t, xy


# ── derived products ─────────────────────────────────────────────────────────
def speed_profile(t, xy):
    """Ego speed (m/s) -> (times, speeds).

    Poses from different sensors can share a timestamp; np.gradient divides by
    the time step, so duplicates have to go before differentiating.
    """
    if len(t) < 2:
        return t, np.zeros_like(t)
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    t, xy = t[keep], xy[keep]
    if len(t) < 2:
        return t, np.zeros_like(t)
    vx = np.gradient(xy[:, 0], t)
    vy = np.gradient(xy[:, 1], t)
    v = np.hypot(vx, vy)
    # Light smoothing: pose noise differentiates into visible speed jitter.
    k = 5
    pad = np.pad(v, (k // 2, k // 2), mode="edge")
    return t, np.convolve(pad, np.ones(k) / k, mode="valid")


def homography_image_to_world(calib, bumper_offset_m):
    """image pixel -> (X forward, Y left) metres on the ground plane."""
    K = np.array(calib["camera_intrinsic"], dtype=np.float64)
    R = quaternion_to_rotation(calib["rotation"])          # camera -> ego
    t = np.array(calib["translation"], dtype=np.float64)   # camera origin in ego

    Rt = R.T
    # Ground plane z = 0 in ego frame: [X, Y, 1] -> camera ray -> pixel.
    G = K @ np.column_stack([Rt[:, 0], Rt[:, 1], -Rt @ t])
    H = np.linalg.inv(G)

    # Shift the world origin from the rear axle to the front bumper.
    T = np.array([[1.0, 0.0, -bumper_offset_m],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0]])
    H = T @ H
    return H / H[2, 2]


def write_h_yaml(path: Path, H):
    rows = ",\n".join(
        ("  data: [ " if i == 0 else "          ") + ", ".join(f"{v:.15e}" for v in H[i])
        for i in range(3)
    ) + " ]"
    path.write_text(
        "%YAML:1.0\n---\n"
        "# Camera homography transform matrix (image px -> world X fwd / Y left, metres)\n"
        "# Auto-generated from nuScenes CAM_FRONT intrinsics + extrinsics.\n"
        "H: !!opencv-matrix\n  rows: 3\n  cols: 3\n  dt: d\n"
        + rows + "\n"
    )


def encode_video(frame_paths, fps, out_path: Path, quality):
    """Encode an explicit frame list at a constant fps with ffmpeg."""
    list_file = out_path.with_suffix(".txt")
    with open(list_file, "w") as f:
        for p in frame_paths:
            f.write(f"file '{p}'\nduration {1.0 / fps:.6f}\n")
        f.write(f"file '{frame_paths[-1]}'\n")  # concat demuxer needs a final entry

    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "concat", "-safe", "0", "-i", str(list_file),
        "-vsync", "cfr", "-r", str(fps),
        "-c:v", "libx264", "-preset", "medium", "-crf", str(quality),
        "-pix_fmt", "yuv420p", str(out_path),
    ]
    subprocess.run(cmd, check=True)
    list_file.unlink()


def convert_scene(nusc, scene, out_dir: Path, fps, bumper_offset_m, quality):
    name = scene["name"]
    cam = nusc.scene_sample_data(scene, channel="CAM_FRONT")
    if not cam:
        print(f"  {name}: no CAM_FRONT data, skipping")
        return None

    src_t = np.array([sd["timestamp"] for sd in cam], dtype=np.float64) / 1e6
    src_t -= src_t[0]
    duration = src_t[-1]

    # Nearest-neighbour resample onto a constant fps grid. Keeps wall-clock
    # timing correct (nuScenes cameras run at ~12 Hz), so a 30 fps video plays
    # back at true speed with some frames repeated.
    n_out = int(np.floor(duration * fps)) + 1
    out_t = np.arange(n_out) / fps
    idx = np.abs(src_t[None, :] - out_t[:, None]).argmin(axis=1)

    frame_paths = [str(nusc.root / cam[i]["filename"]) for i in idx]
    missing = [p for p in frame_paths[:5] if not os.path.exists(p)]
    if missing:
        sys.exit(f"error: image file not found: {missing[0]}")

    video_path = out_dir / f"{name}.mp4"
    encode_video(frame_paths, fps, video_path, quality)

    # Ego speed on the camera's own timeline, interpolated onto the frame grid.
    cam_poses = [nusc.ego_pose[sd["ego_pose_token"]] for sd in cam]
    ego_t = np.array([p["timestamp"] for p in cam_poses], dtype=np.float64) / 1e6
    ego_t -= ego_t[0]
    ego_xy = np.array([p["translation"][:2] for p in cam_poses], dtype=np.float64)
    speeds = np.interp(out_t, *speed_profile(ego_t, ego_xy))

    speed_path = out_dir / f"{name}_speed.txt"
    # A few spare lines: VisionPilot reads one speed per inference frame and
    # throws if it ever runs past the end of the file.
    speed_path.write_text("\n".join(f"{v:.4f}" for v in speeds)
                          + "\n" + f"{speeds[-1]:.4f}\n" * 10)

    calib = nusc.calib[cam[0]["calibrated_sensor_token"]]
    H = homography_image_to_world(calib, bumper_offset_m)
    h_path = out_dir / f"{name}_H.yaml"
    write_h_yaml(h_path, H)

    print(f"  {name}: {n_out} frames @ {fps} fps ({duration:.1f}s), "
          f"speed {speeds.mean():.1f} m/s mean / {speeds.max():.1f} max, "
          f"{cam[0]['width']}x{cam[0]['height']}")
    return {"name": name, "video": video_path, "speed": speed_path,
            "H": h_path, "frames": n_out, "mean_speed": float(speeds.mean())}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, required=True, help="nuScenes root directory")
    ap.add_argument("--version", default="v1.0-mini", help="metadata subdirectory")
    ap.add_argument("--out", type=Path, required=True, help="output directory")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--num-scenes", type=int, default=3,
                    help="how many scenes to convert when --scenes is not given "
                         "(picks the fastest-moving ones)")
    ap.add_argument("--scenes", nargs="*", default=None, help="explicit scene names")
    ap.add_argument("--bumper-offset", type=float, default=DEFAULT_BUMPER_OFFSET_M,
                    help="rear axle -> front bumper distance, metres")
    ap.add_argument("--crf", type=int, default=18, help="x264 quality (lower = better)")
    args = ap.parse_args()

    if shutil.which("ffmpeg") is None:
        sys.exit("error: ffmpeg not found on PATH")

    nusc = NuScenes(args.root, args.version)
    args.out.mkdir(parents=True, exist_ok=True)

    if args.scenes:
        by_name = {s["name"]: s for s in nusc.scenes}
        unknown = [n for n in args.scenes if n not in by_name]
        if unknown:
            sys.exit(f"error: unknown scene(s): {unknown}\n"
                     f"available: {sorted(by_name)}")
        selected = [by_name[n] for n in args.scenes]
    else:
        ranked = []
        for scene in nusc.scenes:
            t, xy = nusc.ego_track(scene)
            ranked.append((speed_profile(t, xy)[1].mean(), scene))
        ranked.sort(key=lambda r: -r[0])
        selected = [s for _, s in ranked[:args.num_scenes]]
        print("Selected the fastest-moving scenes:")
        for v, s in ranked[:args.num_scenes]:
            print(f"  {s['name']}  mean {v:.1f} m/s  — {s['description']}")

    print(f"\nConverting {len(selected)} scene(s) -> {args.out}")
    results = [convert_scene(nusc, s, args.out, args.fps, args.bumper_offset, args.crf)
               for s in selected]

    print("\nDone:")
    for r in filter(None, results):
        print(f"  {r['video']}  ({r['frames']} frames)")


if __name__ == "__main__":
    main()
