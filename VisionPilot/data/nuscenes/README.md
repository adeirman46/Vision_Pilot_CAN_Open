# nuScenes clips for VisionPilot

Full guide: [`../../docs/nuscenes_howto.pdf`](../../docs/nuscenes_howto.pdf) · setup and build:
[`../../../README.md`](../../../README.md)

## Run inference

```bash
cd VisionPilot/scripts
./run_inference.sh scene-0796                         # clean HUD window
./run_inference.sh scene-1077 -- --debug-viz          # telemetry overlay
./run_inference.sh scene-0655 --fast -- --no-window   # headless
./run_inference.sh /path/to/any.mp4 --speed-const 25  # your own footage
```

`./run_inference.sh --help` lists every option. A bare clip name resolves against this
directory; any other path is used as-is.

## Clips

Converted from nuScenes v1.0-mini CAM_FRONT (1600x900), the three fastest-moving scenes.

| clip | frames | length | mean speed | scene |
|---|---|---|---|---|
| `scene-0796.mp4` | 588 | 19.6 s | 12.1 m/s | day, big street, overtaking car |
| `scene-1077.mp4` | 599 | 19.9 s | 12.6 m/s | night, big street, high speed |
| `scene-0655.mp4` | 594 | 19.8 s | 8.2 m/s | parking lot, bendy bus, jaywalker |

Each clip has two siblings, both picked up automatically by `run_inference.sh`:

- `*_speed.txt` — ego speed in m/s, one value per frame, differentiated from the
  scene's `ego_pose` trajectory.
- `*_H.yaml` — ground-plane homography (image px -> world X forward / Y left, metres,
  origin at the front bumper) derived from that scene's own CAM_FRONT intrinsics and
  extrinsics.

nuScenes cameras run at ~12 Hz. The clips are resampled onto a 30 fps constant grid by
nearest neighbour in wall-clock time, so playback speed is true to life with some frames
repeated.

## Convert more scenes

```bash
cd VisionPilot/scripts
/usr/bin/python3 nuscenes_to_visionpilot.py \
    --root ~/vision_pilot_data/nuscenes \
    --out ../data/nuscenes --num-scenes 5
```

The raw dataset lives outside the repo — `~/vision_pilot_data/nuscenes/` on the reference
machine.
