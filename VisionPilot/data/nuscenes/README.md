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

Converted from nuScenes v1.0-mini CAM_FRONT (1600x900), fastest-moving scene first.

| clip             | frames | length | mean speed | scene                                    |
| ---------------- | ------ | ------ | ---------- | ---------------------------------------- |
| `scene-1077.mp4` | 600    | 20.0 s | 12.6 m/s   | night, big street, high speed            |
| `scene-0796.mp4` | 589    | 19.6 s | 12.1 m/s   | day, big street, overtaking car          |
| `scene-0655.mp4` | 595    | 19.8 s | 8.2 m/s    | parking lot, bendy bus, jaywalker        |
| `scene-0103.mp4` | 584    | 19.5 s | 6.1 m/s    | many pedestrians, cyclist, turning car   |
| `scene-0061.mp4` | 576    | 19.2 s | 4.7 m/s    | construction, left turn, following a van |

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
