# scripts

Grouped by what you are trying to do, because the flat list had grown to the
point where finding the right script took longer than running it.

| folder | for |
| --- | --- |
| [build/](build/) | set the machine up, compile models, build the map — the things you do once |
| [run/](run/) | drive: inference on a clip or a live camera, with or without the map panel |
| [sensors/](sensors/) | the hardware: flash the Nucleo, check the GPS/IMU stream, view the IMU |
| [docker/](docker/) | build and run the container |

Every script takes `--help`, and each one prints the exact command that fixes
whatever it refuses to do.

## First time on this machine

```bash
./scripts/build/setup.sh              # packages, ONNX Runtime, build. One command.
./scripts/build/build_osm_map.sh      # the offline Bandung map, once
./scripts/run/run_inference.sh scene-0796
```

## Driving

```bash
./scripts/run/inference_cam.sh                    # live camera, HUD only
./scripts/run/inference_cam_nav.sh                # + map panel, position from a phone
./scripts/run/inference_cam_nav_imu.sh            # + IMU on the PC, fused on the host
./scripts/run/inference_cam_nav_imu_gps_ddpai.sh   # the DDPai dashcam + Nucleo GPS/IMU
```

They differ in where the picture and the position come from:

| script | camera | position source | rate | heading when stopped |
| --- | --- | --- | --- | --- |
| `inference_cam.sh` | phone / any URL | none | — | — |
| `inference_cam_nav.sh` | phone / any URL | phone (phyphox / IP Webcam) | ~1 Hz | lost |
| `inference_cam_nav_imu.sh` | phone / any URL | phone GPS + IMU on the PC, host EKF | IMU rate | held |
| `inference_cam_nav_imu_gps_ddpai.sh` | DDPai N5 dashcam | Nucleo: GNSS-RTK + IMU, EKF on the board | 20 Hz | held |

The last one pairs the dashcam with the Nucleo for a reason: the dashcam's Wi-Fi
has no internet and this machine can only be on one network at a time, so a
phone serving position over Wi-Fi is unreachable while the camera is connected.
Position has to arrive over USB — which is what the Nucleo does.

## The Nucleo (GNSS-RTK rover + IMU)

Both sensors are wired to a Nucleo-G474RE, which fuses them and publishes
positions over its ST-LINK USB port. Firmware lives in
[VisionPilot/modules/sensing/gps_rtk_dfrobot/](../VisionPilot/modules/sensing/gps_rtk_dfrobot/).

```bash
./scripts/sensors/flash_nucleo.sh --list     # what firmwares exist
./scripts/sensors/flash_nucleo.sh fused      # GPS + IMU fused on the board
./scripts/sensors/gps_imu_check.sh           # is it publishing? is the IMU in?
./scripts/run/inference_cam_nav_imu_gps_ddpai.sh   # drive on it
```

When something is wrong, the order that saves the most time:

1. `gps_imu_check.sh` — it names which of the four failure modes you have.
2. `flash_nucleo.sh sniff -m` — if the IMU is silent, this finds its UART pins.
3. `flash_nucleo.sh probe -m` — if the GPS answers `no-device`.
4. `flash_nucleo.sh rtklink -m` — if `base_id` stays 0 and you expected RTK.

## The DDPai N5 dashcam

Reached over Wi-Fi, not USB: the camera raises its own access point and serves
RTSP. It never appears as `/dev/video*` — it is not a UVC device, so no cable
makes it one. The stream URLs, confirmed on v8.0.0.4 firmware:

| lens | URL |
| --- | --- |
| front | `rtsp://193.168.0.1:554/liveRTSP/av4` |
| rear | `rtsp://193.168.0.1:554/liveRTSP/av5` |

One script does everything — joins the camera's Wi-Fi, opens the window, and
puts your own network back when you quit:

```bash
./scripts/sensors/ddpai_stream.sh                  # watch it
./scripts/sensors/ddpai_stream.sh --rear           # the rear lens
./scripts/sensors/ddpai_stream.sh --record drive.mp4
./scripts/sensors/ddpai_stream.sh --hud            # fps, dropped frames, uptime
./scripts/sensors/ddpai_stream.sh --udp            # lower delay, occasional tearing
```

The camera burns its name and a clock into the bottom of every frame. That is a
firmware OSD, drawn before encoding, so it cannot be switched off from here —
the strip carrying it is cropped away instead (`--keep-watermark` to keep it).
Turning the watermark off in the DDPai app removes it everywhere, including from
the inference input, which the crop cannot reach.

Joining the dashcam's AP takes this machine off its normal network, so it cannot
be combined with a phone streaming position over Wi-Fi — use the Nucleo for
position instead.
