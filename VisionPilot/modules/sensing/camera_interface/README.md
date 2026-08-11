# V4L2 Interface Module

## I. Overview

This **V4L2 Interface** module provides a robust C++ wrapper for capturing video frames from V4L2 (Video4Linux2) camera devices and converting them to OpenCV `cv::Mat` format. 

This module is essential for low-level camera frame acquisition in VisionPilot 1.0, featuring:

- V4L2 camera capturing whose interface directly connects to V4L2 mounted devices (`/dev/video0`, `/dev/video1`, etc.)
- Seamless conversion to OpenCV `cv::Mat` for downstream processing
- Mutex-protected thread-safe frame and statistics
- Able to specify desired FPS and codec settings
- Monitor frames captured, dropped, and errors, etc.

## II. Architecture & module structure

### 1. Architecture

The `V4L2Reader` class is the main interface for V4L2 camera operations. It follows a similar pattern to the ROS2 camera subscriber but operates at a lower hardware level.

Basically, for the V4L2 capture, we use a simple, off-the-shelf yet effective [OpenCV's VideoCapture](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html) :

```cpp

cv::Mat frame;
camera_capture >> frame;  // Core OpenCV VideoCapture operation

```

This uses OpenCV's VideoCapture with the CAP_V4L2 backend for direct V4L2 device access.

```

┌─────────────────────────────────────────┐
│   Linux V4L2 Framework (/dev/videoX)    │
│   USB Video Device Driver               │
│   Camera Hardware                       │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│   OpenCV VideoCapture (CAP_V4L2)         │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│   V4L2 Interface Module                  │
│  ┌─────────────────────────────────────┐│
│  │  V4L2Reader Class                    ││
│  │  - get_latest_frame()                ││
│  │  - Statistics & monitoring           ││
│  └─────────────────────────────────────┘│
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│   VisionPilot Application Layer          │
└─────────────────────────────────────────┘

```

### 2. Module structure

```

v4l2_interface/
├── CMakeLists.txt
├── README.md
├── include/
│   └── v4l2_interface/
│       └── v4l2_reader.hpp
└── src/
    └── v4l2_reader.cpp

```

## III. Build

### 1. Prerequisites

- ROS2 (tested on ROS2 Humble / Ubuntu 22.04).
    - `source /opt/ros/humble/setup.bash`
- Required packages:
    - `OpenCV`

### 2. Steps

```bash

# 1. From root to release directory
cd VisionPilot/development_releases/1.0

# 2. Create build dir
mkdir -p build && cd build

# 3. Source ROS2 just in case you forgot to
source /opt/ros/humble/setup.bash

# 4. CMake configure
cmake ..

# 5. Compile
make -j$(nproc)

```

### 3. Expected output

```bash

[ 97%] Building CXX object app/CMakeFiles/VisionPilot.dir/vision_pilot.cpp.o
[100%] Linking CXX executable ../VisionPilot
[100%] Built target VisionPilot

```

## IV. Test

This module features V4L2 => OpenCV stream conversion, thus a V4L2 image stream is required.

You can do a simple test using a local video by simply just:

1. Publishing it as a V4L2 stream via [ffmpeg](https://ffmpeg.org/).

```bash

# 1. Install package
sudo apt update
sudo apt install ffmpeg -y
sudo apt install v4l2loopback-dkms -y

# 2. Load the module (assuming you gonna stream it at `/dev/video9`)
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback video_nr=9 card_label="Virtual Camera" exclusive_caps=1

# 3. Publish looping video at that mount
ffmpeg -re -stream_loop -1 -i <absolute path to local video> -f v4l2 -pix_fmt yuv420p /dev/video9

```

With this the V4L2 stream at `dev/video9` is now active. You can have a look at it from another terminal using:

```bash

ffplay /dev/video9

```

2. Then, on another terminal (or back to the terminal that you built VisionPilot), we can test it inside the main VisionPilot loop:

```bash

# First argument `1` means starting this with V4L2 input
# Second argument being mounted V4L2 stream
# Third argument being desired FPS
./VisionPilot 1 /dev/video9 10

```

## V. DDPai N5 dashcam

The N5 is not a V4L2 device and never will be. Its USB-C port carries power and,
on firmware that supports it, the SD card as mass storage — there is no USB Video
Class interface, so the kernel creates no `/dev/video*` node for it whatever
cable is used. `VideoCapture(0)`, `VideoCapture(1)` and the rest can only open
cameras the kernel already knows about.

The live view is served over the dashcam's own Wi-Fi access point, as RTSP. So
the way in is a URL, not a device index. Confirmed on firmware v8.0.0.4
(`DDPAI DN2508_Overseas`):

| lens | URL |
| --- | --- |
| front | `rtsp://193.168.0.1:554/liveRTSP/av4` |
| rear | `rtsp://193.168.0.1:554/liveRTSP/av5` |

`193.168.0.1` is not a typo for `192.168` — DDPai's AP really does hand out
`193.168.0.x`, with the camera itself as the gateway.

### The viewer

[`tools/ddpai_stream.py`](tools/ddpai_stream.py) is `cv2.VideoCapture` on that
URL with the parts a live link actually needs. One script drives it, joining the
camera's Wi-Fi and restoring yours afterwards:

```bash
./scripts/sensors/ddpai_stream.sh                  # watch it
./scripts/sensors/ddpai_stream.sh --rear           # the rear lens
./scripts/sensors/ddpai_stream.sh --record drive.mp4
./scripts/sensors/ddpai_stream.sh --hud            # fps, dropped frames, uptime
./scripts/sensors/ddpai_stream.sh --udp            # lower delay, occasional tearing
```

Keys: `q` quit, `s` snapshot, `r` record, `h` HUD, `f` fullscreen.

### Latency

Three things stand between the road and the window, and two of them are ours:

**The decoder.** FFmpeg defaults to frame-threaded H.264 decoding — 16 threads
on this machine — and a frame-threaded decoder does not emit a frame until
`threads - 1` more have been fed in. At 30 fps that is half a second of picture
sitting inside libavcodec before anything reaches OpenCV. Opening the capture
with `CAP_PROP_N_THREADS = 1` removes it outright; one thread decodes 720p far
faster than real time, so it costs nothing. This is why an MJPEG IP camera feels
snappier than an H.264 dashcam at the same resolution: MJPEG has no frame
threading to delay it.

**The demuxer queue.** `fflags nobuffer`, `flags low_delay`, `max_delay 0` and
`reorder_queue_size 0`, set through `OPENCV_FFMPEG_CAPTURE_OPTIONS` before the
capture is opened — OpenCV exposes no API for them.

**The read queue.** `read()` hands over the *oldest* frame still queued, so every
frame the UI cannot keep up with adds a permanent frame of lag. A grab thread
drains the stream and keeps only the newest, so the display is always handed the
present.

What remains is the camera's own encoder and the Wi-Fi link, which nothing on
this side can shorten. `--udp` trades retransmission stalls for occasional
tearing and is usually a little quicker than the default TCP.

### The watermark

The camera burns `DDPAI N5 Dual` and a clock into the bottom of every frame. It
is a firmware OSD drawn before encoding, so no decoder setting removes it — the
viewer crops the bottom 12% away instead, giving 1280x632 from a 1280x720
stream. `--keep-watermark` keeps the full frame.

That crop only applies to this viewer. **The inference pipeline opens the RTSP
URL itself and sees the watermark**, so if it matters there, turn the watermark
off in the DDPai app (camera settings) — that removes it at the source, for
everything.

### The camera's API

The app talks to the camera over a JSON API on port 80, `POST` to
`/vcam/cmd.cgi?cmd=<COMMAND>` with an optional `sessionid` header. It is not
needed to stream — the RTSP URLs above work on their own — but it is how the
camera is interrogated:

```bash
curl -s -X POST "http://193.168.0.1/vcam/cmd.cgi?cmd=API_GetBaseInfo"
curl -s -X POST "http://193.168.0.1/vcam/cmd.cgi?cmd=API_RequestSessionID"
curl -s -X POST "http://193.168.0.1/vcam/cmd.cgi?cmd=API_GetGpsState"
```

Responses are JSON with `errcode` 0 on success and `data` carrying a JSON
*string* that has to be parsed a second time. The endpoint list came from
[awexandrr/ddpai-api](https://github.com/awexandrr/ddpai-api).

Its HTTP server is fragile: a burst of requests, or a port scan, wedges it until
the camera is power-cycled. Query it gently.

### Feeding the pipeline

VisionPilot takes the URL directly — `FileInterface` already treats anything with
an `rtsp://` scheme as a live stream:

```conf
source.mode        = video
source.input_video = rtsp://193.168.0.1:554/liveRTSP/av4
```

Or run the whole stack, which joins the camera's Wi-Fi and takes position from
the Nucleo over USB:

```bash
./scripts/run/inference_cam_nav_imu_gps_ddpai.sh
```

Connecting to the dashcam's AP drops this machine off its normal network — the
camera does not route to the internet. That is why position for that run comes
from the Nucleo on USB and not from a phone over Wi-Fi: the two cannot both be
on their own access points.
