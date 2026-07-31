#!/usr/bin/env bash
#
# Run VisionPilot inference on a live camera instead of a recorded clip.
#
# Usage:
#   ./inference_cam.sh [source] [options] [-- <VisionPilot args>]
#
# The source is either a stream URL or a local camera:
#   http://<ip>:8080/video   phone running IP Webcam (Android) — the default form
#   rtsp://<host>/<path>     any RTSP camera
#   0 | /dev/video0          a USB/laptop camera, i.e. the cv2.VideoCapture(0) case
# Default: $VP_CAM_URL, else http://10.131.42.214:8080/video
#
# Options:
#   --speed-const <m/s>  ego speed to feed the planner (15). A live camera carries no
#                        speed signal, so one constant value is synthesised per frame.
#   --hours <h>          how long the synthesised speed track must last (6). The run
#                        stops when it is exhausted.
#   --calib <H.yaml>     camera homography (image px -> world X fwd / Y left, m).
#                        Default: keep whatever is in VisionPilot/config/H.yaml — which
#                        is the nuScenes front camera, NOT your phone. Distances and the
#                        bird's-eye warp are only right once this matches your lens.
#   --precision <p>      which weights to run: default | fp32 | int8 (see run_inference.sh)
#   --provider <p>       execution provider: cpu, cuda or tensorrt.
#   --probe              just check the stream opens, print its resolution, and exit.
#
# Everything after `--` is forwarded to the VisionPilot binary:
#   --debug-viz          annotated telemetry overlay instead of the clean HUD
#   --no-window          headless, console output only
#
# Examples:
#   ./inference_cam.sh                                   # the default phone stream
#   ./inference_cam.sh http://192.168.1.5:8080/video
#   ./inference_cam.sh 0                                 # local /dev/video0
#   ./inference_cam.sh --probe
#   ./inference_cam.sh --speed-const 0 -- --debug-viz
#
# Env overrides: VP_ROOT (VisionPilot source dir), VP_BUILD (build dir), VP_CAM_URL
set -euo pipefail

VP_ROOT="${VP_ROOT:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}"
BUILD_DIR="${VP_BUILD:-$VP_ROOT/build}"
DEFAULT_URL="${VP_CAM_URL:-http://10.142.152.101:8080/video}"

usage() { sed -n '2,38p' "$0"; exit "${1:-1}"; }

SOURCE=""
case "${1:-}" in
    -h|--help) usage 0 ;;
    ""|-*|--) ;;
    *) SOURCE="$1"; shift ;;
esac
SOURCE="${SOURCE:-$DEFAULT_URL}"

SPEED_CONST="15"
HOURS="6"
CALIB=""
PRECISION=""
PROVIDER=""
PROBE="false"
VP_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --speed-const) [ $# -ge 2 ] || { echo "error: --speed-const needs a value" >&2; exit 1; }
                       SPEED_CONST="$2"; shift 2 ;;
        --hours)       [ $# -ge 2 ] || { echo "error: --hours needs a value" >&2; exit 1; }
                       HOURS="$2"; shift 2 ;;
        --calib)       [ $# -ge 2 ] || { echo "error: --calib needs a value" >&2; exit 1; }
                       CALIB="$(readlink -f "$2")"; shift 2 ;;
        --precision)   [ $# -ge 2 ] || { echo "error: --precision needs a value" >&2; exit 1; }
                       case "$2" in
                           default|fp32|int8) PRECISION="$2" ;;
                           *) echo "error: --precision must be default, fp32 or int8" >&2; exit 1 ;;
                       esac
                       shift 2 ;;
        --provider)    [ $# -ge 2 ] || { echo "error: --provider needs a value" >&2; exit 1; }
                       case "$2" in cpu|cuda|tensorrt) ;; *) echo "error: --provider must be cpu, cuda or tensorrt" >&2; exit 1 ;; esac
                       PROVIDER="$2"; shift 2 ;;
        --probe)       PROBE="true"; shift ;;
        -h|--help)     usage 0 ;;
        --)            shift; VP_ARGS=("$@"); break ;;
        *)             echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

# ── Resolve the source ───────────────────────────────────────────────────────
# VisionPilot opens a URL directly (video mode) and a /dev/videoN device through its
# V4L2 interface, so a bare index like `0` becomes /dev/video0.
case "$SOURCE" in
    [0-9]*)                    [ -e "/dev/video$SOURCE" ] || { echo "error: /dev/video$SOURCE does not exist" >&2; exit 1; }
                               DEVICE="/dev/video$SOURCE"; MODE="v4l2" ;;
    /dev/video*)               [ -e "$SOURCE" ] || { echo "error: $SOURCE does not exist" >&2; exit 1; }
                               DEVICE="$SOURCE"; MODE="v4l2" ;;
    http://*|https://*|rtsp://*|rtmp://*|udp://*|tcp://*|rtp://*)
                               URL="$SOURCE"; MODE="stream" ;;
    *) echo "error: unrecognised source '$SOURCE' — expected a stream URL, /dev/videoN or an index" >&2; exit 1 ;;
esac

# ── Reachability check ───────────────────────────────────────────────────────
# A phone that went to sleep or moved to another network is the usual failure, and the
# error it produces deep inside FFmpeg is not obvious. Say it here instead.
if [ "$MODE" = "stream" ]; then
    HOSTPORT="${URL#*://}"; HOSTPORT="${HOSTPORT%%/*}"
    HOST="${HOSTPORT%%:*}"
    PORT="${HOSTPORT##*:}"; [ "$PORT" = "$HOST" ] && PORT=""
    case "$URL" in http://*|https://*)
        if command -v curl >/dev/null 2>&1; then
            # HEAD the server root, not the stream: the stream never ends, so any
            # request against it would only come back on the timeout.
            curl -s -m 5 -I -o /dev/null "http://$HOSTPORT/" || {
                echo "error: cannot reach $HOSTPORT" >&2
                echo "       Check the phone is awake, IP Webcam is started, and both are on" >&2
                echo "       the same network. The IP is shown at the bottom of the app screen." >&2
                exit 1; }
        fi ;;
    esac
    if command -v ffprobe >/dev/null 2>&1; then
        INFO="$(timeout 15 ffprobe -v error -select_streams v:0 \
                -show_entries stream=width,height -of csv=p=0 "$URL" 2>/dev/null | tr ',' 'x')"
    fi
    SOURCE_DESC="$URL${INFO:+  (${INFO})}"
else
    SOURCE_DESC="$DEVICE"
fi

if [ "$PROBE" = "true" ]; then
    echo "source: $SOURCE_DESC"
    [ "$MODE" = "stream" ] && [ -z "${INFO:-}" ] && echo "warning: ffprobe could not read a video stream from it"
    exit 0
fi

[ -x "$BUILD_DIR/VisionPilot" ] || {
    echo "error: $BUILD_DIR/VisionPilot not built" >&2
    echo "       Set the machine up and build it in one step:" >&2
    echo "         $(cd "$VP_ROOT/.." && pwd)/setup.sh" >&2
    exit 1; }

WORK="$BUILD_DIR/run_inputs"
mkdir -p "$WORK"

# ── Camera calibration ───────────────────────────────────────────────────────
# The preprocessor warps each frame to the canonical bird's-eye view through a C matrix
# derived from H, so H has to be installed *and* C regenerated.
if [ -n "$CALIB" ]; then
    [ -f "$CALIB" ] || { echo "error: calibration not found: $CALIB" >&2; exit 1; }
    cp "$CALIB" "$VP_ROOT/config/H.yaml"
    cp "$CALIB" "$BUILD_DIR/config/H.yaml"
    CALIB_DESC="$CALIB"
else
    CALIB_DESC="$VP_ROOT/config/H.yaml (unchanged — nuScenes CAM_FRONT, not your camera)"
fi
( cd "$BUILD_DIR" && /usr/bin/python3 "$VP_ROOT/scripts/find_homography_C_matrix.py" \
    --output "$BUILD_DIR/config/homography_C_matrix.yaml" >/dev/null )

# ── Model precision / execution provider ─────────────────────────────────────
MAIN_CONF="$BUILD_DIR/config/vision_pilot.conf"
SRC_CONF="$VP_ROOT/config/vision_pilot.conf"
[ -f "$MAIN_CONF" ] || { echo "error: $MAIN_CONF not found — run cmake first" >&2; exit 1; }

if [ -n "$PRECISION" ] || [ -n "$PROVIDER" ]; then
    if [ "$PRECISION" = "default" ]; then
        [ -f "$SRC_CONF" ] || { echo "error: $SRC_CONF not found" >&2; exit 1; }
        cp "$SRC_CONF" "$MAIN_CONF"
        PRECISION=""
    fi
    if [ -n "$PRECISION" ]; then
        sed -i -e "s|^model\.precision  *=.*|model.precision    = $PRECISION|" \
               -e "/^model\.precision\.autodrive/d" "$MAIN_CONF"
        # int8 under cuda is dequantised, not accelerated — default to tensorrt.
        if [ "$PRECISION" = "int8" ] && [ -z "$PROVIDER" ]; then
            PROVIDER="tensorrt"
        fi
    fi
    [ -n "$PROVIDER" ] && sed -i "s|^engine\.provider  *=.*|engine.provider     = $PROVIDER|" "$MAIN_CONF"
fi

# ── Input source ─────────────────────────────────────────────────────────────
# source.mode lives in the main config, not the test config, so it has to be rewritten
# in place. A URL runs through video mode (the same FileInterface, opened on a stream);
# a real device runs through v4l2 mode.
if [ "$MODE" = "v4l2" ]; then
    sed -i -e "s|^source\.mode  *=.*|source.mode             = v4l2|" \
           -e "s|^source\.v4l2_device  *=.*|source.v4l2_device      = $DEVICE|" "$MAIN_CONF"
else
    sed -i "s|^source\.mode  *=.*|source.mode             = video|" "$MAIN_CONF"
fi

PRECISION_DESC="$(awk -F= '/^model\.precision  *=/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF")"
PROVIDER_DESC="$(awk -F= '/^engine\.provider/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF")"
AD_DESC="$(awk -F= '/^model\.precision\.autodrive/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF")"
[ -n "$AD_DESC" ] && PRECISION_DESC="$PRECISION_DESC (AutoDrive $AD_DESC)"

if [ "$PRECISION_DESC" != "${PRECISION_DESC#int8}" ] && [ "$PROVIDER_DESC" != "tensorrt" ]; then
    echo "note: int8 weights under the '$PROVIDER_DESC' provider are dequantised, not run as"
    echo "      int8 kernels — expect it to be slower than fp32. Add --provider tensorrt."
fi
# Compiling TensorRT engines takes minutes and several GB of RAM; a live camera run is
# not the place for it. build_models.sh caches them, one model per process.
if [ "$PROVIDER_DESC" = "tensorrt" ]; then
    CACHE_DIR="$(awk -F= '/^engine\.cache_dir/ {gsub(/^[ \t]+|[ \t]+$/,"",$2); print $2}' "$MAIN_CONF")"
    CACHE_DIR="${CACHE_DIR:-../trt_cache}"
    CACHE_DIR="${CACHE_DIR/#\~/$HOME}"
    # A relative cache_dir is resolved against the build directory, so it means the same
    # thing here as it does to VisionPilot, which runs with that as its cwd.
    case "$CACHE_DIR" in /*) ;; *) CACHE_DIR="$BUILD_DIR/$CACHE_DIR" ;; esac
    WANT="${PRECISION_DESC%% *}"
    CACHED=0
    for engine in "$CACHE_DIR"/*"_${WANT}_"*.engine; do
        [ -e "$engine" ] && CACHED=$((CACHED + 1))
    done
    if [ "$CACHED" -lt 3 ]; then
        echo "error: only $CACHED of 3 $WANT TensorRT engines are cached in $CACHE_DIR" >&2
        echo "       Build them once: ./build_models.sh --precision $WANT" >&2
        exit 1
    fi
fi

# ── Test config (video mode only) ────────────────────────────────────────────
# A live camera carries no speed signal, and the planner reads one value per inference
# frame — it stops when the track runs out, so synthesise enough for --hours at 30 fps.
if [ "$MODE" = "stream" ]; then
    LINES=$(awk -v h="$HOURS" 'BEGIN { printf "%d", h * 3600 * 30 }')
    [ "$LINES" -gt 0 ] 2>/dev/null || { echo "error: --hours must be a positive number" >&2; exit 1; }
    SPEED="$WORK/cam_speed.txt"
    awk -v n="$LINES" -v v="$SPEED_CONST" 'BEGIN { for (i = 0; i < n; i++) print v }' > "$SPEED"

    cat > "$BUILD_DIR/config/vision_pilot_test.conf" <<EOF
# Generated by inference_cam.sh

source.video_realtime      = false
source.video_loop          = false

source.input_video         = $URL
source.input_vehicle_speed = $SPEED
source.dataset             = camera
EOF
    SPEED_DESC="synthesised constant ${SPEED_CONST} m/s (${HOURS} h)"
else
    # v4l2 mode reads speed from the CAN interface, which currently reports 0 m/s.
    SPEED_DESC="CAN interface (stub — reports 0 m/s)"
fi

echo "=================================================="
echo " VisionPilot live camera inference"
echo "   source: $SOURCE_DESC"
echo "   speed:  $SPEED_DESC"
echo "   calib:  $CALIB_DESC"
echo "   model:  $PRECISION_DESC on $PROVIDER_DESC"
echo "   extra:  ${VP_ARGS[*]:-<none>}"
echo "   stop with Ctrl-C"
echo "=================================================="

cd "$BUILD_DIR"
exec ./VisionPilot ${VP_ARGS[@]+"${VP_ARGS[@]}"}
