#!/usr/bin/env bash
#
# Run VisionPilot inference on any video file.
#
# Usage:
#   ./run_inference.sh <video.mp4> [options] [-- <VisionPilot args>]
#
# Options:
#   --speed <file>       vehicle speed track, one value in m/s per frame.
#                        Default: <video>_speed.txt next to the video, else a
#                        constant track (see --speed-const).
#   --speed-const <m/s>  constant speed to synthesise when no track exists (15).
#   --calib <H.yaml>     camera homography (image px -> world X fwd / Y left, m).
#                        Default: <video>_H.yaml next to the video, else keep
#                        whatever is currently in VisionPilot/config/H.yaml.
#   --loop               replay the video in a loop.
#   --fast               ignore the video frame rate, run as fast as possible.
#   --precision <p>      which weights to run:
#                          default  restore VisionPilot's shipped config verbatim
#                                   (fp32 weights *and* its engine.provider)
#                          fp32     fp32 weights, leaving the provider alone
#                          int8     quantised models — selects the tensorrt provider,
#                                   since that is the only one with real int8 kernels
#                        Omit to leave vision_pilot.conf exactly as it is.
#   --provider <p>       execution provider: cpu, cuda or tensorrt. Selected automatically
#                        (tensorrt) by --precision int8 unless given explicitly.
#   --allow-build        let this run compile missing TensorRT engines itself. Off by
#                        default: inference should run, not build. Use build_models.sh.
#
# Everything after `--` is forwarded to the VisionPilot binary:
#   --debug-viz          annotated telemetry overlay instead of the clean HUD
#   --no-window          headless, console output only
#
# A bare scene name resolves against data/nuscenes/, so these are equivalent:
#   ./run_inference.sh scene-0796
#   ./run_inference.sh ../data/nuscenes/scene-0796.mp4
#
# Examples:
#   ./run_inference.sh scene-0796
#   ./run_inference.sh scene-1077 -- --debug-viz
#   ./run_inference.sh /path/to/my_dashcam.mp4 --speed-const 25
#   ./run_inference.sh scene-0796 --precision int8 --provider tensorrt
#   ./run_inference.sh scene-0796 --precision fp32
#
# Env overrides: VP_ROOT (VisionPilot source dir), VP_BUILD (build dir)
set -euo pipefail

VP_ROOT="${VP_ROOT:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}"
BUILD_DIR="${VP_BUILD:-$VP_ROOT/build}"
CLIP_DIR="$VP_ROOT/data/nuscenes"

usage() { sed -n '2,30p' "$0"; exit "${1:-1}"; }

[ $# -ge 1 ] || usage
case "$1" in -h|--help) usage 0 ;; esac

# Accept a bare clip name as shorthand for the bundled nuScenes clips.
if [ -f "$1" ]; then
    VIDEO="$(readlink -f "$1")"
elif [ -f "$CLIP_DIR/${1%.mp4}.mp4" ]; then
    VIDEO="$(readlink -f "$CLIP_DIR/${1%.mp4}.mp4")"
else
    echo "error: video not found: $1" >&2
    echo "       bundled clips: $(ls "$CLIP_DIR"/*.mp4 2>/dev/null | xargs -n1 basename 2>/dev/null | tr '\n' ' ')" >&2
    exit 1
fi
shift

SPEED=""
SPEED_CONST="15"
CALIB=""
LOOP="false"
REALTIME="true"
PRECISION=""
PROVIDER=""
ALLOW_BUILD="false"
VP_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --speed)       [ $# -ge 2 ] || { echo "error: --speed needs a value" >&2; exit 1; }
                       SPEED="$(readlink -f "$2")"; shift 2 ;;
        --speed-const) [ $# -ge 2 ] || { echo "error: --speed-const needs a value" >&2; exit 1; }
                       SPEED_CONST="$2"; shift 2 ;;
        --calib)       [ $# -ge 2 ] || { echo "error: --calib needs a value" >&2; exit 1; }
                       CALIB="$(readlink -f "$2")"; shift 2 ;;
        --loop)        LOOP="true"; shift ;;
        --fast)        REALTIME="false"; shift ;;
        --precision)   [ $# -ge 2 ] || { echo "error: --precision needs a value" >&2; exit 1; }
                       case "$2" in
                           default|fp32|int8) PRECISION="$2" ;;
                           *) echo "error: --precision must be default, fp32 or int8" >&2; exit 1 ;;
                       esac
                       shift 2 ;;
        --allow-build) ALLOW_BUILD="true"; shift ;;
        --provider)    [ $# -ge 2 ] || { echo "error: --provider needs a value" >&2; exit 1; }
                       case "$2" in cpu|cuda|tensorrt) ;; *) echo "error: --provider must be cpu, cuda or tensorrt" >&2; exit 1 ;; esac
                       PROVIDER="$2"; shift 2 ;;
        -h|--help)     usage 0 ;;
        --)            shift; VP_ARGS=("$@"); break ;;
        *)             echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

[ -f "$VIDEO" ] || { echo "error: video not found: $VIDEO" >&2; exit 1; }
[ -x "$BUILD_DIR/VisionPilot" ] || {
    echo "error: $BUILD_DIR/VisionPilot not built" >&2
    echo "       Set the machine up and build it in one step:" >&2
    echo "         $(cd "$VP_ROOT/.." && pwd)/setup.sh" >&2
    exit 1; }

BASE="${VIDEO%.*}"
WORK="$BUILD_DIR/run_inputs"
mkdir -p "$WORK"

FRAMES="$(ffprobe -v error -select_streams v:0 -count_frames \
          -show_entries stream=nb_read_frames -of csv=p=0 "$VIDEO" 2>/dev/null || echo 0)"
FPS="$(ffprobe -v error -select_streams v:0 -show_entries stream=r_frame_rate \
       -of csv=p=0 "$VIDEO" 2>/dev/null || echo "?")"
[ "$FRAMES" -gt 0 ] 2>/dev/null || FRAMES=100000

# ── Speed track ──────────────────────────────────────────────────────────────
# VisionPilot reads one value per inference frame and aborts if it runs past
# the end of the file, so always keep a margin.
if [ -z "$SPEED" ] && [ -f "${BASE}_speed.txt" ]; then
    SPEED="${BASE}_speed.txt"
fi
if [ -n "$SPEED" ]; then
    HAVE="$(grep -c '[0-9]' "$SPEED" || true)"
    if [ "$HAVE" -lt "$FRAMES" ]; then
        echo "note: $SPEED has $HAVE values for $FRAMES frames — padding with the last value"
        LAST="$(grep '[0-9]' "$SPEED" | tail -1)"
        cp "$SPEED" "$WORK/speed.txt"
        for _ in $(seq 1 $((FRAMES - HAVE + 10))); do echo "$LAST"; done >> "$WORK/speed.txt"
        SPEED="$WORK/speed.txt"
    fi
    SPEED_DESC="$SPEED"
else
    SPEED="$WORK/speed.txt"
    for _ in $(seq 1 $((FRAMES + 10))); do echo "$SPEED_CONST"; done > "$SPEED"
    SPEED_DESC="synthesised constant ${SPEED_CONST} m/s"
fi

# ── Camera calibration ───────────────────────────────────────────────────────
# The preprocessor warps each frame to the canonical bird's-eye view through a
# C matrix derived from H, so H has to be installed *and* C regenerated.
if [ -z "$CALIB" ] && [ -f "${BASE}_H.yaml" ]; then
    CALIB="${BASE}_H.yaml"
fi
if [ -n "$CALIB" ]; then
    [ -f "$CALIB" ] || { echo "error: calibration not found: $CALIB" >&2; exit 1; }
    cp "$CALIB" "$VP_ROOT/config/H.yaml"
    cp "$CALIB" "$BUILD_DIR/config/H.yaml"
    CALIB_DESC="$CALIB"
else
    CALIB_DESC="$VP_ROOT/config/H.yaml (unchanged — may not match this camera)"
fi
( cd "$BUILD_DIR" && /usr/bin/python3 "$VP_ROOT/scripts/find_homography_C_matrix.py" \
    --output "$BUILD_DIR/config/homography_C_matrix.yaml" >/dev/null )

# ── Model precision / execution provider ─────────────────────────────────────
# These live in vision_pilot.conf, which CMake copies into build/config/ — the copy
# the binary actually reads. Only touch it when asked, so a hand-edited config is
# left alone by default.
MAIN_CONF="$BUILD_DIR/config/vision_pilot.conf"
SRC_CONF="$VP_ROOT/config/vision_pilot.conf"
if [ -n "$PRECISION" ] || [ -n "$PROVIDER" ]; then
    [ -f "$MAIN_CONF" ] || { echo "error: $MAIN_CONF not found — run cmake first" >&2; exit 1; }

    if [ "$PRECISION" = "default" ]; then
        # "default" means VisionPilot's shipped settings, engine included — restore the
        # whole file from config/vision_pilot.conf rather than only swapping the weights,
        # so a provider left over from an int8 run does not linger.
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
PRECISION_DESC="$(awk -F= '/^model\.precision  *=/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF" 2>/dev/null)"
PROVIDER_DESC="$(awk -F= '/^engine\.provider/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF" 2>/dev/null)"
AD_DESC="$(awk -F= '/^model\.precision\.autodrive/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF" 2>/dev/null)"
[ -n "$AD_DESC" ] && PRECISION_DESC="$PRECISION_DESC (AutoDrive $AD_DESC)"

if [ "$PRECISION_DESC" != "${PRECISION_DESC#int8}" ] && [ "$PROVIDER_DESC" != "tensorrt" ]; then
    echo "note: int8 weights under the '$PROVIDER_DESC' provider are dequantised, not run as"
    echo "      int8 kernels — expect it to be slower than fp32. Add --provider tensorrt."
fi
# Inference should only ever run inference. Compiling TensorRT engines takes minutes and
# several GB of RAM, and doing it inside this process has been enough to get the machine
# OOM-killed — so if the engines are not cached yet, stop and say so instead.
if [ "$PROVIDER_DESC" = "tensorrt" ]; then
    CACHE_DIR="$(awk -F= '/^engine\.cache_dir/ {gsub(/^[ \t]+|[ \t]+$/,"",$2); print $2}' "$MAIN_CONF")"
    CACHE_DIR="${CACHE_DIR/#\~/$HOME}"
    CACHE_DIR="${CACHE_DIR:-/tmp/visionpilot_trt_cache}"
    WANT="${PRECISION_DESC%% *}"
    # Counted with a glob rather than `ls | wc -l`: under `set -e -o pipefail` a failing
    # ls (no cache directory at all) would abort the script with no message.
    CACHED=0
    for engine in "$CACHE_DIR"/*"_${WANT}_"*.engine; do
        [ -e "$engine" ] && CACHED=$((CACHED + 1))
    done
    if [ "$CACHED" -lt 3 ] && [ "$ALLOW_BUILD" = "false" ]; then
        echo "error: only $CACHED of 3 $WANT TensorRT engines are cached in $CACHE_DIR" >&2
        echo "       Build them once, one model per process:" >&2
        echo "         ./build_models.sh --precision $WANT" >&2
        echo "       Then this script starts in seconds. To compile inside this run anyway" >&2
        echo "       (minutes, several GB of RAM), pass --allow-build." >&2
        exit 1
    fi
    if [ "$CACHED" -lt 3 ]; then
        echo "note: only $CACHED of 3 $WANT engines cached — compiling the rest in-process"
    fi
fi

# ── Test config ──────────────────────────────────────────────────────────────
cat > "$BUILD_DIR/config/vision_pilot_test.conf" <<EOF
# Generated by run_inference.sh

source.video_realtime      = $REALTIME
source.video_loop          = $LOOP

source.input_video         = $VIDEO
source.input_vehicle_speed = $SPEED
source.dataset             = nuscenes
EOF

echo "=================================================="
echo " VisionPilot inference"
echo "   video: $VIDEO"
echo "   size:  $FRAMES frames @ $FPS fps"
echo "   speed: $SPEED_DESC"
echo "   calib: $CALIB_DESC"
echo "   model: $PRECISION_DESC on $PROVIDER_DESC"
echo "   extra: ${VP_ARGS[*]:-<none>}"
echo "=================================================="

cd "$BUILD_DIR"
exec ./VisionPilot ${VP_ARGS[@]+"${VP_ARGS[@]}"}
