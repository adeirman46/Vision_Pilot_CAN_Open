#!/usr/bin/env bash
#
# Build the TensorRT engines VisionPilot runs on, one model per process.
#
# Do this once per precision before running inference. TensorRT compiles each ONNX model
# into a GPU-specific engine the first time it is used; that compile takes minutes and
# several GB of RAM, and doing all three models inside one process has been enough to get
# it OOM-killed on a 16 GB laptop. This script builds them one at a time, under a memory
# cap when systemd is available, and caches the result — after which run_inference.sh
# starts in seconds.
#
# Usage:
#   ./build_models.sh                     # int8 engines (the reason to pre-build)
#   ./build_models.sh --precision fp32    # fp32 engines
#   ./build_models.sh --model autosteer   # one model only
#   ./build_models.sh --mem-max 8G        # tighter cap on the builder
#   ./build_models.sh --clean             # discard cached engines first
#
# On success the build config is left on that precision with the tensorrt provider, so
#   ./run_inference.sh <clip>
# runs straight away. Switch back to the shipped fp32/cuda setup any time with
#   ./run_inference.sh <clip> --precision default
#
# Env overrides: VP_ROOT (VisionPilot source dir), VP_BUILD (build dir)
set -euo pipefail

VP_ROOT="${VP_ROOT:-$(cd "$(dirname "$(readlink -f "$0")")/../../VisionPilot" && pwd)}"
BUILD_DIR="${VP_BUILD:-$VP_ROOT/build}"
CLIP_DIR="$VP_ROOT/data/nuscenes"

usage() { sed -n '2,27p' "$0"; exit "${1:-1}"; }

PRECISION="int8"
MODEL="all"
MEM_MAX="9G"
CLEAN="false"

while [ $# -gt 0 ]; do
    case "$1" in
        --precision) [ $# -ge 2 ] || { echo "error: --precision needs a value" >&2; exit 1; }
                     case "$2" in fp32|int8) ;; *) echo "error: --precision must be fp32 or int8" >&2; exit 1 ;; esac
                     PRECISION="$2"; shift 2 ;;
        --model)     [ $# -ge 2 ] || { echo "error: --model needs a value" >&2; exit 1; }
                     case "$2" in all|autodrive|autosteer|autospeed) ;; *) echo "error: --model must be all, autodrive, autosteer or autospeed" >&2; exit 1 ;; esac
                     MODEL="$2"; shift 2 ;;
        --mem-max)   [ $# -ge 2 ] || { echo "error: --mem-max needs a value" >&2; exit 1; }
                     MEM_MAX="$2"; shift 2 ;;
        --clean)     CLEAN="true"; shift ;;
        -h|--help)   usage 0 ;;
        *)           echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

TOOL="$BUILD_DIR/build_trt_engines"
[ -x "$TOOL" ] || {
    echo "error: $TOOL not built." >&2
    echo "       Set the machine up and build it in one step:" >&2
    echo "         $(cd "$VP_ROOT/.." && pwd)/scripts/build/setup.sh" >&2
    exit 1; }

MAIN_CONF="$BUILD_DIR/config/vision_pilot.conf"
[ -f "$MAIN_CONF" ] || { echo "error: $MAIN_CONF not found — run cmake first" >&2; exit 1; }

# Engines are only produced by the tensorrt provider, and the weights file is chosen by
# model.precision, so both have to be set before the tool runs.
sed -i -e "s|^engine\.provider  *=.*|engine.provider     = tensorrt|" \
       -e "s|^model\.precision  *=.*|model.precision    = $PRECISION|" \
       -e "/^model\.precision\.autodrive/d" "$MAIN_CONF"

# load_vision_pilot_config() validates the video paths in vision_pilot_test.conf even
# though engine building never reads a frame, so make sure it points at something real.
TEST_CONF="$BUILD_DIR/config/vision_pilot_test.conf"
NEED_TEST_CONF="true"
if [ -f "$TEST_CONF" ]; then
    VID="$(awk -F= '/^source\.input_video/ {gsub(/^[ \t]+|[ \t]+$/,"",$2); print $2}' "$TEST_CONF")"
    SPD="$(awk -F= '/^source\.input_vehicle_speed/ {gsub(/^[ \t]+|[ \t]+$/,"",$2); print $2}' "$TEST_CONF")"
    [ -f "${VID:-/nonexistent}" ] && [ -f "${SPD:-/nonexistent}" ] && NEED_TEST_CONF="false"
fi
if [ "$NEED_TEST_CONF" = "true" ]; then
    CLIP="$(ls "$CLIP_DIR"/*.mp4 2>/dev/null | head -1 || true)"
    [ -n "$CLIP" ] || { echo "error: no clips in $CLIP_DIR to point the config at" >&2; exit 1; }
    SPEED="${CLIP%.mp4}_speed.txt"
    [ -f "$SPEED" ] || { echo "error: $SPEED missing" >&2; exit 1; }
    cat > "$TEST_CONF" <<EOF
# Placeholder written by build_models.sh — run_inference.sh overwrites this per run.
source.video_realtime      = false
source.video_loop          = false

source.input_video         = $CLIP
source.input_vehicle_speed = $SPEED
source.dataset             = nuscenes
EOF
fi

CACHE_DIR="$(awk -F= '/^engine\.cache_dir/ {gsub(/^[ \t]+|[ \t]+$/,"",$2); print $2}' "$MAIN_CONF")"
CACHE_DIR="${CACHE_DIR:-../trt_cache}"
CACHE_DIR="${CACHE_DIR/#\~/$HOME}"
# A relative cache_dir is resolved against the build directory, so it means the same thing
# here as it does to build_trt_engines, which runs with that as its cwd.
case "$CACHE_DIR" in /*) ;; *) CACHE_DIR="$BUILD_DIR/$CACHE_DIR" ;; esac

if [ "$CLEAN" = "true" ]; then
    echo "removing cached ${PRECISION} engines in $CACHE_DIR"
    rm -f "$CACHE_DIR"/*"_${PRECISION}_"*.engine "$CACHE_DIR"/*"_${PRECISION}_"*.profile
fi

# A memory cap turns a runaway builder into one failed process instead of a frozen
# machine. Without systemd we simply run unconstrained.
RUNNER=()
CAP_DESC="none"
if systemd-run --user --scope -q -p MemoryMax=64M true >/dev/null 2>&1; then
    RUNNER=(systemd-run --user --scope -q -p "MemoryMax=$MEM_MAX")
    CAP_DESC="$MEM_MAX (systemd scope)"
else
    echo "note: systemd-run unavailable — building without a memory cap"
fi

echo "=================================================="
echo " VisionPilot TensorRT engine build"
echo "=================================================="
echo " precision:  $PRECISION"
echo " model(s):   $MODEL"
echo " cache dir:  $CACHE_DIR"
echo " memory cap: $CAP_DESC"
echo "=================================================="
echo "This takes a few minutes per model the first time. Engines are cached afterwards."

cd "$BUILD_DIR"
if ! "${RUNNER[@]}" ./build_trt_engines "$MODEL"; then
    echo
    echo "error: engine build failed." >&2
    echo "       If it was killed, raise --mem-max or close other applications." >&2
    echo "       If TensorRT rejected the graph, regenerate the weights with" >&2
    echo "       scripts/quantize_models.py — see the precision section of README.md." >&2
    exit 1
fi

echo
echo "cached engines in $CACHE_DIR:"
ls -1sh "$CACHE_DIR"/*.engine 2>/dev/null | sed 's|^|  |' || echo "  (none — nothing was cached)"
echo
echo "config left at: $(awk -F= '/^model\.precision  *=/ {gsub(/ /,"",$2); print "precision="$2}' "$MAIN_CONF") $(awk -F= '/^engine\.provider/ {gsub(/ /,"",$2); print "provider="$2}' "$MAIN_CONF")"
echo "now run:  ./run_inference.sh <clip>"
