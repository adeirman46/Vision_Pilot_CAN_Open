#!/usr/bin/env bash
#
# One-command setup for Vision Pilot CAN Open.
#
# Installs the system packages, fixes the coin-or include path, fetches ONNX Runtime
# and builds VisionPilot — everything the README's Setup section does by hand. Run it
# once after cloning, then run inference:
#
#   ./setup.sh
#   VisionPilot/scripts/run_inference.sh scene-0796
#
# Usage:
#   ./setup.sh [options]
#
# Options:
#   --gpu | --cpu        force the CUDA/TensorRT build or the CPU-only build.
#                        Default: GPU when nvidia-smi reports a device.
#   --ort-version <v>    ONNX Runtime release to fetch (default 1.27.1).
#   --ort-root <dir>     use an ONNX Runtime already unpacked at <dir>; skip the download.
#   --ort-dir <dir>      where releases are unpacked (default ~/onnxruntime).
#   --jobs <n>           parallel compile jobs (default: cores, capped by free RAM).
#   --webrtc             also install the GStreamer/Boost/json stack, so the HUD can
#                        stream over WebRTC instead of opening a local window.
#   --ros2               build the ROS 2 camera/vehicle interfaces. Source ROS 2 first.
#   --engines <p>        after the build, compile the TensorRT engines for fp32 or int8
#                        (minutes per model; only needed for the tensorrt provider).
#   --skip-apt           do not touch system packages — assume they are already there.
#   --clean              delete VisionPilot/build before configuring. Fetched dependencies
#                        (build/_deps) are kept, so this costs a rebuild, not a re-download.
#   --verify             finish by running scene-0796 headless as a smoke test.
#   -h, --help           this message.
#
# Examples:
#   ./setup.sh                          # detect the GPU, install, build
#   ./setup.sh --cpu                    # CPU-only machine
#   ./setup.sh --skip-apt --clean       # rebuild from scratch, leave the system alone
#   ./setup.sh --engines int8 --verify  # full setup, int8 engines, smoke test
#
# Env overrides: VP_BUILD (build dir), ORT_DIR (ONNX Runtime install dir)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
VP_ROOT="$REPO_ROOT/VisionPilot"
BUILD_DIR="${VP_BUILD:-$VP_ROOT/build}"

usage() { sed -n '2,39p' "$0"; exit "${1:-1}"; }

ORT_VERSION="1.27.1"
ORT_DIR="${ORT_DIR:-$HOME/onnxruntime}"
ORT_ROOT=""
TARGET=""            # gpu | cpu — empty means autodetect
JOBS=""
WEBRTC="false"
ROS2="false"
ENGINES=""
SKIP_APT="false"
CLEAN="false"
VERIFY="false"

while [ $# -gt 0 ]; do
    case "$1" in
        --gpu)         TARGET="gpu"; shift ;;
        --cpu)         TARGET="cpu"; shift ;;
        --ort-version) [ $# -ge 2 ] || { echo "error: --ort-version needs a value" >&2; exit 1; }
                       ORT_VERSION="$2"; shift 2 ;;
        --ort-root)    [ $# -ge 2 ] || { echo "error: --ort-root needs a value" >&2; exit 1; }
                       ORT_ROOT="$(readlink -f "$2")"; shift 2 ;;
        --ort-dir)     [ $# -ge 2 ] || { echo "error: --ort-dir needs a value" >&2; exit 1; }
                       ORT_DIR="$2"; shift 2 ;;
        --jobs|-j)     [ $# -ge 2 ] || { echo "error: --jobs needs a value" >&2; exit 1; }
                       JOBS="$2"; shift 2 ;;
        --webrtc)      WEBRTC="true"; shift ;;
        --ros2)        ROS2="true"; shift ;;
        --engines)     [ $# -ge 2 ] || { echo "error: --engines needs a value" >&2; exit 1; }
                       case "$2" in fp32|int8) ;; *) echo "error: --engines must be fp32 or int8" >&2; exit 1 ;; esac
                       ENGINES="$2"; shift 2 ;;
        --skip-apt)    SKIP_APT="true"; shift ;;
        --clean)       CLEAN="true"; shift ;;
        --verify)      VERIFY="true"; shift ;;
        -h|--help)     usage 0 ;;
        *)             echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

if [ -t 1 ]; then B=$'\033[1m'; D=$'\033[0m'; else B=""; D=""; fi
step() { printf '\n%s==> %s%s\n' "$B" "$*" "$D"; }
note() { printf '    %s\n' "$*"; }
die()  { printf '\nerror: %s\n' "$*" >&2; exit 1; }

[ -d "$VP_ROOT" ] || die "$VP_ROOT not found — run setup.sh from inside the repository."

# Only the package install and the coin-or symlink need root, so sudo is checked where
# it is used — --skip-apt on a machine without sudo is a perfectly good way to run this.
SUDO=""
[ "$(id -u)" -eq 0 ] || SUDO="sudo"
need_root() {
    [ -n "$SUDO" ] && ! command -v sudo >/dev/null 2>&1 &&
        die "$1 needs root, and sudo was not found. Re-run as root, or pass --skip-apt."
    return 0
}

# ── What are we building for? ────────────────────────────────────────────────
# The driver's "CUDA Version:" is the highest release it supports, not what is
# installed, so the toolkit on disk decides which ONNX Runtime asset we need.
# `|| true` throughout: a no-match dpkg-query exits 1, and pipefail turns any
# early-exiting reader into a failed pipeline — either would abort under set -e.
cuda_major() {
    local d ver best=""
    for d in /usr/local/cuda-[0-9]*; do
        [ -d "$d" ] || continue
        ver="${d#/usr/local/cuda-}"
        case "$ver" in
            [0-9]*) best="$(printf '%s\n%s\n' "$best" "${ver%%.*}" | sort -n | tail -1)" ;;
        esac
    done
    if [ -z "$best" ]; then
        best="$(dpkg-query -W -f='${Package}\n' 'cuda-cudart-*' 2>/dev/null |
                sed -n 's/^cuda-cudart-\([0-9]\+\)-[0-9]\+$/\1/p' | sort -n | tail -1 || true)"
    fi
    if [ -z "$best" ]; then
        best="$(nvidia-smi 2>/dev/null | sed -n 's/.*CUDA Version: \([0-9]\+\)\..*/\1/p' |
                sort -u | tail -1 || true)"
    fi
    printf '%s' "$best"
}
# The exact toolkit on disk, e.g. 12-8, to match NVIDIA's per-minor package names.
cuda_minor_tag() {
    local d ver best=""
    for d in /usr/local/cuda-[0-9]*.[0-9]*; do
        [ -d "$d" ] || continue
        ver="${d#/usr/local/cuda-}"
        best="$(printf '%s\n%s\n' "$best" "$ver" | sort -V | tail -1)"
    done
    printf '%s' "${best:+${best/./-}}"
}

if [ -z "$TARGET" ]; then
    TARGET="cpu"
    if command -v nvidia-smi >/dev/null 2>&1; then
        case "$(nvidia-smi -L 2>/dev/null || true)" in *GPU*) TARGET="gpu" ;; esac
    fi
fi

CUDA_MAJOR=""
if [ "$TARGET" = "gpu" ]; then
    CUDA_MAJOR="$(cuda_major)"
    [ -n "$CUDA_MAJOR" ] || die "GPU build requested but no CUDA toolkit was found.
       Install CUDA 12 or 13, or set up for CPU with: ./setup.sh --cpu"
fi

JOBS_NOTE=""
if [ -z "$JOBS" ]; then
    JOBS="$(nproc)"
    # One heavy C++ translation unit per GB of free RAM. The planner's CppAD/Ipopt
    # templates are big enough that -j<cores> has been OOM-killed on a 16 GB laptop.
    AVAIL_GB="$(awk '/^MemAvailable:/ {printf "%d", $2/1048576}' /proc/meminfo 2>/dev/null || echo 0)"
    if [ "${AVAIL_GB:-0}" -gt 0 ] && [ "$AVAIL_GB" -lt "$JOBS" ]; then
        JOBS_NOTE="capped at $AVAIL_GB by ${AVAIL_GB} GB free RAM — override with --jobs"
        JOBS="$AVAIL_GB"
    fi
fi

step "Vision Pilot CAN Open — setup"
note "repository:  $REPO_ROOT"
note "target:      $TARGET$([ -n "$CUDA_MAJOR" ] && echo " (CUDA $CUDA_MAJOR)")"
note "build dir:   $BUILD_DIR"
note "compile:     -j$JOBS${JOBS_NOTE:+  ($JOBS_NOTE)}"
note "webrtc:      $WEBRTC     ros 2: $ROS2"

# ── 1. Submodules ────────────────────────────────────────────────────────────
step "1/6  Submodules"
if [ -f "$REPO_ROOT/.gitmodules" ] && [ -d "$REPO_ROOT/.git" ]; then
    git -C "$REPO_ROOT" submodule update --init --recursive
    note "up to date"
else
    note "no submodules to initialise"
fi

# ── 2. System packages ───────────────────────────────────────────────────────
step "2/6  System packages"

PKGS=(build-essential cmake git wget ca-certificates ffmpeg
      python3 python3-numpy python3-opencv
      libopencv-dev coinor-libipopt-dev libcppad-dev liblapack-dev libblas-dev)

if [ "$WEBRTC" = "true" ]; then
    PKGS+=(libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
           libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-good
           gstreamer1.0-plugins-bad gstreamer1.0-nice libnice-dev libsrtp2-dev
           libboost-system-dev nlohmann-json3-dev)
fi

# Highest-versioned package matching a name pattern, empty when none is available.
apt_pick() { apt-cache search --names-only "$1" 2>/dev/null | awk '{print $1}' | sort -V | tail -1 || true; }
# Matched against one captured string rather than piped into grep: under `set -o pipefail`
# a grep -q that exits on its first match makes the writer die of SIGPIPE, and the pipeline
# then reports 141 — a library that is present would read as missing.
LDCONFIG_CACHE="$(ldconfig -p 2>/dev/null || true)"
have_lib() { case "$LDCONFIG_CACHE" in *"$1"*) return 0 ;; *) return 1 ;; esac; }

MISSING_CUDA_LIBS=()
HAVE_TRT="false"
if [ "$TARGET" = "gpu" ]; then
    # ONNX Runtime's CUDA provider dlopens these; without them it fails at session
    # creation with "Failed to load libonnxruntime_providers_cuda.so".
    MINOR_TAG="$(cuda_minor_tag)"
    for lib in cudnn cufft curand; do
        have_lib "lib${lib}.so" && continue
        case "$lib" in
            cudnn) pkg="$(apt_pick "^libcudnn9-cuda-${CUDA_MAJOR}\$")" ;;
            *)     pkg=""
                   [ -n "${MINOR_TAG:-}" ] && pkg="$(apt_pick "^lib${lib}-${MINOR_TAG}\$")"
                   [ -n "$pkg" ] || pkg="$(apt_pick "^lib${lib}-${CUDA_MAJOR}-[0-9]+\$")" ;;
        esac
        if [ -n "$pkg" ]; then PKGS+=("$pkg"); else MISSING_CUDA_LIBS+=("lib${lib}"); fi
    done
    # TensorRT is optional — only engine.provider = tensorrt (and therefore int8) needs it.
    # Left to the user on purpose: the libnvinfer* candidate in NVIDIA's repo is usually
    # built for the newest CUDA, so installing it by name onto an older toolkit silently
    # mismatches. Pin the +cuda${CUDA_MAJOR} version by hand if you want it.
    have_lib "libnvinfer.so" && HAVE_TRT="true"
fi

if [ "$SKIP_APT" = "true" ]; then
    note "--skip-apt: leaving system packages alone"
else
    need_root "installing packages"
    note "apt-get install: ${PKGS[*]}"
    $SUDO apt-get update
    $SUDO apt-get install -y --no-install-recommends "${PKGS[@]}"
    $SUDO ldconfig
fi

if [ "${#MISSING_CUDA_LIBS[@]}" -gt 0 ]; then
    note "note: ${MISSING_CUDA_LIBS[*]} not installed and not in any configured apt repository."
    note "      Add NVIDIA's CUDA repo, or run on the cpu provider (./setup.sh --cpu)."
fi
if [ "$TARGET" = "gpu" ] && [ "$HAVE_TRT" = "false" ]; then
    note "note: no TensorRT (libnvinfer) found. The cuda provider works without it; the"
    note "      tensorrt provider and int8 weights do not. Install the libnvinfer* build"
    note "      tagged +cuda${CUDA_MAJOR} from NVIDIA's repo if you want them."
fi

# ── 3. coin-or include path ──────────────────────────────────────────────────
# CppAD's cppad/ipopt/solve.hpp includes <coin-or/...>; Ubuntu ships those headers in
# /usr/include/coin. Resolve wherever they actually are instead of hardcoding.
step "3/6  Ipopt headers"
if [ -f /usr/include/coin-or/IpIpoptApplication.hpp ]; then
    note "/usr/include/coin-or already resolves"
else
    HDR_DIR="$(dirname "$(find /usr/include -maxdepth 3 -name IpIpoptApplication.hpp 2>/dev/null | head -1)")"
    if [ ! -d "$HDR_DIR" ] || [ "$HDR_DIR" = "." ]; then
        die "IpIpoptApplication.hpp not found. Install it with:
       sudo apt install coinor-libipopt-dev"
    elif [ "$SKIP_APT" = "true" ]; then
        die "CppAD needs /usr/include/coin-or. Create it once with:
       sudo ln -sfn $HDR_DIR /usr/include/coin-or"
    else
        need_root "linking /usr/include/coin-or"
        $SUDO ln -sfn "$HDR_DIR" /usr/include/coin-or
        note "linked /usr/include/coin-or -> $HDR_DIR"
    fi
fi

# ── 4. ONNX Runtime ──────────────────────────────────────────────────────────
# Not installed system-wide: unpacked somewhere and handed to CMake as a path.
step "4/6  ONNX Runtime"
if [ "$TARGET" = "gpu" ]; then
    ORT_NAME="onnxruntime-linux-x64-gpu_cuda${CUDA_MAJOR}-${ORT_VERSION}"
else
    ORT_NAME="onnxruntime-linux-x64-${ORT_VERSION}"
fi

if [ -n "$ORT_ROOT" ]; then
    note "using $ORT_ROOT"
elif [ -f "$ORT_DIR/$ORT_NAME/lib/libonnxruntime.so" ]; then
    ORT_ROOT="$ORT_DIR/$ORT_NAME"
    note "already unpacked at $ORT_ROOT"
else
    URL="https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ORT_NAME}.tgz"
    note "downloading $ORT_NAME"
    mkdir -p "$ORT_DIR"
    TMP="$(mktemp -d)"
    trap 'rm -rf "$TMP"' EXIT
    if command -v wget >/dev/null 2>&1; then
        wget -q --show-progress -O "$TMP/ort.tgz" "$URL" || die "download failed: $URL
       Check the release exists, or pass --ort-version / --ort-root."
    else
        curl -fSL --progress-bar -o "$TMP/ort.tgz" "$URL" || die "download failed: $URL"
    fi
    tar -xzf "$TMP/ort.tgz" -C "$ORT_DIR"
    rm -rf "$TMP"; trap - EXIT
    ORT_ROOT="$ORT_DIR/$ORT_NAME"
    note "unpacked to $ORT_ROOT"
fi

[ -f "$ORT_ROOT/lib/libonnxruntime.so" ] || die "$ORT_ROOT is not an ONNX Runtime directory
       (no lib/libonnxruntime.so)."
if [ "$TARGET" = "gpu" ] && [ ! -f "$ORT_ROOT/lib/libonnxruntime_providers_cuda.so" ]; then
    die "$ORT_ROOT is a CPU-only ONNX Runtime build, but this is a GPU setup.
       Remove it and re-run, or pass --cpu."
fi

# ── 5. Build ─────────────────────────────────────────────────────────────────
# VisionPilot/build by default: that is where run_inference.sh and build_models.sh
# look, and the binary resolves config/ and the weights relative to its working
# directory — both of which CMake copies into the build tree.
step "5/6  Build"
if [ "$CLEAN" = "true" ] && [ -d "$BUILD_DIR" ]; then
    # Keep build/_deps: it is fetched third-party source (Eigen, cloned from GitLab),
    # not build output. Re-cloning it costs far more than the rebuild it wraps.
    KEEP=""
    if [ -d "$BUILD_DIR/_deps" ]; then
        KEEP="$(mktemp -d)"
        mv "$BUILD_DIR/_deps" "$KEEP/_deps"
        note "removing $BUILD_DIR (keeping the fetched dependencies in _deps/)"
    else
        note "removing $BUILD_DIR"
    fi
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    [ -n "$KEEP" ] && { mv "$KEEP/_deps" "$BUILD_DIR/_deps"; rmdir "$KEEP"; }
fi
mkdir -p "$BUILD_DIR"
[ -d "$BUILD_DIR/_deps" ] || note "first configure clones Eigen — slow on a poor connection"

CMAKE_ARGS=(
    "-DONNXRUNTIME_ROOT=$ORT_ROOT"
    "-DGPU=$([ "$TARGET" = "gpu" ] && echo ON || echo OFF)"
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_CXX_STANDARD_LIBRARIES=-lcppad_lib"
)
if [ "$ROS2" = "true" ]; then
    [ -n "${ROS_DISTRO:-}" ] || die "--ros2 needs a sourced ROS 2 environment:
       source /opt/ros/jazzy/setup.bash"
    CMAKE_ARGS+=("-DENABLE_ROS2_INTERFACE=ON")
fi

( cd "$BUILD_DIR" && cmake "${CMAKE_ARGS[@]}" "$VP_ROOT" && make -j"$JOBS" )
[ -x "$BUILD_DIR/VisionPilot" ] || die "the build produced no $BUILD_DIR/VisionPilot binary."

# CMake copies config/ into build/config/ — that copy is what the binary reads, so a
# CPU-only machine gets its provider fixed there and the shipped config stays untouched.
MAIN_CONF="$BUILD_DIR/config/vision_pilot.conf"
if [ "$TARGET" = "cpu" ] && [ -f "$MAIN_CONF" ]; then
    sed -i "s|^engine\.provider  *=.*|engine.provider     = cpu|" "$MAIN_CONF"
    note "set engine.provider = cpu in build/config/vision_pilot.conf"
fi
PROVIDER="$(awk -F= '/^engine\.provider/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF" 2>/dev/null || true)"
PRECISION="$(awk -F= '/^model\.precision  *=/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF" 2>/dev/null || true)"

# ── 6. TensorRT engines (optional) ───────────────────────────────────────────
step "6/6  TensorRT engines"
if [ -n "$ENGINES" ] && [ "$HAVE_TRT" = "false" ]; then
    die "--engines $ENGINES needs TensorRT, and no libnvinfer was found.
       Install the libnvinfer* build tagged +cuda${CUDA_MAJOR:-<your CUDA>} from NVIDIA's
       repo, then: VisionPilot/scripts/build_models.sh --precision $ENGINES
       Everything else is built and ready — run_inference.sh works on the cuda provider."
elif [ -n "$ENGINES" ]; then
    "$VP_ROOT/scripts/build_models.sh" --precision "$ENGINES"
    PROVIDER="tensorrt"; PRECISION="$ENGINES"
else
    note "skipped — only the tensorrt provider needs them (./setup.sh --engines int8,"
    note "or VisionPilot/scripts/build_models.sh --precision int8 later)"
fi

# ── Done ─────────────────────────────────────────────────────────────────────
step "Ready in $((SECONDS / 60))m $((SECONDS % 60))s"
note "binary:       $BUILD_DIR/VisionPilot"
note "onnxruntime:  $ORT_ROOT"
note "config:       $PRECISION weights on the $PROVIDER provider"
printf '\n    Run inference:\n\n'
printf '      cd %s/VisionPilot/scripts\n' "$REPO_ROOT"
printf '      ./run_inference.sh scene-0796\n\n'

if [ "$VERIFY" = "true" ]; then
    step "Smoke test — scene-0796, headless"
    "$VP_ROOT/scripts/run_inference.sh" scene-0796 --fast -- --no-window
    step "Smoke test passed"
fi
