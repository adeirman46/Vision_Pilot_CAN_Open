#!/usr/bin/env bash
#
# Run VisionPilot on a live camera WITH the offline map panel: the camera frame
# and its HUD on the left, the Bandung map, route and next manoeuvre on the right.
#
#   ┌──────────────────────────┬──────────────┐
#   │  camera frame + HUD      │  map + route │
#   │  1024 x 512              │  512 x 512   │
#   └──────────────────────────┴──────────────┘
#
# This is inference_cam.sh plus navigation, kept as its own script so the plain
# camera run stays exactly as it was. It never writes nav.enabled into the config
# either — navigation is switched on with the binary's --nav flag — so running
# inference_cam.sh afterwards still gives you a camera-only window.
#
# Usage:
#   ./inference_cam_nav.sh [camera-source] [options] [-- <VisionPilot args>]
#
# The camera source is a stream URL or a local camera, same as inference_cam.sh:
#   http://<ip>:8080/video   phone running IP Webcam (Android) — the default form
#   rtsp://<host>/<path>     any RTSP camera
#   0 | /dev/video0          a USB/laptop camera
# Default: $VP_CAM_URL, else http://10.142.152.101:8081/video
#
# Position options:
#   --gps <src>          phyphox (default) | ipwebcam | fixed | file | none
#   --gps-url <url>      GPS host. Default $VP_GPS_URL, else http://10.154.111.223:8080
#                        phyphox: open the "Location (GPS)" experiment, three-dot
#                        menu -> Allow remote access, press play. It answers on
#                        port 8080, so it and IP Webcam cannot share one port.
#                        ipwebcam: omit to reuse the camera's own host and port.
#   --gps-file <file>    recorded track for --gps file, one
#                        "lat,lon[,speed_ms[,bearing_deg]]" per line
#   --fixed-pos <lat,lon>  position for --gps fixed (default Bandung centre)
#
# Map options:
#   --dest <lat,lon|name>  destination — draws the route and the turn-by-turn
#                        banner. Either coordinates or a street name looked up in
#                        the map itself, e.g. --dest "Jalan Braga". The match
#                        nearest your current position wins.
#   --find <text>        list places and roads matching <text>, then exit
#   --ask                ask for the destination in the terminal instead of
#                        typing it into the app window
#   --dest-name <text>   label for that destination on the panel
#   --map <file>         .vpmap to load. Default VisionPilot/maps/bandung-roads.vpmap
#   --zoom <m>           metres across the panel width (400)
#   --panel <px>         panel width in px (512)
#   --north-up           north-up map instead of heading-up
#   --output <WxH>       canvas size (1920x1080). The camera view and the panel
#                        are composed onto exactly this, so the window matches
#                        the screen and nothing pads it with a white border.
#   --check              check the map, the camera and the GPS feed, then exit
#
# Camera options (same meaning as inference_cam.sh):
#   --speed-const <m/s>  ego speed fed to the planner (15)
#   --hours <h>          how long the synthesised speed track lasts (6)
#   --calib <H.yaml>     camera homography. Default: leave config/H.yaml alone
#   --precision <p>      default | fp32 | int8
#   --provider <p>       cpu | cuda | tensorrt
#
# Everything after `--` is forwarded to the VisionPilot binary:
#   --debug-viz          annotated telemetry overlay instead of the clean HUD
#   --no-window          headless, console output only
#
# Examples:
#   ./inference_cam_nav.sh
#   ./inference_cam_nav.sh --gps-url http://172.27.74.24:8080
#   ./inference_cam_nav.sh --dest "Jalan Braga"
#   ./inference_cam_nav.sh --find braga
#   ./inference_cam_nav.sh --dest -6.9034,107.6181 --dest-name "Gedung Sate"
#   ./inference_cam_nav.sh --gps fixed --zoom 800        # no phone GPS needed
#   ./inference_cam_nav.sh --check
#
# Build the map once first:  ./scripts/build_osm_map.sh
#
# Env overrides: VP_ROOT, VP_BUILD, VP_CAM_URL, VP_GPS_URL
set -euo pipefail

VP_ROOT="${VP_ROOT:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}"
BUILD_DIR="${VP_BUILD:-$VP_ROOT/build}"
DEFAULT_URL="${VP_CAM_URL:-http://10.142.152.101:8081/video}"
DEFAULT_GPS="${VP_GPS_URL:-http://10.154.111.223:8080}"

usage() { sed -n '2,64p' "$0"; exit "${1:-1}"; }

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
CHECK="false"
ASK="false"       # --ask brings back the old terminal prompt
VP_ARGS=()

GPS="phyphox"
GPS_URL=""
GPS_FILE=""
FIXED_POS=""
DEST=""
DEST_NAME=""
FIND=""
MAP_FILE=""
OUTPUT_SIZE="1920x1080"
ZOOM="400"
PANEL="512"
HEADING_UP="true"

while [ $# -gt 0 ]; do
    case "$1" in
        --gps)         [ $# -ge 2 ] || { echo "error: --gps needs a value" >&2; exit 1; }
                       case "$2" in phyphox|ipwebcam|fixed|file|none) ;;
                           *) echo "error: --gps must be phyphox, ipwebcam, fixed, file or none" >&2; exit 1 ;;
                       esac
                       GPS="$2"; shift 2 ;;
        --gps-url)     [ $# -ge 2 ] || { echo "error: --gps-url needs a value" >&2; exit 1; }
                       GPS_URL="$2"; shift 2 ;;
        --gps-file)    [ $# -ge 2 ] || { echo "error: --gps-file needs a value" >&2; exit 1; }
                       GPS_FILE="$(readlink -f "$2")"; GPS="file"; shift 2 ;;
        --fixed-pos)   [ $# -ge 2 ] || { echo "error: --fixed-pos needs lat,lon" >&2; exit 1; }
                       FIXED_POS="$2"; GPS="fixed"; shift 2 ;;
        --dest)        [ $# -ge 2 ] || { echo "error: --dest needs lat,lon or a name" >&2; exit 1; }
                       DEST="$2"; shift 2 ;;
        --find)        [ $# -ge 2 ] || { echo "error: --find needs a search term" >&2; exit 1; }
                       FIND="$2"; shift 2 ;;
        --dest-name)   [ $# -ge 2 ] || { echo "error: --dest-name needs a value" >&2; exit 1; }
                       DEST_NAME="$2"; shift 2 ;;
        --map)         [ $# -ge 2 ] || { echo "error: --map needs a value" >&2; exit 1; }
                       MAP_FILE="$(readlink -f "$2")"; shift 2 ;;
        --zoom)        [ $# -ge 2 ] || { echo "error: --zoom needs a value" >&2; exit 1; }
                       ZOOM="$2"; shift 2 ;;
        --panel)       [ $# -ge 2 ] || { echo "error: --panel needs a value" >&2; exit 1; }
                       PANEL="$2"; shift 2 ;;
        --north-up)    HEADING_UP="false"; shift ;;
        --check)       CHECK="true"; shift ;;
        --ask)         ASK="true"; shift ;;
        --no-ask)      ASK="false"; shift ;;
        --output)      [ $# -ge 2 ] || { echo "error: --output needs WxH" >&2; exit 1; }
                       OUTPUT_SIZE="$2"; shift 2 ;;
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
        -h|--help)     usage 0 ;;
        --)            shift; VP_ARGS=("$@"); break ;;
        *)             echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

case "$FIXED_POS" in
    "") ;;
    *,*) ;;
    *) echo "error: --fixed-pos expects lat,lon, got '$FIXED_POS'" >&2; exit 1 ;;
esac

# ── Resolve the camera source ────────────────────────────────────────────────
# VisionPilot opens a URL directly (video mode) and a /dev/videoN device through
# its V4L2 interface, so a bare index like `0` becomes /dev/video0.
case "$SOURCE" in
    [0-9]*)      [ -e "/dev/video$SOURCE" ] || { echo "error: /dev/video$SOURCE does not exist" >&2; exit 1; }
                 DEVICE="/dev/video$SOURCE"; MODE="v4l2" ;;
    /dev/video*) [ -e "$SOURCE" ] || { echo "error: $SOURCE does not exist" >&2; exit 1; }
                 DEVICE="$SOURCE"; MODE="v4l2" ;;
    http://*|https://*|rtsp://*|rtmp://*|udp://*|tcp://*|rtp://*)
                 URL="$SOURCE"; MODE="stream" ;;
    *) echo "error: unrecognised source '$SOURCE' — expected a stream URL, /dev/videoN or an index" >&2; exit 1 ;;
esac

# ── Resolve the GPS host ─────────────────────────────────────────────────────
# IP Webcam serves position and frames off one host and one port, so it can reuse
# the camera URL. phyphox is a separate app on its own port and never can.
case "$GPS" in
    ipwebcam)
        if [ -z "$GPS_URL" ] && [ "$MODE" = "stream" ]; then
            GPS_URL="${URL%/*}"
        fi
        [ -n "$GPS_URL" ] || {
            echo "error: --gps ipwebcam needs --gps-url when the camera is a local device" >&2
            exit 1; } ;;
    phyphox)
        GPS_URL="${GPS_URL:-$DEFAULT_GPS}" ;;
    file)
        [ -f "$GPS_FILE" ] || { echo "error: --gps file needs --gps-file <track>" >&2; exit 1; } ;;
esac

# ── Checks ───────────────────────────────────────────────────────────────────
CAM_OK="unknown"
if [ "$MODE" = "stream" ]; then
    HOSTPORT="${URL#*://}"; HOSTPORT="${HOSTPORT%%/*}"
    case "$URL" in http://*|https://*)
        if command -v curl >/dev/null 2>&1; then
            # HEAD the server root, not the stream: the stream never ends, so a
            # request against it would only come back on the timeout.
            if curl -s -m 5 -I -o /dev/null "http://$HOSTPORT/"; then
                CAM_OK="reachable"
            else
                echo "error: cannot reach the camera at $HOSTPORT" >&2
                echo "       Check the phone is awake, IP Webcam is started, and both" >&2
                echo "       are on the same network." >&2
                exit 1
            fi
        fi ;;
    esac
    if command -v ffprobe >/dev/null 2>&1; then
        # `|| true`: under `set -e -o pipefail` a probe that cannot read the
        # stream would otherwise abort the whole script, and not being able to
        # name the resolution is not a reason to refuse to run.
        INFO="$(timeout 15 ffprobe -v error -select_streams v:0 \
                -show_entries stream=width,height -of csv=p=0 "$URL" 2>/dev/null | tr ',' 'x' || true)"
    fi
    SOURCE_DESC="$URL${INFO:+  (${INFO})}"
else
    SOURCE_DESC="$DEVICE"
    CAM_OK="local device"
fi

# A dead GPS feed is a warning, never a stop: the camera and the planner do not
# depend on it, and the panel says what is wrong.
GPS_OK="not checked"
NEAR=""            # current fix, if any — biases the --dest name search
if command -v curl >/dev/null 2>&1; then
    case "$GPS" in
        phyphox)
            if REPLY_JSON="$(curl -s -m 5 "$GPS_URL/get?lat&lon" 2>/dev/null)" &&
               printf '%s' "$REPLY_JSON" | grep -q '"lat"'; then
                GPS_OK="answering"
                # phyphox prints scientific notation ("1.0765047E2"); awk turns
                # that back into the plain decimals the tools expect.
                LATLON="$(printf '%s' "$REPLY_JSON" |
                          grep -oE '"(lat|lon)":\{[^}]*\}' |
                          grep -oE '\[[^]]*\]' | tr -d '[]' |
                          awk '{ printf "%s%.6f", (NR>1 ? "," : ""), $0 + 0 }')"
                if [ -n "$LATLON" ]; then
                    GPS_OK="answering  ($LATLON)"
                    NEAR="$LATLON"   # biases a --dest name search to nearby roads
                fi
            else
                GPS_OK="NOT answering"
                echo "warning: phyphox is not answering at $GPS_URL" >&2
                echo "         Open the 'Location (GPS)' experiment, three-dot menu ->" >&2
                echo "         Allow remote access, then press play." >&2
            fi ;;
        ipwebcam)
            if curl -s -m 5 "$GPS_URL/sensors.json?sense=gps" 2>/dev/null | grep -q '"gps"'; then
                GPS_OK="answering"
            else
                GPS_OK="NOT answering"
                echo "warning: no GPS from IP Webcam at $GPS_URL" >&2
                echo "         Many builds have no GPS entry under Data logging ->" >&2
                echo "         Active data logging sources at all; on those it never will." >&2
                echo "         Use --gps phyphox instead." >&2
            fi ;;
    esac
fi

# ── Map ──────────────────────────────────────────────────────────────────────
MAP_FILE="${MAP_FILE:-$VP_ROOT/maps/bandung-roads.vpmap}"
[ -f "$MAP_FILE" ] || {
    echo "error: navigation map not found: $MAP_FILE" >&2
    echo "       Build it once (greater Bandung, a few minutes):" >&2
    echo "         $VP_ROOT/scripts/build_osm_map.sh" >&2
    exit 1; }

# ── Destination by name ──────────────────────────────────────────────────────
# The extract carries every road name in greater Bandung, so a destination can be
# a street rather than a coordinate. nav_preview does the lookup against the same
# map the panel draws, biased to whatever is nearest the current fix.
NAV_PREVIEW="$BUILD_DIR/nav_preview"

lookup() {   # <query> -> "lat,lon<TAB>name<TAB>km" lines, best first
    [ -x "$NAV_PREVIEW" ] || {
        echo "error: $NAV_PREVIEW not built — run: make -C $BUILD_DIR nav_preview" >&2
        exit 1; }
    "$NAV_PREVIEW" --map "$MAP_FILE" --find "$1" \
        ${NEAR:+--at "$NEAR"} 2>/dev/null | grep -v '^\[INFO\]'
}

if [ -n "$FIND" ]; then
    HITS="$(lookup "$FIND")" || true
    [ -n "$HITS" ] || { echo "no road matching \"$FIND\" in $MAP_FILE" >&2; exit 1; }
    echo "roads matching \"$FIND\"${NEAR:+ (nearest to $NEAR first)}:"
    printf '%s\n' "$HITS" | awk -F'\t' '{ printf "  %-38s %8s km   --dest %s\n", $2, $3, $1 }'
    exit 0
fi

case "$DEST" in
    "") ;;
    *[!0-9.,+-]*)   # contains a letter, so treat it as a name to look up
        HIT="$(lookup "$DEST" | head -1)" || true
        [ -n "$HIT" ] || {
            echo "error: no road matching \"$DEST\" in the map" >&2
            echo "       List what is there:  $0 --find \"$DEST\"" >&2
            exit 1; }
        DEST_NAME="${DEST_NAME:-$(printf '%s' "$HIT" | cut -f2)}"
        DEST="$(printf '%s' "$HIT" | cut -f1)"
        echo "destination: $DEST_NAME  ($DEST)" ;;
    *,*) ;;
    *) echo "error: --dest expects lat,lon or a road name, got '$DEST'" >&2; exit 1 ;;
esac

# ── Ask where to go (opt-in) ─────────────────────────────────────────────────
# Off by default: with no --dest the app opens its own destination box in the
# video window, the way a car's head unit does. --ask is for scripted or
# headless runs where there is no window to type into.
if [ -z "$DEST" ] && [ "$CHECK" = "false" ] && [ "$ASK" = "true" ] && [ -t 0 ]; then
    echo ""
    echo "┌─ Where do you want to go? ──────────────────────────────────────────┐"
    echo "│  a place or street name   e.g. Trans Studio Mall, Jalan Braga       │"
    echo "│  coordinates              e.g. -6.9034,107.6181                     │"
    echo "│  Enter on its own         no route, just the moving map             │"
    echo "└─────────────────────────────────────────────────────────────────────┘"
    while : ; do
        printf "destination> "
        IFS= read -r ANSWER || ANSWER=""
        [ -z "$ANSWER" ] && { echo "no destination - free-drive map"; break; }

        case "$ANSWER" in
            *[!0-9.,+-]*)
                HITS="$(lookup "$ANSWER")" || true
                if [ -z "$HITS" ]; then
                    echo "  nothing in the map matches \"$ANSWER\" - try another name"
                    continue
                fi
                printf '%s\n' "$HITS" |
                    awk -F'\t' '{ printf "  %2d) %-44s %s\n", NR, $2, ($3+0 > 0 ? $3" km of road" : "") }'
                printf "pick a number [1], or Enter another name> "
                IFS= read -r PICK || PICK=""
                case "$PICK" in
                    "")       PICK=1 ;;
                    *[!0-9]*) continue ;;   # they typed a new search instead
                esac
                LINE="$(printf '%s\n' "$HITS" | sed -n "${PICK}p")"
                [ -n "$LINE" ] || { echo "  no such choice"; continue; }
                DEST_NAME="$(printf '%s' "$LINE" | cut -f2)"
                DEST="$(printf '%s' "$LINE" | cut -f1)" ;;
            *,*) DEST="$ANSWER" ;;
            *)   echo "  expected a name or lat,lon"; continue ;;
        esac
        echo "going to: ${DEST_NAME:-$DEST}  ($DEST)"
        break
    done
    echo ""
fi

if [ "$CHECK" = "true" ]; then
    echo "camera: $SOURCE_DESC  [$CAM_OK]"
    echo "gps:    $GPS${GPS_URL:+ @ $GPS_URL}  [$GPS_OK]"
    echo "map:    $MAP_FILE  ($(du -h "$MAP_FILE" | cut -f1))"
    [ -n "$DEST" ] && echo "dest:   ${DEST_NAME:-$DEST}  ($DEST)"
    [ "$MODE" = "stream" ] && [ -z "${INFO:-}" ] &&
        echo "warning: ffprobe could not read a video stream from the camera"
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
# The preprocessor warps each frame to the canonical bird's-eye view through a C
# matrix derived from H, so H has to be installed *and* C regenerated.
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

MAIN_CONF="$BUILD_DIR/config/vision_pilot.conf"
SRC_CONF="$VP_ROOT/config/vision_pilot.conf"
[ -f "$MAIN_CONF" ] || { echo "error: $MAIN_CONF not found — run cmake first" >&2; exit 1; }

# Replace a key in place, or append it when the config predates it.
set_conf() {
    local key="$1" val="$2" file="$3" esc
    esc="$(printf '%s' "$key" | sed 's/\./\\./g')"
    val="${val//&/\\&}"
    if grep -qE "^${esc}[[:space:]]*=" "$file"; then
        sed -i -E "s|^(${esc})[[:space:]]*=.*|\1 = ${val}|" "$file"
    else
        printf '%s = %s\n' "$key" "$val" >> "$file"
    fi
}

# ── Model precision / execution provider ─────────────────────────────────────
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
    [ -n "$PROVIDER" ] && set_conf engine.provider "$PROVIDER" "$MAIN_CONF"
fi

# ── Input source ─────────────────────────────────────────────────────────────
# A URL runs through video mode (FileInterface, opened on a stream); a real
# device runs through v4l2 mode.
if [ "$MODE" = "v4l2" ]; then
    set_conf source.mode        "v4l2"   "$MAIN_CONF"
    set_conf source.v4l2_device "$DEVICE" "$MAIN_CONF"
else
    set_conf source.mode "video" "$MAIN_CONF"
fi

# ── Navigation ───────────────────────────────────────────────────────────────
# nav.enabled is deliberately NOT written: navigation is turned on with the
# binary's --nav flag, so inference_cam.sh keeps giving a camera-only window.
set_conf nav.map_file    "$MAP_FILE" "$MAIN_CONF"
set_conf nav.gps_source  "$GPS"      "$MAIN_CONF"
set_conf nav.gps_url     "$GPS_URL"  "$MAIN_CONF"
set_conf nav.zoom_m      "$ZOOM"     "$MAIN_CONF"
set_conf nav.panel_width "$PANEL"    "$MAIN_CONF"
set_conf nav.heading_up  "$HEADING_UP" "$MAIN_CONF"
set_conf nav.output_size "$OUTPUT_SIZE" "$MAIN_CONF"
set_conf nav.gps_file    "$GPS_FILE" "$MAIN_CONF"
[ -n "$FIXED_POS" ] && set_conf nav.fixed_position   "$FIXED_POS" "$MAIN_CONF"
set_conf nav.destination      "$DEST"      "$MAIN_CONF"
set_conf nav.destination_name "$DEST_NAME" "$MAIN_CONF"

PRECISION_DESC="$(awk -F= '/^model\.precision  *=/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF")"
PROVIDER_DESC="$(awk -F= '/^engine\.provider/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF")"
AD_DESC="$(awk -F= '/^model\.precision\.autodrive/ {gsub(/ /,"",$2); print $2}' "$MAIN_CONF")"
[ -n "$AD_DESC" ] && PRECISION_DESC="$PRECISION_DESC (AutoDrive $AD_DESC)"

if [ "$PRECISION_DESC" != "${PRECISION_DESC#int8}" ] && [ "$PROVIDER_DESC" != "tensorrt" ]; then
    echo "note: int8 weights under the '$PROVIDER_DESC' provider are dequantised, not run as"
    echo "      int8 kernels — expect it to be slower than fp32. Add --provider tensorrt."
fi
# Compiling TensorRT engines takes minutes and several GB of RAM; a live camera
# run is not the place for it. build_models.sh caches them, one model per process.
if [ "$PROVIDER_DESC" = "tensorrt" ]; then
    CACHE_DIR="$(awk -F= '/^engine\.cache_dir/ {gsub(/^[ \t]+|[ \t]+$/,"",$2); print $2}' "$MAIN_CONF")"
    CACHE_DIR="${CACHE_DIR:-../trt_cache}"
    CACHE_DIR="${CACHE_DIR/#\~/$HOME}"
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

# ── Test config (stream mode only) ───────────────────────────────────────────
# A live camera carries no speed signal, and the planner reads one value per
# inference frame — it stops when the track runs out, so synthesise enough for
# --hours at 30 fps.
if [ "$MODE" = "stream" ]; then
    LINES=$(awk -v h="$HOURS" 'BEGIN { printf "%d", h * 3600 * 30 }')
    [ "$LINES" -gt 0 ] 2>/dev/null || { echo "error: --hours must be a positive number" >&2; exit 1; }
    SPEED="$WORK/cam_speed.txt"
    awk -v n="$LINES" -v v="$SPEED_CONST" 'BEGIN { for (i = 0; i < n; i++) print v }' > "$SPEED"

    cat > "$BUILD_DIR/config/vision_pilot_test.conf" <<EOF
# Generated by inference_cam_nav.sh

source.video_realtime      = false
source.video_loop          = false

source.input_video         = $URL
source.input_vehicle_speed = $SPEED
source.dataset             = camera
EOF
    SPEED_DESC="synthesised constant ${SPEED_CONST} m/s (${HOURS} h)"
else
    SPEED_DESC="CAN interface (stub — reports 0 m/s)"
fi

NAV_DESC="$GPS${GPS_URL:+ @ $GPS_URL} [$GPS_OK]"
[ -n "$DEST" ] && NAV_DESC="$NAV_DESC -> $DEST${DEST_NAME:+ ($DEST_NAME)}"

echo "=================================================="
echo " VisionPilot live camera + map navigation"
echo "   camera: $SOURCE_DESC"
echo "   speed:  $SPEED_DESC"
echo "   calib:  $CALIB_DESC"
echo "   model:  $PRECISION_DESC on $PROVIDER_DESC"
echo "   map:    $MAP_FILE"
echo "   gps:    $NAV_DESC"
echo "   layout: ${OUTPUT_SIZE} canvas, ${PANEL}px panel, ${ZOOM} m across, $([ "$HEADING_UP" = "true" ] && echo "heading up" || echo "north up")"
echo "   extra:  ${VP_ARGS[*]:-<none>}"
echo "   stop with Ctrl-C"
echo "=================================================="

cd "$BUILD_DIR"
exec ./VisionPilot --nav ${VP_ARGS[@]+"${VP_ARGS[@]}"}
