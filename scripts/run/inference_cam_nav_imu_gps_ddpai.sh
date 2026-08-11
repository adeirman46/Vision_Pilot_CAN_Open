#!/usr/bin/env bash
#
# Run VisionPilot on the DDPai N5 dashcam with the map panel and the Nucleo's
# position — camera, inference, map, GPS and IMU, the whole stack in one run.
#
#   ┌──────────────────────────┬──────────────┐
#   │  camera frame + HUD      │  map + route │
#   │  1024 x 512              │  512 x 512   │
#   └──────────────────────────┴──────────────┘
#
# Two things differ from the phone-based runs, and they belong together:
#
#   camera     the DDPai N5, over its own Wi-Fi as RTSP. This script joins that
#              network itself and puts yours back when the run ends.
#   position   the Nucleo on USB — a DFRobot GNSS-RTK rover on I2C and a Yahboom
#              IMU on UART, already fused on the board by a 6-state EKF:
#
#     phone GPS   a fix a second, several metres of jump between them, and a
#                 heading that spins on its own when you stop
#     this        20 Hz position, moving smoothly, with a heading that survives
#                 a stop, a tunnel or a row of tall buildings
#
# The pairing is not a coincidence. The dashcam's access point has no internet,
# and this machine can only be on one Wi-Fi at a time — so a phone serving
# position over Wi-Fi is unreachable while the camera is connected. Position has
# to arrive over USB, which is exactly what the Nucleo does.
#
# Before the first run:
#   scripts/sensors/flash_nucleo.sh fused    put the fused firmware on the board
#   scripts/sensors/gps_imu_check.sh         confirm it publishes, and the IMU is in
#   scripts/sensors/ddpai_stream.sh          confirm the camera streams
#   scripts/build/build_osm_map.sh           once, builds maps/bandung-roads.vpmap
#
# Usage:
#   ./inference_cam_nav_imu_gps_ddpai.sh [options] [-- <VisionPilot args>]
#
# Camera options:
#   --rear               the dashcam's rear lens instead of the front one
#   --url <url>          an explicit stream URL — also skips the Wi-Fi handling
#   --no-wifi            do not touch Wi-Fi; already on the camera's network
#   --keep-wifi          stay on the camera's network after the run ends
#
# Position options:
#   --port <dev>         the Nucleo's serial port. Default: first /dev/ttyACM*
#   --baud <n>           line speed (115200)
#   --compare            also draw the raw receiver beside the fused position,
#                        so the difference is visible while driving
#   --no-gps-check       skip the pre-flight read of the port
#
# Map options:
#   --dest <lat,lon|name>  destination — draws the route and the turn-by-turn
#                        banner. Either coordinates or a street name looked up in
#                        the map itself, e.g. --dest "Jalan Braga".
#   --find <text>        list places and roads matching <text>, then exit
#   --ask                ask for the destination in the terminal instead of
#                        typing it into the app window
#   --dest-name <text>   label for that destination on the panel
#   --map <file>         .vpmap to load. Default VisionPilot/maps/bandung-roads.vpmap
#   --zoom <m>           metres across the panel width (400)
#   --light              light / daytime map ('t' toggles it while running)
#   --opencv             the old OpenCV window instead of the Qt front-end
#   --panel <px>         map panel width in px (760)
#   --north-up           north-up map instead of heading-up
#   --output <WxH|fit>   canvas size. Default: the screen minus the title bar
#   --check              check the camera, the map and the board, then exit
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
#   ./inference_cam_nav_imu_gps_ddpai.sh
#   ./inference_cam_nav_imu_gps_ddpai.sh --check
#   ./inference_cam_nav_imu_gps_ddpai.sh --dest "Jalan Braga" --compare
#   ./inference_cam_nav_imu_gps_ddpai.sh --rear --zoom 800
#
# Env overrides: VP_ROOT, VP_BUILD
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
VP_ROOT="${VP_ROOT:-$(cd "$HERE/../../VisionPilot" && pwd)}"
BUILD_DIR="${VP_BUILD:-$VP_ROOT/build}"
DDPAI="$HERE/../sensors/ddpai_stream.sh"

usage() { sed -n '2,84p' "$0"; exit "${1:-1}"; }

# The camera is the dashcam — there is no positional source argument, because
# there is only one camera this script is for.
CAM_URL=""
CAM_REAR="false"
CAM_WIFI="true"
KEEP_WIFI="false"

GPS_PORT=""
GPS_BAUD="115200"
GPS_CHECK="true"
COMPARE="false"

SPEED_CONST="15"
HOURS="6"
CALIB=""
PRECISION=""
PROVIDER=""
CHECK="false"
ASK="false"
VP_ARGS=()

DEST=""
DEST_NAME=""
FIND=""
MAP_FILE=""
OUTPUT_SIZE=""
ZOOM="400"
PANEL="760"
HEADING_UP="true"
DARK="true"
USE_QT="true"

while [ $# -gt 0 ]; do
    case "$1" in
        --port)        [ $# -ge 2 ] || { echo "error: --port needs a device" >&2; exit 1; }
                       GPS_PORT="$2"; shift 2 ;;
        --baud)        [ $# -ge 2 ] || { echo "error: --baud needs a value" >&2; exit 1; }
                       GPS_BAUD="$2"; shift 2 ;;
        --compare)     COMPARE="true"; shift ;;
        --no-gps-check) GPS_CHECK="false"; shift ;;
        --rear)        CAM_REAR="true"; shift ;;
        --url)         [ $# -ge 2 ] || { echo "error: --url needs a value" >&2; exit 1; }
                       CAM_URL="$2"; CAM_WIFI="false"; shift 2 ;;
        --no-wifi)     CAM_WIFI="false"; shift ;;
        --keep-wifi)   KEEP_WIFI="true"; shift ;;
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
        --qt)          USE_QT="true"; shift ;;
        --opencv)      USE_QT="false"; shift ;;
        --light)       DARK="false"; shift ;;
        --dark)        DARK="true"; shift ;;
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

# ── The camera ───────────────────────────────────────────────────────────────
# ddpai_stream.sh already owns the Wi-Fi handling and knows the stream URLs, so
# it is asked for the working one rather than duplicating any of that here. It
# deliberately leaves the link up (--print-url implies --keep-wifi); putting the
# previous network back is this script's job, on every exit path including
# Ctrl-C, so a finished drive does not leave the machine off the internet.
MODE="stream"
PREV_WIFI=""

restore_wifi() {
    [ "$KEEP_WIFI" = "true" ] && return 0
    [ -n "$PREV_WIFI" ] || return 0
    echo ""
    echo "restoring Wi-Fi: $PREV_WIFI"
    nmcli connection modify "$PREV_WIFI" connection.autoconnect yes >/dev/null 2>&1 || true
    nmcli connection up "$PREV_WIFI" >/dev/null 2>&1 || true
}

if [ -z "$CAM_URL" ]; then
    [ -x "$DDPAI" ] || { echo "error: $DDPAI is missing" >&2; exit 1; }

    if [ "$CAM_WIFI" = "true" ] && command -v nmcli >/dev/null 2>&1; then
        PREV_WIFI="$(nmcli -t -f NAME,TYPE connection show --active 2>/dev/null \
                     | awk -F: '$2=="802-11-wireless" && $1 !~ /^DDPAI/ {print $1; exit}')"
        [ -n "$PREV_WIFI" ] && trap restore_wifi EXIT INT TERM
    fi

    DDPAI_ARGS=(--print-url)
    [ "$CAM_REAR" = "true" ] && DDPAI_ARGS+=(--rear)
    [ "$CAM_WIFI" = "false" ] && DDPAI_ARGS+=(--no-wifi)

    CAM_URL="$("$DDPAI" "${DDPAI_ARGS[@]}")" || {
        echo "error: could not reach the dashcam — see the messages above" >&2
        exit 1; }
    [ -n "$CAM_URL" ] || { echo "error: the dashcam returned no stream URL" >&2; exit 1; }
fi
URL="$CAM_URL"

# ── The Nucleo ───────────────────────────────────────────────────────────────
# An ST-LINK enumerates as CDC-ACM, so the board is /dev/ttyACM*. A ttyUSB is a
# USB-serial bridge — if the IMU is still plugged into the PC, that is what it
# is, and it is not what publishes fixes now.
if [ -z "$GPS_PORT" ]; then
    for p in /dev/ttyACM*; do
        [ -e "$p" ] && { GPS_PORT="$p"; break; }
    done
fi

GPS_OK="not checked"
if [ -z "$GPS_PORT" ] || [ ! -e "$GPS_PORT" ]; then
    # A missing board is fatal here in a way a missing phone GPS is not: this
    # script exists to drive on the Nucleo's position, and with no port there is
    # no position at all. inference_cam_nav.sh is the one to use without it.
    echo "error: no Nucleo serial port found${GPS_PORT:+ at $GPS_PORT}" >&2
    echo "       The board should appear as /dev/ttyACM*. Check:" >&2
    echo "         lsusb | grep -i st-link" >&2
    echo "         $HERE/../sensors/flash_nucleo.sh fused" >&2
    echo "       To drive on a phone's GPS instead:  $HERE/inference_cam_nav.sh" >&2
    exit 1
fi
[ -r "$GPS_PORT" ] && [ -w "$GPS_PORT" ] || {
    echo "error: no permission to read $GPS_PORT" >&2
    echo "       sudo usermod -aG dialout $USER   (then log out and back in)" >&2
    exit 1; }

if command -v fuser >/dev/null 2>&1 && [ -n "$(fuser "$GPS_PORT" 2>/dev/null || true)" ]; then
    echo "error: something else already has $GPS_PORT" >&2
    echo "       Two readers on one port steal bytes from each other. Close the" >&2
    echo "       other one — usually 'pio device monitor' or gps_imu_check.sh." >&2
    exit 1
fi

# A three-second listen, purely to report what is on the wire before the window
# opens. Not fatal: a board that has not got a fix yet will get one outside.
if [ "$GPS_CHECK" = "true" ]; then
    stty -F "$GPS_PORT" "$GPS_BAUD" raw -echo 2>/dev/null || true
    SAMPLE="$(timeout 3 cat "$GPS_PORT" 2>/dev/null || true)"
    N_FUS="$(printf '%s' "$SAMPLE" | grep -ac 'VPFUS' || true)"
    N_RTK="$(printf '%s' "$SAMPLE" | grep -ac 'VPRTK' || true)"
    if [ "${N_FUS:-0}" -gt 0 ]; then
        IMU_HZ="$(printf '%s' "$SAMPLE" | grep -a 'VPFUS' | tail -1 | cut -d, -f9)"
        GPS_OK="fused, IMU at ${IMU_HZ:-?} Hz"
        case "${IMU_HZ%%.*}" in
            0|"") echo "warning: the board is fusing but the IMU is silent — position will" >&2
                  echo "         jump once a second and lose heading at a stop." >&2
                  echo "         Find it:  $HERE/../sensors/flash_nucleo.sh sniff -m" >&2
                  GPS_OK="fused but IMU silent" ;;
        esac
    elif [ "${N_RTK:-0}" -gt 0 ]; then
        GPS_OK="raw GPS only (not the fused firmware)"
        echo "warning: the board publishes raw fixes but nothing fused." >&2
        echo "         Flash the fused firmware:  $HERE/../sensors/flash_nucleo.sh fused" >&2
    else
        GPS_OK="NOT publishing"
        echo "warning: nothing arrived from $GPS_PORT in 3 s." >&2
        echo "         Check it:  $HERE/../sensors/gps_imu_check.sh" >&2
    fi
fi

# ── Camera ───────────────────────────────────────────────────────────────────
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
        # `|| true`: not being able to name the resolution is not a reason to
        # refuse to run, and under `set -e -o pipefail` it otherwise would be.
        INFO="$(timeout 15 ffprobe -v error -select_streams v:0 \
                -show_entries stream=width,height -of csv=p=0 "$URL" 2>/dev/null | tr ',' 'x' || true)"
    fi
    SOURCE_DESC="$URL${INFO:+  (${INFO})}"
else
    SOURCE_DESC="$DEVICE"
    CAM_OK="local device"
fi

# ── Map ──────────────────────────────────────────────────────────────────────
MAP_FILE="${MAP_FILE:-$VP_ROOT/maps/bandung-roads.vpmap}"
[ -f "$MAP_FILE" ] || {
    echo "error: navigation map not found: $MAP_FILE" >&2
    echo "       Build it once (greater Bandung, a few minutes):" >&2
    echo "         $HERE/../build/build_osm_map.sh" >&2
    exit 1; }

# ── Destination by name ──────────────────────────────────────────────────────
# The extract carries every road name in greater Bandung, so a destination can be
# a street rather than a coordinate. nav_preview does the lookup against the same
# map the panel draws.
NAV_PREVIEW="$BUILD_DIR/nav_preview"

lookup() {   # <query> -> "lat,lon<TAB>name<TAB>km" lines, best first
    [ -x "$NAV_PREVIEW" ] || {
        echo "error: $NAV_PREVIEW not built — run: make -C $BUILD_DIR nav_preview" >&2
        exit 1; }
    "$NAV_PREVIEW" --map "$MAP_FILE" --find "$1" 2>/dev/null | grep -v '^\[INFO\]'
}

if [ -n "$FIND" ]; then
    HITS="$(lookup "$FIND")" || true
    [ -n "$HITS" ] || { echo "no road matching \"$FIND\" in $MAP_FILE" >&2; exit 1; }
    echo "roads matching \"$FIND\":"
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
# video window, the way a car's head unit does. --ask is for scripted runs.
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
    echo "gps:    nucleo @ $GPS_PORT ${GPS_BAUD} baud  [$GPS_OK]"
    echo "map:    $MAP_FILE  ($(du -h "$MAP_FILE" | cut -f1))"
    [ -n "$DEST" ] && echo "dest:   ${DEST_NAME:-$DEST}  ($DEST)"
    [ "$MODE" = "stream" ] && [ -z "${INFO:-}" ] &&
        echo "warning: ffprobe could not read a video stream from the camera"
    exit 0
fi

[ -x "$BUILD_DIR/VisionPilot" ] || {
    echo "error: $BUILD_DIR/VisionPilot not built" >&2
    echo "       Set the machine up and build it in one step:" >&2
    echo "         $HERE/../build/setup.sh" >&2
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
if [ "$MODE" = "v4l2" ]; then
    set_conf source.mode        "v4l2"   "$MAIN_CONF"
    set_conf source.v4l2_device "$DEVICE" "$MAIN_CONF"
else
    set_conf source.mode "video" "$MAIN_CONF"
fi

# ── Canvas size ──────────────────────────────────────────────────────────────
# With no --output, size the canvas so the camera view exactly fills the space
# left of the panel and nothing is padded.
if [ "$OUTPUT_SIZE" = "fit" ]; then
    CANVAS_W=1920
    CANVAS_H=$(( (CANVAS_W - PANEL) / 2 ))
    OUTPUT_SIZE="${CANVAS_W}x${CANVAS_H}"
    FIT_DESC=" (camera-fitted, no black bars)"
elif [ -z "$OUTPUT_SIZE" ]; then
    # A canvas as tall as the screen does not fit once the title bar is added,
    # so the window manager shrinks the window and the toolkit pads the leftover
    # with white. Leaving room for the decoration means there is nothing to pad.
    SCREEN="$(xdotool getdisplaygeometry 2>/dev/null || true)"
    SCREEN_W="${SCREEN%% *}"
    SCREEN_H="${SCREEN##* }"
    case "$SCREEN_W$SCREEN_H" in *[!0-9]*|"") SCREEN_W=1920; SCREEN_H=1080 ;; esac
    CANVAS_W="$SCREEN_W"
    CANVAS_H=$(( SCREEN_H - 96 ))
    OUTPUT_SIZE="${CANVAS_W}x${CANVAS_H}"
    FIT_DESC=" (fits the screen, no white border)"
else
    FIT_DESC=""
fi

# ── Navigation ───────────────────────────────────────────────────────────────
# nav.enabled is deliberately NOT written: navigation is turned on with the
# binary's --nav flag, so inference_cam.sh keeps giving a camera-only window.
#
# nav.imu_enabled stays false on purpose. That flag runs the host's own GPS/IMU
# filter, which is the right thing when the IMU is plugged into the PC — but the
# board has already fused these two, and filtering an already-filtered position
# does not average out, it lags.
set_conf nav.map_file    "$MAP_FILE"    "$MAIN_CONF"
set_conf nav.gps_source  "nucleo"       "$MAIN_CONF"
set_conf nav.gps_port    "$GPS_PORT"    "$MAIN_CONF"
set_conf nav.gps_baud    "$GPS_BAUD"    "$MAIN_CONF"
set_conf nav.imu_enabled "false"        "$MAIN_CONF"
set_conf nav.compare_gps "$COMPARE"     "$MAIN_CONF"
set_conf nav.zoom_m      "$ZOOM"        "$MAIN_CONF"
set_conf nav.panel_width "$PANEL"       "$MAIN_CONF"
set_conf nav.heading_up  "$HEADING_UP"  "$MAIN_CONF"
set_conf nav.dark        "$DARK"        "$MAIN_CONF"
set_conf nav.output_size "$OUTPUT_SIZE" "$MAIN_CONF"
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
        echo "       Build them once: $HERE/../build/build_models.sh --precision $WANT" >&2
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
# Generated by inference_cam_nav_imu_gps_ddpai.sh

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

NAV_DESC="nucleo @ $GPS_PORT [$GPS_OK]"
[ -n "$DEST" ] && NAV_DESC="$NAV_DESC -> $DEST${DEST_NAME:+ ($DEST_NAME)}"

echo "=================================================="
echo " VisionPilot — DDPai N5 + map + Nucleo (GPS/IMU fused on the board)"
echo "   camera: $SOURCE_DESC"
echo "   speed:  $SPEED_DESC"
echo "   calib:  $CALIB_DESC"
echo "   model:  $PRECISION_DESC on $PROVIDER_DESC"
echo "   map:    $MAP_FILE"
echo "   gps:    $NAV_DESC"
echo "   fusion: on the board — GPS 1 Hz + IMU 25 Hz -> 20 Hz position"
[ "$COMPARE" = "true" ] &&
    echo "   mode:   comparing — blue ring is the raw receiver, green is fused"
echo "   layout: ${OUTPUT_SIZE}${FIT_DESC} canvas, ${PANEL}px panel, ${ZOOM} m across, $([ "$HEADING_UP" = "true" ] && echo "heading up" || echo "north up")"
echo "   extra:  ${VP_ARGS[*]:-<none>}"
echo "   stop with Ctrl-C"
echo "=================================================="

cd "$BUILD_DIR"
if [ "$USE_QT" = "true" ]; then
    [ -x "$BUILD_DIR/VisionPilotQt" ] || {
        echo "error: $BUILD_DIR/VisionPilotQt not built" >&2
        echo "       Needs Qt5: sudo apt-get install -y qtbase5-dev, then rebuild." >&2
        exit 1; }
    exec ./VisionPilotQt --nav ${VP_ARGS[@]+"${VP_ARGS[@]}"}
fi
exec ./VisionPilot --nav ${VP_ARGS[@]+"${VP_ARGS[@]}"}
