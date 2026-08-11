#!/usr/bin/env bash
#
# Stream the DDPai N5 dashcam. One command, live camera only.
#
# It does the whole job: joins the camera's Wi-Fi, waits for the camera to
# answer, opens the window — and puts your normal Wi-Fi back when you quit.
#
# Usage:
#   ./ddpai_stream.sh [options]
#
# Options:
#       --rear           the rear camera instead of the front one
#   -u, --url <url>      an explicit URL instead of the known ones
#   -r, --record <file>  record while watching (.mp4)
#       --udp            RTSP over UDP — drops late packets instead of stalling
#                        to retransmit them. Lower delay, occasional tearing
#       --hud            show the stats overlay (fps, dropped, uptime)
#       --keep-watermark keep the camera's burnt-in name and clock
#       --scale <x>      resize the view, e.g. 0.5
#       --print-url      find the working URL, print it, and exit (stays on the
#                        camera's Wi-Fi — used by the inference scripts)
#       --ssid <name>    the camera's network (default: DDPAI_N5Dual_4C83)
#       --password <pw>  needed only the first time; after that it is saved
#       --keep-wifi      stay on the camera's network after quitting
#       --no-wifi        do not touch Wi-Fi, assume it is already connected
#
# Keys: q/Esc quit · s snapshot · r record · h HUD · f fullscreen
#
# The camera's access point has no internet, so this machine drops off its
# normal network while streaming — unavoidable, the dashcam is only reachable
# over its own AP. It never appears as /dev/video*: it is not a USB Video Class
# device, so no cable makes it one.
set -euo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
VIEWER="$ROOT/VisionPilot/modules/sensing/camera_interface/tools/ddpai_stream.py"

SSID="DDPAI_N5Dual_4C83"
HOST="193.168.0.1"           # DDPai really uses 193.168.0.x, not 192.168

# Confirmed against the camera, and matching the API that the DDPai app itself
# drives (github.com/awexandrr/ddpai-api): av4 is the front lens, av5 the rear.
URL_FRONT="rtsp://$HOST:554/liveRTSP/av4"
URL_REAR="rtsp://$HOST:554/liveRTSP/av5"

PASSWORD=""
URL=""
REAR="false"
RECORD=""
SCALE=""
TRANSPORT="tcp"
HUD="false"
KEEP_WATERMARK="false"
PRINT_URL="false"
KEEP_WIFI="false"
TOUCH_WIFI="true"

usage() { sed -n '2,32p' "$0"; exit "${1:-1}"; }

while [ $# -gt 0 ]; do
    case "$1" in
        -u|--url)         [ $# -ge 2 ] || { echo "error: --url needs a value" >&2; exit 1; }
                          URL="$2"; shift 2 ;;
        -r|--record)      [ $# -ge 2 ] || { echo "error: --record needs a file" >&2; exit 1; }
                          RECORD="$2"; shift 2 ;;
        --scale)          [ $# -ge 2 ] || { echo "error: --scale needs a value" >&2; exit 1; }
                          SCALE="$2"; shift 2 ;;
        --ssid)           [ $# -ge 2 ] || { echo "error: --ssid needs a name" >&2; exit 1; }
                          SSID="$2"; shift 2 ;;
        --password)       [ $# -ge 2 ] || { echo "error: --password needs a value" >&2; exit 1; }
                          PASSWORD="$2"; shift 2 ;;
        --rear)           REAR="true"; shift ;;
        --udp)            TRANSPORT="udp"; shift ;;
        --tcp)            TRANSPORT="tcp"; shift ;;
        --hud)            HUD="true"; shift ;;
        --keep-watermark) KEEP_WATERMARK="true"; shift ;;
        --print-url)      PRINT_URL="true"; KEEP_WIFI="true"; shift ;;
        --keep-wifi)      KEEP_WIFI="true"; shift ;;
        --no-wifi)        TOUCH_WIFI="false"; shift ;;
        -h|--help)        usage 0 ;;
        *)                echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

[ -f "$VIEWER" ] || { echo "error: $VIEWER is missing" >&2; exit 1; }

# Everything the script says about its progress goes to stderr under
# --print-url, so that stdout carries the URL and nothing else.
say() { if [ "$PRINT_URL" = "true" ]; then echo "$@" >&2; else echo "$@"; fi; }

if [ "$PRINT_URL" = "false" ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "error: no display — this opens a window, so it needs a desktop session." >&2
    echo "       Over ssh: ssh -X" >&2
    exit 1
fi

# ── cv2 ──────────────────────────────────────────────────────────────────────
# The C++ OpenCV on this machine is a separate install and ships no Python
# bindings, so the wheel is what makes `import cv2` work.
if [ "$PRINT_URL" = "false" ] && ! python3 -c "import cv2" >/dev/null 2>&1; then
    say "installing OpenCV for Python (cv2) ..."
    python3 -m pip install --quiet opencv-python || {
        echo "error: pip could not install opencv-python." >&2; exit 1; }
fi

# ── Wi-Fi ────────────────────────────────────────────────────────────────────
# NetworkManager pulls back to a known network within seconds, because the
# camera's AP has no internet — so the previous network's autoconnect has to be
# switched off for the duration, and switched back on afterwards. That restore
# runs on every exit path, including Ctrl-C.
PREV_WIFI=""

restore_wifi() {
    [ "$KEEP_WIFI" = "true" ] && return 0
    [ -n "$PREV_WIFI" ] || return 0
    say ""
    say "restoring Wi-Fi: $PREV_WIFI"
    nmcli connection modify "$PREV_WIFI" connection.autoconnect yes >/dev/null 2>&1 || true
    nmcli connection up "$PREV_WIFI" >/dev/null 2>&1 || true
}

connect_camera() {
    command -v nmcli >/dev/null 2>&1 || {
        echo "error: nmcli not found — connect to $SSID yourself, then use --no-wifi" >&2
        exit 1; }

    if ip route 2>/dev/null | grep -q "^default via $HOST"; then
        say "already on the camera's network"
        return 0
    fi

    PREV_WIFI="$(nmcli -t -f NAME,TYPE connection show --active 2>/dev/null \
                 | awk -F: '$2=="802-11-wireless" {print $1; exit}')"
    if [ -n "$PREV_WIFI" ]; then
        say "leaving $PREV_WIFI (its autoconnect goes off while streaming)"
        nmcli connection modify "$PREV_WIFI" connection.autoconnect no >/dev/null 2>&1 || true
        trap restore_wifi EXIT INT TERM
    fi

    say "joining $SSID ..."
    if nmcli -t -f NAME connection show 2>/dev/null | grep -qx "$SSID"; then
        nmcli connection up "$SSID" >/dev/null 2>&1 || true      # password already saved
    elif [ -n "$PASSWORD" ]; then
        nmcli device wifi connect "$SSID" password "$PASSWORD" >/dev/null 2>&1 || true
    else
        echo "error: no saved profile for $SSID — pass it once with --password <pw>" >&2
        echo "       (the password is in the DDPai app, camera Wi-Fi settings)" >&2
        exit 1
    fi

    for _ in $(seq 1 15); do
        if ping -c1 -W1 "$HOST" >/dev/null 2>&1; then
            say "camera answers at $HOST"
            return 0
        fi
        sleep 1
    done

    echo "" >&2
    echo "error: joined the network but $HOST does not answer." >&2
    echo "       Power-cycle the camera (unplug, wait five seconds, plug back in)." >&2
    exit 1
}

[ "$TOUCH_WIFI" = "true" ] && connect_camera

if ! ping -c1 -W2 "$HOST" >/dev/null 2>&1; then
    echo "error: $HOST does not answer — not on the camera's Wi-Fi." >&2
    echo "       Drop --no-wifi and this script will join it for you." >&2
    exit 1
fi

# ── The URL ──────────────────────────────────────────────────────────────────
# The path is known now, so there is normally nothing to search for. The search
# is kept only as a fallback for firmware that moves it.
if [ -z "$URL" ]; then
    if [ "$REAR" = "true" ]; then URL="$URL_REAR"; else URL="$URL_FRONT"; fi

    if command -v ffprobe >/dev/null 2>&1; then
        if ! timeout 10 ffprobe -v error -rtsp_transport tcp \
                 -show_entries stream=width,height -of csv=p=0 "$URL" >/dev/null 2>&1; then
            say "$URL did not answer — searching ..."
            for candidate in \
                "rtsp://$HOST/liveRTSP/av4" \
                "rtsp://$HOST/live" \
                "rtsp://$HOST:8554/live" \
                "rtsp://$HOST:6200/live"; do
                say "  trying $candidate"
                if timeout 8 ffprobe -v error -rtsp_transport tcp \
                       -show_entries stream=width,height -of csv=p=0 "$candidate" >/dev/null 2>&1; then
                    URL="$candidate"
                    break
                fi
            done
        fi
    fi
fi

if [ "$PRINT_URL" = "true" ]; then
    echo "$URL"          # stdout: the URL alone, for the inference scripts
    exit 0
fi

# ── Go ───────────────────────────────────────────────────────────────────────
echo ""
echo "=================================================="
echo " DDPai N5 — live ($([ "$REAR" = "true" ] && echo "rear" || echo "front") camera)"
echo "   $URL over $TRANSPORT"
echo "   keys: q quit · s snapshot · r record · h HUD · f fullscreen"
echo "=================================================="

ARGS=("$URL" --transport "$TRANSPORT")
[ -n "$RECORD" ] && ARGS+=(--record "$RECORD")
[ -n "$SCALE" ]  && ARGS+=(--scale "$SCALE")
[ "$HUD" = "true" ] && ARGS+=(--hud)
[ "$KEEP_WATERMARK" = "true" ] && ARGS+=(--keep-watermark)

# Not exec: the Wi-Fi restore trap has to run after the viewer exits.
python3 "$VIEWER" "${ARGS[@]}" || true
