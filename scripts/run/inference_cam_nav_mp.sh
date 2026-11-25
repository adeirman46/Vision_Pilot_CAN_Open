#!/usr/bin/env bash
#
# Run VisionPilot on a live camera with the map panel drawn on SATELLITE
# IMAGERY: the camera frame and its HUD on the left, aerial photography with the
# road network, the route and the next manoeuvre on the right.
#
#   ┌──────────────────────────┬──────────────┐
#   │  camera frame + HUD      │  satellite   │
#   │                          │  map + route │
#   └──────────────────────────┴──────────────┘
#
# Same camera, same GPS and same options as inference_cam_nav.sh — this is that
# script with the drawn land replaced by imagery, plus one thing it cannot do:
#
#   CLICK ANYWHERE ON THE MAP AND IT ROUTES THERE.
#
# No name to type, no coordinates to look up. Tap the junction, the gate, the
# corner of the complex you are heading for and the route is computed from where
# you are to where you tapped. The search bar still works exactly as before; the
# click is an addition, not a replacement. Drag still pans — a press that
# travels moves the map, a press that stays put sets the destination.
#
# The imagery is cached on disk (VisionPilot/maps/tiles). The first drive down a
# street downloads it; every drive after that needs no network at all. With no
# network and nothing cached the panel falls back to the drawn map rather than
# going blank, and says so.
#
# Usage:
#   ./inference_cam_nav_mp.sh [camera-source] [options] [-- <VisionPilot args>]
#
# The camera source is a stream URL or a local camera, same as inference_cam.sh:
#   http://<ip>:8080/video   phone running IP Webcam (Android) — the default form
#   rtsp://<host>/<path>     any RTSP camera
#   0 | /dev/video0          a USB/laptop camera
# Default: $VP_CAM_URL, else http://10.130.12.119:8081/video
#
# Position options:
#   --gps <src>          phyphox (default) | ipwebcam | fixed | file | none
#   --gps-url <url>      GPS host. Default $VP_GPS_URL, else http://172.27.74.24:8080
#                        phyphox: open the "Location (GPS)" experiment, three-dot
#                        menu -> Allow remote access, press play. It answers on
#                        port 8080, so it and IP Webcam cannot share one port.
#                        ipwebcam: omit to reuse the camera's own host and port.
#   --gps-file <file>    recorded track for --gps file, one
#                        "lat,lon[,speed_ms[,bearing_deg]]" per line
#   --fixed-pos <lat,lon>  position for --gps fixed (default Bandung centre)
#
# Imagery options:
#   --vector             drawn map instead of imagery — i.e. inference_cam_nav.sh's
#                        panel, but keeping the click-to-route. ('Satellite' in
#                        the window toggles it back at any time.)
#   --roads <mode>       road network traced over the imagery:
#                        all (default) | major | off. "all" is the same network,
#                        at the same widths and in the same order, that
#                        inference_cam_nav.sh draws — the photograph shows where
#                        the tarmac is, those lines show where the map thinks it
#                        is, and the route and the speed limit come from the map.
#   --road-opacity <a>   how solidly it is drawn, 0..1 (0.85). 1.0 is exactly the
#                        drawn map; lower lets more of the photograph through.
#   --tiles <dir>        tile cache. Default VisionPilot/maps/tiles
#   --tile-url <tpl>     tile server, with {z}/{x}/{y} placeholders. Default is
#                        Esri World Imagery, which serves them as z/y/x.
#   --tile-zoom <n>      deepest zoom to request (19). Asking past what the
#                        provider holds only fills the cache with misses.
#   --offline-tiles      never download; draw only what is already cached
#   --credit <text>      attribution printed in the corner of the map
#
# Map options:
#   --dest <lat,lon|name>  destination — draws the route and the turn-by-turn
#                        banner. Either coordinates or a street name looked up in
#                        the map itself, e.g. --dest "Jalan Braga". The match
#                        nearest your current position wins. Usually unnecessary
#                        now: tap the map instead.
#   --find <text>        list places and roads matching <text>, then exit
#   --ask                ask for the destination in the terminal instead of
#                        typing it into the app window
#   --dest-name <text>   label for that destination on the panel
#   --map <file>         .vpmap to load. Default VisionPilot/maps/bandung-roads.vpmap
#   --zoom <m>           metres across the panel width (300 — imagery is worth
#                        looking at closer than the drawn map is)
#   --light              light / daytime map ('t' toggles it while running).
#                        Imagery is drawn as-is in light mode and dimmed a little
#                        in dark, so the route and the car stay readable at night.
#   --opencv             use the old OpenCV highgui window instead of the Qt
#                        front-end. Click-to-route works there too; the search
#                        bar does not — press 'd' for the typed destination box.
#   --panel <px>         map panel width in px (760)
#   --north-up           north-up map instead of heading-up
#
# IMU options (see scripts/run/inference_cam_nav_imu.sh):
#   --imu                fuse the Yahboom IMU with the GPS. Position updates at
#                        IMU rate instead of once a second, stops jumping between
#                        fixes, and keeps a heading while the car is stopped.
#   --imu-port <dev>     serial port. Default: the first /dev/ttyUSB* found
#   --imu-baud <n>       line speed. Default: auto-detect
#   --gps-only           raw receiver, no IMU — the baseline to compare against
#   --compare            fuse, and draw the raw receiver beside the fused
#                        position so the difference is visible while driving
#   --output <WxH|fit>   canvas size. Default: the screen minus room for the
#                        title bar, so the window manager never has to shrink
#                        the window and the toolkit never pads it with white.
#                        "fit" instead derives the height so the camera view
#                        fills its half exactly — no black bars, short window.
#   --check              check the map, the camera, the GPS feed and the tile
#                        server, then exit
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
#   ./inference_cam_nav_mp.sh                       # then tap the map
#   ./inference_cam_nav_mp.sh --zoom 200            # closer in on the imagery
#   ./inference_cam_nav_mp.sh --dest "Jalan Braga"  # route before it even starts
#   ./inference_cam_nav_mp.sh --offline-tiles       # no network, cache only
#   ./inference_cam_nav_mp.sh --vector              # drawn map, click still works
#   ./inference_cam_nav_mp.sh --check
#
# Build the map once first:  ./scripts/build/build_osm_map.sh
#
# Env overrides: VP_ROOT, VP_BUILD, VP_CAM_URL, VP_GPS_URL, VP_TILE_DIR
set -euo pipefail

VP_ROOT="${VP_ROOT:-$(cd "$(dirname "$(readlink -f "$0")")/../../VisionPilot" && pwd)}"
BUILD_DIR="${VP_BUILD:-$VP_ROOT/build}"
DEFAULT_URL="${VP_CAM_URL:-http://10.130.9.41:8081/video}"
DEFAULT_GPS="${VP_GPS_URL:-http://172.27.74.24:8080}"

# ── Finding the phone ────────────────────────────────────────────────────────
# The address IP Webcam and phyphox print on the phone's own screen is the first
# interface they find, and with a VPN running that is the tunnel — 172.16.x.x,
# 10.x.x.x, reachable from the phone and from nowhere else. Worse, it usually
# still answers ping, because it routes off to some unrelated machine, so it
# looks alive right up until the HTTP request times out.
#
# When the phone is tethering, it is this laptop's default gateway, and the
# route table knows that no matter what the phone believes about itself. So a
# host that does not answer is retried against the gateway before giving up.
GATEWAY="$(ip route show default 2>/dev/null | awk '/^default/ { print $3; exit }')"

# Does something speak HTTP at <host:port>? Root, not the stream: an MJPEG
# endpoint never finishes, so a request against it only returns on the timeout.
http_alive() {
    command -v curl >/dev/null 2>&1 || return 0   # cannot tell — do not block
    curl -s -m 4 -o /dev/null "http://$1/" 2>/dev/null
}

# Echoes a working "host:port" for the port in $2, or nothing. $1 is the
# host:port to try first.
find_phone() {
    local hostport="$1" port="${1##*:}" host="${1%:*}"
    if http_alive "$hostport"; then printf '%s' "$hostport"; return 0; fi
    if [ -n "$GATEWAY" ] && [ "$host" != "$GATEWAY" ] && http_alive "$GATEWAY:$port"; then
        printf '%s' "$GATEWAY:$port"
        return 0
    fi
    return 1
}

usage() { sed -n '2,120p' "$0"; exit "${1:-1}"; }

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

IMU="false"          # --imu turns on the GPS/IMU filter
COMPARE="false"      # --compare also draws the unfiltered receiver
IMU_PORT=""
IMU_BAUD=""

GPS="phyphox"
GPS_URL=""
GPS_FILE=""
FIXED_POS=""
DEST=""
DEST_NAME=""
FIND=""
MAP_FILE=""
OUTPUT_SIZE=""            # empty = fit the screen; or WxH, or "fit"
ZOOM="300"
PANEL="760"
HEADING_UP="true"
DARK="true"
USE_QT="true"    # --opencv falls back to the old highgui window

# ── Imagery ──────────────────────────────────────────────────────────────────
# This is what separates this script from inference_cam_nav.sh.
SATELLITE="true"
TILE_DIR="${VP_TILE_DIR:-$VP_ROOT/maps/tiles}"
TILE_URL="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
TILE_ZOOM="19"
TILE_OFFLINE="false"
CREDIT="Imagery (c) Esri"
# The whole road network over the imagery, as inference_cam_nav.sh draws it.
ROADS="all"
ROAD_OPACITY="0.85"

while [ $# -gt 0 ]; do
    case "$1" in
        --gps)         [ $# -ge 2 ] || { echo "error: --gps needs a value" >&2; exit 1; }
                       case "$2" in phyphox|ipwebcam|fixed|file|none) ;;
                           *) echo "error: --gps must be phyphox, ipwebcam, fixed, file or none" >&2; exit 1 ;;
                       esac
                       GPS="$2"; shift 2 ;;
        --gps-url)     [ $# -ge 2 ] || { echo "error: --gps-url needs a value" >&2; exit 1; }
                       GPS_URL="$2"; shift 2 ;;
        --phone)       # One phone, two apps, two ports — say the address once.
                       [ $# -ge 2 ] || { echo "error: --phone needs an IP" >&2; exit 1; }
                       PHONE="$2"; shift 2 ;;
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
        --satellite)   SATELLITE="true";  shift ;;
        --vector|--no-satellite)
                       SATELLITE="false"; shift ;;
        --tiles)       [ $# -ge 2 ] || { echo "error: --tiles needs a directory" >&2; exit 1; }
                       TILE_DIR="$2"; SATELLITE="true"; shift 2 ;;
        --tile-url)    [ $# -ge 2 ] || { echo "error: --tile-url needs a template" >&2; exit 1; }
                       TILE_URL="$2"; SATELLITE="true"; shift 2 ;;
        --tile-zoom)   [ $# -ge 2 ] || { echo "error: --tile-zoom needs a value" >&2; exit 1; }
                       case "$2" in *[!0-9]*|"") echo "error: --tile-zoom must be a number" >&2; exit 1 ;; esac
                       [ "$2" -ge 1 ] && [ "$2" -le 22 ] ||
                           { echo "error: --tile-zoom must be between 1 and 22" >&2; exit 1; }
                       TILE_ZOOM="$2"; shift 2 ;;
        --roads)       [ $# -ge 2 ] || { echo "error: --roads needs all, major or off" >&2; exit 1; }
                       case "$2" in all|major|off|none) ;;
                           *) echo "error: --roads must be all, major or off" >&2; exit 1 ;;
                       esac
                       ROADS="$2"; shift 2 ;;
        --road-opacity) [ $# -ge 2 ] || { echo "error: --road-opacity needs 0..1" >&2; exit 1; }
                       awk -v a="$2" 'BEGIN { exit !(a >= 0 && a <= 1) }' ||
                           { echo "error: --road-opacity must be between 0 and 1" >&2; exit 1; }
                       ROAD_OPACITY="$2"; shift 2 ;;
        --offline-tiles) TILE_OFFLINE="true"; shift ;;
        --credit)      [ $# -ge 2 ] || { echo "error: --credit needs a value" >&2; exit 1; }
                       CREDIT="$2"; shift 2 ;;
        --imu)         IMU="true"; shift ;;
        --no-imu)      IMU="false"; shift ;;
        --gps-only)    IMU="false"; COMPARE="false"; shift ;;
        --compare)     IMU="true";  COMPARE="true";  shift ;;
        --imu-port)    [ $# -ge 2 ] || { echo "error: --imu-port needs a device" >&2; exit 1; }
                       IMU_PORT="$2"; IMU="true"; shift 2 ;;
        --imu-baud)    [ $# -ge 2 ] || { echo "error: --imu-baud needs a value" >&2; exit 1; }
                       IMU_BAUD="$2"; IMU="true"; shift 2 ;;
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

case "$FIXED_POS" in
    "") ;;
    *,*) ;;
    *) echo "error: --fixed-pos expects lat,lon, got '$FIXED_POS'" >&2; exit 1 ;;
esac

# The URL is written into a config file the binary reads, and the tile fetcher
# substitutes only digits into it. Refuse anything that is not a plain http(s)
# template rather than passing whatever was typed through to a downloader.
case "$TILE_URL" in
    http://*|https://*) ;;
    *) echo "error: --tile-url must start with http:// or https://" >&2; exit 1 ;;
esac
case "$TILE_URL" in
    *"{z}"*) ;;
    *) echo "error: --tile-url needs {z}, {x} and {y} placeholders" >&2; exit 1 ;;
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
        GPS_URL="${GPS_URL:-$DEFAULT_GPS}"
        # phyphox is a second app on a second port of the same phone, so it goes
        # missing for exactly the same reason the camera does. Same recovery.
        GPS_HOSTPORT="${GPS_URL#*://}"; GPS_HOSTPORT="${GPS_HOSTPORT%%/*}"
        if FOUND_GPS="$(find_phone "$GPS_HOSTPORT")" &&
           [ "$FOUND_GPS" != "$GPS_HOSTPORT" ]; then
            GPS_URL="${GPS_URL/$GPS_HOSTPORT/$FOUND_GPS}"
            echo "note: phyphox not at $GPS_HOSTPORT — using $FOUND_GPS instead."
        fi ;;
    file)
        [ -f "$GPS_FILE" ] || { echo "error: --gps file needs --gps-file <track>" >&2; exit 1; } ;;
esac

# ── Checks ───────────────────────────────────────────────────────────────────
CAM_OK="unknown"
if [ "$MODE" = "stream" ]; then
    HOSTPORT="${URL#*://}"; HOSTPORT="${HOSTPORT%%/*}"
    case "$URL" in http://*|https://*)
        if command -v curl >/dev/null 2>&1; then
            if FOUND="$(find_phone "$HOSTPORT")"; then
                if [ "$FOUND" != "$HOSTPORT" ]; then
                    # Rewrite the whole URL, not just the check: the binary opens
                    # this address, so telling the user it moved and then handing
                    # the old one to the pipeline would be worse than failing.
                    URL="${URL/$HOSTPORT/$FOUND}"
                    echo "note: nothing answered at $HOSTPORT — using $FOUND instead."
                    echo "      That is this laptop's default gateway, i.e. the phone that is"
                    echo "      sharing its connection. The address on the phone's own screen"
                    echo "      is its VPN interface, which is only reachable from the phone."
                    HOSTPORT="$FOUND"
                fi
                CAM_OK="reachable"
            else
                echo "error: cannot reach the camera at $HOSTPORT" >&2
                [ -n "$GATEWAY" ] &&
                    echo "       Nothing on ${GATEWAY}:${HOSTPORT##*:} either." >&2
                echo "       Check the phone is awake, IP Webcam is started, and both" >&2
                echo "       are on the same network." >&2
                echo "       If the phone shows a VPN address (172.16.x.x, 10.x.x.x), that" >&2
                echo "       is the tunnel, not the LAN — turn the VPN off, or pass the" >&2
                echo "       phone's real address:  $0 --phone <ip>" >&2
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

# ── Imagery ──────────────────────────────────────────────────────────────────
# Cached tiles are the point: they make the panel work with the phone's data off
# and the network gone, which is where this ends up being used. Count what is
# already there, and check the server can be reached at all before a drive that
# expects it to be.
TILES_DESC="off — drawn map"
if [ "$SATELLITE" = "true" ]; then
    mkdir -p "$TILE_DIR" 2>/dev/null || {
        echo "warning: cannot create tile cache $TILE_DIR — imagery will not be kept" >&2
        TILE_DIR=""
    }
    CACHED=0
    [ -n "$TILE_DIR" ] && CACHED="$(find "$TILE_DIR" -type f -name '*.jpg' 2>/dev/null | wc -l)"

    if [ "$TILE_OFFLINE" = "true" ]; then
        TILES_DESC="offline, $CACHED tiles cached in ${TILE_DIR:-<none>}"
        [ "$CACHED" -gt 0 ] || {
            echo "warning: --offline-tiles with an empty cache — the panel will fall back" >&2
            echo "         to the drawn map. Run once with a network first." >&2; }
    elif command -v curl >/dev/null 2>&1; then
        # One real tile over Bandung, so this fails the same way the app would.
        PROBE="${TILE_URL//\{z\}/13}"; PROBE="${PROBE//\{y\}/4256}"; PROBE="${PROBE//\{x\}/6544}"
        if curl -s -f -m 8 -o /dev/null "$PROBE"; then
            TILES_DESC="on, $CACHED tiles cached, server reachable"
        else
            TILES_DESC="on, $CACHED tiles cached, SERVER UNREACHABLE"
            echo "warning: no answer from the tile server" >&2
            echo "         $PROBE" >&2
            echo "         Cached tiles still draw; anywhere new falls back to the drawn map." >&2
        fi
    else
        TILES_DESC="on, $CACHED tiles cached (curl missing — no downloads)"
        echo "warning: curl is not installed; tiles can only come from the cache" >&2
    fi
fi

# ── Map ──────────────────────────────────────────────────────────────────────
MAP_FILE="${MAP_FILE:-$VP_ROOT/maps/bandung-roads.vpmap}"
[ -f "$MAP_FILE" ] || {
    echo "error: navigation map not found: $MAP_FILE" >&2
    echo "       Build it once (greater Bandung, a few minutes):" >&2
    echo "         $(cd "$VP_ROOT/.." && pwd)/scripts/build/build_osm_map.sh" >&2
    exit 1; }

# ── Destination by name ──────────────────────────────────────────────────────
# Still here, and still useful before the window is even open. Once it is, the
# quicker answer is to tap the place on the map.
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
# video window, and the map itself is now the fastest way to pick a place.
# --ask is for scripted or headless runs where there is no window to tap.
if [ -z "$DEST" ] && [ "$CHECK" = "false" ] && [ "$ASK" = "true" ] && [ -t 0 ]; then
    echo ""
    echo "┌─ Where do you want to go? ──────────────────────────────────────────┐"
    echo "│  a place or street name   e.g. Trans Studio Mall, Jalan Braga       │"
    echo "│  coordinates              e.g. -6.9034,107.6181                     │"
    echo "│  Enter on its own         no route — tap the map once it is open    │"
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
    echo "tiles:  $TILES_DESC"
    [ "$SATELLITE" = "true" ] && echo "        $TILE_URL"
    [ -n "$DEST" ] && echo "dest:   ${DEST_NAME:-$DEST}  ($DEST)"
    [ "$MODE" = "stream" ] && [ -z "${INFO:-}" ] &&
        echo "warning: ffprobe could not read a video stream from the camera"
    exit 0
fi

[ -x "$BUILD_DIR/VisionPilot" ] || {
    echo "error: $BUILD_DIR/VisionPilot not built" >&2
    echo "       Set the machine up and build it in one step:" >&2
    echo "         $(cd "$VP_ROOT/.." && pwd)/scripts/build/setup.sh" >&2
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

# ── Canvas size ──────────────────────────────────────────────────────────────
# With no --output, size the canvas so the camera view exactly fills the space
# left of the panel and nothing is padded. The HUD frame is 2:1 (1024x512), so
# for a canvas W wide with a P-wide panel the camera needs (W - P) / 2 of height
# — make that the canvas height and there are no black bars anywhere.
if [ "$OUTPUT_SIZE" = "fit" ]; then
    # Height derived so the 2:1 camera view fills its half exactly: no black
    # bars anywhere, at the cost of a short window.
    CANVAS_W=1920
    CANVAS_H=$(( (CANVAS_W - PANEL) / 2 ))
    OUTPUT_SIZE="${CANVAS_W}x${CANVAS_H}"
    FIT_DESC=" (camera-fitted, no black bars)"
elif [ -z "$OUTPUT_SIZE" ]; then
    # Fit the screen's usable area. A canvas as tall as the screen does not fit
    # once the title bar is added, so the window manager shrinks the window and
    # the toolkit pads the leftover with its own background — that is the white
    # border. Leaving room for the decoration means there is nothing to pad.
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
set_conf nav.map_file    "$MAP_FILE" "$MAIN_CONF"
set_conf nav.gps_source  "$GPS"      "$MAIN_CONF"
set_conf nav.gps_url     "$GPS_URL"  "$MAIN_CONF"
set_conf nav.zoom_m      "$ZOOM"     "$MAIN_CONF"
set_conf nav.panel_width "$PANEL"    "$MAIN_CONF"
set_conf nav.heading_up  "$HEADING_UP" "$MAIN_CONF"
set_conf nav.dark        "$DARK"       "$MAIN_CONF"
set_conf nav.output_size "$OUTPUT_SIZE" "$MAIN_CONF"
set_conf nav.gps_file    "$GPS_FILE" "$MAIN_CONF"
set_conf nav.imu_enabled "$IMU"      "$MAIN_CONF"
set_conf nav.imu_port    "$IMU_PORT" "$MAIN_CONF"
set_conf nav.imu_baud    "${IMU_BAUD:-0}" "$MAIN_CONF"
set_conf nav.compare_gps "$COMPARE"  "$MAIN_CONF"
[ -n "$FIXED_POS" ] && set_conf nav.fixed_position   "$FIXED_POS" "$MAIN_CONF"
set_conf nav.destination      "$DEST"      "$MAIN_CONF"
set_conf nav.destination_name "$DEST_NAME" "$MAIN_CONF"
# This script plans a single destination, so it clears any multi-waypoint
# mission left in the shared config by inference_cam_nav_follow.sh. Without this
# line a mission planned there would still be in force here, and the route drawn
# would tour points nobody asked this script for.
set_conf nav.waypoints        ""           "$MAIN_CONF"

# ── Imagery ──────────────────────────────────────────────────────────────────
# The only keys that differ from inference_cam_nav.sh. They are written every
# run, including nav.satellite = false under --vector, so switching between the
# two scripts never leaves the other one's setting behind.
set_conf nav.satellite        "$SATELLITE"    "$MAIN_CONF"
set_conf nav.tile_url         "$TILE_URL"     "$MAIN_CONF"
set_conf nav.tile_cache_dir   "$TILE_DIR"     "$MAIN_CONF"
set_conf nav.tile_attribution "$CREDIT"       "$MAIN_CONF"
set_conf nav.tile_max_zoom    "$TILE_ZOOM"    "$MAIN_CONF"
set_conf nav.tile_offline     "$TILE_OFFLINE" "$MAIN_CONF"
set_conf nav.satellite_roads         "$ROADS"        "$MAIN_CONF"
set_conf nav.satellite_roads_opacity "$ROAD_OPACITY" "$MAIN_CONF"

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
    CACHED_ENGINES=0
    for engine in "$CACHE_DIR"/*"_${WANT}_"*.engine; do
        [ -e "$engine" ] && CACHED_ENGINES=$((CACHED_ENGINES + 1))
    done
    if [ "$CACHED_ENGINES" -lt 3 ]; then
        echo "error: only $CACHED_ENGINES of 3 $WANT TensorRT engines are cached in $CACHE_DIR" >&2
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
# Generated by inference_cam_nav_mp.sh

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
echo " VisionPilot live camera + satellite map navigation"
echo "   camera: $SOURCE_DESC"
echo "   speed:  $SPEED_DESC"
echo "   calib:  $CALIB_DESC"
echo "   model:  $PRECISION_DESC on $PROVIDER_DESC"
echo "   map:    $MAP_FILE"
echo "   tiles:  $TILES_DESC"
[ "$SATELLITE" = "true" ] && echo "           cache $TILE_DIR"
[ "$SATELLITE" = "true" ] && echo "   roads:  $ROADS over the imagery at ${ROAD_OPACITY} opacity"
echo "   gps:    $NAV_DESC"
if [ "$IMU" = "true" ]; then
    echo "   imu:    ${IMU_PORT:-auto} @ ${IMU_BAUD:-auto} — GPS/IMU filter on"
    [ "$COMPARE" = "true" ] &&
        echo "   mode:   comparing — blue ring is the raw receiver, green is fused"
else
    echo "   imu:    off — raw GPS only (baseline)"
fi
echo "   layout: ${OUTPUT_SIZE}${FIT_DESC} canvas, ${PANEL}px panel, ${ZOOM} m across, $([ "$HEADING_UP" = "true" ] && echo "heading up" || echo "north up")"
echo "   extra:  ${VP_ARGS[*]:-<none>}"
echo ""
echo "   CLICK THE MAP to route there. Drag to pan, wheel to zoom."
echo "   Search bar, 'Satellite'/'Map', 'Clear route' are along the panel."
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
