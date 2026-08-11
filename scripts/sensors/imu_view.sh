#!/usr/bin/env bash
#
# Build if needed, make sure the IMU's serial port exists, then open the live 3D
# view. Turn the module in your hand and the aircraft on screen turns with it —
# that is the functional check the whole package exists for.
#
# Usage:
#   ./imu_view.sh [options]
#
# Options:
#   --port <dev>       serial port. Default: the first /dev/ttyUSB* found
#   --baud <n>         line speed. Default: scan 115200, 9600, 921600, ...
#   --protocol <p>     wit | yesense | ascii. Default: auto-detect
#   --raw-attitude     fuse accel/gyro/mag here instead of trusting the module's
#                      own attitude output — use this to check the raw channels
#   --no-mag           leave the magnetometer out of the fusion
#   --monitor          console readout instead of the 3D window (works over ssh)
#   --raw              hex-dump the serial stream and exit
#   --probe            report port, baud rate and protocol, then exit
#   --list             list serial ports, then exit
#   --fix-port         re-bind the USB-serial driver, then exit (needs sudo)
#   --rebuild          configure and build from scratch
#
# Examples:
#   ./imu_view.sh
#   ./imu_view.sh --probe
#   ./imu_view.sh --port /dev/ttyUSB0 --baud 9600
#   ./imu_view.sh --monitor
set -euo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "$0")")/../../VisionPilot/modules/sensing/imu_yahboom_10axis" && pwd)"
BUILD="$ROOT/build"

usage() { sed -n '2,32p' "$0"; exit "${1:-1}"; }

MODE="view"
REBUILD="false"
FIX_PORT="false"
PORT=""
ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --port)          [ $# -ge 2 ] || { echo "error: --port needs a value" >&2; exit 1; }
                         PORT="$2"; ARGS+=(--port "$2"); shift 2 ;;
        --baud)          [ $# -ge 2 ] || { echo "error: --baud needs a value" >&2; exit 1; }
                         ARGS+=(--baud "$2"); shift 2 ;;
        --protocol)      [ $# -ge 2 ] || { echo "error: --protocol needs a value" >&2; exit 1; }
                         ARGS+=(--protocol "$2"); shift 2 ;;
        --raw-attitude)  ARGS+=(--raw-attitude); shift ;;
        --no-mag)        ARGS+=(--no-mag); shift ;;
        --monitor)       MODE="monitor"; shift ;;
        --raw)           MODE="raw"; shift ;;
        --probe)         MODE="probe"; shift ;;
        --list)          MODE="list"; shift ;;
        --csv)           MODE="csv"; shift ;;
        --fix-port)      FIX_PORT="true"; shift ;;
        --rebuild)       REBUILD="true"; shift ;;
        -h|--help)       usage 0 ;;
        *)               echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

# ── Serial port ──────────────────────────────────────────────────────────────
# These boards sit behind a CH340/CP2102/FTDI bridge. The node only appears once
# the kernel serial driver owns the USB interface — anything else holding it
# (libusb through usbfs, a stray flashing tool) leaves the port missing with no
# obvious explanation.
usb_bridge() {   # -> "<usb-id> <driver> <label>" for the first bridge found
    local d vid pid
    for d in /sys/bus/usb/devices/*/; do
        [ -f "$d/idVendor" ] || continue
        vid="$(cat "$d/idVendor")"; pid="$(cat "$d/idProduct" 2>/dev/null || true)"
        case "$vid:$pid" in
            1a86:7523|1a86:5523) echo "$(basename "$d") ch341 CH340"; return 0 ;;
            1a86:55d4)           echo "$(basename "$d") ch341 CH9102"; return 0 ;;
            10c4:ea60)           echo "$(basename "$d") cp210x CP2102"; return 0 ;;
            0403:6001|0403:6015) echo "$(basename "$d") ftdi_sio FTDI"; return 0 ;;
        esac
    done
    return 1
}

fix_port() {
    local info id driver label bound
    if ! info="$(usb_bridge)"; then
        echo "error: no USB-serial bridge is plugged in — check the IMU's USB cable" >&2
        return 1
    fi
    read -r id driver label <<< "$info"
    echo "found $label bridge at USB $id"

    bound="$(basename "$(readlink -f "/sys/bus/usb/devices/$id:1.0/driver" 2>/dev/null || echo none)")"
    if [ "$bound" = "$driver" ] && ls /dev/ttyUSB* >/dev/null 2>&1; then
        echo "already bound to $driver — $(ls /dev/ttyUSB* | tr '\n' ' ')"
        return 0
    fi

    sudo modprobe "$driver" 2>/dev/null || true
    if [ "$bound" != "none" ] && [ "$bound" != "$driver" ]; then
        echo "interface is held by '$bound' — handing it back to $driver"
        echo "$id:1.0" | sudo tee "/sys/bus/usb/drivers/$bound/unbind" >/dev/null 2>&1 || true
    fi
    echo "$id:1.0" | sudo tee "/sys/bus/usb/drivers/$driver/bind" >/dev/null 2>&1 || true
    sleep 1

    if ls /dev/ttyUSB* >/dev/null 2>&1; then
        echo "port ready: $(ls /dev/ttyUSB* | tr '\n' ' ')"
    else
        echo "still no /dev/ttyUSB* — unplug the USB cable, plug it back in, and retry" >&2
        return 1
    fi
}

if [ "$FIX_PORT" = "true" ]; then
    fix_port
    exit $?
fi

# ── Build ────────────────────────────────────────────────────────────────────
if [ "$REBUILD" = "true" ]; then
    rm -rf "$BUILD"
fi
if [ ! -x "$BUILD/imu_monitor" ] || [ "$REBUILD" = "true" ]; then
    echo "building the IMU package in $BUILD ..."
    cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "$BUILD" -j"$(nproc)" >/dev/null
    echo "built."
fi

MONITOR="$BUILD/imu_monitor"
VIEWER="$BUILD/imu_view"

# ── Port check ───────────────────────────────────────────────────────────────
# One glob at a time: `ls /dev/ttyUSB* /dev/ttyACM*` exits non-zero whenever
# *either* pattern matches nothing, so testing both together reports "no port"
# on a machine that has one.
first_port() {
    local p
    for p in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$p" ] && { echo "$p"; return 0; }
    done
    return 1
}

if [ -n "$PORT" ]; then
    [ -e "$PORT" ] || { echo "error: $PORT does not exist" >&2; "$MONITOR" --list || true; exit 1; }
elif ! first_port >/dev/null; then
    echo "no serial port yet — trying to bind the USB-serial driver"
    fix_port || {
        echo "" >&2
        echo "The IMU is not reachable. Check the USB cable, then run:" >&2
        echo "  $0 --fix-port" >&2
        exit 1; }
fi

FOUND_PORT="${PORT:-$(first_port || true)}"
if [ -n "$FOUND_PORT" ] && [ ! -r "$FOUND_PORT" ]; then
    echo "warning: no read permission on $FOUND_PORT." >&2
    echo "         sudo usermod -aG dialout $USER    (then log out and back in)" >&2
fi

case "$MODE" in
    list)    exec "$MONITOR" --list ;;
    probe)   exec "$MONITOR" --probe ${PORT:+--port "$PORT"} ;;
    raw)     exec "$MONITOR" --raw ${PORT:+--port "$PORT"} ;;
    csv)     exec "$MONITOR" --csv "${ARGS[@]+${ARGS[@]}}" ;;
    monitor) exec "$MONITOR" "${ARGS[@]+${ARGS[@]}}" ;;
esac

if [ ! -x "$VIEWER" ]; then
    echo "error: the 3D viewer was not built — Qt5 is missing." >&2
    echo "       sudo apt-get install -y qtbase5-dev, then: $0 --rebuild" >&2
    echo "       In the meantime the console readout works: $0 --monitor" >&2
    exit 1
fi

echo "=================================================="
echo " Yahboom 10-axis IMU — live 3D attitude"
echo "   port:  ${PORT:-auto ($FOUND_PORT)}"
echo "   keys:  Space zero here · R reset position · C recalibrate · V reset view"
echo "   drag to orbit the camera, wheel to zoom"
echo "=================================================="

exec "$VIEWER" "${ARGS[@]+${ARGS[@]}}"
