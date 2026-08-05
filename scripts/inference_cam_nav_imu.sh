#!/usr/bin/env bash
#
# inference_cam_nav.sh with the Yahboom 10-axis IMU fused into the position.
#
# Same window as the plain nav run — camera and HUD on the left, map and route on
# the right — but the dot on the map is now the output of a GPS/IMU filter rather
# than the raw receiver:
#
#   GPS alone           one fix a second, jumping several metres between them,
#                       and no idea which way the car points when it is stopped
#   GPS + IMU           position at IMU rate, moving smoothly, heading that
#                       survives a stop, a tunnel, or a row of tall buildings
#
# The filter is a 6-state EKF — north, east, velocity north/east, heading, and
# the gyro's yaw-rate bias. The IMU propagates it at 25 Hz; each GPS fix corrects
# it, weighted by the accuracy that fix reports.
#
# Everything from inference_cam_nav.sh works here too; this only adds the IMU.
#
# Usage:
#   ./inference_cam_nav_imu.sh [camera-source] [options] [-- <VisionPilot args>]
#
# IMU options:
#   --imu-port <dev>     serial port. Default: the first /dev/ttyUSB* found
#   --imu-baud <n>       line speed. Default: auto-detect
#   --check-imu          check the IMU, print what it reports, then exit
#   --no-imu             run without it — identical to inference_cam_nav.sh
#
# Everything else is passed straight through:
#   --dest "Jalan Braga"     --gps-url http://<phone>:8080     --light
#   --zoom 800               --check                           --opencv
#
# Examples:
#   ./inference_cam_nav_imu.sh
#   ./inference_cam_nav_imu.sh --check-imu
#   ./inference_cam_nav_imu.sh --dest "Jalan Braga" --zoom 600
#   ./inference_cam_nav_imu.sh --imu-port /dev/ttyUSB0 --imu-baud 115200
#
# The IMU is an accessory, never a blocker: if it is unplugged or the port is
# busy, the run says so and carries on with GPS alone.
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
NAV="$HERE/inference_cam_nav.sh"
IMU_VIEW="$HERE/imu_view.sh"

usage() { sed -n '2,38p' "$0"; exit "${1:-1}"; }

CHECK_IMU="false"
WANT_IMU="true"
PASS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --check-imu) CHECK_IMU="true"; shift ;;
        --gps-only)  WANT_IMU="false"; PASS+=(--gps-only); shift ;;
        --compare)   PASS+=(--compare); shift ;;
        --no-imu)    WANT_IMU="false"; shift ;;
        -h|--help)   usage 0 ;;
        *)           PASS+=("$1"); shift ;;
    esac
done

# ── The serial port ──────────────────────────────────────────────────────────
# On Ubuntu, brltty claims CH340 bridges on sight and holds them through usbfs,
# so /dev/ttyUSB* never appears and the IMU looks dead. imu_view.sh --fix-port
# knows how to get it back; a failure here is only a warning, because the drive
# does not depend on the IMU.
if [ "$WANT_IMU" = "true" ]; then
    have_port="false"
    for p in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$p" ] && { have_port="true"; break; }
    done
    if [ "$have_port" = "false" ]; then
        echo "no serial port for the IMU yet — trying to bind the USB-serial driver"
        "$IMU_VIEW" --fix-port || {
            echo "warning: the IMU is not reachable; continuing with GPS alone" >&2
            WANT_IMU="false"; }
    fi
fi

# ── Check mode ───────────────────────────────────────────────────────────────
if [ "$CHECK_IMU" = "true" ]; then
    echo "=================================================="
    echo " IMU check"
    echo "=================================================="
    "$IMU_VIEW" --probe || exit 1
    echo ""
    echo "live readout for 5 s — tilt the module and watch roll/pitch/yaw move:"
    timeout 5 "$IMU_VIEW" --monitor || true
    echo ""
    echo "3D view:  $IMU_VIEW"
    exit 0
fi

if [ "$WANT_IMU" = "true" ]; then
    exec "$NAV" --imu ${PASS[@]+"${PASS[@]}"}
fi
exec "$NAV" --no-imu ${PASS[@]+"${PASS[@]}"}
