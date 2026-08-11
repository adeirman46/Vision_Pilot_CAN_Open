#!/usr/bin/env bash
#
# Build and flash the Nucleo-G474RE that carries the GNSS-RTK rover and the IMU.
#
# One firmware per job, so this script is mostly a menu of them:
#
#   fused     GPS + IMU fused on the board, 20 Hz position   <- the one to use
#   gps       GPS only, 1 Hz, no IMU, no RTK required        <- the baseline
#   gps10     GPS only, 10 Hz — worth it once RTK is fixed
#   sniff     where is the IMU? sweeps UART pins and bauds
#   probe     where is the GPS? sweeps the rover's baud rates
#   rate      why is the rover still at 1 Hz after $PAIR050
#   rtklink   why are no RTCM corrections arriving
#
# Usage:
#   ./flash_nucleo.sh [firmware] [options]
#
# Options:
#   -m, --monitor    open the serial monitor afterwards (Ctrl-C, then Ctrl-] to quit)
#   -b, --build      build only, do not flash
#   -l, --list       list the firmwares and exit
#   -p, --port <dev> upload/monitor port. Default: whatever PlatformIO finds
#
# Examples:
#   ./flash_nucleo.sh                 # flash the fused firmware
#   ./flash_nucleo.sh fused -m        # flash it and watch the stream
#   ./flash_nucleo.sh sniff -m        # hunt for the IMU's UART pins
#   ./flash_nucleo.sh gps             # fall back to GPS only
#
# After flashing, check what is coming out with:  ./gps_imu_check.sh
#   (from the repo root:  scripts/sensors/gps_imu_check.sh)
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
PROJ="$HERE/../../VisionPilot/modules/sensing/gps_rtk_dfrobot"

usage() { sed -n '2,31p' "$0"; exit "${1:-1}"; }

# PlatformIO installs into its own venv and does not always put itself on PATH.
PIO="$(command -v pio || true)"
[ -n "$PIO" ] || PIO="$HOME/.platformio/penv/bin/pio"
[ -x "$PIO" ] || {
    echo "error: PlatformIO not found (looked on PATH and in ~/.platformio/penv/bin)" >&2
    echo "       pip install --user platformio" >&2
    exit 1; }

# Friendly name -> pio env, plus one line about what it does.
name_to_env() {
    case "$1" in
        fused|ekf|gps_imu_ekf)   echo "gps_imu_ekf" ;;
        gps|1hz|gps_1hz)         echo "gps_1hz" ;;
        gps10|10hz|nucleo_g474re) echo "nucleo_g474re" ;;
        sniff|imu|diag_imu_uart) echo "diag_imu_uart" ;;
        probe|diag)              echo "diag" ;;
        rate|diag_rate)          echo "diag_rate" ;;
        rtklink|diag_rtk_link)   echo "diag_rtk_link" ;;
        legacy|legacy_1hz)       echo "legacy_1hz" ;;
        *)                       echo "" ;;
    esac
}

list_firmwares() {
    cat <<'EOF'
firmware   pio env         what it does
────────   ─────────────   ────────────────────────────────────────────────────
fused      gps_imu_ekf     GPS (I2C, 1 Hz) + IMU (USART1, 25 Hz) fused on the
                           board by a 6-state EKF. Publishes $VPFUS at 20 Hz and
                           the raw $VPRTK at 1 Hz. Works with no RTK base.
gps        gps_1hz         GPS only, 1 Hz, RTK optional. The baseline to compare
                           the fused track against, and the fallback when the
                           IMU is not working.
gps10      nucleo_g474re   GPS only, 10 Hz. Worth flashing once an RTK base
                           station is running — at centimetre accuracy there is
                           something to gain from ten fixes a second.
sniff      diag_imu_uart   Sweeps four UART pin pairs x six baud rates and says
                           what arrives on each. For "imu-silent".
probe      diag            Sweeps the rover's baud rates. For "no-device".
rate       diag_rate       Why the rover ignores $PAIR050.
rtklink    diag_rtk_link   Why no RTCM corrections arrive (base_id stays 0).
legacy     legacy_1hz      The original UART-wired 1 Hz build.
EOF
}

FIRMWARE="fused"
MONITOR="false"
BUILD_ONLY="false"
PORT=""

case "${1:-}" in
    -h|--help) usage 0 ;;
    -l|--list) list_firmwares; exit 0 ;;
    ""|-*) ;;
    *) FIRMWARE="$1"; shift ;;
esac

while [ $# -gt 0 ]; do
    case "$1" in
        -m|--monitor) MONITOR="true"; shift ;;
        -b|--build)   BUILD_ONLY="true"; shift ;;
        -l|--list)    list_firmwares; exit 0 ;;
        -p|--port)    [ $# -ge 2 ] || { echo "error: --port needs a device" >&2; exit 1; }
                      PORT="$2"; shift 2 ;;
        -h|--help)    usage 0 ;;
        *)            echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

ENV_NAME="$(name_to_env "$FIRMWARE")"
[ -n "$ENV_NAME" ] || {
    echo "error: unknown firmware '$FIRMWARE'" >&2
    echo "" >&2
    list_firmwares >&2
    exit 1; }

# ── The board ────────────────────────────────────────────────────────────────
# An ST-LINK enumerates as a CDC-ACM device, so /dev/ttyACM* is the Nucleo and
# /dev/ttyUSB* is a USB-serial bridge — a different board. Only a warning: the
# upload goes over the ST-LINK's debug interface, not the serial port, so a
# board with no /dev/ttyACM yet can still be flashed.
if [ "$BUILD_ONLY" = "false" ]; then
    if ! lsusb 2>/dev/null | grep -qi "STMicroelectronics.*ST-?LINK"; then
        echo "warning: no ST-LINK on USB — is the Nucleo plugged into the PC?" >&2
    fi
fi

echo "=================================================="
echo " Nucleo firmware: $FIRMWARE  (pio env $ENV_NAME)"
list_firmwares | awk -v f="$FIRMWARE" '$1 == f { $1=""; $2=""; print "  " $0 }'
echo "=================================================="

TARGET=()
[ "$BUILD_ONLY" = "false" ] && TARGET=(-t upload)

# shellcheck disable=SC2086   # PORT is one word or nothing
"$PIO" run -d "$PROJ" -e "$ENV_NAME" ${TARGET[@]+"${TARGET[@]}"} ${PORT:+--upload-port "$PORT"}

if [ "$BUILD_ONLY" = "true" ]; then
    echo ""
    echo "built, not flashed. Drop --build to flash it."
    exit 0
fi

echo ""
echo "flashed. The board reboots and starts publishing immediately."
case "$ENV_NAME" in
    gps_imu_ekf)
        echo "  check it:   $HERE/gps_imu_check.sh"
        echo "  drive with: $HERE/../run/inference_cam_nav_gps.sh"
        echo ""
        echo "If it reports imu-silent, the IMU is not reaching PA10 (D2):"
        echo "  $0 sniff -m" ;;
    diag_imu_uart|diag|diag_rate|diag_rtk_link)
        echo "  this is a diagnostic — read its output:"
        echo "    $0 $FIRMWARE -m" ;;
    *)
        echo "  check it:   $HERE/gps_imu_check.sh" ;;
esac

if [ "$MONITOR" = "true" ]; then
    echo ""
    echo "opening the monitor — Ctrl-C then Ctrl-] to quit"
    echo ""
    exec "$PIO" device monitor -d "$PROJ" ${PORT:+--port "$PORT"}
fi
