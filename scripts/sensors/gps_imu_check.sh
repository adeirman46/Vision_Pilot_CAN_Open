#!/usr/bin/env bash
#
# Is the Nucleo publishing usable positions? Reads its serial port, checks every
# line's checksum, decodes what it finds and says what is working.
#
# This is the script to run after flashing and before driving. It answers, in
# one pass, the four questions that otherwise cost a drive each:
#
#   is the board sending anything at all?
#   is the GPS solving, and at what quality?
#   is the IMU actually feeding the filter, or is it silent?
#   is the fused position moving smoothly, or jumping like the raw one?
#
# Usage:
#   ./gps_imu_check.sh [options]
#
# Options:
#   -p, --port <dev>   serial port. Default: the first /dev/ttyACM* found
#   -b, --baud <n>     line speed (115200)
#   -s, --seconds <n>  how long to listen (15)
#   -f, --follow       keep listening and print each fix as it arrives
#   -r, --raw          dump the port verbatim, no decoding
#
# Examples:
#   ./gps_imu_check.sh              # 15 s health check, then a verdict
#   ./gps_imu_check.sh -f           # watch fixes live
#   ./gps_imu_check.sh -s 60        # a longer sample, e.g. while walking
#
# Nothing else may hold the port at the same time: two readers steal bytes from
# each other and both see corruption. Close `pio device monitor` first.
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

usage() { sed -n '2,30p' "$0"; exit "${1:-1}"; }

PORT=""
BAUD="115200"
SECONDS_TO_RUN="15"
FOLLOW="false"
RAW="false"

while [ $# -gt 0 ]; do
    case "$1" in
        -p|--port)    [ $# -ge 2 ] || { echo "error: --port needs a device" >&2; exit 1; }
                      PORT="$2"; shift 2 ;;
        -b|--baud)    [ $# -ge 2 ] || { echo "error: --baud needs a value" >&2; exit 1; }
                      BAUD="$2"; shift 2 ;;
        -s|--seconds) [ $# -ge 2 ] || { echo "error: --seconds needs a value" >&2; exit 1; }
                      SECONDS_TO_RUN="$2"; shift 2 ;;
        -f|--follow)  FOLLOW="true"; shift ;;
        -r|--raw)     RAW="true"; shift ;;
        -h|--help)    usage 0 ;;
        *)            echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

# ── The port ─────────────────────────────────────────────────────────────────
# An ST-LINK is a CDC-ACM device, so the Nucleo is /dev/ttyACM*. A /dev/ttyUSB*
# is a USB-serial bridge — the IMU's CH340, if it is still plugged into the PC —
# and reading that here would just print IMU frames as line noise.
if [ -z "$PORT" ]; then
    for p in /dev/ttyACM*; do
        [ -e "$p" ] && { PORT="$p"; break; }
    done
fi
[ -n "$PORT" ] && [ -e "$PORT" ] || {
    echo "error: no Nucleo serial port found${PORT:+ at $PORT}" >&2
    echo "       An ST-LINK appears as /dev/ttyACM*. Check:" >&2
    echo "         lsusb | grep -i st-link      the board is on USB at all" >&2
    echo "         ls -l /dev/ttyACM*           the port exists" >&2
    echo "       If the board is new or unflashed:  $HERE/flash_nucleo.sh" >&2
    exit 1; }

[ -r "$PORT" ] && [ -w "$PORT" ] || {
    echo "error: no permission to read $PORT" >&2
    echo "       sudo usermod -aG dialout $USER   (then log out and back in)" >&2
    exit 1; }

# Two readers on one port steal bytes from each other, which shows up as
# characters dropped mid-word — corruption that looks exactly like a bad cable.
if command -v fuser >/dev/null 2>&1; then
    HOLDER="$(fuser "$PORT" 2>/dev/null || true)"
    if [ -n "$HOLDER" ]; then
        echo "warning: something else already has $PORT (pid$HOLDER)." >&2
        echo "         Close it — probably 'pio device monitor'. Reading anyway," >&2
        echo "         but expect torn lines that are not the board's fault." >&2
    fi
fi

stty -F "$PORT" "$BAUD" raw -echo -echoe -echok -crtscts 2>/dev/null || {
    echo "error: cannot configure $PORT at $BAUD baud" >&2; exit 1; }

if [ "$RAW" = "true" ]; then
    echo "raw dump of $PORT at $BAUD baud — Ctrl-C to stop"
    exec cat "$PORT"
fi

echo "=================================================="
echo " Nucleo GPS/IMU check"
echo "   port:   $PORT at $BAUD baud"
if [ "$FOLLOW" = "true" ]; then
    echo "   mode:   following — Ctrl-C to stop"
else
    echo "   window: ${SECONDS_TO_RUN}s"
fi
echo "=================================================="

# The decoder. Python rather than awk because the checksum is an XOR over the
# line, and the awk on a stock Ubuntu (mawk) has no xor().
#
# Reading the port with `timeout cat` rather than from Python keeps the timeout
# honest: a board that stops mid-line cannot wedge this, because the read that
# blocks is the one being killed.
DECODE=$(cat <<'PYEOF'
import sys, time

def verify(line):
    """Return the body of a checksummed NMEA-style line, or None."""
    if '$' not in line or '*' not in line:
        return None
    body = line[line.index('$') + 1:line.rindex('*')]
    given = line[line.rindex('*') + 1:].strip()[:2]
    if len(given) < 2:
        return None
    sum_ = 0
    for ch in body:
        sum_ ^= ord(ch) & 0xFF
    try:
        return body if sum_ == int(given, 16) else None
    except ValueError:
        return None

follow = sys.argv[1] == 'true'

n_fus = n_rtk = n_sta = n_bad = 0
first_t = last_t = None
last_fus = last_rtk = None
statuses = []
imu_hz = 0.0
quality = 0
sats = 0
base_id = 0
sigma = 0.0
fus_positions = []

for raw in sys.stdin:
    body = verify(raw)
    if body is None:
        if raw.strip():
            n_bad += 1
        continue

    now = time.time()
    if first_t is None:
        first_t = now
    last_t = now
    f = body.split(',')

    if body.startswith('VPFUS,') and len(f) >= 10:
        n_fus += 1
        last_fus = f
        sigma = float(f[6])
        quality = int(f[7])
        imu_hz = float(f[8])
        fus_positions.append((float(f[2]), float(f[3])))
        if follow and n_fus % 10 == 0:      # 20 Hz is too fast to read
            print(f"  fused  {f[2]},{f[3]}  {float(f[4]):5.2f} m/s  "
                  f"hdg {float(f[5]):5.1f}  sigma {sigma:4.2f} m  imu {imu_hz:4.1f} Hz")

    elif body.startswith('VPRTK,') and len(f) >= 13:
        n_rtk += 1
        last_rtk = f
        quality = int(f[7])
        sats = int(f[8])
        base_id = int(f[12])
        if follow:
            print(f"  raw    {f[3]},{f[4]}  q={f[7]} sats={f[8]} "
                  f"hdop={f[9]} acc={f[10]}m base={f[12]}")

    elif body.startswith('VPSTA,'):
        n_sta += 1
        msg = f[2] if len(f) > 2 else body
        msg = ','.join(f[2:])
        statuses.append(msg)
        if follow:
            print(f"  status {msg}")

span = (last_t - first_t) if (first_t and last_t and last_t > first_t) else 0.0

print()
print("--- what arrived ---------------------------------")
if span > 0:
    print(f"  $VPFUS  fused position   {n_fus:5d}   {n_fus / span:5.1f} Hz")
    print(f"  $VPRTK  raw receiver     {n_rtk:5d}   {n_rtk / span:5.1f} Hz")
else:
    print(f"  $VPFUS  fused position   {n_fus:5d}")
    print(f"  $VPRTK  raw receiver     {n_rtk:5d}")
print(f"  $VPSTA  status lines     {n_sta:5d}")
if n_bad:
    print(f"  bad checksum / torn    {n_bad:5d}   "
          "(a few at the start are normal — the port was opened mid-line)")

if statuses:
    print()
    print("--- last status lines ----------------------------")
    for s in statuses[-6:]:
        print(f"  {s}")

if last_rtk:
    print()
    print("--- GPS ------------------------------------------")
    qnames = {0: 'no fix', 1: 'standalone GPS', 2: 'DGPS', 4: 'RTK FIXED',
              5: 'RTK float', 6: 'dead reckoning'}
    print(f"  position   {last_rtk[3]}, {last_rtk[4]}")
    print(f"  quality    {quality} ({qnames.get(quality, 'unknown')}), "
          f"{sats} satellites, hdop {last_rtk[9]}")
    print(f"  accuracy   {last_rtk[10]} m")
    print(f"  base       {base_id if base_id else 'none — no RTK corrections'}")

if last_fus:
    print()
    print("--- filter ---------------------------------------")
    print(f"  position   {last_fus[2]}, {last_fus[3]}")
    print(f"  speed      {float(last_fus[4]):.2f} m/s   heading {float(last_fus[5]):.1f} deg")
    print(f"  sigma      {sigma:.2f} m   (the filter's own error estimate)")
    print(f"  imu        {imu_hz:.1f} Hz   yaw bias {float(last_fus[9]):.3f} deg/s")

# ── Verdict ──────────────────────────────────────────────────────────────────
# Each branch names the one next thing to do, because "something is wrong" with
# a list of six possibilities is what makes hardware take all day.
print()
print("--- verdict --------------------------------------")
rc = 0
if n_fus == 0 and n_rtk == 0 and n_sta == 0:
    print("  NOTHING arrived. Either the board is not flashed, another program")
    print("  holds the port, or this is the wrong /dev/ttyACM*.")
    print("    scripts/sensors/flash_nucleo.sh fused -m")
    rc = 1
elif n_fus == 0 and n_rtk == 0:
    print("  The board is alive but has no fix yet — only status lines. Give the")
    print("  GNSS antenna a clear view of the sky and wait a minute or two.")
    rc = 1
elif n_fus == 0:
    print("  GPS only: raw fixes are arriving but nothing is fused. That is the")
    print("  gps_1hz or 10 Hz firmware, not the fused one.")
    print("    scripts/sensors/flash_nucleo.sh fused")
    rc = 1
elif imu_hz < 1.0:
    print("  The filter is running on GPS alone — the IMU is not feeding it.")
    print("  Position will still be right, but it will jump once a second and")
    print("  lose its heading whenever you stop. Find the IMU with:")
    print("    scripts/sensors/flash_nucleo.sh sniff -m")
    rc = 1
elif imu_hz < 15.0:
    print(f"  Fused, but the IMU is only managing {imu_hz:.1f} Hz (expect ~25).")
    print("  Usually a marginal connection or a module sharing its UART.")
else:
    moved = 0.0
    if len(fus_positions) > 1:
        a, b = fus_positions[0], fus_positions[-1]
        moved = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5 * 111000
    print(f"  GPS + IMU fused and healthy: {imu_hz:.1f} Hz IMU, "
          f"quality {quality}, sigma {sigma:.2f} m.")
    if moved < 2.0:
        print(f"  Stationary ({moved:.1f} m over the window), which is all this can")
        print("  tell you sitting still. The heading and the smoothing only prove")
        print("  themselves moving:  scripts/run/inference_cam_nav_gps.sh")
    else:
        print(f"  Moved {moved:.0f} m during the window — the filter is tracking.")
    if quality == 1:
        print("  No RTK: metre-level, not centimetre-level. Expected with no base")
        print("  station running, and the filter weights the fixes accordingly.")

sys.exit(rc)
PYEOF
)

if [ "$FOLLOW" = "true" ]; then
    # Ctrl-C ends `cat`, Python then prints its summary and exits.
    cat "$PORT" | /usr/bin/python3 -c "$DECODE" true
else
    timeout "$SECONDS_TO_RUN" cat "$PORT" | /usr/bin/python3 -c "$DECODE" false
fi
