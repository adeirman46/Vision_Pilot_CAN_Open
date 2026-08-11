/*!
 * @file  main.cpp
 * @brief GPS and IMU fused on the Nucleo, published to the host as one position
 *        stream at IMU rate.
 *
 * Hardware: ST Nucleo-G474RE, with both sensors on it —
 *   - DFRobot GNSS-RTK rover (TEL0171-LoRa, Quectel LC29HDA) on I2C, 1 Hz
 *   - Yahboom MRA02-V1.0 10-axis IMU on USART1, 25 Hz
 *
 * What each sensor contributes, and why neither is enough alone:
 *
 *   GPS   absolute position, once a second, jumping a couple of metres between
 *         fixes, and no idea which way the car points when it is stopped
 *   IMU   acceleration and yaw rate at 25 Hz, smooth and honest about every
 *         small change in motion — but its errors integrate, so on its own it
 *         walks away within a minute
 *
 * The EKF in ekf.cpp keeps the strengths of both: GPS anchors the absolute
 * position and bounds the drift, the IMU fills in the second between fixes and
 * carries the heading through a stop, a tunnel, or a row of tall buildings.
 *
 * Why the filter is here and not on the PC. VisionPilot has the same filter on
 * the host side (navigation::FusedImuGps), and it works — but it only works
 * when the IMU is plugged into the PC. Both sensors are wired to this board, so
 * this is where the two streams are already together and where the timestamps
 * are honest: `dt` between IMU samples is measured by the MCU that received
 * them, not inferred after a USB round trip and a scheduler delay. The host then
 * has nothing left to do but read positions.
 *
 * Flash it with:
 *     pio run -e gps_imu_ekf -t upload && pio device monitor
 *
 * RTK is optional. With no base station broadcasting, the rover solves
 * standalone at quality 1 — metre-level rather than centimetre-level — and the
 * fix reports an accuracy that says so. The filter weights each fix by that
 * number, so nothing here needs changing when the base station does arrive: the
 * same fix simply pulls harder.
 */

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_RTK_LoRa.h>

#include "ekf.hpp"
#include "imu_mra02.hpp"

/* ─── Wiring ──────────────────────────────────────────────────────────────
 *
 * Two sensors, two buses, and they must not be swapped: the GPS is on I2C
 * because only I2C relays configuration through to the LC29H (see the transport
 * note in ../src/main.cpp), which leaves USART1 — the pins the old UART GPS
 * build used — free for the IMU.
 *
 *   DFRobot rover, Gravity 4-pin      Nucleo-G474RE
 *   ────────────────────────────      ─────────────────────────
 *   Black   GND                  ->   GND
 *   Red     VCC   3.3-5 V        ->   5V        (3V3 also works)
 *   Green   SDA                  ->   PB9   D14
 *   Blue    SCL                  ->   PB8   D15
 *
 *   Yahboom MRA02 IMU                 Nucleo-G474RE
 *   ────────────────────────────      ─────────────────────────
 *   GND                          ->   GND        (must be common with the above)
 *   VCC   3.3-5 V                ->   5V
 *   TX    module output          ->   PA10  D2   USART1_RX
 *   RX    module input           ->   PA9   D8   USART1_TX   (optional)
 *
 * The IMU pair crosses over — its TX to the Nucleo's RX. The GPS pair does not:
 * SDA to SDA, SCL to SCL. Getting the IMU straight-through instead of crossed
 * is the usual reason `imu=silent` never clears.
 *
 * Only PA10 is actually required: nothing here ever commands the IMU, it just
 * streams. Leaving its RX unconnected is fine.
 *
 * Set the switch on the rover to I2C, not UART. It is sampled at power-up, so
 * move it before applying power.
 *
 * PB9/PB8 are named explicitly because the G474 variant does not define
 * PIN_WIRE_SDA/SCL, so a bare `Wire` would fall back to PA8/PA9 — and PA9 is
 * now the IMU's pin.
 */
TwoWire GnssWire(/*sda=*/PB9, /*scl=*/PB8);
DFRobot_RTK_LoRa_I2C rtk(&GnssWire, DEVICE_ADDR);

HardwareSerial ImuSerial(/*rx=*/PA10, /*tx=*/PA9);

/* ─── Mounting ────────────────────────────────────────────────────────────
 * The filter needs acceleration in the vehicle's own frame (x forward, y left)
 * and a yaw rate that is positive when the car turns *right*. Two constants
 * carry every assumption about how the module is bolted in, so a module mounted
 * differently is a one-line change rather than a hunt through the maths.
 *
 * kYawRateSign is the one to check on the first drive, because it is the one
 * that is wrong quietly. The MRA02's gyro is right-handed about a z axis that
 * points *up*, so a right turn — clockwise seen from above — reads negative;
 * the filter's heading is clockwise from north, so it needs the opposite sign.
 * Hence -1.
 *
 *   How to verify, in ten seconds, parked:  watch the <heading> field of $VPFUS
 *   and rotate the whole car (or, on the bench, the module) clockwise. Heading
 *   must count UP: 350 -> 0 -> 10. If it counts down, set this to +1.
 *
 * kBodyYIsLeft is the module's y axis. Left is the convention the filter uses;
 * a module whose silkscreen has y pointing right needs `false`. Getting it
 * wrong sends every cornering acceleration sideways the wrong way — visible as
 * the fused track cutting to the inside of bends the GPS track goes round the
 * outside of. It matters less than the yaw sign, because GPS speed corrects the
 * velocity every second.
 */
static const double kYawRateSign = -1.0;
static const bool   kBodyYIsLeft = true;

/* ─── Rates ───────────────────────────────────────────────────────────────
 * 1 Hz for the GPS: it is the receiver's default and, on this kit, what it
 * actually sustains — the 10 Hz build asks for 100 ms and still measures
 * `rate=1.0hz`. Position between fixes is the IMU's job now, which is the whole
 * point of the filter, so there is nothing to gain by asking for more.
 *
 * The rate lives in the receiver rather than in this firmware, so $PAIR050 is
 * re-sent on every boot: flashing this over a running 10 Hz session would
 * otherwise leave the rover at 10 Hz until it next lost power.
 */
static const uint16_t kFixHz         = 1;
static const uint16_t kFixIntervalMs = 1000 / kFixHz;
static const bool     kSaveToFlash   = false;

// How often a fused position goes out. Faster than the IMU's 25 Hz would just
// repeat the same state; 20 Hz costs ~1.9 kB/s of the host link's 11.5 kB/s.
static const uint32_t kPublishMs = 50;

// Selecting the LoRa backend is a rover-side setting and does not need a base
// station to exist, but it is not worth a boot loop when no corrections are
// coming anyway.
static const uint32_t kLoraSelectTimeoutMs = 3000;

/* ─── IMU link ────────────────────────────────────────────────────────────
 * 115200 is the MRA02's factory rate. The others are here because a module that
 * has been reconfigured, or a different unit from the same family, is a far
 * more likely reason for silence than a broken board — and the scan costs a
 * couple of seconds only when nothing is decoding anyway.
 */
static const uint32_t kImuBauds[]  = {115200, 921600, 460800, 230400, 57600, 9600};
static const size_t   kImuBaudN    = sizeof(kImuBauds) / sizeof(kImuBauds[0]);
static const uint32_t kImuProbeMs  = 300;    // listen this long per candidate
static const uint32_t kImuSilentMs = 3000;   // silence before starting a sweep
static const uint32_t kImuSweepGap = 10000;  // rest between failed sweeps

/* The filter must keep stepping even when the IMU does not.
 *
 * predict() is the only thing that grows the covariance, so a filter that only
 * ever gets GPS updates becomes more and more certain of a position it is no
 * longer propagating: after a minute it has a sigma of centimetres, ignores the
 * receiver, and sits still while the car drives away. So when no IMU sample has
 * arrived for longer than an IMU interval, the filter is stepped with zero
 * acceleration and zero yaw rate — the plain constant-velocity model, which is
 * exactly what a GPS-only estimate is, with its uncertainty growing honestly at
 * the accel_noise rate.
 *
 * 60 ms is comfortably longer than the 40 ms of a healthy 25 Hz stream, so this
 * never fires while the IMU is alive.
 */
static const uint32_t kCoastMs = 60;

/* ─── Fix quality ─────────────────────────────────────────────────────────
 * The GGA quality digit, straight from the receiver. 1 is what you get with no
 * base station: an ordinary standalone GNSS fix. 4 is RTK Fixed — the reason the
 * kit exists, but not a precondition for anything here.
 */
enum : uint8_t {
    kQualityNone  = 0,
    kQualityGps   = 1,
    kQualityDgps  = 2,
    kQualityFixed = 4,
    kQualityFloat = 5,
};

static const char* quality_name(uint8_t q)
{
    switch (q) {
        case kQualityNone:  return "no-fix";
        case kQualityGps:   return "gps-standalone";
        case kQualityDgps:  return "dgps";
        case 3:             return "pps";
        case kQualityFixed: return "rtk-fixed";
        case kQualityFloat: return "rtk-float";
        case 6:             return "dead-reckoning";
        default:            return "unknown";
    }
}

/* A nominal 1-sigma horizontal error, in metres, and the number the filter
 * divides by: a fix claiming 2 cm overrides the IMU completely, one claiming 5 m
 * barely nudges it. The receiver reports no accuracy of its own, so this is the
 * accuracy class the quality digit stands for.
 *
 * Standalone GNSS is HDOP times the user-equivalent range error, 2.5 m being the
 * usual single-frequency figure — but HDOP under open sky drops to ~0.6, which
 * would claim 1.5 m from a receiver whose real spread is several metres. So the
 * standalone case is floored: better to trust the IMU too much for a second than
 * to chase a fix that is wrong by more than it admits.
 */
static double accuracy_for(uint8_t q, double hdop)
{
    switch (q) {
        case kQualityFixed: return 0.02;
        case kQualityFloat: return 0.50;
        case kQualityDgps:  return 1.00;
        case kQualityGps: {
            const double est = (hdop > 0.0) ? hdop * 2.5 : 5.0;
            return est < 2.5 ? 2.5 : est;
        }
        default: return 0.0;   // 0 = unknown, matching GpsFix on the host
    }
}

/* ─── Output ──────────────────────────────────────────────────────────────
 * Three sentences, NMEA-framed so the host can resynchronise after opening the
 * port mid-line. The checksum is the XOR of everything between '$' and '*'; a
 * line that does not verify is line noise and is dropped rather than parsed
 * into a wrong position.
 *
 *   $VPFUS,<uptime_ms>,<lat>,<lon>,<speed_ms>,<heading_deg>,<sigma_m>,
 *          <quality>,<imu_hz>,<yaw_bias_dps>*<xx>
 *
 *      The filter's output, 20 Hz. <sigma_m> is its own estimate of how good
 *      the position is — the host uses it as the fix accuracy, so it grows
 *      visibly when GPS drops out and the position is pure dead reckoning.
 *      <quality> is the GPS quality digit behind it, so a host can still tell
 *      an RTK-anchored track from a standalone one.
 *
 *   $VPRTK,...   the raw receiver, 1 Hz, unchanged from the GPS-only builds.
 *                Kept so the host can draw both and the fusion can be judged
 *                against the thing it replaced.
 *   $VPSTA,...   everything that is not a position.
 */
static uint8_t nmea_checksum(const char* body)
{
    uint8_t sum = 0;
    for (const char* p = body; *p != '\0'; ++p) {
        sum ^= static_cast<uint8_t>(*p);
    }
    return sum;
}

// Built into one buffer and pushed as a single write: at 20 Hz the handful of
// Serial.print() calls this replaces would sit in the path that also has to
// service a 25 Hz IMU and a 1 Hz burst of register reads.
static void emit(const char* body)
{
    char line[256];
    const int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, nmea_checksum(body));
    if (n > 0) {
        const size_t len = (static_cast<size_t>(n) < sizeof(line)) ? static_cast<size_t>(n)
                                                                   : sizeof(line) - 1;
        Serial.write(reinterpret_cast<const uint8_t*>(line), len);
    }
}

static void emit_status(const char* fmt, ...)
{
    char body[192];
    int n = snprintf(body, sizeof(body), "VPSTA,%lu,", static_cast<unsigned long>(millis()));
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body + n, sizeof(body) - n, fmt, ap);
    va_end(ap);
    emit(body);
}

// ─── State ───────────────────────────────────────────────────────────────────

static GpsImuEkf    g_ekf;
static Mra02Decoder g_imu;

static uint32_t g_imu_baud      = 0;      // 0 = not found yet
static uint32_t g_last_imu_ms   = 0;      // last decoded reading
static uint32_t g_imu_samples   = 0;      // since the last rate report
static double   g_imu_hz        = 0.0;
static uint32_t g_imu_window_ms = 0;
static uint32_t g_imu_bytes     = 0;      // raw bytes seen, this sweep
static uint32_t g_last_predict_ms = 0;    // last filter step, from either source

// Baud sweep, run as a state machine rather than a loop — see service_probe().
static bool     g_probing        = false;
static size_t   g_probe_idx      = 0;
static uint32_t g_probe_until_ms = 0;
static uint32_t g_next_sweep_ms  = 0;

static uint32_t g_last_fix_ms   = 0;      // last published $VPRTK
static uint32_t g_last_pub_ms   = 0;      // last published $VPFUS
static uint8_t  g_quality       = 0;

// Previous fix, for deriving speed and course. The register interface reports
// neither, and the filter wants both: speed pins the velocity, and course is
// the only thing that keeps heading honest without trusting the magnetometer.
static bool     g_have_prev     = false;
static double   g_prev_lat      = 0.0;
static double   g_prev_lon      = 0.0;
static uint32_t g_prev_fix_ms   = 0;

// ─── GPS helpers ─────────────────────────────────────────────────────────────

static bool link_alive()
{
    GnssWire.beginTransmission(DEVICE_ADDR);
    return GnssWire.endTransmission() == 0;
}

static const char* send_pair(const char* body)
{
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", body, nmea_checksum(body));
    return rtk.transmitAT(cmd);
}

/* PAIR commands are acknowledged with `$PAIR001,<cmd>,<result>`, 0 being
 * success. Returns that digit, or -1 if no matching ack came back — which is
 * "inconclusive", not "failed": the co-processor's AT buffer holds one reply and
 * an unrelated sentence can land there first. The readback is the real proof.
 */
static int pair_ack_result(const char* reply, const char* cmd_id)
{
    if (reply == nullptr) return -1;
    const char* ack = strstr(reply, "PAIR001,");
    if (ack == nullptr) return -1;
    ack += 8;
    if (strncmp(ack, cmd_id, strlen(cmd_id)) != 0) return -1;
    ack += strlen(cmd_id);
    if (*ack != ',') return -1;
    return isdigit(static_cast<unsigned char>(ack[1])) ? (ack[1] - '0') : -1;
}

static bool configure_fix_rate(uint16_t interval_ms)
{
    char body[32];
    snprintf(body, sizeof(body), "PAIR050,%u", static_cast<unsigned>(interval_ms));
    const int result = pair_ack_result(send_pair(body), "050");
    if (result > 0) {
        emit_status("fix-rate-rejected,interval=%ums,pair001=%d",
                    static_cast<unsigned>(interval_ms), result);
        return false;
    }

    const char* reply = send_pair("PAIR051");
    const char* field = (reply != nullptr) ? strstr(reply, "PAIR051,") : nullptr;
    if (field == nullptr) {
        emit_status("fix-rate-unverified,requested=%ums", static_cast<unsigned>(interval_ms));
        return false;
    }
    const unsigned reported = strtoul(field + 8, nullptr, 10);
    if (reported != interval_ms) {
        emit_status("fix-rate-mismatch,requested=%ums,reported=%ums",
                    static_cast<unsigned>(interval_ms), reported);
        return false;
    }
    emit_status("fix-rate=%ums,%uhz", reported,
                static_cast<unsigned>(1000u / (reported ? reported : 1000u)));
    return true;
}

// Metres per degree at this latitude, for turning two fixes into a speed and a
// course. Same expressions the filter uses for its own tangent plane.
static void metres_per_degree(double lat, double& per_lat, double& per_lon)
{
    const double phi = lat * (3.14159265358979323846 / 180.0);
    per_lat = 111132.92 - 559.82 * cos(2 * phi) + 1.175 * cos(4 * phi);
    per_lon = 111412.84 * cos(phi) - 93.5 * cos(3 * phi);
}

// ─── IMU ─────────────────────────────────────────────────────────────────────

static void imu_listen_at(uint32_t baud)
{
    ImuSerial.end();
    ImuSerial.begin(baud);
    g_imu.reset();
}

/* Find the IMU's line speed by listening — one candidate per few hundred
 * milliseconds, spread across loop iterations rather than run as a loop of its
 * own.
 *
 * The first version of this did scan in a loop, and it was wrong in a way worth
 * recording: six candidates at 400 ms each is 2.4 seconds during which nothing
 * else runs. With the IMU unplugged that sweep repeated every 3 s, so the board
 * spent most of its life inside it — the fused output fell from 20 Hz to 4 Hz
 * and roughly a third of the GPS fixes were never read at all. A diagnostic that
 * degrades the thing it is diagnosing is worse than no diagnostic.
 *
 * A module at the wrong baud is not silent, it delivers a steady stream of bytes
 * that never decode — so the test is "do whole checksummed frames come out", not
 * "are there bytes". The byte count is kept anyway, because the difference
 * between "bytes but no frames" and "no bytes at all" is the difference between
 * a baud problem and a wiring problem.
 */
static void service_probe()
{
    if (!g_probing) return;

    // Two clean frames at this candidate: that is the rate.
    if (g_imu.frames_ok() >= 2) {
        g_probing = false;
        const uint32_t found = kImuBauds[g_probe_idx];
        if (found != g_imu_baud) {
            emit_status("imu-ok,mra02,baud=%lu", static_cast<unsigned long>(found));
        }
        g_imu_baud    = found;
        g_last_imu_ms = 0;      // the next sample starts a fresh dt
        return;
    }

    if (static_cast<int32_t>(millis() - g_probe_until_ms) < 0) return;

    if (++g_probe_idx < kImuBaudN) {
        imu_listen_at(kImuBauds[g_probe_idx]);
        g_probe_until_ms = millis() + kImuProbeMs;
        return;
    }

    // Swept everything and nothing decoded.
    g_probing = false;
    if (g_imu_bytes == 0) {
        emit_status("imu-silent,bytes=0,rx=pa10,check=tx-to-pa10|gnd-common|power,"
                    "continuing=gps-only");
    } else {
        emit_status("imu-undecoded,bytes=%lu,rx=pa10,frames-bad=%lu,"
                    "check=not-an-mra02|framing,continuing=gps-only",
                    static_cast<unsigned long>(g_imu_bytes),
                    static_cast<unsigned long>(g_imu.frames_bad()));
    }
    g_imu_baud      = 0;
    g_next_sweep_ms = millis() + kImuSweepGap;
    imu_listen_at(kImuBauds[0]);   // park at the factory rate and keep listening
}

// Start a sweep if the IMU has gone quiet and one is not already running.
static void maybe_start_probe()
{
    if (g_probing) return;

    const uint32_t now = millis();
    if (g_last_imu_ms != 0 && now - g_last_imu_ms < kImuSilentMs) return;
    if (static_cast<int32_t>(now - g_next_sweep_ms) < 0) return;

    if (g_imu_baud != 0) {
        emit_status("imu-lost,after=%lums,continuing=gps-only",
                    static_cast<unsigned long>(now - g_last_imu_ms));
        g_imu_baud = 0;
    }
    g_probing   = true;
    g_probe_idx = 0;
    g_imu_bytes = 0;
    imu_listen_at(kImuBauds[0]);
    g_probe_until_ms = now + kImuProbeMs;
}

/* Drain whatever the IMU has sent and, for each complete reading, take one
 * filter step.
 *
 * The conversion is the part worth reading twice. The filter wants horizontal
 * acceleration in the *vehicle's* frame with gravity removed, and the raw
 * accelerometer gives neither: it measures gravity plus motion, in whatever
 * attitude the module happens to be sitting at. A module tilted by 5 degrees
 * leaks 0.85 m/s^2 of gravity into the horizontal plane — which the filter would
 * integrate into 3 m of position error in three seconds, and which looks exactly
 * like gentle acceleration.
 *
 * So: rotate the measurement into the module's world frame using its own
 * quaternion, subtract g from the vertical, then rotate the horizontal part back
 * by the module's own yaw. The two rotations cancel in azimuth, which is the
 * point — it means nothing here depends on what the module thinks north is, only
 * on its tilt, which it knows from gravity itself and which no magnetic
 * disturbance can spoil. Heading comes from the GPS course instead, inside the
 * filter.
 */
static void service_imu()
{
    uint8_t buf[128];
    while (ImuSerial.available() > 0) {
        size_t n = 0;
        while (ImuSerial.available() > 0 && n < sizeof(buf)) {
            buf[n++] = static_cast<uint8_t>(ImuSerial.read());
        }
        g_imu_bytes += n;
        g_imu.feed(buf, n);
    }

    ImuSample s;
    while (g_imu.pop(s)) {
        if (!s.has_accel || !s.has_gyro) continue;

        double dt = 0.0;
        if (g_last_imu_ms != 0 && s.stamp_ms > g_last_imu_ms) {
            dt = static_cast<double>(s.stamp_ms - g_last_imu_ms) * 1e-3;
        }
        g_last_imu_ms = s.stamp_ms;
        ++g_imu_samples;

        if (!g_ekf.initialised() || dt <= 0.0) continue;

        // Orientation: the module's own quaternion if it sent one, otherwise
        // built from its Euler angles. Both come from the module's internal
        // fusion, which carries a factory magnetometer calibration that nothing
        // here could reproduce.
        double qw = s.qw, qx = s.qx, qy = s.qy, qz = s.qz;
        if (!s.has_quat) {
            if (!s.has_euler) {
                // No attitude at all: assume the module is level. Wrong by
                // whatever the real tilt is, but better than skipping the step.
                qw = 1.0; qx = qy = qz = 0.0;
            } else {
                const double cr = cos(s.roll_rad * 0.5),  sr = sin(s.roll_rad * 0.5);
                const double cp = cos(s.pitch_rad * 0.5), sp = sin(s.pitch_rad * 0.5);
                const double cy = cos(s.yaw_rad * 0.5),   sy = sin(s.yaw_rad * 0.5);
                qw = cr * cp * cy + sr * sp * sy;
                qx = sr * cp * cy - cr * sp * sy;
                qy = cr * sp * cy + sr * cp * sy;
                qz = cr * cp * sy - sr * sp * cy;
            }
        }
        const double qn = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (qn > 1e-9) { qw /= qn; qx /= qn; qy /= qn; qz /= qn; }

        // Body -> world:  v + 2 * q_vec x (q_vec x v + w * v)
        const double vx = s.ax, vy = s.ay, vz = s.az;
        const double t1x = qy * vz - qz * vy;
        const double t1y = qz * vx - qx * vz;
        const double t1z = qx * vy - qy * vx;
        const double ux = t1x + qw * vx, uy = t1y + qw * vy, uz = t1z + qw * vz;
        const double r1x = qy * uz - qz * uy;
        const double r1y = qz * ux - qx * uz;
        const double r1z = qx * uy - qy * ux;
        const double wx = vx + 2.0 * r1x;
        const double wy = vy + 2.0 * r1y;
        const double wz = vz + 2.0 * r1z - 9.80665;    // gravity out

        // Back into the body's horizontal plane by the module's own yaw.
        const double yaw = atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz));
        const double cy2 = cos(yaw), sy2 = sin(yaw);
        const double ax_body =  wx * cy2 + wy * sy2;
        double       ay_body = -wx * sy2 + wy * cy2;
        (void)wz;   // vertical acceleration is not part of a 2-D road model

        if (!kBodyYIsLeft) ay_body = -ay_body;

        g_ekf.predict(ax_body, ay_body, kYawRateSign * s.gz, dt);
        g_last_predict_ms = millis();
    }
}

// Step the filter on the plain constant-velocity model when the IMU has nothing
// to say. See kCoastMs for why this exists at all.
static void coast_predict()
{
    if (!g_ekf.initialised()) return;

    const uint32_t now = millis();
    if (g_last_predict_ms == 0) { g_last_predict_ms = now; return; }

    const uint32_t gap = now - g_last_predict_ms;
    if (gap < kCoastMs) return;

    g_ekf.predict(0.0, 0.0, 0.0, gap * 1e-3);
    g_last_predict_ms = now;
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }

    emit_status("boot,gps-imu-ekf,nucleo-g474re,gps=i2c-pb9-pb8,imu=usart1-pa10,"
                "yaw-sign=%d,y-is-left=%d",
                static_cast<int>(kYawRateSign), kBodyYIsLeft ? 1 : 0);

    // ── GPS ──────────────────────────────────────────────────────────────────
    // begin() runs Wire.begin() and probes the address, so a failure here is
    // wiring or the mode switch, never the sky. This is the one loop that
    // blocks: with no receiver there is no origin, and with no origin the filter
    // has nothing to anchor dead reckoning to — an IMU alone would just report
    // a plausible position drifting away from an unknown starting point.
    uint32_t attempt = 0;
    while (!rtk.begin()) {
        emit_status("no-device,attempt=%lu,check=i2c-switch|sda-pb9|scl-pb8|pullups|power",
                    static_cast<unsigned long>(++attempt));
        delay(1000);
    }
    emit_status("gps-device-ok");

    rtk.setModule(module_lora);
    const uint32_t lora_deadline = millis() + kLoraSelectTimeoutMs;
    while (rtk.getModule() != module_lora && millis() < lora_deadline) {
        delay(100);
    }
    emit_status("lora-mode=%s", rtk.getModule() == module_lora ? "ok" : "unset,continuing");

    if (!configure_fix_rate(kFixIntervalMs)) {
        emit_status("continuing-at-receiver-default-rate");
    }
    if (kSaveToFlash) {
        send_pair("PAIR513");
        emit_status("config-saved-to-flash");
    }

    // ── IMU ──────────────────────────────────────────────────────────────────
    // An IMU that never turns up is not fatal. The filter degrades to the plain
    // constant-velocity model — the same 1 Hz position the GPS-only build
    // publishes, just through this pipeline — and says so once per sweep rather
    // than stopping the drive. The sweep starts here and finishes in the loop,
    // so the GPS is being served from the first millisecond either way.
    g_probing        = true;
    g_probe_idx      = 0;
    g_imu_bytes      = 0;
    imu_listen_at(kImuBauds[0]);
    g_probe_until_ms = millis() + kImuProbeMs;

    g_imu_window_ms = millis();
    emit_status("waiting-for-fix");
}

// ─── Loop ────────────────────────────────────────────────────────────────────

/* Publish the filter's state. Every field is derived from the same instant, so
 * a host plotting speed against heading is not mixing two epochs.
 */
static void publish_fused()
{
    char body[192];
    snprintf(body, sizeof(body),
             "VPFUS,%lu,%.7f,%.7f,%.2f,%.1f,%.2f,%u,%.1f,%.3f",
             static_cast<unsigned long>(millis()),
             g_ekf.latitude(), g_ekf.longitude(),
             g_ekf.speed_ms(), g_ekf.heading_deg(), g_ekf.position_sigma_m(),
             static_cast<unsigned>(g_quality), g_imu_hz, g_ekf.yaw_rate_bias_dps());
    emit(body);
}

/* Once every ten seconds, one line saying whether each half of the filter is
 * actually running. The IMU rate is measured rather than assumed — a module that
 * has dropped to a fraction of 25 Hz still produces a smooth-looking track right
 * up until it doesn't — and the yaw-rate bias is the filter's own opinion of the
 * gyro, which should settle within a degree per second of a constant and stay
 * there. A bias that keeps climbing means the heading is being corrected against
 * something the gyro disagrees with: check kYawRateSign first.
 */
static void report_health()
{
    static const uint32_t kWindowMs = 10000;

    const uint32_t now  = millis();
    const uint32_t span = now - g_imu_window_ms;
    if (span < kWindowMs) return;

    g_imu_hz        = (span > 0) ? (g_imu_samples * 1000.0 / span) : 0.0;
    g_imu_window_ms = now;
    g_imu_samples   = 0;

    emit_status("imu=%.1fhz,baud=%lu,bytes=%lu,frames=%lu,bad=%lu,"
                "gps-quality=%u,filter=%s,sigma=%.2fm,yaw-bias=%.3fdps",
                g_imu_hz, static_cast<unsigned long>(g_imu_baud),
                static_cast<unsigned long>(g_imu_bytes),
                static_cast<unsigned long>(g_imu.frames_ok()),
                static_cast<unsigned long>(g_imu.frames_bad()),
                static_cast<unsigned>(g_quality),
                g_ekf.initialised() ? "running" : "waiting-for-first-fix",
                g_ekf.position_sigma_m(), g_ekf.yaw_rate_bias_dps());
}

void loop()
{
    // ── IMU: predict ─────────────────────────────────────────────────────────
    // First and last thing in the loop, because everything else here can block
    // for tens of milliseconds and the module keeps sending regardless. The RX
    // buffer is enlarged in platformio.ini for the same reason.
    service_imu();
    service_probe();
    maybe_start_probe();

    // ── GPS: correct ─────────────────────────────────────────────────────────
    if (rtk.getDataFlush()) {
        /* The register file is not a snapshot — the co-processor overwrites it
         * as each solution lands, so the fields that must agree with each other
         * are read first, right after the flush latch said the data is fresh.
         * The tail is metadata that changes slowly. */
        sLonLat_t      lat_raw = rtk.getLat();
        sLonLat_t      lon_raw = rtk.getLon();
        const double   alt     = rtk.getAlt();
        const uint8_t  quality = rtk.getQuality();

        const double   sep     = rtk.getSep();
        const double   hdop    = rtk.getHdop();
        const uint8_t  sats    = rtk.getNumSatUsed();
        const uint16_t base_id = rtk.getSiteID();
        const double   age     = rtk.getDifTime();
        sTim_t         utc     = rtk.getUTC();
        sTim_t         date    = rtk.getDate();

        service_imu();      // the reads above took a while; do not fall behind

        /* The library fills latitudeDegree from the magnitude and hands you the
         * hemisphere separately — it never applies the sign. Used as it comes,
         * every fix lands in the northern and eastern hemispheres, which for
         * Bandung at 6.9 S is a plausible coordinate in Malaysia rather than an
         * obvious error. */
        double lat = lat_raw.latitudeDegree;
        double lon = lon_raw.lonitudeDegree;
        if (lat_raw.latDirection == 'S') lat = -lat;
        if (lon_raw.lonDirection == 'W') lon = -lon;

        // Before the first fix the direction bytes are 0, not 'N'/'S', and the
        // degrees are zero — publishing that puts the car in the Gulf of Guinea.
        const bool have_hemisphere =
            (lat_raw.latDirection == 'N' || lat_raw.latDirection == 'S') &&
            (lon_raw.lonDirection == 'E' || lon_raw.lonDirection == 'W');

        if (quality != kQualityNone && have_hemisphere) {
            const uint32_t now = millis();
            const double   acc = accuracy_for(quality, hdop);
            g_quality = quality;

            char body[224];
            snprintf(body, sizeof(body),
                     "VPRTK,%lu,%04u-%02u-%02uT%02u:%02u:%02uZ,"
                     "%.7f,%.7f,%.2f,%.2f,%u,%u,%.2f,%.2f,%.1f,%u",
                     static_cast<unsigned long>(now),
                     static_cast<unsigned>(date.year), static_cast<unsigned>(date.month),
                     static_cast<unsigned>(date.date), static_cast<unsigned>(utc.hour),
                     static_cast<unsigned>(utc.minute), static_cast<unsigned>(utc.second),
                     lat, lon, alt, sep,
                     static_cast<unsigned>(quality), static_cast<unsigned>(sats),
                     hdop, acc, age, static_cast<unsigned>(base_id));
            emit(body);
            g_last_fix_ms = now;

            /* Speed and course from the previous fix. The register interface
             * reports neither, and both matter: speed pins the velocity — which
             * is what stops the IMU integrating a phantom crawl at traffic
             * lights — and course is the only heading reference the filter has
             * that no magnetic disturbance can spoil.
             *
             * Only while genuinely moving. Two fixes a second apart with the car
             * parked differ by GPS noise alone, and the "course" of that noise
             * is a random direction that would spin the map. */
            double per_lat = 111132.0, per_lon = 111320.0;
            metres_per_degree(lat, per_lat, per_lon);

            double speed = 0.0, course = 0.0;
            bool   have_course = false;
            if (g_have_prev && now > g_prev_fix_ms) {
                const double dt = (now - g_prev_fix_ms) * 1e-3;
                const double dn = (lat - g_prev_lat) * per_lat;
                const double de = (lon - g_prev_lon) * per_lon;
                const double d  = sqrt(dn * dn + de * de);
                speed = d / dt;
                if (speed > 1.5) {
                    course = atan2(de, dn) * (180.0 / 3.14159265358979323846);
                    if (course < 0.0) course += 360.0;
                    have_course = true;
                }
            }
            g_prev_lat    = lat;
            g_prev_lon    = lon;
            g_prev_fix_ms = now;
            g_have_prev   = true;

            if (!g_ekf.initialised()) {
                g_ekf.initialise(lat, lon, course, have_course);
                emit_status("filter-initialised,%.7f,%.7f,acc=%.2fm", lat, lon, acc);
            } else {
                g_ekf.update_position(lat, lon, acc);
                g_ekf.update_velocity(speed, course, have_course);
            }

            static uint8_t last_quality = 0xFF;
            if (quality != last_quality) {
                last_quality = quality;
                emit_status("quality=%s,sats=%u,base=%u", quality_name(quality),
                            static_cast<unsigned>(sats), static_cast<unsigned>(base_id));
            }
            static bool announced_solo = false;
            if (base_id == 0 && !announced_solo && millis() > 20000) {
                announced_solo = true;
                emit_status("no-corrections,solving-standalone,"
                            "expect=metre-level,rtk=start-the-base-station");
            }
        } else {
            static uint32_t last_report = 0;
            if (millis() - last_report > 2000) {
                last_report = millis();
                emit_status("no-fix,sats=%u,quality=%u", static_cast<unsigned>(sats),
                            static_cast<unsigned>(quality));
            }
        }
    } else if (millis() - g_last_fix_ms > 3000) {
        // No new fix this poll is normal — we poll far faster than the receiver
        // solves. Only silence that outlasts the solution rate is worth
        // reporting, and it is measured from the last fix actually published.
        static uint32_t last_idle = 0;
        if (millis() - last_idle > 2000) {
            last_idle = millis();
            const bool up = link_alive();
            emit_status("gps-idle,no-fix-for=%lus,link=%s,filter=%s",
                        static_cast<unsigned long>((millis() - g_last_fix_ms) / 1000),
                        up ? "up,rover-has-no-satellites,check=gnss-antenna|sky-view"
                           : "DOWN,rover-not-acking,check=power|sda-scl|pullups|i2c-switch",
                        g_ekf.initialised() ? "dead-reckoning-on-imu" : "no-origin-yet");
        }
    }

    // ── Publish ──────────────────────────────────────────────────────────────
    service_imu();
    coast_predict();
    if (g_ekf.initialised() && millis() - g_last_pub_ms >= kPublishMs) {
        g_last_pub_ms = millis();
        publish_fused();
    }
    report_health();

    // Short enough that a 25 Hz IMU sample never waits a whole tick, long
    // enough that the I2C bus is not saturated with getDataFlush() polls.
    delay(2);
}
