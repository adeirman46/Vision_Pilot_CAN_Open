/*!
 * @file  main.cpp
 * @brief The GPS half of the host's GPS/IMU filter: one fix a second over I2C,
 *        published to the PC as checksummed lines on the ST-LINK serial port.
 *
 * Hardware: ST Nucleo-G474RE + DFRobot GNSS-RTK kit (KIT0198), rover only.
 *
 * This is src/main.cpp with the RTK ambition removed. Same board, same wiring,
 * same I2C transport, same $VPRTK wire format — two differences, and both exist
 * because the thing consuming these fixes is now navigation's GPS/IMU EKF rather
 * than a person watching a monitor:
 *
 *   1 Hz, deliberately.  The receiver's own default, and on this kit what it
 *     actually sustains: the 10 Hz build asks for 100 ms and then measures
 *     `rate=1.0hz` regardless. Asking for a rate the hardware does not deliver
 *     buys nothing and makes the rate line useless as a health check — at 1 Hz
 *     it reads 1.0 and any departure from that is real. Position between fixes
 *     is the IMU's job now, which is the whole point of the filter.
 *
 *   RTK is optional, not awaited.  With no base station broadcasting, the rover
 *     solves standalone at quality 1 and `base_id` stays 0 forever. That is a
 *     usable fix — metre-level instead of centimetre-level — so it is published,
 *     with an `accuracy_m` that says honestly how good it is. The EKF weights
 *     each fix by that number, so a standalone fix simply pulls the filter less
 *     hard than an RTK one would. Nothing here blocks waiting for corrections,
 *     and the LoRa backend is selected on a timeout rather than in a loop.
 *
 * Flash it with:
 *     pio run -e gps_1hz -t upload && pio device monitor
 *
 * When the base station is up, `pio run -t upload` (the default 10 Hz env) is
 * the one that chases centimetres. Both publish the same sentences, so the host
 * does not care which is running.
 */

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_RTK_LoRa.h>

// Also forward the rover's unrounded $GNGGA sentence after each fix. The
// register interface rounds latitude to ~1.9 cm, which is far below standalone
// GNSS error — so at quality 1 this buys nothing. It is here for the day the
// base station arrives.
#define EMIT_RAW_GGA 0

/* ─── Wiring ──────────────────────────────────────────────────────────────
 * Identical to src/main.cpp — I2C, and the mode switch on the rover has to
 * agree. It is sampled at power-up, so move it before applying power.
 *
 *   Rover, Gravity 4-pin              Nucleo-G474RE
 *   ─────────────────────             ─────────────────────────
 *   Black   GND                  ->   GND
 *   Red     VCC   3.3-5 V        ->   5V        (3V3 also works)
 *   Green   SDA                  ->   PB9   D14
 *   Blue    SCL                  ->   PB8   D15
 *
 * No crossover: SDA to SDA, SCL to SCL.
 *
 * PB9/PB8 are named explicitly because the G474 variant does not define
 * PIN_WIRE_SDA/SCL, so a bare `Wire` falls back to PA8/PA9 — not the pins on
 * the Arduino header.
 */
TwoWire GnssWire(/*sda=*/PB9, /*scl=*/PB8);
DFRobot_RTK_LoRa_I2C rtk(&GnssWire, DEVICE_ADDR);

/* ─── Solution rate ───────────────────────────────────────────────────────
 * 1 Hz is the receiver's power-on default, so `$PAIR050,1000` is usually a
 * no-op. It is sent anyway, because the rate is *not* reset by reflashing the
 * Nucleo: flash the 10 Hz build, then this one, and without this line the rover
 * would keep solving at 10 Hz until it next lost power — and this firmware
 * would happily publish every one of those fixes despite its name.
 *
 * The command only lands over I2C. On UART the passthrough goes out and never
 * reaches the LC29H, silently. See the transport note in src/main.cpp.
 */
static const uint16_t kFixHz         = 1;
static const uint16_t kFixIntervalMs = 1000 / kFixHz;
static const bool     kSaveToFlash   = false;

static_assert(kFixIntervalMs >= 100 && kFixIntervalMs <= 1000,
              "$PAIR050 accepts a 100-1000 ms fix interval (1-10 Hz)");

// How long to wait for the LoRa backend to be selected before carrying on
// without it. Selecting it is a rover-side setting and does not need a base to
// exist, but it is not worth a boot loop when no corrections are coming anyway.
static const uint32_t kLoraSelectTimeoutMs = 3000;

/* ─── Fix quality ─────────────────────────────────────────────────────────
 * The GGA quality digit, straight from the receiver. 1 is what you get with no
 * base station: an ordinary standalone GNSS fix, metre-level. 4 is RTK Fixed,
 * centimetre-level, and the reason the kit exists — but it is not a
 * precondition for anything here.
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

/* A nominal 1-sigma horizontal error, in metres. The receiver does not report
 * one, so this is the accuracy class the quality digit stands for.
 *
 * It matters more here than it did in the 10 Hz build, because the host EKF
 * divides by it: a fix claiming 2 cm overrides the IMU completely, one claiming
 * 5 m barely nudges it. Standalone GNSS is HDOP times the user-equivalent range
 * error, and 2.5 m is the usual single-frequency figure for that — but HDOP
 * under open sky drops to ~0.6, which would claim 1.5 m from a receiver whose
 * real spread is several metres. So the standalone case is floored: better to
 * let the filter trust the IMU too much for a second than to have it chase a
 * fix that is wrong by more than it admits.
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
        default: return 0.0;   // 0 = unknown, matching GpsFix
    }
}

/* ─── Output ──────────────────────────────────────────────────────────────
 * Unchanged from the 10 Hz build, so navigation's reader parses either without
 * knowing which is flashed:
 *
 *   $VPRTK,<uptime_ms>,<utc>,<lat>,<lon>,<alt_m>,<geoid_sep_m>,
 *          <quality>,<sats>,<hdop>,<accuracy_m>,<corr_age_s>,<base_id>*<xx>
 *   $VPSTA,<uptime_ms>,<message>*<xx>
 *
 * The checksum is the XOR of everything between '$' and '*'. A line that does
 * not verify is line noise — the host drops it rather than parsing it into a
 * wrong position.
 */
static uint8_t nmea_checksum(const char* body)
{
    uint8_t sum = 0;
    for (const char* p = body; *p != '\0'; ++p) {
        sum ^= static_cast<uint8_t>(*p);
    }
    return sum;
}

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
    char body[160];
    int n = snprintf(body, sizeof(body), "VPSTA,%lu,", static_cast<unsigned long>(millis()));
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body + n, sizeof(body) - n, fmt, ap);
    va_end(ap);
    emit(body);
}

// millis() of the last fix actually published, so the idle heartbeat can tell
// "between two solutions" from "the stream has stopped".
static uint32_t g_last_fix_ms = 0;

/* Is the rover still on the other end of the bus?
 *
 * "No fix" has two causes the library cannot tell apart, because a failed
 * register read and a register that says "no new data" both come back as zero:
 * the link is up and there are no satellites (antenna, sky), or the link is
 * down and nothing is being read at all (power, wiring). An address probe
 * separates them for the cost of one byte.
 */
static bool link_alive()
{
    GnssWire.beginTransmission(DEVICE_ADDR);
    return GnssWire.endTransmission() == 0;
}

/* ─── Receiver configuration ──────────────────────────────────────────────
 * transmitAT() relays a sentence to the LC29H behind the co-processor and
 * returns its reply. Checksums are computed rather than pasted: a stale one is
 * rejected silently, which looks exactly like the command having no effect.
 */
static const char* send_pair(const char* body)
{
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", body, nmea_checksum(body));
    return rtk.transmitAT(cmd);
}

/* PAIR commands are acknowledged with `$PAIR001,<cmd>,<result>`, 0 being
 * success. Returns that digit, or -1 if no matching ack came back.
 *
 * -1 is "inconclusive", not "failed": the co-processor's AT buffer holds one
 * reply and an unrelated sentence can land there first. The readback below, and
 * ultimately the measured rate, are the real proof.
 */
static int pair_ack_result(const char* reply, const char* cmd_id)
{
    if (reply == nullptr) {
        return -1;
    }
    const char* ack = strstr(reply, "PAIR001,");
    if (ack == nullptr) {
        return -1;
    }
    ack += 8;                                   // past "PAIR001,"
    if (strncmp(ack, cmd_id, strlen(cmd_id)) != 0) {
        return -1;
    }
    ack += strlen(cmd_id);
    if (*ack != ',') {
        return -1;
    }
    return isdigit(static_cast<unsigned char>(ack[1])) ? (ack[1] - '0') : -1;
}

// Set the fix interval, then read it back with $PAIR051. Returns whether the
// receiver's own view of its configuration agrees with what was asked for.
static bool configure_fix_rate(uint16_t interval_ms)
{
    char body[32];

    snprintf(body, sizeof(body), "PAIR050,%u", static_cast<unsigned>(interval_ms));
    const int result = pair_ack_result(send_pair(body), "050");
    if (result > 0) {
        // 1 unsupported packet type, 2 invalid parameter, 3 execution failed.
        emit_status("fix-rate-rejected,interval=%ums,pair001=%d",
                    static_cast<unsigned>(interval_ms), result);
        return false;
    }

    const char* reply = send_pair("PAIR051");
    const char* field = (reply != nullptr) ? strstr(reply, "PAIR051,") : nullptr;
    if (field == nullptr) {
        emit_status("fix-rate-unverified,requested=%ums,ack=%d",
                    static_cast<unsigned>(interval_ms), result);
        return false;
    }

    const unsigned reported = strtoul(field + 8, nullptr, 10);
    if (reported != interval_ms) {
        emit_status("fix-rate-mismatch,requested=%ums,reported=%ums",
                    static_cast<unsigned>(interval_ms), reported);
        return false;
    }

    emit_status("fix-rate=%ums,%uhz,ack=%d", reported,
                static_cast<unsigned>(1000u / (reported ? reported : 1000u)), result);
    return true;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }

    emit_status("boot,gps-1hz,nucleo-g474re,rover-tel0171-lora,i2c-pb9-pb8,addr=0x%02X",
                static_cast<unsigned>(DEVICE_ADDR));

    // begin() runs Wire.begin() and probes the address, so a failure here is
    // wiring or the mode switch, never the sky. This is the one loop that does
    // block: with no rover on the bus there is nothing to publish at all.
    uint32_t attempt = 0;
    while (!rtk.begin()) {
        emit_status("no-device,attempt=%lu,check=i2c-switch|sda-pb9|scl-pb8|pullups|power",
                    static_cast<unsigned long>(++attempt));
        delay(1000);
    }
    emit_status("device-ok");

    /* The same board is sold with a 4G/NTRIP backend, and selecting LoRa is
     * what makes it listen to a base station over the radio. Without a base
     * there is nothing to listen to, so this is set on a timeout rather than in
     * the 10 Hz build's `while (getModule() != module_lora)` loop: a rover that
     * will not switch backends is a reason to note it and keep publishing
     * standalone fixes, not a reason to never boot. */
    rtk.setModule(module_lora);
    const uint32_t lora_deadline = millis() + kLoraSelectTimeoutMs;
    while (rtk.getModule() != module_lora && millis() < lora_deadline) {
        delay(100);
    }
    emit_status("lora-mode=%s", rtk.getModule() == module_lora ? "ok" : "unset,continuing");

    // A failure here is not fatal either: the receiver keeps solving at whatever
    // rate it was already using, and 1 Hz is its default in any case.
    if (!configure_fix_rate(kFixIntervalMs)) {
        emit_status("continuing-at-receiver-default-rate");
    }

    if (kSaveToFlash) {
        send_pair("PAIR513");
        emit_status("config-saved-to-flash");
    }

    emit_status("waiting-for-fix");
}

/* ─── Rate measurement ────────────────────────────────────────────────────
 * What the host actually receives, measured between the first and last fix of
 * the window rather than over the wall clock. Dividing by the wall clock counts
 * the time before the receiver had a fix at all as though it were time spent
 * solving slowly, so the first window after boot reads about half the true rate
 * and a window containing an outage reads low in proportion to it. Fix-to-fix
 * cannot do that: n fixes spanning t ms are exactly n-1 intervals.
 *
 * At 1 Hz this line is the cheapest health check there is. It should read 1.0.
 */
static uint32_t g_window_first_ms = 0;
static uint32_t g_window_last_ms  = 0;
static uint32_t g_window_fixes    = 0;

static void note_fix()
{
    const uint32_t now = millis();
    if (g_window_fixes == 0) {
        g_window_first_ms = now;
    }
    g_window_last_ms = now;
    ++g_window_fixes;
}

static void report_rate()
{
    static const uint32_t kWindowMs = 10000;

    if (g_window_fixes < 2) {
        return;
    }
    const uint32_t span = g_window_last_ms - g_window_first_ms;
    if (span < kWindowMs) {
        return;
    }

    // Tenths of a hertz in integer arithmetic — %f is already why this build
    // links _printf_float, and a housekeeping line does not need to spend it.
    const uint32_t intervals = g_window_fixes - 1;
    const uint32_t hz_tenths = (intervals * 10000UL + span / 2) / span;
    emit_status("rate=%lu.%luhz,fixes=%lu,span=%lums,target=%uhz",
                static_cast<unsigned long>(hz_tenths / 10),
                static_cast<unsigned long>(hz_tenths % 10),
                static_cast<unsigned long>(g_window_fixes),
                static_cast<unsigned long>(span),
                static_cast<unsigned>(kFixHz));

    g_window_fixes = 0;
}

/* Say once, and only once, that this is a standalone fix rather than an RTK
 * one. base_id = 0 with corr_age = 0 means no correction has ever arrived; that
 * is expected here and not a fault, but it should be visible in the log so that
 * "the position is drifting by two metres" is never a mystery. Repeating it
 * every second would drown the fix stream.
 */
static void note_correction_state(uint16_t base_id)
{
    static bool announced_solo = false;
    static bool announced_rtk  = false;

    if (base_id == 0) {
        if (!announced_solo && millis() > 20000) {
            announced_solo = true;
            emit_status("no-corrections,solving-standalone,"
                        "expect=metre-level,rtk=start-the-base-station");
        }
        return;
    }
    if (!announced_rtk) {
        announced_rtk = true;
        emit_status("corrections-arriving,base=%u", static_cast<unsigned>(base_id));
    }
}

void loop()
{
    if (!rtk.getDataFlush()) {
        // No new fix *this poll* is the normal state — we poll faster than the
        // receiver solves, so this runs between every pair of fixes. Only
        // silence that outlasts the solution rate is worth reporting, measured
        // from the last fix actually published.
        if (millis() - g_last_fix_ms > 3000) {
            static uint32_t last_idle = 0;
            if (millis() - last_idle > 2000) {
                last_idle = millis();
                const bool up = link_alive();
                emit_status("idle,no-fix-for=%lus,link=%s",
                            static_cast<unsigned long>((millis() - g_last_fix_ms) / 1000),
                            up ? "up,rover-has-no-satellites,check=gnss-antenna|sky-view"
                               : "DOWN,rover-not-acking,check=power|sda-scl|pullups|i2c-switch");
            }
        }
        report_rate();
        // A 1 s epoch has room to spare, so poll gently: every poll is a
        // register round trip on the same bus as the reads below.
        delay(20);
        return;
    }

    /* The register file is not a snapshot — the co-processor overwrites it as
     * each solution lands, so a fix assembled from ~11 sequential reads can
     * straddle two epochs. At 1 Hz the reads are a few percent of the interval
     * and this is close to impossible, but the ordering costs nothing: the
     * fields that have to agree with each other are read first, right after the
     * flush latch said the data is fresh. The tail is metadata that changes
     * slowly, where one epoch late is harmless. */
    sLonLat_t       lat_raw = rtk.getLat();
    sLonLat_t       lon_raw = rtk.getLon();
    const double    alt     = rtk.getAlt();
    const uint8_t   quality = rtk.getQuality();

    const double    sep     = rtk.getSep();
    const double    hdop    = rtk.getHdop();
    const uint8_t   sats    = rtk.getNumSatUsed();
    const uint16_t  base_id = rtk.getSiteID();
    const double    age     = rtk.getDifTime();
    sTim_t          utc     = rtk.getUTC();
    sTim_t          date    = rtk.getDate();

    /* The library fills latitudeDegree from the magnitude and hands you the
     * hemisphere separately — it never applies the sign. Used as it comes, every
     * fix lands in the northern and eastern hemispheres, which for Bandung at
     * 6.9 S is a plausible coordinate in Malaysia rather than an obvious error. */
    double lat = lat_raw.latitudeDegree;
    double lon = lon_raw.lonitudeDegree;
    if (lat_raw.latDirection == 'S') { lat = -lat; }
    if (lon_raw.lonDirection == 'W') { lon = -lon; }

    // Before the first fix the direction bytes are 0, not 'N'/'S', and the
    // degrees are zero. Publishing that would put the car in the Gulf of Guinea.
    const bool have_hemisphere =
        (lat_raw.latDirection == 'N' || lat_raw.latDirection == 'S') &&
        (lon_raw.lonDirection == 'E' || lon_raw.lonDirection == 'W');

    if (quality == kQualityNone || !have_hemisphere) {
        static uint32_t last_report = 0;
        if (millis() - last_report > 2000) {
            last_report = millis();
            emit_status("no-fix,sats=%u,quality=%u", static_cast<unsigned>(sats),
                        static_cast<unsigned>(quality));
        }
        report_rate();
        delay(50);
        return;
    }

    char body[224];
    snprintf(body, sizeof(body),
             "VPRTK,%lu,%04u-%02u-%02uT%02u:%02u:%02uZ,"
             "%.7f,%.7f,%.2f,%.2f,%u,%u,%.2f,%.2f,%.1f,%u",
             static_cast<unsigned long>(millis()),
             static_cast<unsigned>(date.year), static_cast<unsigned>(date.month),
             static_cast<unsigned>(date.date), static_cast<unsigned>(utc.hour),
             static_cast<unsigned>(utc.minute), static_cast<unsigned>(utc.second),
             lat, lon, alt, sep,
             static_cast<unsigned>(quality), static_cast<unsigned>(sats),
             hdop, accuracy_for(quality, hdop), age,
             static_cast<unsigned>(base_id));
    emit(body);
    g_last_fix_ms = millis();
    note_fix();

    // Say in words what the quality digit means, once per transition, so the
    // monitor shows RTK engaging — if it ever does — without anyone decoding
    // the field.
    static uint8_t last_quality = 0xFF;
    if (quality != last_quality) {
        last_quality = quality;
        emit_status("quality=%s,sats=%u,base=%u", quality_name(quality),
                    static_cast<unsigned>(sats), static_cast<unsigned>(base_id));
    }
    note_correction_state(base_id);

#if EMIT_RAW_GGA
    char* gga = rtk.getGnssMessage(gnGGA);
    if (gga != nullptr && gga[0] != '\0') {
        Serial.print(gga);          // already a complete NMEA sentence
    }
#endif

    report_rate();

    /* No delay here. getDataFlush() at the top of the loop is the pacer — it
     * returns false until the receiver has something new, so the loop free-runs
     * against the receiver's own cadence. A sleep the length of the epoch would
     * alias against it and drop roughly every other fix. */
}
