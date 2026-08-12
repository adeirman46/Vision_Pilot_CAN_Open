/*!
 * @file  main.cpp
 * @brief Read centimetre-level fixes from a DFRobot GNSS-RTK rover and publish
 *        them to the host PC as checksummed lines on the ST-LINK serial port.
 *
 * Hardware: ST Nucleo-G474RE + DFRobot GNSS-RTK kit (KIT0198).
 *   - rover / mobile station: TEL0171-LoRa, Quectel LC29HDA
 *   - base station:           Quectel LC29HBS, broadcasting RTCM 3.x over LoRa
 *
 * The rover is not an NMEA receiver on this interface. A co-processor on the
 * board parses NMEA for you and exposes the result as registers, read over the
 * same wires for both I2C and UART:
 *
 *     read   0x55 | reg & 0x7F | len | 0xAA   -> len bytes back
 *     write  0x55 | reg | 0x80  | data | 0xAA
 *
 * DFRobot_RTK_LoRa wraps that, so nothing here speaks it directly. What that
 * costs is precision: latitude arrives as degrees, minutes and five decimals of
 * a minute, which quantises to ~1.9 cm. That is below the kit's own accuracy,
 * but it is not nothing — set EMIT_RAW_GGA to also forward the receiver's own
 * GGA sentence, which is not rounded.
 */

#include <Arduino.h>
#include <DFRobot_RTK_LoRa.h>

// Also forward the rover's unrounded $GNGGA sentence after each fix.
#define EMIT_RAW_GGA 0

/* ─── Wiring ──────────────────────────────────────────────────────────────
 *
 * `Serial` on a Nucleo-64 is USART2 on PA2/PA3, soldered to the ST-LINK
 * virtual COM port. That is the link to the host PC, so the rover cannot have
 * it — it gets USART1 instead.
 *
 *   Rover, Gravity 4-pin              Nucleo-G474RE
 *   ─────────────────────             ─────────────────────────
 *   Black   GND                  ->   GND
 *   Red     VCC   3.3-5 V        ->   5V        (3V3 also works)
 *   Green   TX    rover output   ->   PA10  D2  USART1_RX
 *   Blue    RX    rover input    ->   PA9   D8  USART1_TX
 *
 * Green to PA10 and blue to PA9 — the pair crosses over. Straight-through is
 * the usual reason begin() never returns true.
 *
 * Set the switch on the rover to UART, not I2C. It is read at power-up, so
 * move it before applying power.
 */
static const uint32_t kRoverBaud = 115200;

HardwareSerial GnssSerial(/*rx=*/PA10, /*tx=*/PA9);
DFRobot_RTK_LoRa_UART rtk(&GnssSerial, kRoverBaud);

/* ─── Fix quality ─────────────────────────────────────────────────────────
 * The GGA quality digit, straight from the receiver. 4 is the one to wait for:
 * the carrier ambiguities are resolved and the fix is centimetre-level. 5 means
 * corrections are arriving but the solution has not converged yet, which is
 * still decimetre-level and much better than 1.
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
        case kQualityGps:   return "gps";
        case kQualityDgps:  return "dgps";
        case 3:             return "pps";
        case kQualityFixed: return "rtk-fixed";
        case kQualityFloat: return "rtk-float";
        case 6:             return "dead-reckoning";
        default:            return "unknown";
    }
}

// A nominal 1-sigma horizontal error, so the host has something to weight the
// fix by. The receiver does not report one, so these are the accuracy classes
// the quality digit stands for rather than a measurement.
static double accuracy_for(uint8_t q, double hdop)
{
    switch (q) {
        case kQualityFixed: return 0.02;
        case kQualityFloat: return 0.50;
        case kQualityDgps:  return 1.00;
        case kQualityGps:   return hdop > 0.0 ? hdop * 2.5 : 5.0;
        default:            return 0.0;   // 0 = unknown, matching GpsFix
    }
}

/* ─── Output ──────────────────────────────────────────────────────────────
 * NMEA-style framing, because the host already has to be tolerant of partial
 * lines on a port that it may open midway through a sentence. The checksum is
 * what makes that recoverable: a line that does not verify is dropped rather
 * than parsed into a wrong position.
 *
 *   $VPRTK,<uptime_ms>,<utc>,<lat>,<lon>,<alt_m>,<geoid_sep_m>,
 *          <quality>,<sats>,<hdop>,<accuracy_m>,<corr_age_s>,<base_id>*<xx>
 */
static void emit(const char* body)
{
    uint8_t sum = 0;
    for (const char* p = body; *p != '\0'; ++p) {
        sum ^= static_cast<uint8_t>(*p);
    }
    Serial.print('$');
    Serial.print(body);
    Serial.print('*');
    if (sum < 0x10) {
        Serial.print('0');
    }
    Serial.println(sum, HEX);
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

/* ─── Framing recovery ────────────────────────────────────────────────────
 * readReg() in the library reads exactly `len` bytes and gives up after 200 ms,
 * but it never discards what did arrive. A reply that is cut short therefore
 * leaves its tail in the RX buffer, and every later read is offset by that
 * many bytes — the coordinates stay plausible and are silently wrong.
 *
 * The library's own `_serial->flush()` does not help: on Arduino it drains the
 * *transmit* buffer. So drop stale bytes here, at the top of a cycle, when the
 * bus has been idle since the last read and anything still pending is junk.
 */
// millis() of the last fix actually published, so the idle heartbeat can tell
// "between two 1 Hz solutions" from "the stream has stopped".
static uint32_t g_last_fix_ms = 0;

static void drain_rx()
{
    while (GnssSerial.available() > 0) {
        (void)GnssSerial.read();
    }
}

/* Is the rover still on the other end of the UART?
 *
 * "No fix" has two completely different causes that the library cannot tell
 * apart, because a timed-out register read and a register that genuinely says
 * "no new data" both come back as zero:
 *
 *   - the link is up and the receiver simply has no satellites  -> antenna/sky
 *   - the link is down, so nothing is being read at all         -> power/wiring
 *
 * Asking for the device-ID register separates them: 0x20 comes back only if
 * something is alive and framing correctly. This re-sends the same request
 * DFRobot_RTK_LoRa_UART::begin() uses, but without begin()'s side effect of
 * re-opening the port.
 */
static bool link_alive()
{
    drain_rx();
    GnssSerial.write(0x55);
    GnssSerial.write((uint8_t)(50 & 0x7F));   // REG_I2C_ID
    GnssSerial.write((uint8_t)1);
    GnssSerial.write(0xAA);

    uint32_t start = millis();
    while (millis() - start < 200) {
        if (GnssSerial.available() > 0) {
            return GnssSerial.read() == 0x20;   // DEVICE_ADDR
        }
    }
    return false;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }

    emit_status("boot,nucleo-g474re,rover-tel0171-lora,usart1-pa9-pa10");

    // begin() reads the device-ID register back and checks it is 0x20, so a
    // failure here is the wiring or the mode switch, not the sky.
    uint32_t attempt = 0;
    while (!rtk.begin()) {
        emit_status("no-device,attempt=%lu,check=uart-switch|tx-rx-crossed|power",
                    static_cast<unsigned long>(++attempt));
        delay(1000);
    }
    emit_status("device-ok");

    // The same board is sold with a 4G/NTRIP backend. Selecting the LoRa
    // backend is what makes it listen to the base station over the radio.
    rtk.setModule(module_lora);
    while (rtk.getModule() != module_lora) {
        emit_status("waiting-lora-mode");
        delay(1000);
    }
    emit_status("lora-mode-ok,waiting-for-fix");
}

void loop()
{
    drain_rx();

    if (!rtk.getDataFlush()) {
        // No new fix *this poll*, which is the normal state: the rover solves
        // at about 1 Hz and we poll faster than that, so this branch runs
        // between every pair of fixes. Warning on that alone would print
        // "check your antenna" in the middle of a healthy 1 Hz stream.
        //
        // What is worth reporting is silence that outlasts the solution rate.
        // Measure it from the last fix actually published, so the heartbeat
        // only appears once the data really has stopped — still covering the
        // case it exists for, a board whose UART died after a good start.
        if (millis() - g_last_fix_ms > 3000) {
            static uint32_t last_idle = 0;
            if (millis() - last_idle > 2000) {
                last_idle = millis();
                const bool up = link_alive();
                emit_status("idle,no-fix-for=%lus,link=%s",
                            static_cast<unsigned long>((millis() - g_last_fix_ms) / 1000),
                            up ? "up,rover-has-no-satellites,check=gnss-antenna|sky-view"
                               : "DOWN,rover-not-answering,check=power|wires|uart-switch");
            }
        }
        delay(50);
        return;
    }

    sTim_t          utc     = rtk.getUTC();
    sTim_t          date    = rtk.getDate();
    sLonLat_t       lat_raw = rtk.getLat();
    sLonLat_t       lon_raw = rtk.getLon();
    const double    alt     = rtk.getAlt();
    const double    sep     = rtk.getSep();
    const double    hdop    = rtk.getHdop();
    const uint8_t   sats    = rtk.getNumSatUsed();
    const uint8_t   quality = rtk.getQuality();
    const uint16_t  base_id = rtk.getSiteID();
    const double    age     = rtk.getDifTime();

    /* The library builds latitudeDegree and lonitudeDegree from the magnitude
     * only and hands you the hemisphere separately — it never applies the sign.
     * Use them as they come and every fix lands in the northern and eastern
     * hemispheres, which for Bandung at 6.9 S reads as a plausible coordinate
     * in Malaysia rather than as an obvious error. Apply it here. */
    double lat = lat_raw.latitudeDegree;
    double lon = lon_raw.lonitudeDegree;
    if (lat_raw.latDirection == 'S') { lat = -lat; }
    if (lon_raw.lonDirection == 'W') { lon = -lon; }

    // Before the first fix the hemisphere bytes are 0, not 'N'/'S'. Publishing
    // 0,0 as a position would put the car in the Gulf of Guinea.
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
        delay(200);
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

    // Say in words what the quality digit means, once per transition, so the
    // monitor shows RTK engaging without anyone decoding the field.
    static uint8_t last_quality = 0xFF;
    if (quality != last_quality) {
        last_quality = quality;
        emit_status("quality=%s,base=%u", quality_name(quality),
                    static_cast<unsigned>(base_id));
    }

#if EMIT_RAW_GGA
    char* gga = rtk.getGnssMessage(gnGGA);
    if (gga != nullptr && gga[0] != '\0') {
        Serial.print(gga);          // already a complete NMEA sentence
    }
#endif

    delay(100);   // the rover solves at 1-10 Hz; getDataFlush() paces the rest
}
