/*!
 * @file  main.cpp
 * @brief Find the IMU. Sweeps candidate RX pins and baud rates, and reports
 *        what — if anything — is arriving on each.
 *
 * Flash this when gps_imu_ekf keeps saying `imu-silent`. The question it answers
 * is not "which baud rate?" but the one before it:
 *
 *     do any bytes at all reach this board, on any pin?
 *
 * Three faults look identical from outside — dead power, TX and RX
 * straight-through instead of crossed, and the wire being on a different pin
 * from the one the firmware listens to. Permuting them by hand means a reflash
 * per guess. This sweeps the lot in about twelve seconds and repeats for ever,
 * so you can move one wire at a time and watch the verdict change.
 *
 *   pio run -e diag_imu_uart -t upload && pio device monitor
 *
 * The GPS is not touched: nothing here opens I2C, so the rover can stay wired.
 */

#include <Arduino.h>

#include "../gps_imu_ekf/imu_mra02.hpp"

/* ─── What to try ─────────────────────────────────────────────────────────
 *
 * Every UART pin pair on the Nucleo-G474RE that a sensor is plausibly wired to,
 * with the Arduino-header label each one carries. The IMU's TX goes to the RX
 * pin — the pair crosses over — and TX here is listed only so the table matches
 * what is printed on the board.
 *
 * PA2/PA3 (D1/D0) are deliberately absent: they are USART2, soldered to the
 * ST-LINK virtual COM port, which is the link carrying this report to the PC.
 * Listening on them would mean talking to ourselves.
 */
struct Candidate
{
    const char* name;
    uint32_t    rx;
    uint32_t    tx;
};

static const Candidate kPins[] = {
    {"PA10/PA9  D2/D8    USART1", PA10, PA9},
    {"PB7/PB6   -/D10    USART1", PB7,  PB6},
    {"PC11/PC10 CN7-2/1  USART3", PC11, PC10},
    {"PB11/PB10 -/D6     USART3", PB11, PB10},
};
static const size_t kPinN = sizeof(kPins) / sizeof(kPins[0]);

// 115200 is the MRA02's factory rate and is tried first, so a correctly wired
// module is found in the first half second rather than at the end of the sweep.
static const uint32_t kBauds[] = {115200, 9600, 57600, 230400, 460800, 921600};
static const size_t   kBaudN   = sizeof(kBauds) / sizeof(kBauds[0]);

static const uint32_t kListenMs = 500;   // per pin/baud combination

static Mra02Decoder decoder;

// One result per combination tried, so the summary can name the winner.
struct Result
{
    uint32_t bytes  = 0;
    uint32_t frames = 0;
    uint32_t bad    = 0;
};

static void hex_dump(const uint8_t* d, size_t n)
{
    char line[80];
    size_t used = 0;
    for (size_t i = 0; i < n && used + 3 < sizeof(line); ++i) {
        used += snprintf(line + used, sizeof(line) - used, "%02X ", d[i]);
    }
    line[used] = '\0';
    Serial.print("       first bytes: ");
    Serial.println(line);
}

/* Listen on one pin pair at one baud rate.
 *
 * `sample` keeps the first handful of bytes: when something arrives but nothing
 * decodes, the bytes themselves are the evidence. A stream of 0x00 or 0xFF is a
 * floating pin or a baud mismatch; readable ASCII is a module speaking a
 * different protocol; 0x7E 0x23 with a bad checksum is an MRA02 at a baud close
 * enough to frame but not to sample cleanly.
 */
static Result listen(const Candidate& c, uint32_t baud)
{
    Result   r;
    uint8_t  sample[16];
    size_t   sample_n = 0;

    HardwareSerial port(c.rx, c.tx);
    port.begin(baud);
    decoder.reset();

    const uint32_t deadline = millis() + kListenMs;
    while (static_cast<int32_t>(millis() - deadline) < 0) {
        uint8_t buf[64];
        size_t  n = 0;
        while (port.available() > 0 && n < sizeof(buf)) {
            buf[n++] = static_cast<uint8_t>(port.read());
        }
        if (n > 0) {
            for (size_t i = 0; i < n && sample_n < sizeof(sample); ++i) {
                sample[sample_n++] = buf[i];
            }
            r.bytes += n;
            decoder.feed(buf, n);
        }
        delay(1);
    }
    port.end();

    r.frames = decoder.frames_ok();
    r.bad    = decoder.frames_bad();

    if (r.bytes > 0) {
        char msg[96];
        snprintf(msg, sizeof(msg), "  %-28s %7lu baud  %4lu bytes  %2lu frames  %2lu bad",
                 c.name, static_cast<unsigned long>(baud),
                 static_cast<unsigned long>(r.bytes),
                 static_cast<unsigned long>(r.frames),
                 static_cast<unsigned long>(r.bad));
        Serial.println(msg);
        if (r.frames == 0) hex_dump(sample, sample_n);
    }
    return r;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }
    Serial.println();
    Serial.println("=== IMU UART sniffer =======================================");
    Serial.println("Yahboom MRA02: 0x7E 0x23 framing, 115200 baud, 25 Hz.");
    Serial.println("Only lines for combinations that received something are shown.");
    Serial.println("Move one wire at a time — the sweep repeats for ever.");
}

void loop()
{
    Serial.println();
    Serial.println("--- sweeping -----------------------------------------------");

    uint32_t total_bytes = 0;
    bool     found       = false;

    for (size_t p = 0; p < kPinN; ++p) {
        for (size_t b = 0; b < kBaudN; ++b) {
            const Result r = listen(kPins[p], kBauds[b]);
            total_bytes += r.bytes;

            if (r.frames >= 2) {
                found = true;
                char msg[128];
                snprintf(msg, sizeof(msg),
                         "\nFOUND: MRA02 on %s at %lu baud.",
                         kPins[p].name, static_cast<unsigned long>(kBauds[b]));
                Serial.println(msg);
                if (kPins[p].rx != PA10) {
                    Serial.println("       gps_imu_ekf listens on PA10. Either move the wire");
                    Serial.println("       to PA10 (D2), or change ImuSerial in");
                    Serial.println("       gps_imu_ekf/main.cpp to this pair.");
                } else if (kBauds[b] != 115200) {
                    Serial.println("       Not the factory rate, but gps_imu_ekf scans for it.");
                } else {
                    Serial.println("       This is what gps_imu_ekf expects. Flash it.");
                }
            }
        }
    }

    if (found) return;

    /* Nothing decoded anywhere. The byte count is what separates the two
     * remaining faults, and they need completely different fixes — so say which
     * one it is rather than printing a checklist of everything. */
    Serial.println();
    if (total_bytes == 0) {
        Serial.println("NOTHING on any pin, at any baud. Not a baud problem —");
        Serial.println("nothing is transmitting into this board at all. In the order");
        Serial.println("these actually go wrong:");
        Serial.println("  1. Power. Is the module's LED on? VCC to 5V, and measure it.");
        Serial.println("  2. Ground. The IMU's GND must be common with the Nucleo's.");
        Serial.println("     A module powered from somewhere else with no shared");
        Serial.println("     ground transmits into nothing.");
        Serial.println("  3. The pair is not crossed. The module's TX goes to the");
        Serial.println("     Nucleo's RX (PA10/D2), not to its TX.");
        Serial.println("  4. The wire is in a pin not on this list, or in a header");
        Serial.println("     hole next to the right one.");
    } else {
        Serial.println("Bytes arrived, but no frame ever checksummed. The wiring is");
        Serial.println("fine — the framing is not. Read the hex above:");
        Serial.println("  all 0x00 / all 0xFF   floating pin, or a baud far off");
        Serial.println("  readable ASCII        a module speaking something else");
        Serial.println("  0x7E 0x23, bad sums   an MRA02 at a near-miss baud rate");
        Serial.println("  0x55 headers          a WitMotion-protocol module: the host");
        Serial.println("                        decoder handles it, this one does not");
    }
}
