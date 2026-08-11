/*!
 * @file  test_nucleo_gps.cpp
 * @brief Drive NucleoGps from a pseudo-terminal and check what comes out.
 *
 * The board is not always plugged in, and the failure this guards against is
 * silent: a field counted from the wrong end still parses, still passes the
 * checksum, and still puts a plausible coordinate on the map. Only comparing
 * against known bytes catches that.
 *
 * The sentences below are a verbatim capture from a Nucleo running the
 * gps_imu_ekf firmware in Bandung, checksums included — so this also pins the
 * wire format itself. If someone reorders a field in the firmware, this fails.
 *
 *   ./test_nucleo_gps
 */

#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include <navigation/gps_sources.hpp>

namespace {

int g_failures = 0;

void check(const char* what, double got, double want, double tol)
{
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %-28s %14.7f  (want %.7f +-%g)  %s\n", what, got, want, tol,
                ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

void check_true(const char* what, bool ok)
{
    std::printf("  %-28s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

// A PTY pair stands in for the board: the slave is a real character device that
// termios accepts, so NucleoGps opens it exactly as it would open /dev/ttyACM0
// and nothing in the code under test knows the difference.
struct Pty
{
    int         master = -1;
    std::string slave;

    bool open()
    {
        master = posix_openpt(O_RDWR | O_NOCTTY);
        if (master < 0) return false;
        if (grantpt(master) != 0 || unlockpt(master) != 0) return false;
        const char* name = ptsname(master);
        if (!name) return false;
        slave = name;
        // Raw, or the line discipline would echo everything we write straight
        // back at us and the reader would see its own bytes.
        termios tio{};
        if (tcgetattr(master, &tio) == 0) {
            cfmakeraw(&tio);
            tcsetattr(master, TCSANOW, &tio);
        }
        return true;
    }

    void send(const std::string& s) const
    {
        [[maybe_unused]] ssize_t n = ::write(master, s.data(), s.size());
    }
};

// The reader runs on its own thread, so give it a moment — but bail out as soon
// as the predicate is true rather than sleeping a fixed time.
template <typename Fn>
bool wait_for(Fn&& fn, int timeout_ms = 3000)
{
    for (int i = 0; i < timeout_ms / 10; ++i) {
        if (fn()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

/* Send a sentence until the reader reacts to it.
 *
 * Not just belt and braces: SerialPort::open() flushes the input buffer, which
 * is the right thing against real hardware — whatever arrived before the port
 * was opened is a partial line — but it means anything written here before the
 * reader thread gets to open() is discarded. Repeating is also what the board
 * actually does: it publishes on a timer and does not care who is listening.
 */
template <typename Fn>
bool feed_until(const Pty& pty, const std::string& line, Fn&& fn, int timeout_ms = 3000)
{
    for (int i = 0; i < timeout_ms / 100; ++i) {
        pty.send(line);
        if (wait_for(fn, 100)) return true;
    }
    return false;
}

}  // namespace

int main()
{
    Pty pty;
    if (!pty.open()) {
        std::printf("cannot allocate a pty — skipping\n");
        return 0;
    }

    std::printf("nucleo gps reader, fed from %s\n\n", pty.slave.c_str());
    navigation::NucleoGps gps(pty.slave, 115200);

    // ── A raw fix, opened mid-sentence ───────────────────────────────────────
    // The leading fragment is what a host sees when it opens the port while the
    // board is halfway through a line. It must be dropped, not parsed.
    std::printf("raw $VPRTK (after a torn partial line)\n");
    const std::string torn = "505123,716.30,3.26,1,17,0.67\r\n";
    const std::string rtk  = "$VPRTK,1213365,2026-08-07T07:31:05Z,-6.9349973,107.6505123,"
                             "716.30,3.26,1,17,0.67,1.68,0.0,0*7F\r\n";
    check_true("a fix arrived",
               feed_until(pty, torn + rtk, [&] { return gps.read().has_value(); }));
    auto fix = gps.read();
    if (fix) {
        check("latitude",  fix->latitude,   -6.9349973, 1e-7);
        check("longitude", fix->longitude,  107.6505123, 1e-7);
        check("accuracy_m", fix->accuracy_m, 1.68, 1e-6);
    } else {
        ++g_failures;
    }
    check_true("raw_fix() returns it too", gps.raw_fix().has_value());

    // ── A bad checksum must change nothing ───────────────────────────────────
    // Same sentence, one digit of latitude altered and the checksum left alone.
    // Parsing it would move the car 100 m; dropping it is the whole point of
    // framing the lines this way.
    std::printf("\ncorrupted line (checksum no longer matches)\n");
    pty.send("$VPRTK,1213366,2026-08-07T07:31:06Z,-6.9359973,107.6505123,"
             "716.30,3.26,1,17,0.67,1.68,0.0,0*7F\r\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    auto after_bad = gps.read();
    check_true("position unchanged",
               after_bad && std::fabs(after_bad->latitude - (-6.9349973)) < 1e-7);

    // ── The fused sentence wins ──────────────────────────────────────────────
    std::printf("\nfused $VPFUS\n");
    check_true("fused stream detected",
               feed_until(pty,
                          "$VPFUS,187950,-6.9349896,107.6505035,0.00,190.6,0.32,1,0.0,0.000*55\r\n",
                          [&] { return gps.fused(); }));
    auto fused = gps.read();
    if (fused) {
        check("latitude",   fused->latitude,   -6.9349896, 1e-7);
        check("longitude",  fused->longitude,  107.6505035, 1e-7);
        check("heading",    fused->bearing_deg, 190.6, 1e-3);
        check("sigma -> accuracy", fused->accuracy_m, 0.32, 1e-6);
        check_true("carries a bearing", fused->has_bearing);
    } else {
        ++g_failures;
    }

    // raw_fix() must still be the receiver, not the filter — that is what makes
    // nav.compare_gps able to draw the two apart.
    auto raw = gps.raw_fix();
    check_true("raw_fix() is still the receiver",
               raw && std::fabs(raw->latitude - (-6.9349973)) < 1e-7);

    // ── Status lines are not positions ───────────────────────────────────────
    std::printf("\n$VPSTA health line\n");
    pty.send("$VPSTA,208959,imu=25.0hz,baud=115200,bytes=41230,frames=4102,bad=0,"
             "gps-quality=1,filter=running,sigma=0.61m,yaw-bias=0.004dps*4C\r\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    auto after_sta = gps.read();
    check_true("position unchanged by status",
               after_sta && std::fabs(after_sta->latitude - (-6.9349896)) < 1e-7);

    std::printf("\n%s\n", g_failures == 0 ? "PASS" : "FAIL");
    ::close(pty.master);
    return g_failures == 0 ? 0 : 1;
}
