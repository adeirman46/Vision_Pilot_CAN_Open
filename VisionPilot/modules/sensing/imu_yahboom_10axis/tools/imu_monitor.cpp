// Console side of the Yahboom 10-axis IMU package: find the port, work out the
// wire format, and print what the sensor is saying. No GUI, so it also works
// over ssh on the car's compute box.
//
//   imu_monitor                 live readout, refreshed in place
//   imu_monitor --raw           hex dump of the stream (for an unknown format)
//   imu_monitor --csv           one line per sample, for logging to a file
//   imu_monitor --probe         report port, baud and protocol, then exit
//   imu_monitor --list          list the serial ports on this machine
//
//   --port /dev/ttyUSB0         default: the first USB serial port found
//   --baud 115200               default: scan the usual rates
//   --protocol wit|yesense|ascii   default: auto-detect

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <imu/imu_decoder.hpp>
#include <imu/imu_reader.hpp>
#include <imu/serial_port.hpp>

namespace {

std::atomic<bool> g_run{true};

void on_sigint(int) { g_run = false; }

void usage()
{
    std::printf(
        "usage: imu_monitor [--port DEV] [--baud N] [--protocol wit|yesense|ascii]\n"
        "                   [--raw | --csv | --probe | --list]\n\n"
        "  --raw     dump the bytes as hex+ASCII instead of decoding them\n"
        "  --csv     stamp,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,temp\n"
        "  --probe   report the port, baud rate and protocol, then exit\n"
        "  --list    list serial ports and exit\n");
}

imu::Protocol parse_protocol(const std::string& v)
{
    if (v == "yahboom" || v == "mra02" || v == "mra")  return imu::Protocol::Yahboom;
    if (v == "wit" || v == "witmotion" || v == "wt901") return imu::Protocol::WitMotion;
    if (v == "yesense" || v == "yis")                   return imu::Protocol::Yesense;
    if (v == "ascii" || v == "text")                    return imu::Protocol::Ascii;
    return imu::Protocol::Unknown;
}

int cmd_list()
{
    const auto ports = imu::list_serial_ports();
    if (ports.empty()) {
        std::printf("no serial ports found\n\n%s\n", imu::port_diagnosis("").c_str());
        return 1;
    }
    std::printf("serial ports:\n");
    for (const auto& p : ports) std::printf("  %s\n", p.c_str());
    return 0;
}

// Raw mode never decodes: it opens the port and prints exactly what arrives,
// which is the only useful thing to do when the format is not one we know.
int cmd_raw(const std::string& device, int baud)
{
    imu::SerialPort port;
    std::string     err;
    if (!port.open(device, baud > 0 ? baud : 115200, err)) {
        std::fprintf(stderr, "error: %s\n\n%s\n", err.c_str(),
                     imu::port_diagnosis(device).c_str());
        return 1;
    }

    std::printf("raw dump of %s @ %d baud — Ctrl-C to stop\n\n", device.c_str(),
                port.baud());

    imu::ImuDecoder decoder;   // runs alongside, purely to report what it sees
    uint8_t         buf[512];
    uint64_t        offset = 0;

    while (g_run) {
        const int n = port.read(buf, sizeof(buf), 200);
        if (n < 0) { std::fprintf(stderr, "read error\n"); return 1; }
        if (n == 0) continue;
        std::fputs(imu::hex_dump(buf, static_cast<size_t>(n), offset).c_str(), stdout);
        std::fflush(stdout);
        offset += static_cast<uint64_t>(n);
        decoder.feed(buf, static_cast<size_t>(n));
    }

    std::printf("\n%llu bytes read\n", static_cast<unsigned long long>(offset));
    std::printf("%s\n", decoder.sniff_report().c_str());
    return 0;
}

int cmd_probe(const std::string& device_in, int baud)
{
    std::string device = device_in;
    if (device.empty()) {
        const auto ports = imu::list_serial_ports();
        if (ports.empty()) {
            std::fprintf(stderr, "%s\n", imu::port_diagnosis("").c_str());
            return 1;
        }
        device = ports.front();
    }

    std::printf("probing %s ...\n", device.c_str());
    int           found_baud = 0;
    imu::Protocol proto      = imu::Protocol::Unknown;
    std::string   err;
    if (!imu::ImuReader::probe(device, baud, found_baud, proto, err)) {
        std::fprintf(stderr, "error: %s\n\n%s\n", err.c_str(),
                     imu::port_diagnosis(device).c_str());
        return 1;
    }
    std::printf("  port:     %s\n  baud:     %d\n  protocol: %s\n", device.c_str(),
                found_baud, imu::protocol_name(proto));
    return 0;
}

void print_live(const imu::ImuState& st, bool first)
{
    if (!first) std::printf("\033[11A");   // back over the block we drew last time

    const auto& s = st.sample;
    const char* clr = "\033[K";

    std::printf("%s %s%s\n", st.connected ? "\033[32m●\033[0m" : "\033[31m●\033[0m",
                st.status.c_str(), clr);
    std::printf("  %s%s\n", imu::format_stats(st.stats).c_str(), clr);
    std::printf("%s\n", clr);
    std::printf("  attitude   roll %8.2f°   pitch %8.2f°   yaw %8.2f°%s\n",
                st.euler_deg.x, st.euler_deg.y, st.euler_deg.z, clr);
    std::printf("  heading    %6.1f° %-4s %s%s\n", st.heading_deg,
                st.mag_yaw ? "(mag)" : "(gyro)",
                st.calibrating
                    ? "· calibrating gyro bias, keep it still"
                    : (st.still ? "· at rest" : "· moving"),
                clr);
    std::printf("%s\n", clr);
    std::printf("  accel      %8.3f  %8.3f  %8.3f  m/s²   |a| %6.3f%s\n", s.accel.x,
                s.accel.y, s.accel.z, s.accel.norm(), clr);
    std::printf("  gyro       %8.2f  %8.2f  %8.2f  °/s%s\n", s.gyro.x * imu::kRad2Deg,
                s.gyro.y * imu::kRad2Deg, s.gyro.z * imu::kRad2Deg, clr);
    std::printf("  mag        %8.1f  %8.1f  %8.1f  %s%s\n", s.mag.x, s.mag.y, s.mag.z,
                s.has_mag ? "" : "(none)", clr);
    std::printf("  bias       %8.3f  %8.3f  %8.3f  °/s%s\n", st.gyro_bias_dps.x,
                st.gyro_bias_dps.y, st.gyro_bias_dps.z, clr);
    std::printf("  temp %5.1f °C   pressure %8.0f Pa   alt %7.2f m%s\n",
                s.temperature_c, s.pressure_pa, s.altitude_m, clr);
    std::fflush(stdout);
}

int cmd_live(const imu::ReaderConfig& cfg, bool csv)
{
    imu::ImuReader reader(cfg);
    std::string    err;
    if (!reader.start(err)) {
        std::fprintf(stderr, "error: %s\n", err.c_str());
        return 1;
    }

    if (csv) {
        std::printf("# stamp_us,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,temp_c\n");
        uint64_t last = 0;
        while (g_run) {
            const auto st = reader.state();
            if (st.sample.stamp_us != last && st.sample.stamp_us != 0) {
                last = st.sample.stamp_us;
                const auto& s = st.sample;
                std::printf("%llu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,"
                            "%.3f,%.3f,%.3f,%.2f\n",
                            static_cast<unsigned long long>(s.stamp_us), s.accel.x,
                            s.accel.y, s.accel.z, s.gyro.x, s.gyro.y, s.gyro.z,
                            s.mag.x, s.mag.y, s.mag.z, st.euler_deg.x, st.euler_deg.y,
                            st.euler_deg.z, s.temperature_c);
                std::fflush(stdout);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return 0;
    }

    std::printf("\033[?25l");    // hide the cursor while the block redraws
    bool first = true;
    while (g_run) {
        print_live(reader.state(), first);
        first = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::printf("\033[?25h\n");
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);
    std::signal(SIGTERM, on_sigint);

    imu::ReaderConfig cfg;
    std::string       device;
    int               baud = 0;
    bool raw = false, csv = false, probe = false, list = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "error: %s needs a value\n", what);
                std::exit(1);
            }
            return argv[++i];
        };

        if (a == "--port" || a == "-p")        device = next("--port");
        else if (a == "--baud" || a == "-b")   baud = std::stoi(next("--baud"));
        else if (a == "--protocol")            cfg.protocol = parse_protocol(next("--protocol"));
        else if (a == "--raw")                 raw = true;
        else if (a == "--csv")                 csv = true;
        else if (a == "--probe")               probe = true;
        else if (a == "--list")                list = true;
        else if (a == "-h" || a == "--help")   { usage(); return 0; }
        else {
            std::fprintf(stderr, "error: unknown option '%s'\n", a.c_str());
            usage();
            return 1;
        }
    }

    if (list) return cmd_list();
    if (probe) return cmd_probe(device, baud);

    if (raw) {
        if (device.empty()) {
            const auto ports = imu::list_serial_ports();
            if (ports.empty()) {
                std::fprintf(stderr, "%s\n", imu::port_diagnosis("").c_str());
                return 1;
            }
            device = ports.front();
        }
        return cmd_raw(device, baud);
    }

    cfg.device = device;
    cfg.baud   = baud;
    return cmd_live(cfg, csv);
}
