#include <imu/imu_reader.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

#include <dirent.h>
#include <unistd.h>

namespace imu {

namespace {

uint64_t now_us()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

std::string read_sys(const std::string& path)
{
    std::ifstream f(path);
    std::string   s;
    std::getline(f, s);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
        s.pop_back();
    return s;
}

// USB-to-serial bridges used by these modules, so the diagnosis can say which
// kernel driver should have claimed the device.
struct Bridge { const char* vid; const char* pid; const char* driver; const char* label; };

const Bridge kBridges[] = {
    {"1a86", "7523", "ch341", "CH340"},
    {"1a86", "5523", "ch341", "CH341"},
    {"1a86", "55d4", "ch341", "CH9102"},
    {"10c4", "ea60", "cp210x", "CP2102"},
    {"0403", "6001", "ftdi_sio", "FT232"},
    {"0403", "6015", "ftdi_sio", "FT231X"},
};

}  // namespace

std::string port_diagnosis(const std::string& device)
{
    std::ostringstream os;

    if (!device.empty() && ::access(device.c_str(), F_OK) == 0) {
        if (::access(device.c_str(), R_OK | W_OK) != 0) {
            os << device << " exists but is not readable by this user.\n"
               << "  Add yourself to the dialout group, then log out and back in:\n"
               << "    sudo usermod -aG dialout $USER\n"
               << "  Or, just for now:  sudo chmod a+rw " << device;
            return os.str();
        }
        return device + " looks fine — nothing decoded from it";
    }

    // No device node. If the USB bridge is plugged in but its interface is bound
    // to something other than the kernel serial driver, no /dev/ttyUSB* is ever
    // created — that is the case worth naming, because the fix is one command.
    DIR* dir = ::opendir("/sys/bus/usb/devices");
    if (dir) {
        while (dirent* e = ::readdir(dir)) {
            const std::string base = std::string("/sys/bus/usb/devices/") + e->d_name;
            const std::string vid  = read_sys(base + "/idVendor");
            const std::string pid  = read_sys(base + "/idProduct");
            if (vid.empty()) continue;

            for (const Bridge& b : kBridges) {
                if (vid != b.vid || pid != b.pid) continue;

                const std::string iface = base + "/" + e->d_name + ":1.0";
                char link[512]{};
                const ssize_t n = ::readlink((iface + "/driver").c_str(), link, sizeof(link) - 1);
                std::string drv;
                if (n > 0) {
                    drv = link;
                    const auto slash = drv.find_last_of('/');
                    if (slash != std::string::npos) drv = drv.substr(slash + 1);
                }

                os << b.label << " USB-serial bridge found at " << e->d_name;
                if (drv == b.driver) {
                    os << ", driver " << b.driver << " loaded, but no /dev/ttyUSB* node.\n"
                       << "  Try replugging the cable.";
                } else if (drv.empty()) {
                    os << ", but no driver is bound to it.\n"
                       << "  Load the driver and bind it:\n"
                       << "    sudo modprobe " << b.driver << "\n"
                       << "    echo '" << e->d_name << ":1.0' | sudo tee /sys/bus/usb/drivers/"
                       << b.driver << "/bind";
                } else {
                    os << ", but its interface is bound to '" << drv << "' instead of "
                       << b.driver << ",\n"
                       << "  so the kernel never creates a serial device for it.\n"
                       << "  Hand it back to the serial driver:\n"
                       << "    echo '" << e->d_name << ":1.0' | sudo tee /sys/bus/usb/drivers/"
                       << drv << "/unbind\n"
                       << "    echo '" << e->d_name << ":1.0' | sudo tee /sys/bus/usb/drivers/"
                       << b.driver << "/bind\n"
                       << "  Unplugging and replugging the cable does the same thing.";
                }
                ::closedir(dir);
                return os.str();
            }
        }
        ::closedir(dir);
    }

    os << "no serial port found.\n"
       << "  Plug the IMU in over USB and check it appears:  ls /dev/ttyUSB*\n"
       << "  and that the kernel saw it:                     dmesg | tail";
    return os.str();
}

// ─── probe ───────────────────────────────────────────────────────────────────

bool ImuReader::probe(const std::string& device, int baud_hint, int& baud_out,
                      Protocol& proto_out, std::string& error)
{
    SerialPort port;
    std::vector<int> bauds;
    if (baud_hint > 0) bauds.push_back(baud_hint);
    for (int b : candidate_bauds())
        if (b != baud_hint) bauds.push_back(b);

    if (!port.open(device, bauds.front(), error)) return false;

    for (int baud : bauds) {
        if (!port.set_baud(baud, error)) continue;
        port.flush_input();

        ImuDecoder decoder;
        uint8_t    buf[512];
        const uint64_t deadline = now_us() + 700000;   // 0.7 s per rate

        while (now_us() < deadline) {
            const int n = port.read(buf, sizeof(buf), 120);
            if (n < 0) { error = "read error on " + device; return false; }
            if (n == 0) continue;
            decoder.feed(buf, static_cast<size_t>(n));
            if (decoder.protocol() != Protocol::Unknown) {
                baud_out  = baud;
                proto_out = decoder.protocol();
                return true;
            }
        }
        if (decoder.stats().bytes == 0 && baud == bauds.back()) {
            error = "port opens but no bytes arrive at any baud rate — check power,"
                    " the TX/RX wiring, and that the module is actually streaming";
            return false;
        }
    }

    error = "bytes arrive but no known protocol decodes them — run `imu_monitor --raw`"
            " to dump the stream";
    return false;
}

// ─── ImuReader ───────────────────────────────────────────────────────────────

ImuReader::ImuReader(const ReaderConfig& cfg) : cfg_(cfg)
{
    att_           = std::make_unique<AttitudeEstimator>(cfg_.attitude);
    state_.device  = cfg_.device;
    state_.baud    = cfg_.baud;
    state_.status  = "not started";
}

ImuReader::~ImuReader()
{
    stop();
}

bool ImuReader::open_and_detect(std::string& error)
{
    if (cfg_.device.empty()) {
        const auto ports = list_serial_ports();
        if (ports.empty()) {
            error = port_diagnosis("");
            return false;
        }
        cfg_.device = ports.front();
    }

    int      baud  = cfg_.baud;
    Protocol proto = cfg_.protocol;

    if (proto == Protocol::Unknown || baud <= 0) {
        int      found_baud  = 0;
        Protocol found_proto = Protocol::Unknown;
        std::string perr;
        if (probe(cfg_.device, cfg_.baud, found_baud, found_proto, perr)) {
            baud  = found_baud;
            proto = cfg_.protocol == Protocol::Unknown ? found_proto : cfg_.protocol;
        } else if (cfg_.baud > 0) {
            // An explicit baud rate is an instruction, not a hint: open it anyway
            // and let the stream statistics show what is happening.
            baud = cfg_.baud;
        } else {
            error = perr + "\n" + port_diagnosis(cfg_.device);
            return false;
        }
    }

    if (!port_.open(cfg_.device, baud, error)) {
        error += "\n" + port_diagnosis(cfg_.device);
        return false;
    }

    decoder_.reset();
    if (proto != Protocol::Unknown) decoder_.force_protocol(proto);

    std::lock_guard<std::mutex> lock(mtx_);
    state_.device    = cfg_.device;
    state_.baud      = baud;
    state_.protocol  = proto;
    state_.connected = true;
    state_.status    = cfg_.device + " @ " + std::to_string(baud) + " — " +
                    protocol_name(proto);
    return true;
}

bool ImuReader::start(std::string& error)
{
    if (running_) return true;
    if (!open_and_detect(error)) {
        std::lock_guard<std::mutex> lock(mtx_);
        state_.connected = false;
        state_.status    = error;
        return false;
    }
    running_ = true;
    thread_  = std::thread(&ImuReader::run, this);
    return true;
}

void ImuReader::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
    port_.close();
}

void ImuReader::run()
{
    uint8_t buf[1024];

    while (running_) {
        const int n = port_.read(buf, sizeof(buf), 100);
        if (n < 0) {
            std::lock_guard<std::mutex> lock(mtx_);
            state_.connected = false;
            state_.status    = "serial read failed — was the cable unplugged?";
            break;
        }
        if (n > 0) decoder_.feed(buf, static_cast<size_t>(n));

        if (want_recal_.exchange(false)) att_->reset();
        if (want_zero_.exchange(false)) att_->zero_here();
        if (want_reset_pos_.exchange(false)) att_->reset_position();

        ImuSample s;
        bool      got = false;
        while (decoder_.pop(s)) {
            att_->update(s);
            got = true;
        }

        std::lock_guard<std::mutex> lock(mtx_);
        if (got) {
            last_sample_us_    = s.stamp_us;
            state_.sample      = s;
            state_.euler_deg   = att_->euler_deg();
            state_.orientation = att_->orientation();
            state_.gyro_bias_dps = att_->gyro_bias_dps();
            state_.linear_accel  = att_->linear_accel();
            state_.velocity      = att_->velocity();
            state_.position      = att_->position();
            state_.heading_deg   = att_->heading_deg();
            state_.calibrating   = att_->calibrating();
            state_.cal_progress  = att_->calibration_progress();
            state_.still         = att_->still();
            state_.mag_yaw       = att_->has_mag_yaw();
        }
        state_.protocol = decoder_.protocol();
        state_.stats    = decoder_.stats();
        state_.age_ms   = last_sample_us_
                            ? (now_us() - last_sample_us_) / 1000
                            : 0;

        if (state_.protocol == Protocol::Unknown && state_.stats.bytes > 256) {
            state_.status = decoder_.sniff_report();
        } else if (last_sample_us_ == 0) {
            state_.status = state_.stats.bytes == 0
                                ? "waiting for the first byte from " + state_.device
                                : "receiving bytes, waiting for a complete frame";
        } else if (state_.age_ms > 2000) {
            state_.status = "stream stalled — no sample for " +
                            std::to_string(state_.age_ms / 1000) + " s";
        } else {
            state_.status = state_.device + " @ " + std::to_string(state_.baud) +
                            " — " + protocol_name(state_.protocol);
        }
    }
}

ImuState ImuReader::state() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    ImuState copy = state_;
    if (copy.connected && last_sample_us_)
        copy.age_ms = (now_us() - last_sample_us_) / 1000;
    return copy;
}

void ImuReader::zero_here()      { want_zero_      = true; }
void ImuReader::reset_position() { want_reset_pos_ = true; }
void ImuReader::recalibrate()    { want_recal_     = true; }

}  // namespace imu
