#include <navigation/gps_sources.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <logging/logger.hpp>
#include <navigation/osm_map.hpp>   // kEarthR

#include <navigation/fused_gps.hpp>

#include "http_get.hpp"

namespace navigation {

namespace {

uint64_t now_ms()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count());
}

bool plausible(double lat, double lon)
{
    if (std::isnan(lat) || std::isnan(lon)) return false;
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;
    // 0,0 is the Atlantic — in practice it means "the sensor answered with nothing".
    return std::abs(lat) > 1e-7 || std::abs(lon) > 1e-7;
}

std::vector<double> numbers_in(const std::string& s)
{
    std::vector<double> out;
    size_t i = 0;
    while (i < s.size()) {
        if (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '-' || s[i] == '+') {
            char* end = nullptr;
            const double v = std::strtod(s.c_str() + i, &end);
            if (end && end != s.c_str() + i) {
                out.push_back(v);
                i = static_cast<size_t>(end - s.c_str());
                continue;
            }
        }
        ++i;
    }
    return out;
}

// Last innermost [...] holding at least two numbers. That is the newest sample in
//   {"gps":{"data":[[ts,[lat,lon,alt]], ...]}}
// without needing a JSON library for one endpoint.
std::vector<double> last_leaf_array(const std::string& s, size_t from)
{
    std::vector<double> best;
    size_t i = from;
    while (i < s.size()) {
        if (s[i] == '[') {
            size_t j = i + 1;
            bool leaf = true;
            while (j < s.size() && s[j] != ']') {
                if (s[j] == '[') { leaf = false; break; }
                ++j;
            }
            if (leaf && j < s.size()) {
                auto nums = numbers_in(s.substr(i + 1, j - i - 1));
                if (nums.size() >= 2) best = std::move(nums);
                i = j + 1;
                continue;
            }
        }
        ++i;
    }
    return best;
}

// "key": 12.34  → 12.34
bool json_number(const std::string& s, const std::string& key, double& out)
{
    const auto k = s.find("\"" + key + "\"");
    if (k == std::string::npos) return false;
    auto p = s.find(':', k);
    if (p == std::string::npos) return false;
    ++p;
    while (p < s.size() && (s[p] == ' ' || s[p] == '"')) ++p;
    char* end = nullptr;
    const double v = std::strtod(s.c_str() + p, &end);
    if (!end || end == s.c_str() + p) return false;
    out = v;
    return true;
}

// Both shapes seen from phone apps: IP Webcam's sensor log, and the flat
// {"latitude":…, "longitude":…} that GPSLogger-style endpoints emit.
bool parse_gps_json(const std::string& body, GpsFix& fix)
{
    double lat = 0.0, lon = 0.0;
    if (json_number(body, "latitude", lat) && json_number(body, "longitude", lon)) {
        if (!plausible(lat, lon)) return false;
        fix.latitude  = lat;
        fix.longitude = lon;
        json_number(body, "altitude", fix.altitude_m);
        json_number(body, "accuracy", fix.accuracy_m);
        if (json_number(body, "bearing", fix.bearing_deg)) fix.has_bearing = true;
        json_number(body, "speed", fix.speed_ms);
        fix.stamp_ms = now_ms();
        return true;
    }

    const auto gps = body.find("\"gps\"");
    if (gps == std::string::npos) return false;
    const auto data = body.find("\"data\"", gps);
    const auto nums = last_leaf_array(body, data == std::string::npos ? gps : data);
    if (nums.size() < 2 || !plausible(nums[0], nums[1])) return false;

    fix.latitude   = nums[0];
    fix.longitude  = nums[1];
    fix.altitude_m = nums.size() > 2 ? nums[2] : 0.0;
    fix.stamp_ms   = now_ms();
    return true;
}

// phyphox wraps every channel as {"<name>":{...,"buffer":[value]}}. Look the key
// up first, then the "buffer" that follows it — the whole reply is itself nested
// under a key called "buffer", so searching for that alone finds the wrong one.
bool phyphox_buffer(const std::string& s, const std::string& key, double& out)
{
    const auto k = s.find("\"" + key + "\"");
    if (k == std::string::npos) return false;
    const auto b = s.find("\"buffer\"", k);
    if (b == std::string::npos) return false;
    auto p = s.find('[', b);
    if (p == std::string::npos) return false;

    ++p;
    while (p < s.size() && std::isspace(static_cast<unsigned char>(s[p]))) ++p;
    // "[]" or "[null]" — the experiment is running but has no sample yet.
    if (p >= s.size() || s[p] == ']' || s[p] == 'n') return false;

    char* end = nullptr;
    const double v = std::strtod(s.c_str() + p, &end);
    if (!end || end == s.c_str() + p) return false;
    out = v;
    return true;
}

// The config carries the camera stream URL, so accept it as-is and cut the
// stream path off: both phone apps are addressed by origin.
std::string strip_stream_path(const std::string& base_url)
{
    std::string base = base_url;
    while (!base.empty() && base.back() == '/') base.pop_back();
    for (const char* suffix : {"/video", "/videofeed", "/shot.jpg"}) {
        const std::string s(suffix);
        if (base.size() > s.size() &&
            base.compare(base.size() - s.size(), s.size(), s) == 0) {
            base.resize(base.size() - s.size());
            break;
        }
    }
    return base;
}

// IP Webcam and GeoClue report position only. Derive the rest from successive
// fixes, holding the last bearing when stopped so the map does not spin.
void derive_motion(GpsFix& fix, const GpsFix* prev)
{
    if (!prev || fix.stamp_ms <= prev->stamp_ms) return;

    const double dt = (fix.stamp_ms - prev->stamp_ms) * 1e-3;
    const double lat_r = fix.latitude * M_PI / 180.0;
    const double north = (fix.latitude  - prev->latitude)  * M_PI / 180.0 * kEarthR;
    const double east  = (fix.longitude - prev->longitude) * M_PI / 180.0 * kEarthR
                         * std::cos(lat_r);
    const double dist = std::hypot(north, east);

    if (fix.speed_ms <= 0.0 && dt > 0.0) fix.speed_ms = dist / dt;

    if (fix.has_bearing) return;

    // A parked phone still reports a position that wanders by tens of metres,
    // and treating that wander as travel produces a heading that points
    // anywhere. On a heading-up map that is a compass spinning on its own. So a
    // new heading is only believed when the car has actually moved: further
    // than the fix's own uncertainty, and fast enough that it cannot be noise.
    const double moved_enough = std::max(8.0, prev->accuracy_m * 0.75);
    const bool   really_moving = dist > moved_enough && fix.speed_ms > 1.5;

    if (!really_moving) {
        if (prev->has_bearing) {          // hold the last real heading
            fix.bearing_deg = prev->bearing_deg;
            fix.has_bearing = true;
        }
        return;
    }

    const double raw = std::fmod(std::atan2(east, north) * 180.0 / M_PI + 360.0, 360.0);
    if (!prev->has_bearing) {
        fix.bearing_deg = raw;
    } else {
        // Low-pass, the short way round the circle, so a real turn arrives
        // smoothly instead of snapping.
        double delta = std::fmod(raw - prev->bearing_deg + 540.0, 360.0) - 180.0;
        fix.bearing_deg = std::fmod(prev->bearing_deg + 0.35 * delta + 360.0, 360.0);
    }
    fix.has_bearing = true;
}

}  // namespace

// ─── PollingGps ──────────────────────────────────────────────────────────────

PollingGps::PollingGps(double rate_hz)
    : period_s_(rate_hz > 0.0 ? 1.0 / rate_hz : 0.5)
{}

PollingGps::~PollingGps()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void PollingGps::start()
{
    thread_ = std::thread(&PollingGps::poll_loop, this);
}

void PollingGps::poll_loop()
{
    bool warned = false;
    bool prepared = prepare();
    GpsFix prev;
    bool have_prev = false;
    int empty_polls = 0;

    while (running_) {
        GpsFix fix;
        std::string note;

        if (fetch(fix, note)) {
            derive_motion(fix, have_prev ? &prev : nullptr);
            {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_ = fix;
            }
            prev = fix;
            have_prev = true;
            empty_polls = 0;
            if (warned) {
                VP_INFO("[Nav] GPS: fix restored");
                warned = false;
            }
        } else {
            // Retry the one-time setup about once a minute: the phone may have
            // been restarted, or the app reopened, mid-drive.
            if (++empty_polls % 60 == 0 && !prepared) prepared = prepare();
            if (!warned) {
                VP_WARN("[Nav] GPS: no fix from %s (%s).%s", endpoint(),
                        note.empty() ? "empty reply" : note.substr(0, 80).c_str(),
                        hint(prepared));
                warned = true;
            }
        }

        // Sleep in slices so shutdown does not wait out a whole period.
        const int slices = std::max(1, static_cast<int>(period_s_ * 20.0));
        for (int i = 0; i < slices && running_; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(period_s_ * 1000.0 / slices)));
    }
}

std::optional<GpsFix> PollingGps::read()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return latest_;
}

// ─── IpWebcamGps ─────────────────────────────────────────────────────────────

IpWebcamGps::IpWebcamGps(const std::string& base_url, double rate_hz)
    : PollingGps(rate_hz)
{
    base_ = strip_stream_path(base_url);
    url_  = base_ + "/sensors.json?sense=gps";

    VP_INFO("[Nav] GPS: polling %s at %.1f Hz", url_.c_str(), rate_hz);
    start();
}

// The app ships with its GPS sensor off, and /sensors.json is empty until it is
// on. Switching it on is a plain GET — the same one the app's web UI issues — so
// do it here instead of asking anyone to unlock the phone.
bool IpWebcamGps::prepare()
{
    std::string body;
    if (!http::get(base_ + "/settings/gps_active?set=on", body, 4000)) {
        VP_WARN("[Nav] GPS: cannot reach %s (%s)", base_.c_str(),
                body.substr(0, 60).c_str());
        return false;
    }
    // "<result>Ok</result>" on success; anything else means a different app.
    if (body.find("Ok") == std::string::npos) return false;
    VP_INFO("[Nav] GPS: asked %s to switch its GPS sensor on", base_.c_str());
    return true;
}

const char* IpWebcamGps::hint(bool prepared) const
{
    // The empty answer is nearly always one of these two, and the retention
    // window is the one nobody thinks of: a GPS fix that arrives every 20 s is
    // gone from a 5 s buffer long before the next poll reads it.
    return prepared
               ? " Sensor is on. In the app: Data logging -> tick GPS *and* raise"
                 " 'Sensor data retention' well above 5 s. Indoors the phone may"
                 " never lock on — try nav.gps_source = phyphox."
               : " Enable GPS in IP Webcam (Data logging -> GPS).";
}

bool IpWebcamGps::fetch(GpsFix& fix, std::string& note)
{
    std::string body;
    if (!http::get(url_, body, 3000)) { note = body; return false; }
    if (!parse_gps_json(body, fix)) { note = body.substr(0, 80); return false; }
    return true;
}

// ─── PhyphoxGps ──────────────────────────────────────────────────────────────

PhyphoxGps::PhyphoxGps(const std::string& base_url, double rate_hz)
    : PollingGps(rate_hz)
{
    base_ = strip_stream_path(base_url);
    // Bare buffer names return the newest value of each — exactly one fix.
    // Values come back in scientific notation ("1.0765048E2"), which strtod eats.
    url_  = base_ + "/get?lat&lon&z&v&dir&accuracy";

    VP_INFO("[Nav] GPS: polling %s at %.1f Hz", url_.c_str(), rate_hz);
    start();
}

const char* PhyphoxGps::hint(bool) const
{
    return " In phyphox: open the 'Location (GPS)' experiment, three-dot menu ->"
           " Allow remote access, then press play. It must be measuring.";
}

bool PhyphoxGps::fetch(GpsFix& fix, std::string& note)
{
    std::string body;
    if (!http::get(url_, body, 3000)) { note = body; return false; }

    double lat = 0.0, lon = 0.0;
    if (!phyphox_buffer(body, "lat", lat) || !phyphox_buffer(body, "lon", lon)) {
        note = body.substr(0, 80);
        return false;
    }
    if (!plausible(lat, lon)) {
        // Before the first fix phyphox reports null, which reads back as 0,0.
        note = "experiment running but no fix yet";
        return false;
    }

    fix.latitude  = lat;
    fix.longitude = lon;
    fix.stamp_ms  = now_ms();
    phyphox_buffer(body, "z", fix.altitude_m);
    phyphox_buffer(body, "v", fix.speed_ms);
    phyphox_buffer(body, "accuracy", fix.accuracy_m);
    // Android reports bearing as 0 when stopped rather than "unknown", so a
    // standing start would otherwise lock the map pointing north. Let
    // derive_motion() fill it from movement instead.
    // Android reports a bearing even when parked, where it is meaningless and
    // jumps between fixes. Only take it while genuinely moving; otherwise leave
    // it unset and let derive_motion() hold the last real heading.
    double dir = 0.0;
    if (phyphox_buffer(body, "dir", dir) && fix.speed_ms > 1.5 && dir != 0.0) {
        fix.bearing_deg = dir;
        fix.has_bearing = true;
    }
    return true;
}

// ─── NucleoGps ───────────────────────────────────────────────────────────────

namespace {

// XOR of everything between '$' and '*', which is what an NMEA-style line is
// framed with. Returns the body with the framing stripped, or empty on a line
// that does not verify — and a line that does not verify is line noise, not a
// position, so it is dropped rather than parsed.
std::string verified_body(const std::string& line)
{
    const auto dollar = line.find('$');
    if (dollar == std::string::npos) return {};
    const auto star = line.rfind('*');
    if (star == std::string::npos || star < dollar + 2) return {};
    if (star + 2 >= line.size() + 1) return {};

    const std::string body = line.substr(dollar + 1, star - dollar - 1);

    uint8_t sum = 0;
    for (char c : body) sum ^= static_cast<uint8_t>(c);

    const std::string given = line.substr(star + 1, 2);
    if (given.size() < 2) return {};
    char* end = nullptr;
    const long want = std::strtol(given.c_str(), &end, 16);
    if (end != given.c_str() + 2) return {};

    return (static_cast<long>(sum) == want) ? body : std::string{};
}

std::vector<std::string> split(const std::string& s, char sep)
{
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        const auto p = s.find(sep, start);
        out.push_back(s.substr(start, p == std::string::npos ? p : p - start));
        if (p == std::string::npos) break;
        start = p + 1;
    }
    return out;
}

double to_d(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

}  // namespace

NucleoGps::NucleoGps(const std::string& port, int baud)
    : want_port_(port), baud_(baud > 0 ? baud : 115200)
{
    running_ = true;
    thread_  = std::thread(&NucleoGps::run, this);
}

NucleoGps::~NucleoGps()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
    port_.close();
}

/* Open the configured port, or find one.
 *
 * The auto-detect looks for a CDC-ACM device, because that is what an ST-LINK
 * enumerates as — a USB-serial bridge like the IMU's CH340 comes up as ttyUSB
 * and is a different board entirely. Picking the wrong one here would look like
 * a receiver that never sends a fix.
 */
bool NucleoGps::open_port(std::string& error)
{
    std::string dev = want_port_;
    if (dev.empty()) {
        for (const auto& p : imu::list_serial_ports()) {
            if (p.rfind("/dev/ttyACM", 0) == 0) { dev = p; break; }
        }
    }
    if (dev.empty()) {
        error = "no /dev/ttyACM* device — is the Nucleo plugged in?";
        return false;
    }
    return port_.open(dev, baud_, error);
}

void NucleoGps::run()
{
    bool     warned      = false;
    uint64_t last_line_ms = 0;

    while (running_) {
        if (!port_.is_open()) {
            std::string error;
            if (!open_port(error)) {
                if (!warned) {
                    VP_WARN("[Nav] GPS: %s", error.c_str());
                    warned = true;
                }
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    status_ = "no board: " + error;
                }
                // The board may be plugged in later, or be mid-reset after a
                // flash. Retry rather than giving up for the drive.
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            VP_INFO("[Nav] GPS: reading %s at %d baud", port_.device().c_str(), baud_);
            warned       = false;
            partial_.clear();
            last_line_ms = now_ms();
        }

        uint8_t buf[512];
        const int n = port_.read(buf, sizeof(buf), 200);
        if (n < 0) {
            // Unplugged mid-drive. Close and let the loop above find it again.
            VP_WARN("[Nav] GPS: %s went away", port_.device().c_str());
            port_.close();
            continue;
        }

        for (int i = 0; i < n; ++i) {
            const char c = static_cast<char>(buf[i]);
            if (c == '\n' || c == '\r') {
                if (!partial_.empty()) {
                    consume(partial_);
                    partial_.clear();
                    last_line_ms = now_ms();
                }
            } else if (partial_.size() < 512) {
                partial_.push_back(c);
            } else {
                partial_.clear();      // no line is this long; resynchronise
            }
        }

        // The MCU sends something at least every couple of seconds even with no
        // fix — $VPSTA if nothing else. Total silence means it is wedged or the
        // port is being read by something else at the same time.
        if (now_ms() - last_line_ms > 5000) {
            std::lock_guard<std::mutex> lock(mtx_);
            status_ = "silent for " + std::to_string((now_ms() - last_line_ms) / 1000) +
                      "s — is pio device monitor still open?";
        }

        // A fix a second is no longer the whole story: the fused stream is
        // 20 Hz, and dropping to the raw rate on a busy line would throw away
        // exactly the smoothness this source exists to provide.
        if (n == 0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void NucleoGps::consume(const std::string& line)
{
    const std::string body = verified_body(line);
    if (body.empty()) return;

    if (body.rfind("VPFUS,", 0) == 0) {
        parse_fused(body);
    } else if (body.rfind("VPRTK,", 0) == 0) {
        parse_raw(body);
    } else if (body.rfind("VPSTA,", 0) == 0) {
        // Everything that is not a position: boot, wiring faults, quality
        // transitions, the 10 s health line. Worth logging as it arrives —
        // this is where "the IMU is not connected" shows up, and it is the
        // only place it shows up at all.
        const auto comma = body.find(',', 6);
        const std::string msg = (comma == std::string::npos) ? body : body.substr(comma + 1);
        VP_INFO("[Nav] Nucleo: %s", msg.c_str());
        std::lock_guard<std::mutex> lock(mtx_);
        status_ = msg;
    }
}

/* $VPFUS,<uptime_ms>,<lat>,<lon>,<speed_ms>,<heading_deg>,<sigma_m>,
 *        <quality>,<imu_hz>,<yaw_bias_dps>
 *
 * The MCU has already fused GPS with the IMU, so speed and heading are the
 * filter's own and are used as given — no derive_motion() here. <sigma_m> is
 * the filter's estimate of its own position error, which is exactly what
 * accuracy_m means downstream, and it grows honestly when the receiver drops
 * out and the position becomes pure dead reckoning.
 */
bool NucleoGps::parse_fused(const std::string& body)
{
    const auto f = split(body, ',');
    if (f.size() < 7) return false;

    GpsFix fix;
    fix.latitude    = to_d(f[2]);
    fix.longitude   = to_d(f[3]);
    if (!plausible(fix.latitude, fix.longitude)) return false;

    fix.speed_ms    = to_d(f[4]);
    fix.bearing_deg = to_d(f[5]);
    fix.has_bearing = true;
    fix.accuracy_m  = to_d(f[6]);
    fix.stamp_ms    = now_ms();

    std::lock_guard<std::mutex> lock(mtx_);
    fused_         = fix;
    last_fused_ms_ = fix.stamp_ms;
    fused_active_  = true;
    return true;
}

/* $VPRTK,<uptime_ms>,<utc>,<lat>,<lon>,<alt_m>,<geoid_sep_m>,<quality>,
 *        <sats>,<hdop>,<accuracy_m>,<corr_age_s>,<base_id>
 *
 * The raw receiver. It reports no speed or course — the register interface
 * carries neither — so those are derived from successive fixes, the same way
 * every other position-only source here does it.
 */
bool NucleoGps::parse_raw(const std::string& body)
{
    const auto f = split(body, ',');
    if (f.size() < 11) return false;

    GpsFix fix;
    fix.latitude   = to_d(f[3]);
    fix.longitude  = to_d(f[4]);
    if (!plausible(fix.latitude, fix.longitude)) return false;

    fix.altitude_m = to_d(f[5]);
    fix.accuracy_m = to_d(f[10]);
    fix.stamp_ms   = now_ms();

    std::lock_guard<std::mutex> lock(mtx_);
    derive_motion(fix, prev_raw_ ? &*prev_raw_ : nullptr);
    prev_raw_ = fix;
    raw_      = fix;
    return true;
}

std::optional<GpsFix> NucleoGps::read()
{
    std::lock_guard<std::mutex> lock(mtx_);

    // Prefer the filtered position, but only while it is actually arriving. The
    // MCU stops sending $VPFUS if its filter has no origin yet, and a fused
    // position frozen two seconds ago is worse than a fresh raw one.
    if (fused_ && now_ms() - last_fused_ms_ < 2000) return fused_;

    if (fused_active_ && fused_) {
        fused_active_ = false;
        VP_WARN("[Nav] Nucleo: fused stream stopped, falling back to the raw receiver");
    }
    return raw_;
}

std::optional<GpsFix> NucleoGps::raw_fix() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return raw_;
}

std::string NucleoGps::status() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return status_;
}

// ─── FileGps ─────────────────────────────────────────────────────────────────

FileGps::FileGps(const std::string& path) : file_(path)
{
    open_ = file_.is_open();
    if (!open_) VP_ERROR("[Nav] GPS: cannot open track %s", path.c_str());
    else        VP_INFO("[Nav] GPS: replaying track %s", path.c_str());
}

std::optional<GpsFix> FileGps::read()
{
    if (!open_) return last_;

    std::string line;
    while (std::getline(file_, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);

        GpsFix fix;
        if (!(ss >> fix.latitude >> fix.longitude)) continue;
        if (!plausible(fix.latitude, fix.longitude)) continue;
        fix.stamp_ms = now_ms();
        if (ss >> fix.speed_ms) {
            if (ss >> fix.bearing_deg) fix.has_bearing = true;
        }
        derive_motion(fix, last_ ? &*last_ : nullptr);
        last_ = fix;
        return last_;
    }

    // Track exhausted: hold the last position rather than losing the map.
    open_ = false;
    return last_;
}

// ─── FixedGps ────────────────────────────────────────────────────────────────

FixedGps::FixedGps(double lat, double lon)
{
    fix_.latitude   = lat;
    fix_.longitude  = lon;
    fix_.accuracy_m = 5.0;
    fix_.stamp_ms   = now_ms();
    VP_INFO("[Nav] GPS: fixed position %.5f,%.5f (not a real fix)", lat, lon);
}

std::optional<GpsFix> FixedGps::read()
{
    fix_.stamp_ms = now_ms();
    return fix_;
}

// ─── Factory ─────────────────────────────────────────────────────────────────

GpsSource parse_gps_source(const std::string& v)
{
    if (v == "ipwebcam" || v == "phone" || v == "ip") return GpsSource::IpWebcam;
    if (v == "phyphox" || v == "phypox")              return GpsSource::Phyphox;
    if (v == "file"  || v == "track")                 return GpsSource::File;
    if (v == "fixed" || v == "static")                return GpsSource::Fixed;
    if (v == "nucleo" || v == "rtk" || v == "stm32")  return GpsSource::Nucleo;
    if (v == "none"  || v == "off")                   return GpsSource::None;
    throw std::runtime_error("Invalid nav.gps_source: '" + v +
                             "'. Use ipwebcam|phyphox|nucleo|file|fixed|none");
}

RoadOverlay parse_road_overlay(const std::string& v)
{
    if (v == "all"   || v == "full" || v == "true")  return RoadOverlay::All;
    if (v == "major" || v == "arterial")             return RoadOverlay::Major;
    if (v == "off"   || v == "none" || v == "false") return RoadOverlay::Off;
    throw std::runtime_error("Invalid nav.satellite_roads: '" + v +
                             "'. Use all|major|off");
}

std::unique_ptr<GpsInterface> make_gps_source(const Config& cfg)
{
    auto raw = make_raw_gps_source(cfg);

    // The Nucleo has already done this, on the board both sensors are wired to.
    // Wrapping it in the host filter would run two EKFs in series, the second
    // one fed a position that is already smoothed — which does not average out,
    // it lags. Say so rather than quietly ignoring the flag: nav.imu_enabled is
    // how the phone-IMU setup is switched on, and someone will carry it over.
    if (cfg.imu_enabled && cfg.gps_source == GpsSource::Nucleo) {
        VP_WARN("[Nav] nav.imu_enabled ignored: gps_source = nucleo is already "
                "GPS/IMU fused on the board. Wire the IMU to the PC and use a "
                "phone GPS source to run the filter on the host instead.");
        return raw;
    }

    // The IMU wraps whatever receiver was configured rather than replacing it:
    // the filter needs both, and with no GPS at all there is nothing to anchor
    // dead reckoning to, so wrapping nothing would be worse than not wrapping.
    if (cfg.imu_enabled && raw)
        return std::make_unique<FusedImuGps>(std::move(raw), cfg.imu_port, cfg.imu_baud);
    return raw;
}

std::unique_ptr<GpsInterface> make_raw_gps_source(const Config& cfg)
{
    switch (cfg.gps_source) {
    case GpsSource::IpWebcam:
        return std::make_unique<IpWebcamGps>(cfg.gps_url, cfg.gps_rate_hz);
    case GpsSource::Phyphox:
        return std::make_unique<PhyphoxGps>(cfg.gps_url, cfg.gps_rate_hz);
    case GpsSource::File:
        return std::make_unique<FileGps>(cfg.gps_file);
    case GpsSource::Fixed:
        return std::make_unique<FixedGps>(cfg.fixed_lat, cfg.fixed_lon);
    case GpsSource::Nucleo:
        return std::make_unique<NucleoGps>(cfg.gps_port, cfg.gps_baud);
    case GpsSource::None:
    default:
        return nullptr;
    }
}

}  // namespace navigation
