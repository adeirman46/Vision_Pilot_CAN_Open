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
    if (v == "none"  || v == "off")                   return GpsSource::None;
    throw std::runtime_error("Invalid nav.gps_source: '" + v +
                             "'. Use ipwebcam|phyphox|file|fixed|none");
}

std::unique_ptr<GpsInterface> make_gps_source(const Config& cfg)
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
    case GpsSource::None:
    default:
        return nullptr;
    }
}

}  // namespace navigation
