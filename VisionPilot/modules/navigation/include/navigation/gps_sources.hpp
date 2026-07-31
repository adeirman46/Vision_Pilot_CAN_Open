#ifndef VISIONPILOT_GPS_SOURCES_HPP
#define VISIONPILOT_GPS_SOURCES_HPP

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <navigation/gps_interface.hpp>
#include <navigation/nav_config.hpp>

namespace navigation {

// ─── Shared HTTP polling ─────────────────────────────────────────────────────
// Both phone sources are "GET a URL, parse a fix out of it" on a timer. The
// timer lives here, on its own thread, so a phone that went to sleep stalls the
// poll and never the inference loop.
class PollingGps : public GpsInterface
{
public:
    explicit PollingGps(double rate_hz);
    ~PollingGps() override;

    std::optional<GpsFix> read() override;
    bool is_open() const override { return true; }   // a sleeping phone may come back

protected:
    // Derived constructors call this once they are fully built — the poll thread
    // calls fetch(), which is virtual.
    void start();

    // One request. Returns true on a usable fix; `note` explains a failure.
    virtual bool fetch(GpsFix& fix, std::string& note) = 0;
    // Run once before the first poll, and retried while polls come back empty.
    virtual bool prepare() { return true; }
    // Appended to the "no fix" warning to say what to do about it.
    virtual const char* hint(bool prepared) const { (void)prepared; return ""; }
    virtual const char* endpoint() const = 0;

private:
    void poll_loop();

    double            period_s_;
    std::atomic<bool> running_{true};
    std::thread       thread_;

    mutable std::mutex mtx_;
    std::optional<GpsFix> latest_;
};

// ─── IP Webcam (Android) ─────────────────────────────────────────────────────
// The phone that streams the camera also serves its sensors, so position and
// frames come off one device over one network:
//
//   GET /sensors.json?sense=gps
//   {"gps":{"unit":"deg,deg,m","data":[[1690000000000,[-6.9175,107.6191,768]]]}}
//
// The app's GPS sensor is off by default, and while it is off that endpoint
// answers `{}`. It can be switched on remotely — GET /settings/gps_active?set=on
// is what the app's own web UI does — so this source turns it on at startup
// rather than making anyone go and tick a box on the phone.
//
// Polling runs on its own thread: a phone that went to sleep stalls the poll,
// never the inference loop.
class IpWebcamGps : public PollingGps
{
public:
    // base_url is the stream URL without /video, e.g. http://10.0.0.5:8080
    IpWebcamGps(const std::string& base_url, double rate_hz);

    const char* name() const override { return "ipwebcam"; }

protected:
    bool fetch(GpsFix& fix, std::string& note) override;
    // GET /settings/gps_active?set=on. True when the phone acknowledged.
    bool prepare() override;
    const char* hint(bool prepared) const override;
    const char* endpoint() const override { return url_.c_str(); }

private:
    std::string base_;   // http://host:port
    std::string url_;    // full /sensors.json?sense=gps URL
};

// ─── phyphox ─────────────────────────────────────────────────────────────────
// The physics-lab app, and the way out when IP Webcam's sensor buffer stays
// empty: its Location experiment reads the same Android GPS but publishes it
// over a remote-access API with no retention window to expire out of.
//
//   GET /get?lat&lon&z&v&dir
//   {"buffer":{"lat":{"buffer":[-6.9034],...},"lon":{"buffer":[107.6181],...}},
//    "status":{"measuring":true,...}}
//
// On the phone: open the "Location (GPS)" experiment, three-dot menu → Allow
// remote access, then press play. It answers on port 8080, so when it shares a
// phone with IP Webcam one of the two has to move (IP Webcam's port is settable
// under Connection settings → Local broadcasting).
//
// Unlike IP Webcam this reports speed and bearing itself, so they are used as
// given rather than differentiated from successive fixes.
class PhyphoxGps : public PollingGps
{
public:
    PhyphoxGps(const std::string& base_url, double rate_hz);

    const char* name() const override { return "phyphox"; }

protected:
    bool fetch(GpsFix& fix, std::string& note) override;
    const char* hint(bool prepared) const override;
    const char* endpoint() const override { return url_.c_str(); }

private:
    std::string base_;
    std::string url_;
};

// ─── Recorded track ──────────────────────────────────────────────────────────
// One fix per line: "lat,lon", "lat,lon,speed_ms" or "lat,lon,speed_ms,bearing_deg".
// Blank lines and '#' comments are skipped. Advanced one line per read(), which
// is what pairs it with a video clip replayed frame by frame.
class FileGps : public GpsInterface
{
public:
    explicit FileGps(const std::string& path);

    std::optional<GpsFix> read() override;
    bool is_open() const override { return open_; }
    const char* name() const override { return "file"; }

private:
    std::ifstream file_;
    bool          open_ = false;
    std::optional<GpsFix> last_;
};

// ─── Fixed coordinate ────────────────────────────────────────────────────────
// Not a real fix; it parks the ego marker somewhere in the extract so the map,
// the route and the panel can be exercised with no receiver at all.
class FixedGps : public GpsInterface
{
public:
    FixedGps(double lat, double lon);

    std::optional<GpsFix> read() override;
    bool is_open() const override { return true; }
    const char* name() const override { return "fixed"; }

private:
    GpsFix fix_;
};

// Builds the source named by cfg.gps_source. Returns nullptr for GpsSource::None.
std::unique_ptr<GpsInterface> make_gps_source(const Config& cfg);

}  // namespace navigation

#endif  // VISIONPILOT_GPS_SOURCES_HPP
