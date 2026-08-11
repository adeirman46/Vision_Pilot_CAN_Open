#ifndef VISIONPILOT_GPS_SOURCES_HPP
#define VISIONPILOT_GPS_SOURCES_HPP

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <imu/serial_port.hpp>

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

// ─── Nucleo (DFRobot GNSS-RTK, optionally IMU-fused on the MCU) ──────────────
//
// The position source that is not a phone. A Nucleo-G474RE reads the DFRobot
// GNSS-RTK rover over I2C — and, with the gps_imu_ekf firmware, a Yahboom IMU
// over UART as well — and publishes checksummed lines on the ST-LINK serial
// port. See modules/sensing/gps_rtk_dfrobot/.
//
//   $VPFUS,<uptime_ms>,<lat>,<lon>,<speed_ms>,<heading_deg>,<sigma_m>,
//          <quality>,<imu_hz>,<yaw_bias_dps>*<xx>        20 Hz, filtered
//   $VPRTK,<uptime_ms>,<utc>,<lat>,<lon>,<alt_m>,<geoid_sep_m>,<quality>,
//          <sats>,<hdop>,<accuracy_m>,<corr_age_s>,<base_id>*<xx>   1 Hz, raw
//   $VPSTA,<uptime_ms>,<message>*<xx>                     boot, faults, health
//
// Both position sentences are read, which is what makes one source work with
// every firmware in that project: the fused build sends both and the fused one
// wins; the GPS-only builds send $VPRTK alone and it is used directly. If the
// MCU stops fusing mid-drive — IMU unplugged — this falls back to the raw fix
// on its own, without the panel losing position.
//
// `raw_fix()` always returns the unfiltered receiver, so nav.compare_gps can
// draw the two against each other on a real drive. That comparison is the only
// honest way to judge the fusion: on the road there is no ground truth, so the
// only thing to measure against is the thing it replaced.
//
// The checksum is what makes a serial port safe to open mid-sentence: a line
// that does not verify is dropped rather than parsed into a wrong position.
class NucleoGps : public GpsInterface
{
public:
    // port empty = the first ST-LINK-looking device found (/dev/ttyACM*).
    NucleoGps(const std::string& port, int baud);
    ~NucleoGps() override;

    std::optional<GpsFix> read() override;
    // A board that was unplugged may come back, and the reader thread keeps
    // trying — so this stays open the way a sleeping phone does.
    bool                  is_open() const override { return true; }
    const char*           name() const override { return "nucleo"; }
    std::optional<GpsFix> raw_fix() const override;

    // For the panel: whether the MCU is fusing, and what it last said.
    bool        fused() const { return fused_active_; }
    std::string status() const;

private:
    void run();
    bool open_port(std::string& error);
    void consume(const std::string& line);
    bool parse_fused(const std::string& body);
    bool parse_raw(const std::string& body);

    std::string           want_port_;
    int                   baud_;
    imu::SerialPort       port_;
    std::string           partial_;      // line still arriving

    mutable std::mutex    mtx_;
    std::optional<GpsFix> fused_;
    std::optional<GpsFix> raw_;
    std::optional<GpsFix> prev_raw_;     // for deriving speed and course
    std::string           status_;
    uint64_t              last_fused_ms_ = 0;

    std::atomic<bool>     fused_active_{false};
    std::atomic<bool>     running_{false};
    std::thread           thread_;
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
// The configured receiver on its own, without the IMU filter wrapped around it.
std::unique_ptr<GpsInterface> make_raw_gps_source(const Config& cfg);

}  // namespace navigation

#endif  // VISIONPILOT_GPS_SOURCES_HPP
