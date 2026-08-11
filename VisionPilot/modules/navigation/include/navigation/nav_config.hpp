#ifndef VISIONPILOT_NAV_CONFIG_HPP
#define VISIONPILOT_NAV_CONFIG_HPP

#include <string>

// Kept free of OpenCV and of the rest of the navigation module so the config
// module can parse vision_pilot.conf into it without pulling in the renderer.
namespace navigation {

enum class GpsSource {
    None = 0,   // no position — panel shows "no fix"
    IpWebcam,   // phone running IP Webcam: /sensors.json?sense=gps
    Phyphox,    // phone running phyphox's Location experiment: /get?lat&lon
    File,       // recorded track, one "lat,lon[,speed_ms[,bearing_deg]]" per line
    Fixed,      // one static coordinate (exercises the map with no receiver)
    Nucleo,     // Nucleo-G474RE on USB: $VPRTK / $VPFUS lines from the DFRobot
                // GNSS-RTK rover, already GPS/IMU fused on the MCU
};

struct Config {
    bool        enabled   = false;
    std::string map_file  = "../maps/bandung-roads.vpmap";

    // ── Position ─────────────────────────────────────────────────────────────
    GpsSource   gps_source = GpsSource::IpWebcam;
    // Base URL of the phone, i.e. the stream URL without /video. The GPS feed
    // and the camera frames come from the same device.
    std::string gps_url    = "http://10.142.152.101:8080";
    std::string gps_file;
    double      fixed_lat  = -6.9175;    // Bandung city centre
    double      fixed_lon  = 107.6191;
    double      gps_rate_hz = 2.0;

    // ── Nucleo receiver (gps_source = nucleo) ────────────────────────────────
    // The ST-LINK virtual COM port the board publishes on. Empty picks the
    // first /dev/ttyACM* found, which is the right answer whenever there is
    // only one Nucleo plugged in.
    std::string gps_port;
    int         gps_baud = 115200;

    // ── IMU ──────────────────────────────────────────────────────────────────
    // With an IMU attached the position source becomes a GPS/IMU filter instead
    // of the raw receiver: fixes arrive at IMU rate, move smoothly between them,
    // and keep a heading while stopped. Everything downstream is unchanged.
    bool        imu_enabled = false;
    std::string imu_port;              // empty = first USB serial port found
    int         imu_baud    = 0;       // 0 = auto-detect
    // Draw the unfiltered receiver alongside the fused position, so the two can
    // be judged against each other on a real drive.
    bool        compare_gps = false;

    // ── Destination (empty → free-drive map, no route) ───────────────────────
    bool        has_destination = false;
    double      dest_lat = 0.0;
    double      dest_lon = 0.0;
    std::string dest_name;

    // ── Panel ────────────────────────────────────────────────────────────────
    int         panel_width = 512;    // px, appended to the right of the HUD
    double      zoom_m      = 400.0;  // metres covered across the panel width
    bool        heading_up  = true;   // false → north-up
    bool        dark        = true;   // false → light / daytime map. 't' toggles.

    // Compose onto a canvas of exactly this size instead of just widening the
    // frame. The camera view plus a map panel is about 3:1, and a window that
    // shape gets letterboxed by the toolkit — in white — on any normal screen.
    // Producing a 16:9 canvas ourselves means the padding is ours, and black.
    // 0 x 0 keeps the plain side-by-side frame.
    int         out_width   = 1920;
    int         out_height  = 1080;

    // Set when the config could be read but the position source it asks for
    // cannot work here (no GPS host, missing track). Navigation is an accessory:
    // it degrades to a panel that says why, it never stops the drive.
    std::string degraded_reason;
};

// "ipwebcam" | "phyphox" | "nucleo" | "file" | "fixed" | "none" (also accepts
// "phone" for ipwebcam and "rtk"/"stm32" for nucleo).
// Throws std::runtime_error on anything else.
GpsSource parse_gps_source(const std::string& value);

}  // namespace navigation

#endif  // VISIONPILOT_NAV_CONFIG_HPP
