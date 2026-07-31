#ifndef VISIONPILOT_NAVIGATION_HPP
#define VISIONPILOT_NAVIGATION_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>

#include <navigation/gps_interface.hpp>
#include <navigation/map_renderer.hpp>
#include <navigation/nav_config.hpp>
#include <navigation/osm_map.hpp>
#include <navigation/router.hpp>

namespace navigation {

// Owns the position source, the offline map and the active route, and turns the
// three into the panel that sits to the right of the inference view.
//
//   Navigator nav(cfg);
//   ...
//   nav.update();                            // once per frame — cheap
//   display = nav.compose(display_frame);    // HUD | map
//
// Routing runs on a worker thread: a Dijkstra over greater Bandung takes tens of
// milliseconds and must not land in the inference loop.
class Navigator
{
public:
    explicit Navigator(const Config& cfg);
    ~Navigator();

    Navigator(const Navigator&) = delete;
    Navigator& operator=(const Navigator&) = delete;

    // True when the map loaded and a position source exists. A Navigator that is
    // not ready still renders a panel — it just says why.
    bool ready() const { return ready_; }
    const std::string& error() const { return error_; }

    // Pull the newest fix, re-match to the road network, follow the route.
    void update();

    // Retarget. Recomputes on the worker thread; the panel keeps drawing the old
    // route until the new one lands.
    void set_destination(double lat, double lon, const std::string& name = "");
    void clear_destination();

    // ── Destination box ──────────────────────────────────────────────────────
    // Typed straight into the video window: the app has no other UI, and a
    // driver cannot go back to a terminal to retarget. Feed it the key that
    // render_frame() reported. Returns true when the key was consumed, so the
    // caller knows not to treat it as a global shortcut.
    //
    //   d or /        open the box
    //   text          search places and roads as you type
    //   up/down       move through the matches
    //   Enter         go to the highlighted one (or lat,lon typed literally)
    //   Esc           close, keeping the current route
    //   c             (box closed) clear the route
    bool handle_key(int key);
    bool destination_input_active() const { return input_active_; }

    cv::Mat panel(int height);

    // frame | panel, side by side. Returns frame untouched when disabled.
    cv::Mat compose(const cv::Mat& frame);

    // Latest fix, for callers that want the position for something else.
    std::optional<GpsFix> fix() const;

    // Speed limit of the road currently under the ego, km/h. 0 when unknown.
    int speed_limit_kph() const;

private:
    void request_route();
    void route_worker(double from_lat, double from_lon, double heading_deg,
                      bool have_heading, double to_lat, double to_lon,
                      std::string dest_name);

    Config                        cfg_;
    OsmMap                        map_;
    std::unique_ptr<GpsInterface> gps_;

    bool        ready_ = false;
    std::string error_;

    // ── State touched by update() only ───────────────────────────────────────
    std::optional<GpsFix> fix_;
    MapMatch              match_;
    int                   speed_limit_kph_ = 0;
    bool                  speed_inferred_  = false;
    std::string           road_name_;
    uint8_t               matched_class_ = kUnknown;
    RouteProgress         progress_;
    int                   off_route_count_ = 0;
    uint64_t              last_route_ms_   = 0;

    // ── Shared with the routing worker ───────────────────────────────────────
    mutable std::mutex route_mtx_;
    Route              route_;
    std::atomic<bool>  routing_{false};
    std::thread        worker_;
    std::atomic<bool>  shutdown_{false};

    bool        want_route_ = false;
    double      dest_lat_ = 0.0, dest_lon_ = 0.0;
    std::string dest_name_;

    // ── Destination box state ────────────────────────────────────────────────
    bool                        input_active_ = false;
    std::string                 input_text_;
    std::vector<OsmMap::Place>  input_hits_;
    int                         input_sel_ = 0;
    std::string                 input_note_;   // "no match", "go", ...

    void refresh_input_hits();
};

}  // namespace navigation

#endif  // VISIONPILOT_NAVIGATION_HPP
