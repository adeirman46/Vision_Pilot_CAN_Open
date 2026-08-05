#ifndef VISIONPILOT_NAVIGATION_HPP
#define VISIONPILOT_NAVIGATION_HPP

#include <algorithm>
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

    // ── Pan and zoom ─────────────────────────────────────────────────────────
    // Drag the map with the left button, wheel to zoom, 'r' (or a drag that
    // ends back near the car) to follow again. Panning suspends auto-centring
    // so the map does not fight the hand moving it; it resumes by itself after
    // a few seconds of stillness, because a driver should never be left looking
    // at a map that is not where they are.
    void handle_mouse(int x, int y, bool left_down, int wheel);
    void recenter();
    // Move the map by a pixel delta. The UI owns the pointer, so it converts
    // its own coordinates and just says how far.
    void pan_by(double dx, double dy);
    bool panned() const { return panned_; }
    // True while the left button is held on the map. The app uses this to keep
    // repainting between inference frames, so the map tracks the mouse at
    // pointer rate instead of at the ~25 fps the models run at.
    bool dragging() const { return dragging_; }

    // Where the panel was drawn in the composed frame, so mouse coordinates can
    // be tested against it. Set by compose().
    cv::Rect panel_rect() const { return panel_rect_; }

    cv::Mat panel(int height);
    // Render at an explicit size, for a UI that owns its own layout.
    cv::Mat panel(int width, int height);

    // ── For a real UI front-end ──────────────────────────────────────────────
    const OsmMap& map() const { return map_; }
    // Pixels at the top of the panel the front-end's own widgets occupy, so the
    // manoeuvre banner can start below them instead of underneath them.
    void   set_top_inset(int px) { top_inset_ = px; }
    void   set_dark(bool d) { cfg_.dark = d; }
    bool   dark() const { return cfg_.dark; }
    void   set_heading_up(bool h) { cfg_.heading_up = h; }
    bool   heading_up() const { return cfg_.heading_up; }
    void   zoom_by(double factor)
    { cfg_.zoom_m = std::clamp(cfg_.zoom_m * factor, 50.0, 8000.0); }
    double zoom_m() const { return cfg_.zoom_m; }
    bool   has_route() const;
    std::string destination_name() const { return dest_name_; }
    const RouteProgress& progress() const { return progress_; }
    std::string road_name() const { return road_name_; }

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

    // ── Pan / zoom state ─────────────────────────────────────────────────────
    // Offsets are in panel pixels, not metres: a drag should move the map by
    // exactly the distance the hand moved, whatever the rotation or the zoom.
    double   pan_x_ = 0.0, pan_y_ = 0.0;
    bool     panned_ = false;
    bool     dragging_ = false;
    int      drag_last_x_ = 0, drag_last_y_ = 0;
    uint64_t last_pan_ms_ = 0;
    cv::Rect panel_rect_;
    int      top_inset_ = 0;
};

}  // namespace navigation

#endif  // VISIONPILOT_NAVIGATION_HPP
