#ifndef VISIONPILOT_NAVIGATION_HPP
#define VISIONPILOT_NAVIGATION_HPP

#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <navigation/gps_interface.hpp>
#include <navigation/map_renderer.hpp>
#include <navigation/nav_config.hpp>
#include <navigation/osm_map.hpp>
#include <navigation/router.hpp>
#include <navigation/tile_source.hpp>

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
    // route until the new one lands. Replaces the whole mission with this one
    // point — it is "go here", not "go here next".
    void set_destination(double lat, double lon, const std::string& name = "");
    void clear_destination();

    // ── Missions ─────────────────────────────────────────────────────────────
    // A route through a list of points in order, the way a ground station plans
    // one: click, click, click, and the vehicle visits them in that sequence.
    // A single destination is just a mission of length one, so there is one
    // mechanism here and not two.
    //
    // Each leg is routed separately and the legs are stitched into one Route,
    // which means everything downstream — the progress tracker, the turn
    // banner, the corridor that filters the vision waypoints — carries on
    // working without knowing a mission exists.
    // Append a point to the end of the mission and re-route through all of them.
    void add_waypoint(double lat, double lon, const std::string& name = "");
    // Remove the last one. Undo for a misplaced click, which on a map is the
    // most common thing to want.
    void pop_waypoint();
    const std::vector<Waypoint>& mission() const { return mission_; }
    size_t waypoint_count() const { return mission_.size(); }

    // ── Tap the map to go there ──────────────────────────────────────────────
    // A destination the driver can see but cannot name — a junction, a gate, the
    // far corner of a complex — is the normal case, and typing a street name is
    // a poor substitute for pointing at it. `px`,`py` are pixels inside the
    // panel as it was last rendered. Returns false when the point was not on the
    // map (the banner, the footer, or before the first panel was drawn).
    bool set_destination_at_panel_px(double px, double py);
    // The same, in the composed frame's coordinates — for the OpenCV front-end,
    // which has one window and no widget to measure against.
    bool set_destination_at_canvas_px(int x, int y);
    // The same two, appending to the mission instead of replacing it.
    bool add_waypoint_at_panel_px(double px, double py);
    bool add_waypoint_at_canvas_px(int x, int y);
    // The same conversion on its own, for a front-end that wants the coordinate
    // without committing to a route.
    bool panel_px_to_lat_lon(double px, double py, double& lat, double& lon) const;

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
    // Aerial imagery under the map. Turning it on with no tile source built
    // (nav.satellite was off at startup) builds one on the spot, so the toggle
    // works whether or not the run began in satellite mode.
    void   set_satellite(bool s);
    bool   satellite() const { return cfg_.satellite; }
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

    // ── The route, in the frame the vision path lives in ─────────────────────
    // The route's own waypoints converted to ego coordinates: x forward, y left,
    // metres — exactly what LateralFusion projects AutoSteer's waypoints into.
    // That shared frame is the whole point: once the two are in it, the route
    // can say which of the branches leaving a junction the vision waypoints are
    // allowed to describe.
    //
    // Resampled at 1 m, because OSM stores geometry at whatever spacing the
    // mapper drew it with — 5 m round a bend, 80 m down a straight — and a
    // corridor built from those raw nodes would be lumpy in exactly the places
    // it matters.
    //
    // Empty when there is no route, no fix, or no usable heading. An empty
    // result turns the corridor off rather than filtering against a guess.
    std::vector<cv::Point2f> route_ahead_ego(double lookahead_m = 60.0) const;

    // ── The next thing the route asks the car to do ──────────────────────────
    // The router has already found the turns and measured how far away they
    // are; re-deriving that by fitting curves to the geometry throws the answer
    // away and gets a worse one back. A junction is a corner between two OSM
    // ways, and on a road whose nodes are 75 m apart any fixed measuring window
    // either misses it or averages it into the straights either side.
    //
    // `bend_deg` is the heading change across the manoeuvre itself — tangent
    // before against tangent after, +ve left — so a right-angle turn reads as a
    // right angle however far off it is and however sparse the nodes are.
    // Named NextTurn, not Maneuver: navigation::Maneuver is already the enum of
    // turn kinds, and a nested type of that name would shadow it inside every
    // member of this class.
    struct NextTurn {
        bool   valid    = false;
        double dist_m   = 0.0;   // along the route, from the ego
        double bend_deg = 0.0;   // +ve = left
    };
    NextTurn next_maneuver() const;

private:
    // A tap on the panel → a named point. Shared by "go here" and "go here
    // next" so the two cannot disagree about where the finger landed.
    // Name a coordinate after the road under it, for pins that came from a tap
    // and pins that came from the config alike.
    std::string name_at(double lat, double lon) const;
    bool tap_to_point(double px, double py,
                      double& lat, double& lon, std::string& name) const;
    // Composed-frame pixels → panel pixels. The panel is rendered at one size
    // and pasted at another, so a click has to be scaled before it means
    // anything.
    bool canvas_px_to_panel_px(int x, int y, double& px, double& py) const;

    void request_route();
    // Routes every leg and stitches them into one Route. `mission` is copied
    // rather than referenced because it runs on the worker thread and the UI
    // may well have appended another point by the time it finishes.
    void route_worker(double from_lat, double from_lon, double heading_deg,
                      bool have_heading, std::vector<Waypoint> mission);

    Config                        cfg_;
    OsmMap                        map_;
    std::unique_ptr<GpsInterface> gps_;
    std::unique_ptr<TileSource>   tiles_;

    void ensure_tiles();
    // The transform the last panel() used, so a click can be inverted through
    // exactly it rather than through a guess at what it was.
    PanelProjection proj_;

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
    // The mission, in visiting order. Empty means no route. Its last entry is
    // the destination, which is what dest_name_ reports and what the panel
    // labels.
    std::vector<Waypoint> mission_;
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
    // The size the panel was rendered at, which is not always the size it was
    // pasted in at: compose() resizes it to fit a fixed canvas. A click in the
    // window has to be scaled by the ratio before it means anything.
    cv::Size panel_render_size_;
    int      top_inset_ = 0;
};

}  // namespace navigation

#endif  // VISIONPILOT_NAVIGATION_HPP
