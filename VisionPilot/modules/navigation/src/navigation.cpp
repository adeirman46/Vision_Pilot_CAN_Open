#include <navigation/navigation.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include <logging/logger.hpp>
#include <navigation/gps_sources.hpp>

namespace navigation {

namespace {

uint64_t now_ms()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

// Don't thrash the router: a rerouting storm at a junction is worse than being
// briefly off route.
constexpr uint64_t kRerouteCooldownMs = 8000;
// A panned map stops following the car. Snap back on its own so nobody is left
// staring at the wrong part of town mid-drive.
constexpr uint64_t kPanIdleResetMs   = 12000;
constexpr int      kOffRouteFixes     = 4;      // consecutive fixes off the line

}  // namespace

Navigator::Navigator(const Config& cfg) : cfg_(cfg)
{
    if (!cfg_.enabled) return;

    if (!map_.load(cfg_.map_file)) {
        error_ = map_.error();
        VP_ERROR("[Nav] %s", error_.c_str());
    }

    // --nav on the command line turns navigation on after the config was read,
    // so the config's own validation never ran. Re-check what actually matters
    // here: a network source with no URL polls nothing, for ever.
    if ((cfg_.gps_source == GpsSource::Phyphox ||
         cfg_.gps_source == GpsSource::IpWebcam) && cfg_.gps_url.empty()) {
        cfg_.degraded_reason = "no nav.gps_url for the chosen GPS source";
        cfg_.gps_source = GpsSource::None;
    }

    gps_ = make_gps_source(cfg_);
    if (!gps_ && error_.empty())
        error_ = cfg_.degraded_reason.empty() ? "nav.gps_source = none"
                                              : cfg_.degraded_reason;

    ready_ = map_.loaded() && gps_ != nullptr;

    if (cfg_.satellite) ensure_tiles();

    // Via points first, destination last: "--via A --via B --dest C" reads as
    // A, B, C and is planned as A, B, C.
    for (const auto& [wlat, wlon] : cfg_.waypoints)
        mission_.push_back({wlat, wlon, name_at(wlat, wlon)});
    if (cfg_.has_destination)
        mission_.push_back({cfg_.dest_lat, cfg_.dest_lon,
                            cfg_.dest_name.empty() ? name_at(cfg_.dest_lat, cfg_.dest_lon)
                                                   : cfg_.dest_name});

    if (!mission_.empty()) {
        dest_name_  = mission_.back().name;
        want_route_ = true;
        if (mission_.size() > 1)
            VP_INFO("[Nav] mission of %zu waypoints from the config",
                    mission_.size());
    } else if (map_.loaded()) {
        // Started with nowhere to go: open the destination box straight away.
        // A head unit asks "where to?" when you get in — it does not wait to be
        // found. Esc dismisses it and leaves the plain moving map.
        input_active_ = true;
        refresh_input_hits();
    }
}

Navigator::~Navigator()
{
    shutdown_ = true;
    if (worker_.joinable()) worker_.join();
    // The GPS poll thread lives inside the source and stops in its destructor.
    gps_.reset();
}

void Navigator::ensure_tiles()
{
    if (tiles_) return;
    TileSource::Options opt;
    opt.url         = cfg_.tile_url;
    opt.cache_dir   = cfg_.tile_cache_dir;
    opt.attribution = cfg_.tile_attribution;
    opt.max_zoom    = cfg_.tile_max_zoom;
    opt.offline     = cfg_.tile_offline;
    tiles_ = std::make_unique<TileSource>(std::move(opt));
}

void Navigator::set_satellite(bool s)
{
    cfg_.satellite = s;
    if (s) ensure_tiles();
}

bool Navigator::panel_px_to_lat_lon(double px, double py, double& lat, double& lon) const
{
    return navigation::panel_px_to_lat_lon(proj_, px, py, lat, lon);
}

// Name a point after the road it landed on. The router snaps the point to the
// network itself, so the coordinate stays exactly where it was put — the pin
// marks the place that was pointed at, not the nearest thing the map happens to
// know a name for. A coordinate is the fallback, because an unnamed pin on a
// banner is worse than a pair of numbers.
std::string Navigator::name_at(double lat, double lon) const
{
    std::string name;
    if (map_.loaded()) {
        const MapMatch m = map_.match(lat, lon, 250.0);
        if (m.valid) name = map_.way_name(map_.ways()[m.way]);
    }
    if (name.empty()) {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "%.5f, %.5f", lat, lon);
        name = buf;
    }
    return name;
}

// A tap on the panel, turned into a named point. Shared by "go here" and "go
// here next" so the two can never disagree about where the finger landed.
bool Navigator::tap_to_point(double px, double py,
                             double& lat, double& lon, std::string& name) const
{
    if (!panel_px_to_lat_lon(px, py, lat, lon)) return false;
    if (!map_.loaded()) return false;
    name = name_at(lat, lon);
    return true;
}

// Panel pixels → composed-frame pixels. The panel is rendered at one size and
// pasted at another, so a click in the window has to be scaled before it means
// anything.
bool Navigator::canvas_px_to_panel_px(int x, int y, double& px, double& py) const
{
    if (panel_rect_.width <= 0 || panel_rect_.height <= 0) return false;
    if (!panel_rect_.contains(cv::Point(x, y))) return false;
    if (panel_render_size_.width <= 0 || panel_render_size_.height <= 0) return false;

    px = (x - panel_rect_.x)
       * (static_cast<double>(panel_render_size_.width) / panel_rect_.width);
    py = (y - panel_rect_.y)
       * (static_cast<double>(panel_render_size_.height) / panel_rect_.height);
    return true;
}

bool Navigator::set_destination_at_panel_px(double px, double py)
{
    double lat = 0.0, lon = 0.0;
    std::string name;
    if (!tap_to_point(px, py, lat, lon, name)) return false;

    VP_INFO("[Nav] destination tapped on the map: %.6f, %.6f (%s)", lat, lon,
            name.c_str());
    set_destination(lat, lon, name);
    return true;
}

bool Navigator::add_waypoint_at_panel_px(double px, double py)
{
    double lat = 0.0, lon = 0.0;
    std::string name;
    if (!tap_to_point(px, py, lat, lon, name)) return false;

    VP_INFO("[Nav] waypoint %zu tapped on the map: %.6f, %.6f (%s)",
            mission_.size() + 1, lat, lon, name.c_str());
    add_waypoint(lat, lon, name);
    return true;
}

bool Navigator::set_destination_at_canvas_px(int x, int y)
{
    double px = 0.0, py = 0.0;
    if (!canvas_px_to_panel_px(x, y, px, py)) return false;
    return set_destination_at_panel_px(px, py);
}

bool Navigator::add_waypoint_at_canvas_px(int x, int y)
{
    double px = 0.0, py = 0.0;
    if (!canvas_px_to_panel_px(x, y, px, py)) return false;
    return add_waypoint_at_panel_px(px, py);
}

void Navigator::set_destination(double lat, double lon, const std::string& name)
{
    // "Go here", not "go here next": a destination replaces the mission rather
    // than extending it. Appending is add_waypoint().
    mission_    = {{lat, lon, name}};
    dest_name_  = name;
    want_route_ = true;
    last_route_ms_ = 0;   // route now, do not wait out the cooldown
    request_route();
}

void Navigator::add_waypoint(double lat, double lon, const std::string& name)
{
    mission_.push_back({lat, lon, name});
    dest_name_  = name;              // the last point is the destination
    want_route_ = true;
    last_route_ms_ = 0;
    request_route();
}

void Navigator::pop_waypoint()
{
    if (mission_.empty()) return;
    mission_.pop_back();
    if (mission_.empty()) {
        clear_destination();
        return;
    }
    dest_name_ = mission_.back().name;
    last_route_ms_ = 0;
    request_route();
}

void Navigator::clear_destination()
{
    want_route_ = false;
    mission_.clear();
    dest_name_.clear();
    std::lock_guard<std::mutex> lock(route_mtx_);
    route_.clear();
}

void Navigator::request_route()
{
    if (!ready_ || !want_route_ || !fix_) return;
    if (routing_) return;

    const uint64_t t = now_ms();
    if (last_route_ms_ != 0 && t - last_route_ms_ < kRerouteCooldownMs) return;
    last_route_ms_ = t;

    if (worker_.joinable()) worker_.join();
    routing_ = true;

    const GpsFix f = *fix_;
    worker_ = std::thread(&Navigator::route_worker, this,
                          f.latitude, f.longitude, f.bearing_deg, f.has_bearing,
                          mission_);
}

// Route every leg of the mission and stitch them into one Route.
//
// Stitching rather than teaching the router about via-points is deliberate:
// everything downstream — the progress tracker, the manoeuvre banner, the
// corridor that filters the vision waypoints — reads a Route and none of it has
// to learn that a mission exists. A single destination is a one-leg mission, so
// there is one path through this code and not two.
void Navigator::route_worker(double from_lat, double from_lon, double heading,
                             bool have_heading, std::vector<Waypoint> mission)
{
    Route out;

    double lat = from_lat, lon = from_lon;
    double leg_heading      = heading;
    bool   have_leg_heading = have_heading;

    for (size_t i = 0; i < mission.size() && !shutdown_; ++i) {
        const Route leg = compute_route(map_, lat, lon, mission[i].lat, mission[i].lon,
                                        have_leg_heading ? &leg_heading : nullptr);
        // One unroutable leg makes the whole mission unroutable. Half a mission
        // is worse than none: it would draw a line to somewhere the driver did
        // not ask to go and call it the route.
        if (!leg.valid || leg.nodes.size() < 2) { out.clear(); break; }

        // The first node of every leg after the first is the last node of the
        // one before it. Appending it again would put a zero-length segment in
        // the geometry and a duplicate distance in the cumulative table.
        const size_t skip   = out.nodes.empty() ? 0 : 1;
        const double offset = out.length_m;

        for (size_t j = skip; j < leg.nodes.size(); ++j) {
            out.nodes.push_back(leg.nodes[j]);
            out.cumulative_m.push_back(offset + leg.cumulative_m[j]);
        }
        for (const Step& st : leg.steps) {
            // Drop the arrival at an intermediate waypoint and the departure
            // from it: mid-mission they are not manoeuvres, and "Arrive"
            // announced four waypoints early is worse than saying nothing.
            const bool intermediate = (i + 1 < mission.size());
            if (intermediate && st.type == Maneuver::Arrive) continue;
            if (i > 0 && st.type == Maneuver::Depart) continue;
            Step s2 = st;
            s2.distance_from_start_m += offset;
            out.steps.push_back(s2);
        }
        out.length_m   += leg.length_m;
        out.duration_s += leg.duration_s;

        // Only the first leg gets a heading. It is tempting to carry the arrival
        // heading into the next leg to stop it turning round on the spot, but
        // compute_route's heading does not constrain travel at all — it only
        // biases which way the start point snaps to. Feeding it an arrival
        // heading therefore cannot prevent a U-turn, and it can snap a waypoint
        // onto a parallel road that happens to point the right way, which sends
        // the leg kilometres out of its way to somewhere the driver never
        // pointed at.
        have_leg_heading = false;
        lat = mission[i].lat;
        lon = mission[i].lon;
        out.valid = true;
    }

    if (out.valid && !mission.empty()) {
        out.dest_lat  = mission.back().lat;
        out.dest_lon  = mission.back().lon;
        out.dest_name = mission.back().name;
        VP_INFO("[Nav] mission: %zu waypoint%s, %.1f km, %.0f min, %zu steps",
                mission.size(), mission.size() == 1 ? "" : "s",
                out.length_m / 1000.0, out.duration_s / 60.0, out.steps.size());
    }

    if (!shutdown_) {
        std::lock_guard<std::mutex> lock(route_mtx_);
        // Keep the previous route when the new one failed — a momentary bad fix
        // should not wipe the guidance off the panel.
        if (out.valid || !route_.valid) route_ = std::move(out);
    }
    routing_ = false;
}

// ── The route, in the frame the vision path lives in ────────────────────────
// One conversion, and it is the only thing standing between "the map knows the
// turn is left" and "the path fit knows it too".
//
// The ego-frame conversion itself is en_to_ego(), in osm_map.cpp, so the sign
// convention has one home and one test.
//
// ── Where the heading comes from ────────────────────────────────────────────
// The receiver, when it reports one. phyphox stops reporting a bearing below
// 1.5 m/s and derive_motion() then holds the last real one, so a car crawling
// through a junction — which is the only place any of this matters — keeps the
// heading it had a second ago.
//
// Before the car has ever moved there is no such heading to hold, and the
// route's own first leg is used instead: the direction from here to the point
// 25 m along it. That is an assumption — that the car is pointing the way the
// route goes — and it is stated here rather than hidden, because it is only
// true of a car that is on its route and facing forwards. It is true whenever
// this feature is doing anything, and when it is false the car is parked.
//
// What is deliberately NOT used is the direction of the matched way. A way's
// node order is whichever way the mapper happened to draw it and carries no
// sense of travel at all, so half the time it points backwards — and unlike
// having no heading, a backwards one is confidently wrong: it puts the entire
// route behind the car, where the corridor rejects every waypoint there is.
std::vector<cv::Point2f> Navigator::route_ahead_ego(double lookahead_m) const
{
    std::vector<cv::Point2f> out;
    if (!cfg_.enabled || !fix_ || !map_.loaded() || lookahead_m <= 1.0) return out;

    const double lat0 = fix_->latitude;
    const double lon0 = fix_->longitude;

    std::lock_guard<std::mutex> lock(route_mtx_);
    if (!route_.valid || route_.nodes.size() < 2) return out;

    const size_t start =
        std::min<size_t>(progress_.nearest_index, route_.nodes.size() - 1);

    // ── Heading ──────────────────────────────────────────────────────────────
    double      heading_deg = 0.0;
    const char* heading_src = "receiver";
    if (fix_->has_bearing) {
        heading_deg = fix_->bearing_deg;
    } else {
        // Far enough along that a metre of GPS noise does not swing it, near
        // enough that it is still this road and not the turn after next.
        constexpr double kHeadingBaseM = 25.0;
        size_t j = start;
        double along = 0.0;
        for (size_t i = start; i + 1 < route_.nodes.size() && along < kHeadingBaseM; ++i) {
            along += map_.node_distance_m(route_.nodes[i], route_.nodes[i + 1]);
            j = i + 1;
        }
        if (j == start) return out;   // route ends here; nothing to aim at
        const Node& n = map_.nodes()[route_.nodes[j]];
        heading_deg = bearing_deg(lat0, lon0, n.lat(), n.lon());
        heading_src = "route (never moved)";
    }

    {
        static uint64_t last_log = 0;
        const uint64_t  t = now_ms();
        if (t - last_log > 3000) {
            last_log = t;
            VP_INFO("[Nav] route frame: heading %.0f deg (%s), speed %.1f m/s",
                    heading_deg, heading_src, fix_->speed_ms);
        }
    }

    // Walk the route forward from the node the ego last projected onto, and
    // convert as we go. Rotating first and resampling second matters: the
    // spacing that has to be even is the one the waypoints get compared at.
    std::vector<cv::Point2f> raw;
    raw.reserve(route_.nodes.size() - start);
    double walked = 0.0;
    for (size_t i = start; i < route_.nodes.size(); ++i) {
        const Node& n = map_.nodes()[route_.nodes[i]];
        double east = 0.0, north = 0.0, fwd = 0.0, left = 0.0;
        lat_lon_to_en(n.lat(), n.lon(), lat0, lon0, east, north);
        en_to_ego(east, north, heading_deg, fwd, left);
        raw.emplace_back(static_cast<float>(fwd), static_cast<float>(left));
        if (raw.size() >= 2) {
            const cv::Point2f& prev = raw[raw.size() - 2];
            const cv::Point2f& cur  = raw.back();
            walked += std::hypot(cur.x - prev.x, cur.y - prev.y);
            // Overshoot the lookahead: the ego sits partway along the first
            // segment, so stopping exactly at it would come up short.
            if (walked > lookahead_m + 20.0) break;
        }
    }
    if (raw.size() < 2) return out;

    // Resample at a fixed 1 m of ARC LENGTH, not of forward distance.
    //
    // This matters at exactly the place the whole feature exists for. Past a
    // right-angle turn the route stops going forwards: x plateaus and then
    // falls while y runs away sideways. Sampling or clipping by x there thins
    // the corner out, and past 90 degrees drops it entirely — so the one part
    // of the route that is not straight would be the part least well described.
    // Arc length is monotonic whatever the road does.
    constexpr double kStepM = 1.0;
    double next_s = 0.0, acc = 0.0;
    bool   started = false;
    for (size_t i = 0; i + 1 < raw.size() && next_s <= lookahead_m; ++i) {
        const cv::Point2f a = raw[i], b = raw[i + 1];
        const double seg = std::hypot(b.x - a.x, b.y - a.y);
        if (seg < 1e-6) continue;
        while (next_s <= acc + seg && next_s <= lookahead_m) {
            const double t = (next_s - acc) / seg;
            const cv::Point2f p(static_cast<float>(a.x + (b.x - a.x) * t),
                                static_cast<float>(a.y + (b.y - a.y) * t));
            next_s += kStepM;
            // Skip the tail of the route that is still behind the car — the ego
            // sits partway along its first segment — but only before the first
            // accepted point. Dropping points from the middle would put a hole
            // in the arc length the sampling above just made uniform.
            if (!started) {
                if (p.x < -2.0f) continue;
                started = true;
            }
            out.push_back(p);
        }
        acc += seg;
    }
    return out;
}

// ── The next thing the route asks the car to do ─────────────────────────────
// Taken from the router's own step list rather than measured off the geometry.
// The router already decided where the turns are; the only thing left to work
// out is how sharp this one is, and that is a heading change across the step's
// node — which is exact whatever the node spacing either side of it happens to
// be.
Navigator::NextTurn Navigator::next_maneuver() const
{
    NextTurn m;
    std::lock_guard<std::mutex> lock(route_mtx_);
    if (!route_.valid || progress_.next_step < 0) return m;
    if (static_cast<size_t>(progress_.next_step) >= route_.steps.size()) return m;

    const Step& st = route_.steps[progress_.next_step];
    // Arriving is not a manoeuvre to steer through, and departing has already
    // happened by definition.
    if (st.type == Maneuver::Arrive || st.type == Maneuver::Depart) return m;

    // Where the step sits in the node list, so the roads either side of it can
    // be measured. cumulative_m is parallel to nodes and monotonic, so this is
    // a lookup rather than a search over geometry.
    size_t at = 0;
    double best = 1e18;
    for (size_t i = 0; i < route_.nodes.size(); ++i) {
        const double d = std::abs(route_.cumulative_m[i] - st.distance_from_start_m);
        if (d < best) { best = d; at = i; }
    }
    if (at == 0 || at + 1 >= route_.nodes.size()) return m;

    const Node& before = map_.nodes()[route_.nodes[at - 1]];
    const Node& corner = map_.nodes()[route_.nodes[at]];
    const Node& after  = map_.nodes()[route_.nodes[at + 1]];

    const double in_deg  = bearing_deg(before.lat(), before.lon(), corner.lat(), corner.lon());
    const double out_deg = bearing_deg(corner.lat(), corner.lon(), after.lat(),  after.lon());
    // Shortest way round, and positive to the left: bearings grow clockwise,
    // the ego frame's y grows to the left, so the sign flips here once and
    // nowhere else.
    double turn = std::fmod(out_deg - in_deg + 540.0, 360.0) - 180.0;

    m.valid    = true;
    m.bend_deg = -turn;
    m.dist_m   = std::max(0.0, progress_.distance_to_step_m);
    return m;
}

void Navigator::update()
{
    if (!cfg_.enabled || !gps_) return;

    auto fix = gps_->read();
    if (fix) fix_ = fix;
    if (!fix_ || !map_.loaded()) return;

    // Match no tighter than the fix is accurate. A phone indoors reports a
    // network fix good to ~100 m; insisting on 60 m there matches nothing at
    // all, which reads as "no road" when the truth is "not sure which road".
    const double heading = fix_->bearing_deg;
    const double reach   = std::clamp(fix_->accuracy_m, 60.0, 200.0);
    match_ = map_.match(fix_->latitude, fix_->longitude, reach,
                        fix_->has_bearing ? &heading : nullptr);

    if (match_.valid) {
        const Way& w = map_.ways()[match_.way];
        road_name_       = map_.way_name(w);
        matched_class_   = w.road_class;
        speed_limit_kph_ = w.speed_kph;
        speed_inferred_  = (w.flags & kSpeedInferred) != 0;
    } else {
        road_name_.clear();
        matched_class_   = kUnknown;
        speed_limit_kph_ = 0;
        speed_inferred_  = false;
    }

    if (!want_route_) return;

    {
        std::lock_guard<std::mutex> lock(route_mtx_);
        if (route_.valid)
            progress_ = track_route(map_, route_, fix_->latitude, fix_->longitude,
                                    fix_->speed_ms);
    }

    // First route, or a reroute once we have genuinely left the line.
    bool need_route = false;
    {
        std::lock_guard<std::mutex> lock(route_mtx_);
        if (!route_.valid) {
            need_route = true;
        } else if (!progress_.on_route) {
            need_route = (++off_route_count_ >= kOffRouteFixes);
        } else {
            off_route_count_ = 0;
        }
    }
    if (need_route) {
        off_route_count_ = 0;
        request_route();
    }
}

// ─── Destination box ─────────────────────────────────────────────────────────

void Navigator::refresh_input_hits()
{
    input_hits_.clear();
    input_sel_ = 0;
    input_note_.clear();

    // Below three characters every second street in Bandung matches, which is
    // not a list worth drawing.
    if (input_text_.size() < 3) {
        input_note_ = "keep typing...";
        return;
    }
    const double lat = fix_ ? fix_->latitude  : 0.0;
    const double lon = fix_ ? fix_->longitude : 0.0;
    input_hits_ = map_.search(input_text_, 6, fix_ ? &lat : nullptr,
                              fix_ ? &lon : nullptr);
    if (input_hits_.empty()) input_note_ = "no match in this map";
}

bool Navigator::handle_key(int key)
{
    if (!cfg_.enabled || key < 0) return false;

    // OpenCV returns the raw X11 keysym in the high bits on some builds; the
    // low byte is the character.
    const int k = key & 0xFF;

    if (!input_active_) {
        if (k == 'd' || k == 'D' || k == '/') {
            if (!map_.loaded()) return false;
            input_active_ = true;
            input_text_.clear();
            refresh_input_hits();
            return true;
        }
        if (k == 'c' || k == 'C') {          // drop the route, keep driving
            clear_destination();
            return true;
        }
        if (k == 't' || k == 'T') {          // light <-> dark map
            cfg_.dark = !cfg_.dark;
            VP_INFO("[Nav] map theme: %s", cfg_.dark ? "dark" : "light");
            return true;
        }
        if (k == '+' || k == '=') {
            cfg_.zoom_m = std::clamp(cfg_.zoom_m * 0.8, 50.0, 8000.0);
            return true;
        }
        if (k == '-' || k == '_') {
            cfg_.zoom_m = std::clamp(cfg_.zoom_m * 1.25, 50.0, 8000.0);
            return true;
        }
        return false;
    }

    switch (k) {
    case 27:                                  // Esc — leave the route alone
        input_active_ = false;
        return true;

    case 8: case 127:                         // Backspace
        if (!input_text_.empty()) {
            input_text_.pop_back();
            refresh_input_hits();
        }
        return true;

    case 13: case 10: {                       // Enter — go
        double lat = 0.0, lon = 0.0;
        // Coordinates typed literally still work, so a place the extract has
        // never heard of is still reachable.
        const auto comma = input_text_.find(',');
        if (comma != std::string::npos &&
            input_text_.find_first_not_of("0123456789.,+- ") == std::string::npos) {
            lat = std::atof(input_text_.substr(0, comma).c_str());
            lon = std::atof(input_text_.substr(comma + 1).c_str());
            if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
                input_active_ = false;
                set_destination(lat, lon, "");
                return true;
            }
        }
        if (!input_hits_.empty() &&
            input_sel_ < static_cast<int>(input_hits_.size())) {
            const auto& p = input_hits_[input_sel_];
            input_active_ = false;
            set_destination(p.lat, p.lon, p.name);
        }
        return true;
    }

    // Arrow keys arrive as the low byte of the platform keycode; 'j'/'k' work
    // everywhere and cost nothing to support.
    case 82: case 'k':                        // up
        if (input_sel_ > 0) --input_sel_;
        return true;
    case 84: case 'j':                        // down
        if (input_sel_ + 1 < static_cast<int>(input_hits_.size())) ++input_sel_;
        return true;

    default:
        if (k >= 32 && k < 127 && input_text_.size() < 48) {
            input_text_.push_back(static_cast<char>(k));
            refresh_input_hits();
        }
        return true;
    }
}

// ─── Pan and zoom ────────────────────────────────────────────────────────────

void Navigator::recenter()
{
    pan_x_ = pan_y_ = 0.0;
    panned_ = false;
}

void Navigator::pan_by(double dx, double dy)
{
    pan_x_ += dx;
    pan_y_ += dy;
    panned_ = std::hypot(pan_x_, pan_y_) > 12.0;
    last_pan_ms_ = now_ms();
}

void Navigator::handle_mouse(int x, int y, bool left_down, int wheel)
{
    if (!cfg_.enabled || !map_.loaded()) return;

    const bool in_panel = panel_rect_.width > 0 && panel_rect_.contains(cv::Point(x, y));

    if (wheel != 0 && in_panel) {
        // Multiplicative, so each notch feels the same at every scale. Clamped
        // to what the extract can usefully show: below ~50 m the map is one
        // road, above ~8 km it is a grey smear.
        const double factor = wheel > 0 ? 0.8 : 1.25;
        cfg_.zoom_m = std::clamp(cfg_.zoom_m * factor, 50.0, 8000.0);
        last_pan_ms_ = now_ms();
    }

    if (left_down && !dragging_ && in_panel && !input_active_) {
        dragging_ = true;
        drag_last_x_ = x;
        drag_last_y_ = y;
    } else if (!left_down) {
        dragging_ = false;
    }

    if (dragging_) {
        pan_x_ += x - drag_last_x_;
        pan_y_ += y - drag_last_y_;
        drag_last_x_ = x;
        drag_last_y_ = y;
        // A drag that ends up back on the car is the natural "follow me again".
        panned_ = std::hypot(pan_x_, pan_y_) > 12.0;
        last_pan_ms_ = now_ms();
    }

    if (panned_ && !dragging_ && last_pan_ms_ != 0 &&
        now_ms() - last_pan_ms_ > kPanIdleResetMs)
        recenter();
}

std::optional<GpsFix> Navigator::fix() const { return fix_; }

int Navigator::speed_limit_kph() const { return speed_limit_kph_; }

bool Navigator::has_route() const
{
    std::lock_guard<std::mutex> lock(route_mtx_);
    return route_.valid;
}

cv::Mat Navigator::panel(int width, int height)
{
    const int saved = cfg_.panel_width;
    cfg_.panel_width = width;
    cv::Mat out = panel(height);
    cfg_.panel_width = saved;
    return out;
}

cv::Mat Navigator::panel(int height)
{
    MapView view;
    if (cfg_.compare_gps && gps_) view.raw_fix = gps_->raw_fix();
    view.top_inset = top_inset_;
    view.fix             = fix_;
    view.map_loaded      = map_.loaded();
    view.status          = error_.empty()
                               ? (gps_ ? std::string("source: ") + gps_->name() : std::string())
                               : error_;
    view.source_name     = gps_ ? gps_->name() : "none";
    view.match           = match_;
    view.road_name       = road_name_;
    view.matched_class   = matched_class_;
    view.speed_limit_kph = speed_limit_kph_;
    view.speed_inferred  = speed_inferred_;
    view.progress        = progress_;
    view.waypoints       = mission_.empty() ? nullptr : &mission_;
    view.routing         = routing_;
    view.zoom_m          = cfg_.zoom_m;
    view.heading_up      = cfg_.heading_up;
    view.dark            = cfg_.dark;
    view.input_active    = input_active_;
    view.input_text      = input_text_;
    view.input_hits      = input_hits_;
    view.input_sel       = input_sel_;
    view.input_note      = input_note_;
    view.pan_x           = pan_x_;
    view.pan_y           = pan_y_;
    view.panned          = panned_;
    view.satellite       = cfg_.satellite;
    view.tiles           = tiles_.get();
    view.sat_roads       = cfg_.sat_roads;
    view.sat_road_opacity = cfg_.sat_road_opacity;

    // Hold the lock across the render: the worker may swap the route in, and the
    // renderer walks its node list.
    std::lock_guard<std::mutex> lock(route_mtx_);
    if (route_.valid) view.route = &route_;
    panel_render_size_ = cv::Size(cfg_.panel_width, height);
    return render_map_panel(map_, view, cfg_.panel_width, height, &proj_);
}

cv::Mat Navigator::compose(const cv::Mat& frame)
{
    if (!cfg_.enabled || frame.empty()) return frame;

    cv::Mat left = frame;
    if (left.type() != CV_8UC3) {
        cv::Mat conv;
        cv::cvtColor(left, conv,
                     left.channels() == 4 ? cv::COLOR_BGRA2BGR : cv::COLOR_GRAY2BGR);
        left = conv;
    }

    // Plain side-by-side: the frame simply gets wider.
    if (cfg_.out_width <= 0 || cfg_.out_height <= 0) {
        cv::Mat right = panel(left.rows);
        if (right.rows != left.rows)
            cv::resize(right, right, cv::Size(right.cols, left.rows), 0, 0, cv::INTER_AREA);

        cv::Mat out(left.rows, left.cols + right.cols, CV_8UC3);
        left.copyTo(out(cv::Rect(0, 0, left.cols, left.rows)));
        right.copyTo(out(cv::Rect(left.cols, 0, right.cols, right.rows)));
        panel_rect_ = cv::Rect(left.cols, 0, right.cols, right.rows);

        // The destination box goes over the finished frame, not inside the panel:
        // it is a modal, and it gets the whole window.
        if (input_active_) {
            MapView box;
            box.input_active = true;
            box.input_text   = input_text_;
            box.input_hits   = input_hits_;
            box.input_sel    = input_sel_;
            box.input_note   = input_note_;
            box.dark         = cfg_.dark;
            draw_destination_box(out, box);
        }
        return out;
    }

    // Fixed canvas. Camera view plus panel is roughly 3:1, and a window that
    // shape is letterboxed by the toolkit — in white — on a 16:9 screen. Doing
    // the padding here instead means it is black and part of the image, so the
    // window matches the screen exactly and nothing frames it.
    const int W = cfg_.out_width;
    const int H = cfg_.out_height;
    const int panel_w = std::clamp(cfg_.panel_width, 160, W / 2);

    cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::Mat right = panel(H);
    if (right.cols != panel_w || right.rows != H)
        cv::resize(right, right, cv::Size(panel_w, H), 0, 0, cv::INTER_AREA);
    right.copyTo(canvas(cv::Rect(W - panel_w, 0, panel_w, H)));
    panel_rect_ = cv::Rect(W - panel_w, 0, panel_w, H);

    // Camera view scaled to fit its half, aspect preserved, centred. Any
    // leftover is black rather than the toolkit's white, and --output can be
    // sized so there is no leftover at all: height = (width - panel) / 2 for a
    // 2:1 camera frame.
    const int cam_w = W - panel_w;
    const double scale = std::min(static_cast<double>(cam_w) / left.cols,
                                  static_cast<double>(H) / left.rows);
    const int dw = std::max(1, static_cast<int>(left.cols * scale));
    const int dh = std::max(1, static_cast<int>(left.rows * scale));

    cv::Mat scaled;
    cv::resize(left, scaled, cv::Size(dw, dh), 0, 0,
               scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
    scaled.copyTo(canvas(cv::Rect((cam_w - dw) / 2, (H - dh) / 2, dw, dh)));

    // The destination box goes over the finished frame, not inside the panel:
    // it is a modal, and it gets the whole window.
    if (input_active_) {
        MapView box;
        box.input_active = true;
        box.input_text   = input_text_;
        box.input_hits   = input_hits_;
        box.input_sel    = input_sel_;
        box.input_note   = input_note_;
        box.dark         = cfg_.dark;
        draw_destination_box(canvas, box);
    }

    return canvas;
}

}  // namespace navigation
