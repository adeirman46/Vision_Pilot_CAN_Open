#include <navigation/navigation.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

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

    if (cfg_.has_destination) {
        dest_lat_   = cfg_.dest_lat;
        dest_lon_   = cfg_.dest_lon;
        dest_name_  = cfg_.dest_name;
        want_route_ = true;
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

void Navigator::set_destination(double lat, double lon, const std::string& name)
{
    dest_lat_   = lat;
    dest_lon_   = lon;
    dest_name_  = name;
    want_route_ = true;
    last_route_ms_ = 0;   // route now, do not wait out the cooldown
    request_route();
}

void Navigator::clear_destination()
{
    want_route_ = false;
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
                          dest_lat_, dest_lon_, dest_name_);
}

void Navigator::route_worker(double from_lat, double from_lon, double heading,
                             bool have_heading, double to_lat, double to_lon,
                             std::string dest_name)
{
    Route r = compute_route(map_, from_lat, from_lon, to_lat, to_lon,
                            have_heading ? &heading : nullptr);
    r.dest_name = std::move(dest_name);

    if (!shutdown_) {
        std::lock_guard<std::mutex> lock(route_mtx_);
        // Keep the previous route when the new one failed — a momentary bad fix
        // should not wipe the guidance off the panel.
        if (r.valid || !route_.valid) route_ = std::move(r);
    }
    routing_ = false;
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

    // Hold the lock across the render: the worker may swap the route in, and the
    // renderer walks its node list.
    std::lock_guard<std::mutex> lock(route_mtx_);
    if (route_.valid) view.route = &route_;
    return render_map_panel(map_, view, cfg_.panel_width, height);
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
