#include <navigation/map_renderer.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace navigation {

namespace {

constexpr int kFont      = cv::FONT_HERSHEY_DUPLEX;
constexpr int kManeuverH = 88;   // px — arrow + distance + "onto <road>"

// Dark basemap, so the panel sits next to the camera view without fighting it.
const cv::Scalar kBackground(24, 22, 20);
const cv::Scalar kPanelEdge (60, 56, 52);
const cv::Scalar kTextMain  (240, 240, 240);
const cv::Scalar kTextDim   (150, 150, 150);
const cv::Scalar kRouteLine (60, 190, 255);    // BGR — amber
const cv::Scalar kEgo       (120, 255, 120);
const cv::Scalar kBanner    (40, 36, 32);
const cv::Scalar kWarn      (60, 170, 255);    // BGR — warning amber
const cv::Scalar kDest      (120, 210, 255);

// Semi-transparent filled rectangle, clipped to the image.
void fill_rect_alpha(cv::Mat& img, cv::Rect r, const cv::Scalar& color, double alpha)
{
    r &= cv::Rect(0, 0, img.cols, img.rows);
    if (r.width <= 0 || r.height <= 0) return;
    cv::Mat roi = img(r);
    cv::Mat block(roi.size(), roi.type(), color);
    cv::addWeighted(block, alpha, roi, 1.0 - alpha, 0.0, roi);
}

struct RoadStyle {
    cv::Scalar color;
    double     width;      // px at the reference zoom
};

// Wider and lighter as the class goes up, the usual road-atlas hierarchy.
RoadStyle style_for(uint8_t road_class, bool link)
{
    RoadStyle s;
    switch (road_class) {
    case kMotorway:     s = {cv::Scalar(120, 190, 250), 5.0}; break;
    case kTrunk:        s = {cv::Scalar(110, 170, 240), 4.5}; break;
    case kPrimary:      s = {cv::Scalar(140, 190, 235), 4.0}; break;
    case kSecondary:    s = {cv::Scalar(150, 180, 200), 3.2}; break;
    case kTertiary:     s = {cv::Scalar(140, 155, 170), 2.6}; break;
    case kResidential:  s = {cv::Scalar(105, 110, 118), 2.0}; break;
    case kLivingStreet: s = {cv::Scalar(95, 100, 108),  1.6}; break;
    // Dimmer and thinner than a residential street: present so the map is not
    // blank inside a complex, but never mistaken for a through-road.
    case kService:      s = {cv::Scalar(78, 82, 88),    1.3}; break;
    default:            s = {cv::Scalar(100, 104, 110), 1.8}; break;
    }
    if (link) s.width = std::max(1.4, s.width * 0.6);
    return s;
}

const char* road_class_label(uint8_t c)
{
    switch (c) {
    case kMotorway:     return "motorway";
    case kTrunk:        return "trunk road";
    case kPrimary:      return "primary road";
    case kSecondary:    return "secondary road";
    case kTertiary:     return "tertiary road";
    case kResidential:  return "residential street";
    case kUnclassified: return "road";
    case kLivingStreet: return "living street";
    case kService:      return "service road";
    default:            return "road";
    }
}

void text(cv::Mat& img, const std::string& s, cv::Point at, double scale,
          cv::Scalar color, int thickness = 1)
{
    cv::putText(img, s, at + cv::Point(1, 1), kFont, scale, cv::Scalar(0, 0, 0),
                thickness + 1, cv::LINE_AA);
    cv::putText(img, s, at, kFont, scale, color, thickness, cv::LINE_AA);
}

void text_center(cv::Mat& img, const std::string& s, int cx, int baseline,
                 double scale, cv::Scalar color, int thickness = 1)
{
    int bl = 0;
    const cv::Size ts = cv::getTextSize(s, kFont, scale, thickness, &bl);
    text(img, s, cv::Point(cx - ts.width / 2, baseline), scale, color, thickness);
}

// The Hershey fonts OpenCV ships are ASCII only — anything above 0x7F comes out
// as '?'. Road names off OSM are UTF-8, so fold them before drawing.
std::string ascii(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        if (c < 0x80) { out.push_back(static_cast<char>(c)); continue; }
        // Latin-1 supplement and the dashes/quotes OSM names actually contain.
        if (c == 0xC3 && i + 1 < s.size()) {                 // Ã-prefixed accents
            static const char* kFold = "AAAAAAACEEEEIIIIDNOOOOO*OUUUUYPsaaaaaaaceeeeiiiidnooooo/ouuuuypy";
            const unsigned char n = static_cast<unsigned char>(s[++i]);
            const int idx = n - 0x80;
            out.push_back(idx >= 0 && idx < 64 ? kFold[idx] : '?');
        } else if (c == 0xE2 && i + 2 < s.size()) {          // dashes, quotes, bullets
            i += 2;
            out.push_back('-');
        } else if (c == 0xC2 && i + 1 < s.size()) {
            ++i;                                             // NBSP and friends
            out.push_back(' ');
        } else {
            out.push_back('?');
        }
    }
    return out;
}

// Trim to fit a pixel width, with an ellipsis. Road names in Bandung run long
// ("Jalan Raya Bandung-Garut") and would otherwise spill out of the panel.
std::string elide(const std::string& raw, int max_px, double scale, int thickness = 1)
{
    const std::string s = ascii(raw);
    int bl = 0;
    if (s.empty() || cv::getTextSize(s, kFont, scale, thickness, &bl).width <= max_px)
        return s;
    std::string out = s;
    while (out.size() > 1) {
        out.pop_back();
        if (cv::getTextSize(out + "...", kFont, scale, thickness, &bl).width <= max_px)
            return out + "...";
    }
    return out;
}

std::string format_distance(double m)
{
    char buf[32];
    if (m < 950.0) std::snprintf(buf, sizeof(buf), "%.0f m", std::round(m / 10.0) * 10.0);
    else           std::snprintf(buf, sizeof(buf), "%.1f km", m / 1000.0);
    return buf;
}

std::string format_duration(double s)
{
    char buf[32];
    const int mins = static_cast<int>(std::round(s / 60.0));
    if (mins < 60) std::snprintf(buf, sizeof(buf), "%d min", std::max(1, mins));
    else           std::snprintf(buf, sizeof(buf), "%d h %02d", mins / 60, mins % 60);
    return buf;
}

// Manoeuvre glyph: a chevron bent the way the driver has to go. Drawn rather
// than loaded so the panel needs no extra assets.
void draw_maneuver(cv::Mat& img, Maneuver m, cv::Point c, int size, cv::Scalar color)
{
    const int h = size / 2;
    std::vector<cv::Point> shaft;
    double tip_angle = 0.0;   // deg, 0 = up

    switch (m) {
    case Maneuver::Left:
    case Maneuver::SharpLeft:
        shaft = {{c.x + h / 2, c.y + h}, {c.x + h / 2, c.y}, {c.x - h / 2, c.y}};
        tip_angle = -90.0;
        break;
    case Maneuver::Right:
    case Maneuver::SharpRight:
        shaft = {{c.x - h / 2, c.y + h}, {c.x - h / 2, c.y}, {c.x + h / 2, c.y}};
        tip_angle = 90.0;
        break;
    case Maneuver::SlightLeft:
        shaft = {{c.x + h / 2, c.y + h}, {c.x + h / 2, c.y}, {c.x - h / 3, c.y - h / 2}};
        tip_angle = -35.0;
        break;
    case Maneuver::SlightRight:
        shaft = {{c.x - h / 2, c.y + h}, {c.x - h / 2, c.y}, {c.x + h / 3, c.y - h / 2}};
        tip_angle = 35.0;
        break;
    case Maneuver::UTurn:
        shaft = {{c.x + h / 2, c.y + h}, {c.x + h / 2, c.y - h / 3},
                 {c.x - h / 2, c.y - h / 3}, {c.x - h / 2, c.y + h / 3}};
        tip_angle = 180.0;
        break;
    case Maneuver::Arrive:
        cv::circle(img, cv::Point(c.x, c.y), h / 2, color, 2, cv::LINE_AA);
        cv::circle(img, cv::Point(c.x, c.y), h / 6, color, -1, cv::LINE_AA);
        return;
    default:   // Depart / Straight
        shaft = {{c.x, c.y + h}, {c.x, c.y - h / 3}};
        tip_angle = 0.0;
        break;
    }

    cv::polylines(img, shaft, false, color, 4, cv::LINE_AA);

    // Arrow head on the last vertex, pointing along tip_angle.
    const cv::Point tip = shaft.back();
    const double a = tip_angle * M_PI / 180.0;
    const cv::Point2d dir(std::sin(a), -std::cos(a));
    const cv::Point2d perp(-dir.y, dir.x);
    const double head = size * 0.28;
    const std::vector<cv::Point> tri = {
        cv::Point(cvRound(tip.x + dir.x * head), cvRound(tip.y + dir.y * head)),
        cv::Point(cvRound(tip.x - perp.x * head * 0.6), cvRound(tip.y - perp.y * head * 0.6)),
        cv::Point(cvRound(tip.x + perp.x * head * 0.6), cvRound(tip.y + perp.y * head * 0.6)),
    };
    cv::fillConvexPoly(img, tri, color, cv::LINE_AA);
}

// Ways run far past the viewport — a primary road can be the width of the whole
// extract. Draw segment by segment and drop the ones that cannot touch the
// image, so nothing hands OpenCV a line a million pixels long.
void draw_polyline_clipped(cv::Mat& img, const std::vector<cv::Point>& pts,
                           const cv::Scalar& color, int thickness)
{
    const int m = thickness + 2;
    const int x0 = -m, y0 = -m, x1 = img.cols + m, y1 = img.rows + m;

    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const cv::Point& a = pts[i];
        const cv::Point& b = pts[i + 1];
        if ((a.x < x0 && b.x < x0) || (a.x > x1 && b.x > x1) ||
            (a.y < y0 && b.y < y0) || (a.y > y1 && b.y > y1))
            continue;
        cv::line(img, a, b, color, thickness, cv::LINE_AA);
    }
}

void draw_ego(cv::Mat& img, cv::Point c, double heading_rad)
{
    const double s = 11.0;
    auto rot = [&](double x, double y) {
        return cv::Point(cvRound(c.x + x * std::cos(heading_rad) - y * std::sin(heading_rad)),
                         cvRound(c.y + x * std::sin(heading_rad) + y * std::cos(heading_rad)));
    };
    const std::vector<cv::Point> tri = {rot(0, -s), rot(-s * 0.66, s * 0.7), rot(s * 0.66, s * 0.7)};
    cv::circle(img, c, static_cast<int>(s * 1.5), cv::Scalar(60, 90, 60), -1, cv::LINE_AA);
    cv::fillConvexPoly(img, tri, kEgo, cv::LINE_AA);
    cv::polylines(img, tri, true, cv::Scalar(20, 60, 20), 1, cv::LINE_AA);
}

}  // namespace

cv::Mat render_map_panel(const OsmMap& map, const MapView& view, int width, int height)
{
    cv::Mat panel(height, width, CV_8UC3, kBackground);

    // kManeuverH is the arrow/distance part; the destination strip sits under it.
    const int banner_h = view.route && view.route->valid ? kManeuverH + 30 : 0;
    const int footer_h = 74;
    const int map_top    = banner_h;
    const int map_bottom = height - footer_h;
    const int map_h      = std::max(1, map_bottom - map_top);

    // ── Projection: metres → panel pixels, ego near the lower third ──────────
    const double m_per_px = std::max(1e-3, view.zoom_m / std::max(1, width));
    const cv::Point ego_px(width / 2, map_top + static_cast<int>(map_h * 0.62));
    const double rot = (view.heading_up && view.fix && view.fix->has_bearing)
                           ? -view.fix->bearing_deg * M_PI / 180.0
                           : 0.0;
    const double cr = std::cos(rot), sr = std::sin(rot);

    const double ref_lat = view.fix ? view.fix->latitude  : 0.0;
    const double ref_lon = view.fix ? view.fix->longitude : 0.0;

    auto project = [&](double lat, double lon) {
        double e, n;
        lat_lon_to_en(lat, lon, ref_lat, ref_lon, e, n);
        // Rotate so the heading points up, then flip north to screen-up.
        const double rx = e * cr - n * sr;
        const double ry = e * sr + n * cr;
        // Clamped: a destination 40 km away is otherwise a coordinate no
        // rasteriser wants to see, and off-screen is off-screen either way.
        return cv::Point(
            static_cast<int>(std::clamp(ego_px.x + rx / m_per_px, -1e5, 1e5)),
            static_cast<int>(std::clamp(ego_px.y - ry / m_per_px, -1e5, 1e5)));
    };

    if (view.fix && map.loaded()) {
        // Half-diagonal of the map area, so a rotated view still covers corners.
        const double reach_m = std::hypot(width, map_h) * 0.5 * m_per_px + 100.0;
        const double dlat = reach_m / kEarthR * 180.0 / M_PI;
        const double dlon = dlat / std::max(0.2, std::cos(ref_lat * M_PI / 180.0));

        const cv::Rect clip(0, map_top, width, map_h);
        cv::Mat roi = panel(clip);

        const auto ids = map.ways_in_bbox(ref_lat - dlat, ref_lon - dlon,
                                          ref_lat + dlat, ref_lon + dlon);

        // Two passes so the arterials are not cut by the residential streets
        // crossing them: casings first, then the fills.
        for (int pass = 0; pass < 2; ++pass) {
            for (const uint32_t wi : ids) {
                const Way& w = map.ways()[wi];
                const RoadStyle st = style_for(w.road_class, (w.flags & kLink) != 0);

                std::vector<cv::Point> pts;
                pts.reserve(w.nodes.size());
                for (const uint32_t ni : w.nodes) {
                    const Node& n = map.nodes()[ni];
                    cv::Point p = project(n.lat(), n.lon());
                    pts.push_back(cv::Point(p.x, p.y - map_top));
                }
                if (pts.size() < 2) continue;

                if (pass == 0)
                    draw_polyline_clipped(roi, pts, cv::Scalar(12, 11, 10),
                                          cvRound(st.width) + 3);
                else
                    draw_polyline_clipped(roi, pts, st.color,
                                          std::max(1, cvRound(st.width)));
            }
        }

        // ── Route ────────────────────────────────────────────────────────────
        if (view.route && view.route->valid) {
            std::vector<cv::Point> pts;
            pts.reserve(view.route->nodes.size());
            for (const uint32_t ni : view.route->nodes) {
                const Node& n = map.nodes()[ni];
                cv::Point p = project(n.lat(), n.lon());
                pts.push_back(cv::Point(p.x, p.y - map_top));
            }
            if (pts.size() >= 2) {
                draw_polyline_clipped(roi, pts, cv::Scalar(40, 60, 90), 11);
                draw_polyline_clipped(roi, pts, kRouteLine, 6);
            }

            // Destination pin.
            const cv::Point d = project(view.route->dest_lat, view.route->dest_lon);
            const cv::Point dp(d.x, d.y - map_top);
            if (clip.contains(cv::Point(d.x, d.y))) {
                cv::circle(roi, dp, 8, cv::Scalar(60, 60, 240), -1, cv::LINE_AA);
                cv::circle(roi, dp, 8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            }
        }

        draw_ego(panel, ego_px, view.heading_up || !view.fix->has_bearing
                                    ? 0.0
                                    : view.fix->bearing_deg * M_PI / 180.0);

        // ── Scale bar ────────────────────────────────────────────────────────
        {
            const double bar_m = view.zoom_m / 4.0 < 150.0 ? 100.0
                               : (view.zoom_m / 4.0 < 400.0 ? 250.0 : 500.0);
            const int bar_px = static_cast<int>(bar_m / m_per_px);
            const int y = map_bottom - 14;
            cv::line(panel, cv::Point(14, y), cv::Point(14 + bar_px, y), kTextDim, 2, cv::LINE_AA);
            cv::line(panel, cv::Point(14, y - 4), cv::Point(14, y + 4), kTextDim, 2, cv::LINE_AA);
            cv::line(panel, cv::Point(14 + bar_px, y - 4), cv::Point(14 + bar_px, y + 4),
                     kTextDim, 2, cv::LINE_AA);
            text(panel, format_distance(bar_m), cv::Point(18, y - 8), 0.4, kTextDim);
        }
        text(panel, view.heading_up ? "heading up" : "north up",
             cv::Point(width - 96, map_top + 20), 0.38, kTextDim);
    } else {
        // No position, or no map. Never quietly draw a plausible-looking map at
        // some default coordinate — say the position is missing, loudly, because
        // a map showing the wrong place is worse than no map.
        const int cy = map_top + map_h / 2;
        fill_rect_alpha(panel, cv::Rect(0, map_top, width, map_h),
                        cv::Scalar(20, 45, 90), 0.30);

        const int tri = 26;
        const cv::Point c(width / 2, cy - 78);
        const std::vector<cv::Point> warn = {
            {c.x, c.y - tri}, {c.x - tri, c.y + tri * 2 / 3}, {c.x + tri, c.y + tri * 2 / 3}};
        cv::polylines(panel, warn, true, kWarn, 3, cv::LINE_AA);
        cv::line(panel, cv::Point(c.x, c.y - tri / 2), cv::Point(c.x, c.y + tri / 5),
                 kWarn, 3, cv::LINE_AA);
        cv::circle(panel, cv::Point(c.x, c.y + tri / 2), 2, kWarn, -1, cv::LINE_AA);

        text_center(panel, view.map_loaded ? "NO GPS SIGNAL" : "NO MAP LOADED",
                    width / 2, cy - 18, 0.72, kWarn, 2);
        text_center(panel, elide("source: " + std::string(view.source_name), width - 30, 0.44),
                    width / 2, cy + 12, 0.44, kTextMain);
        text_center(panel, elide(view.status, width - 24, 0.40), width / 2,
                    cy + 36, 0.40, kTextDim);
        text_center(panel, "position unknown - map not drawn", width / 2, cy + 64,
                    0.40, kTextDim);
    }

    // ── Manoeuvre banner + destination strip ─────────────────────────────────
    // "Turn right, 60 m" alone never says where you are ultimately going, so the
    // destination gets its own line and stays on screen for the whole route.
    if (banner_h > 0) {
        cv::rectangle(panel, cv::Rect(0, 0, width, banner_h), kBanner, -1);
        cv::line(panel, cv::Point(0, banner_h), cv::Point(width, banner_h), kPanelEdge, 1);

        const Route& r = *view.route;
        const int idx = view.progress.next_step;

        if (!view.fix) {
            // A stale route with no current position: show the target, not a
            // manoeuvre distance that is no longer true.
            text(panel, "Route paused - no position", cv::Point(16, 40), 0.52, kWarn);
        } else if (idx >= 0 && idx < static_cast<int>(r.steps.size())) {
            const Step& s = r.steps[idx];
            draw_maneuver(panel, s.type, cv::Point(52, kManeuverH / 2), 46, kTextMain);
            text(panel, format_distance(view.progress.distance_to_step_m),
                 cv::Point(104, 40), 0.78, kTextMain, 2);
            const std::string what = s.road.empty()
                                         ? maneuver_text(s.type)
                                         : std::string(maneuver_text(s.type)) + " onto " + s.road;
            text(panel, elide(what, width - 118, 0.46), cv::Point(104, 68), 0.46, kTextDim);
        }

        // Destination strip along the bottom of the banner.
        cv::line(panel, cv::Point(12, kManeuverH), cv::Point(width - 12, kManeuverH),
                 kPanelEdge, 1);
        char dest_buf[96];
        if (!r.dest_name.empty()) {
            std::snprintf(dest_buf, sizeof(dest_buf), "to  %s", r.dest_name.c_str());
        } else {
            std::snprintf(dest_buf, sizeof(dest_buf), "to  %.5f, %.5f",
                          r.dest_lat, r.dest_lon);
        }
        const std::string right = format_distance(view.progress.remaining_m) + "  " +
                                  format_duration(view.progress.eta_s);
        int bl = 0;
        const cv::Size rs = cv::getTextSize(right, kFont, 0.44, 1, &bl);
        text(panel, elide(dest_buf, width - rs.width - 40, 0.44),
             cv::Point(16, banner_h - 10), 0.44, kDest);
        text(panel, right, cv::Point(width - rs.width - 14, banner_h - 10), 0.44, kTextMain);
    }

    // ── Footer: road, limit, route summary, GPS state ────────────────────────
    {
        const int y0 = height - footer_h;
        cv::rectangle(panel, cv::Rect(0, y0, width, footer_h), kBanner, -1);
        cv::line(panel, cv::Point(0, y0), cv::Point(width, y0), kPanelEdge, 1);

        // "No name" and "no match" are different facts: most service roads in a
        // complex are unnamed, and calling that "no road matched" reads as a
        // broken map when the position is in fact snapped to a road.
        std::string road = view.road_name;
        if (road.empty())
            road = view.match.valid
                       ? std::string("unnamed ") + road_class_label(view.matched_class)
                       : "no road matched";
        text(panel, elide(road, width - 92, 0.52), cv::Point(14, y0 + 26), 0.52, kTextMain);

        if (view.speed_limit_kph > 0) {
            const int bw = 46, bh = 34;
            const cv::Rect box(width - bw - 12, y0 + 8, bw, bh);
            cv::rectangle(panel, box, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            char lbl[16];
            std::snprintf(lbl, sizeof(lbl), "%d", view.speed_limit_kph);
            text_center(panel, lbl, box.x + bw / 2, box.y + 26, 0.62,
                        view.speed_inferred ? kTextDim : kTextMain, 1);
        }

        std::string line2;
        if (view.route && view.route->valid) {
            line2 = format_distance(view.progress.remaining_m) + "   |   " +
                    format_duration(view.progress.eta_s);
            if (!view.route->dest_name.empty()) line2 += "   |   " + view.route->dest_name;
            if (!view.progress.on_route) line2 += "   |   off route";
        } else if (view.routing) {
            line2 = "routing...";
        } else if (view.fix) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.5f, %.5f   |   %.0f km/h",
                          view.fix->latitude, view.fix->longitude, view.fix->speed_ms * 3.6);
            line2 = buf;
        } else {
            line2 = std::string("gps: ") + view.source_name;
        }
        text(panel, elide(line2, width - 24, 0.42), cv::Point(14, y0 + 52), 0.42, kTextDim);
    }

    // The box itself is drawn over the whole window by draw_destination_box();
    // inside a 512 px panel it would be a postage stamp.
    if (!view.input_active && view.map_loaded)
        text(panel, "press  d  to set destination",
             cv::Point(12, height - footer_h - 10), 0.36, kTextDim);

    cv::rectangle(panel, cv::Rect(0, 0, width, height), kPanelEdge, 1);
    return panel;
}

// ─── Destination box ─────────────────────────────────────────────────────────
// Drawn over the entire window rather than inside the map panel: while it is
// open it owns the keyboard, and it should own the screen to match.
void draw_destination_box(cv::Mat& canvas, const MapView& view)
{
    if (!view.input_active || canvas.empty()) return;

    const int W = canvas.cols, H = canvas.rows;
    fill_rect_alpha(canvas, cv::Rect(0, 0, W, H), cv::Scalar(0, 0, 0), 0.62);

    const int rows   = std::max<int>(1, static_cast<int>(view.input_hits.size()));
    const int row_h  = 46;
    const int box_w  = std::min(860, W - 120);
    const int box_h  = std::min(H - 100, 168 + rows * row_h);
    const cv::Rect box((W - box_w) / 2, std::max(40, H / 2 - box_h / 2), box_w, box_h);

    cv::rectangle(canvas, box, cv::Scalar(28, 26, 24), -1);
    cv::rectangle(canvas, box, kDest, 2, cv::LINE_AA);

    text(canvas, "Where to?", cv::Point(box.x + 22, box.y + 42), 0.78, kDest, 2);

    // Entry line with a blinking caret.
    const cv::Rect entry(box.x + 22, box.y + 60, box.width - 44, 46);
    cv::rectangle(canvas, entry, cv::Scalar(16, 15, 14), -1);
    cv::rectangle(canvas, entry, kPanelEdge, 1);
    const bool caret_on =
        (cv::getTickCount() / static_cast<int64>(cv::getTickFrequency() / 2)) % 2 == 0;
    const std::string typed = ascii(view.input_text);
    text(canvas, elide(typed.empty() ? "type a place or street name" : typed,
                       entry.width - 40, 0.62),
         cv::Point(entry.x + 14, entry.y + 32), 0.62,
         typed.empty() ? kTextDim : kTextMain);
    if (caret_on) {
        int bl = 0;
        const cv::Size ts = cv::getTextSize(typed, kFont, 0.62, 1, &bl);
        const int cx = entry.x + 16 + (typed.empty() ? 0 : ts.width);
        cv::line(canvas, cv::Point(cx, entry.y + 12), cv::Point(cx, entry.y + 34),
                 kTextMain, 2, cv::LINE_AA);
    }

    int y = entry.y + entry.height + 34;
    if (view.input_hits.empty()) {
        text(canvas, view.input_note.empty() ? "e.g. Trans Studio Mall, Jalan Braga"
                                             : view.input_note,
             cv::Point(box.x + 26, y), 0.5, kTextDim);
    } else {
        for (size_t i = 0; i < view.input_hits.size(); ++i) {
            const auto& p = view.input_hits[i];
            const bool sel = static_cast<int>(i) == view.input_sel;
            if (sel)
                fill_rect_alpha(canvas,
                                cv::Rect(box.x + 12, y - 26, box.width - 24, row_h - 6),
                                kDest, 0.30);
            // A mall and a street can share a name; say which this row is.
            const std::string tag = p.is_poi
                                        ? (p.kind.empty() ? "place" : p.kind)
                                        : road_class_label(p.road_class);
            text(canvas, elide(p.name, box.width - 230, 0.54),
                 cv::Point(box.x + 26, y), 0.54, sel ? kTextMain : kTextDim);
            int bl = 0;
            const cv::Size ts = cv::getTextSize(ascii(tag), kFont, 0.42, 1, &bl);
            text(canvas, ascii(tag),
                 cv::Point(box.x + box.width - ts.width - 26, y), 0.42, kTextDim);
            y += row_h;
            if (y > box.y + box.height - 40) break;
        }
    }

    text(canvas, "up/down  choose      Enter  go      Esc  cancel",
         cv::Point(box.x + 22, box.y + box.height - 18), 0.44, kTextDim);
}

}  // namespace navigation
