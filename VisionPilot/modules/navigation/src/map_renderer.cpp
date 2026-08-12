#include <navigation/map_renderer.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace navigation {

namespace {

constexpr int kFont      = cv::FONT_HERSHEY_DUPLEX;
constexpr int kManeuverH = 88;   // px — arrow + distance + "onto <road>"

// Palette. Mutable and swapped per frame by set_theme(): a night drive wants
// the dark map, a bright day wants the light one, and the difference is only
// ever which set of colours these names hold.
cv::Scalar kBackground(24, 22, 20);
cv::Scalar kPanelEdge (60, 56, 52);
cv::Scalar kTextMain  (240, 240, 240);
cv::Scalar kTextDim   (150, 150, 150);
cv::Scalar kRouteLine (60, 190, 255);    // BGR — amber
cv::Scalar kEgo       (120, 255, 120);
cv::Scalar kBanner    (40, 36, 32);
cv::Scalar kWarn      (60, 170, 255);
cv::Scalar kDest      (120, 210, 255);
cv::Scalar kCasing    (12, 11, 10);      // outline under every road
bool       g_dark = true;

void set_theme(bool dark)
{
    g_dark = dark;
    if (dark) {
        kBackground = {24, 22, 20};
        kPanelEdge  = {60, 56, 52};
        kTextMain   = {240, 240, 240};
        kTextDim    = {150, 150, 150};
        kRouteLine  = {60, 190, 255};
        kEgo        = {120, 255, 120};
        kBanner     = {40, 36, 32};
        kWarn       = {60, 170, 255};
        kDest       = {120, 210, 255};
        kCasing     = {12, 11, 10};
    } else {
        // Paper map: near-white land, white roads with a grey casing, which is
        // what makes the hierarchy readable in sunlight.
        kBackground = {235, 238, 240};
        kPanelEdge  = {190, 193, 196};
        kTextMain   = {32, 33, 36};
        kTextDim    = {110, 114, 120};
        kRouteLine  = {40, 140, 240};
        kEgo        = {40, 150, 40};
        kBanner     = {248, 249, 250};
        kWarn       = {30, 110, 220};
        kDest       = {180, 100, 30};
        kCasing     = {186, 190, 194};
    }
}

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
    if (!g_dark) {
        // Light theme: white roads on pale land, wider as the class rises.
        switch (road_class) {
        case kMotorway:     s = {cv::Scalar(150, 205, 255), 5.0}; break;
        case kTrunk:        s = {cv::Scalar(170, 220, 255), 4.5}; break;
        case kPrimary:      s = {cv::Scalar(215, 240, 255), 4.0}; break;
        case kSecondary:    s = {cv::Scalar(255, 255, 255), 3.2}; break;
        case kTertiary:     s = {cv::Scalar(255, 255, 255), 2.6}; break;
        case kResidential:  s = {cv::Scalar(255, 255, 255), 2.0}; break;
        case kLivingStreet: s = {cv::Scalar(250, 250, 250), 1.6}; break;
        case kService:      s = {cv::Scalar(244, 245, 246),  1.3}; break;
        default:            s = {cv::Scalar(250, 250, 250), 1.8}; break;
        }
        if (link) s.width = std::max(1.4, s.width * 0.6);
        return s;
    }
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

// ─── Basemap cache ───────────────────────────────────────────────────────────
// Roads and the route are expensive to draw — every way in view, re-projected
// and antialias-stroked twice — and they only change when the car moves, the
// zoom changes or the route is replaced. All of those happen at GPS rate (2 Hz)
// or on a keypress, never at frame rate. So draw them once into a buffer larger
// than the panel and blit a window of it each frame; a drag then costs a memcpy
// instead of a full redraw, which is what makes it track the mouse smoothly.
struct BasemapCache {
    cv::Mat img;
    bool    valid = false;
    bool    dark = true;
    double  ref_lat = 0, ref_lon = 0, rot = 0, m_per_px = 0;
    int     w = 0, h = 0;
    double  pan_x = 0, pan_y = 0;    // pan the buffer was drawn for
    size_t  route_fp = 0;
    bool    satellite = false;
    RoadOverlay sat_roads = RoadOverlay::All;
    double  sat_road_opacity = 0.85;
    int     tiles_painted = 0;       // 0 = imagery asked for but none arrived
    int     tiles_pending = 0;       // still in flight when this was drawn
    int     served_at = -1;          // TileSource::served() at that moment
    int64   redrawn_at = 0;          // getTickCount(), throttles the tile refill
};
BasemapCache g_basemap;
constexpr int kBasemapMargin = 260;   // px of slack around the panel

// Cheap fingerprint: a different route always differs in one of these.
size_t route_fingerprint(const Route* r)
{
    if (!r || !r->valid) return 0;
    return r->nodes.size() * 1000003u +
           static_cast<size_t>(r->length_m) * 31u +
           static_cast<size_t>(r->dest_lat * 1e5) ^
           static_cast<size_t>(r->dest_lon * 1e5);
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

// `known` is false when nothing has established which way the car is pointing.
// That happens more often than it sounds: a bearing derived from successive GPS
// fixes needs real travel before it means anything, so a car that is stopped,
// crawling, or freshly started has no heading at all. Drawing a confident arrow
// anyway would be inventing one — it would point north and look exactly like a
// car facing north. A blunt disc says "position yes, direction no", which is the
// truth, and the arrow returns the moment the fusion has a heading again.
//
// `accuracy_px` draws the uncertainty the fix reports as a halo. With the
// GPS/IMU filter this shrinks as the filter converges and swells when fixes stop
// arriving, so how much to trust the dot is visible rather than implied.
void draw_ego(cv::Mat& img, cv::Point c, double heading_rad, bool known,
              double accuracy_px)
{
    const double s = 11.0;

    if (accuracy_px > s * 1.6) {
        cv::Mat halo = img.clone();
        cv::circle(halo, c, static_cast<int>(std::min(accuracy_px, 400.0)),
                   cv::Scalar(120, 150, 120), -1, cv::LINE_AA);
        cv::addWeighted(halo, 0.16, img, 0.84, 0.0, img);
        cv::circle(img, c, static_cast<int>(std::min(accuracy_px, 400.0)),
                   cv::Scalar(90, 120, 90), 1, cv::LINE_AA);
    }

    cv::circle(img, c, static_cast<int>(s * 1.5), cv::Scalar(60, 90, 60), -1, cv::LINE_AA);
    // A white ring round the disc so it reads at the same weight as the raw-GPS
    // marker beside it in compare mode, and stays visible on a light basemap.
    cv::circle(img, c, static_cast<int>(s * 1.05), cv::Scalar(40, 190, 40), -1, cv::LINE_AA);
    cv::circle(img, c, static_cast<int>(s * 1.05), cv::Scalar(235, 245, 235), 2, cv::LINE_AA);

    if (!known) {
        cv::circle(img, c, static_cast<int>(s * 0.62), kEgo, -1, cv::LINE_AA);
        cv::circle(img, c, static_cast<int>(s * 0.62), cv::Scalar(20, 60, 20), 1,
                   cv::LINE_AA);
        return;
    }

    auto rot = [&](double x, double y) {
        return cv::Point(cvRound(c.x + x * std::cos(heading_rad) - y * std::sin(heading_rad)),
                         cvRound(c.y + x * std::sin(heading_rad) + y * std::cos(heading_rad)));
    };
    // A ray ahead of the nose: on a moving map the triangle alone is small, and
    // the direction of travel is the one thing that has to be readable at a
    // glance.
    cv::line(img, c, rot(0, -s * 2.6), cv::Scalar(150, 220, 150), 2, cv::LINE_AA);
    const std::vector<cv::Point> tri = {rot(0, -s), rot(-s * 0.66, s * 0.7), rot(s * 0.66, s * 0.7)};
    cv::fillConvexPoly(img, tri, kEgo, cv::LINE_AA);
    cv::polylines(img, tri, true, cv::Scalar(20, 60, 20), 1, cv::LINE_AA);
}

// ─── Aerial imagery ──────────────────────────────────────────────────────────
// The panel works in a local tangent plane around the fix; tiles are squares of
// Web Mercator. Over the few hundred metres a tile covers the difference
// between the two is a conformal map — a rotation and a scale — so each tile is
// placed with the affine transform that takes three of its corners to where the
// panel's own projection puts them. That is a warpAffine per tile rather than a
// per-pixel reprojection, and it is exact to well under a pixel at these zooms.
//
// `project` is the caller's buffer-space projection; `unproject` its inverse,
// needed to work out which tiles the buffer covers at all.
struct Mosaic {
    int painted = 0;   // 0 → fall back to the drawn map
    int pending = 0;   // still in flight, so this buffer is worth redrawing
};

Mosaic draw_tile_mosaic(cv::Mat& dst, TileSource& tiles, double m_per_px,
                        double centre_lat,
                        const std::function<cv::Point2d(double, double)>& project,
                        const std::function<void(double, double, double&, double&)>& unproject)
{
    // Pick the zoom whose pixels are closest to the panel's own, one level in
    // reserve: sharper is cheap to downscale, blurrier can never be recovered.
    int z = 1;
    for (int cand = 1; cand <= tiles.max_zoom(); ++cand) {
        z = cand;
        if (tile_resolution_m(cand, centre_lat) <= m_per_px) break;
    }

    // The tile range the buffer covers, from its own four corners. Corners
    // rather than the centre plus a radius: under rotation the covered region
    // is a rotated rectangle, and a radius would fetch a circle around it.
    double min_tx = 1e18, max_tx = -1e18, min_ty = 1e18, max_ty = -1e18;
    const double cx[4] = {0.0, static_cast<double>(dst.cols), 0.0,
                          static_cast<double>(dst.cols)};
    const double cy[4] = {0.0, 0.0, static_cast<double>(dst.rows),
                          static_cast<double>(dst.rows)};
    for (int i = 0; i < 4; ++i) {
        double lat = 0.0, lon = 0.0;
        unproject(cx[i], cy[i], lat, lon);
        double tx = 0.0, ty = 0.0;
        lat_lon_to_tile(lat, lon, z, tx, ty);
        min_tx = std::min(min_tx, tx); max_tx = std::max(max_tx, tx);
        min_ty = std::min(min_ty, ty); max_ty = std::max(max_ty, ty);
    }

    const int x0 = static_cast<int>(std::floor(min_tx));
    const int x1 = static_cast<int>(std::floor(max_tx));
    const int y0 = static_cast<int>(std::floor(min_ty));
    const int y1 = static_cast<int>(std::floor(max_ty));
    // A corner that unprojected somewhere absurd (no fix, a pathological zoom)
    // must not turn into a million-tile loop.
    if (static_cast<int64_t>(x1 - x0 + 1) * (y1 - y0 + 1) > 400) return {};

    Mosaic result;
    for (int ty = y0; ty <= y1; ++ty) {
        for (int tx = x0; tx <= x1; ++tx) {
            TileSource::State state = TileSource::State::Missing;
            const cv::Mat tile = tiles.get(z, tx, ty, &state);
            if (state == TileSource::State::Pending) ++result.pending;
            if (tile.empty()) continue;

            // Three corners are enough to pin an affine down; the fourth would
            // over-determine it.
            //
            // The source corners are pulled half a pixel inside the tile while
            // the destination corners stay where the projection puts them, which
            // stretches each tile that half pixel past its own edge. Without it
            // the outermost sample of every tile falls on the border and
            // BORDER_TRANSPARENT leaves it alone — a dark one-pixel grid over
            // the whole mosaic, exactly where the tile seams are.
            constexpr float kBleed = 0.5f;
            double lat = 0.0, lon = 0.0;
            cv::Point2f src_pt[3] = {
                {kBleed, kBleed},
                {static_cast<float>(tile.cols) - kBleed, kBleed},
                {kBleed, static_cast<float>(tile.rows) - kBleed}};
            cv::Point2f dst_pt[3];
            const int corner_tx[3] = {tx, tx + 1, tx};
            const int corner_ty[3] = {ty, ty, ty + 1};
            for (int i = 0; i < 3; ++i) {
                tile_to_lat_lon(corner_tx[i], corner_ty[i], z, lat, lon);
                const cv::Point2d p = project(lat, lon);
                dst_pt[i] = cv::Point2f(static_cast<float>(p.x), static_cast<float>(p.y));
            }

            // Where this tile lands, so the warp writes into a few hundred
            // pixels instead of the whole buffer once per tile.
            const cv::Point2f d3(dst_pt[1].x + dst_pt[2].x - dst_pt[0].x,
                                 dst_pt[1].y + dst_pt[2].y - dst_pt[0].y);
            float lo_x = dst_pt[0].x, hi_x = dst_pt[0].x;
            float lo_y = dst_pt[0].y, hi_y = dst_pt[0].y;
            for (const cv::Point2f& p : {dst_pt[1], dst_pt[2], d3}) {
                lo_x = std::min(lo_x, p.x); hi_x = std::max(hi_x, p.x);
                lo_y = std::min(lo_y, p.y); hi_y = std::max(hi_y, p.y);
            }
            cv::Rect roi(static_cast<int>(std::floor(lo_x)) - 1,
                         static_cast<int>(std::floor(lo_y)) - 1,
                         static_cast<int>(std::ceil(hi_x - lo_x)) + 3,
                         static_cast<int>(std::ceil(hi_y - lo_y)) + 3);
            roi &= cv::Rect(0, 0, dst.cols, dst.rows);
            if (roi.width <= 0 || roi.height <= 0) continue;

            cv::Mat m = cv::getAffineTransform(src_pt, dst_pt);
            m.at<double>(0, 2) -= roi.x;
            m.at<double>(1, 2) -= roi.y;

            cv::Mat out = dst(roi);
            // BORDER_TRANSPARENT: the parts of the ROI this tile does not cover
            // belong to its neighbours, and must survive the warp.
            cv::warpAffine(tile, out, m, roi.size(), cv::INTER_LINEAR,
                           cv::BORDER_TRANSPARENT);
            ++result.painted;
        }
    }
    return result;
}

}  // namespace

bool panel_px_to_lat_lon(const PanelProjection& p, double px, double py,
                         double& lat, double& lon)
{
    if (!p.valid || p.m_per_px <= 0.0) return false;
    if (py < p.map_top || py > p.map_bottom) return false;

    // Undo the projection render_map_panel applied: screen offset → rotated
    // metres → east/north → degrees.
    const double rx = (px - p.ego_x) * p.m_per_px;
    const double ry = (p.ego_y - py) * p.m_per_px;
    const double east  =  rx * p.cr + ry * p.sr;
    const double north = -rx * p.sr + ry * p.cr;

    lat = p.ref_lat + north / kEarthR * 180.0 / M_PI;
    lon = p.ref_lon + east / (kEarthR * std::max(0.2, std::cos(p.ref_lat * M_PI / 180.0))) *
                          180.0 / M_PI;
    return true;
}

cv::Mat render_map_panel(const OsmMap& map, const MapView& view, int width, int height,
                         PanelProjection* out_proj)
{
    set_theme(view.dark);
    cv::Mat panel(height, width, CV_8UC3, kBackground);

    // kManeuverH is the arrow/distance part; the destination strip sits under it.
    const int banner_body = view.route && view.route->valid ? kManeuverH + 30 : 0;
    // The inset is part of the banner's footprint: the map has to start below
    // both, or the turn arrow ends up underneath the search field.
    const int banner_top = banner_body > 0 ? view.top_inset : 0;
    const int banner_h   = banner_body > 0 ? banner_top + banner_body : 0;
    const int footer_h = 74;
    const int map_top    = banner_h;
    const int map_bottom = height - footer_h;
    const int map_h      = std::max(1, map_bottom - map_top);

    // ── Projection: metres → panel pixels, ego near the lower third ──────────
    const double m_per_px = std::max(1e-3, view.zoom_m / std::max(1, width));
    // The pan offset moves the whole projection, so a drag shifts the map by
    // exactly the pixels the hand moved regardless of rotation or zoom.
    const cv::Point ego_px(width / 2 + static_cast<int>(view.pan_x),
                           map_top + static_cast<int>(map_h * 0.62) +
                               static_cast<int>(view.pan_y));
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

    if (out_proj) {
        out_proj->valid      = view.fix.has_value() && map.loaded();
        out_proj->ref_lat    = ref_lat;
        out_proj->ref_lon    = ref_lon;
        out_proj->m_per_px   = m_per_px;
        out_proj->cr         = cr;
        out_proj->sr         = sr;
        out_proj->ego_x      = ego_px.x;
        out_proj->ego_y      = ego_px.y;
        out_proj->map_top    = map_top;
        out_proj->map_bottom = map_bottom;
    }

    // Imagery is only ever a background: with no tiles yet the drawn map has to
    // come back, or the panel goes blank the moment the network does.
    bool imagery_drawn = false;

    if (view.fix && map.loaded()) {
        // ── Basemap: drawn into an oversized buffer, reused between frames ───
        const int BW = width + 2 * kBasemapMargin;
        const int BH = map_h + 2 * kBasemapMargin;
        const size_t fp = route_fingerprint(view.route);

        // Re-draw when the view genuinely changed, or when the drag has taken
        // the window near the edge of what the buffer covers.
        const bool drifted = std::abs(view.pan_x - g_basemap.pan_x) > kBasemapMargin - 60 ||
                             std::abs(view.pan_y - g_basemap.pan_y) > kBasemapMargin - 60;
        // Tiles arrive after the frame that asked for them, so a buffer drawn
        // with some still in flight has to be drawn again to pick them up.
        // Two conditions, and both matter: that buffer was actually missing
        // tiles (a square the server does not have never arrives, so the queue
        // is the wrong thing to watch), and a tile has landed since it was
        // drawn. The 150 ms floor only coalesces bursts — it can never lose the
        // last tile, because the served count stays different until the redraw
        // that consumes it.
        const int64 now_tick = cv::getTickCount();
        const bool filling =
            view.satellite && view.tiles && g_basemap.tiles_pending > 0 &&
            view.tiles->served() != g_basemap.served_at &&
            (now_tick - g_basemap.redrawn_at) >
                static_cast<int64>(cv::getTickFrequency() * 0.15);
        const bool stale = !g_basemap.valid || g_basemap.w != BW || g_basemap.h != BH ||
                           g_basemap.ref_lat != ref_lat || g_basemap.ref_lon != ref_lon ||
                           g_basemap.rot != rot || g_basemap.m_per_px != m_per_px ||
                           g_basemap.route_fp != fp || g_basemap.dark != view.dark ||
                           g_basemap.satellite != view.satellite ||
                           g_basemap.sat_roads != view.sat_roads ||
                           g_basemap.sat_road_opacity != view.sat_road_opacity ||
                           drifted || filling;

        if (stale) {
            if (g_basemap.img.rows != BH || g_basemap.img.cols != BW)
                g_basemap.img.create(BH, BW, CV_8UC3);
            g_basemap.img.setTo(kBackground);

            // Ego's position inside the buffer for this pan. When the map has
            // been dragged a long way it lands outside the buffer, which is
            // right: the window is then looking somewhere the car is not.
            const double bx = kBasemapMargin + width / 2.0 + view.pan_x;
            const double by = kBasemapMargin + map_h * 0.62 + view.pan_y;

            auto bproject = [&](double lat, double lon) {
                double e, n;
                lat_lon_to_en(lat, lon, ref_lat, ref_lon, e, n);
                const double rx = e * cr - n * sr;
                const double ry = e * sr + n * cr;
                return cv::Point(
                    static_cast<int>(std::clamp(bx + rx / m_per_px, -1e5, 1e5)),
                    static_cast<int>(std::clamp(by - ry / m_per_px, -1e5, 1e5)));
            };

            // Query around the middle of the buffer, not around the car: after a
            // drag those are different places, and querying the car's
            // neighbourhood would leave the dragged-to area blank.
            const double cx_off = (BW / 2.0) - bx;
            const double cy_off = (BH / 2.0) - by;
            const double rx_m = cx_off * m_per_px;
            const double ry_m = -cy_off * m_per_px;
            const double e_c = rx_m * cr + ry_m * sr;
            const double n_c = -rx_m * sr + ry_m * cr;
            const double c_lat = ref_lat + n_c / kEarthR * 180.0 / M_PI;
            const double c_lon = ref_lon + e_c /
                                 (kEarthR * std::max(0.2, std::cos(ref_lat * M_PI / 180.0)))
                                 * 180.0 / M_PI;

            // ── Aerial imagery, under everything the map draws ───────────────
            Mosaic mosaic;
            if (view.satellite && view.tiles) {
                auto bunproject = [&](double px, double py, double& lat, double& lon) {
                    const double rx = (px - bx) * m_per_px;
                    const double ry = (by - py) * m_per_px;
                    const double e =  rx * cr + ry * sr;
                    const double n = -rx * sr + ry * cr;
                    lat = ref_lat + n / kEarthR * 180.0 / M_PI;
                    lon = ref_lon + e / (kEarthR * std::max(0.2, std::cos(ref_lat * M_PI / 180.0))) *
                                        180.0 / M_PI;
                };
                mosaic = draw_tile_mosaic(
                    g_basemap.img, *view.tiles, m_per_px, ref_lat,
                    [&](double lat, double lon) {
                        double e, n;
                        lat_lon_to_en(lat, lon, ref_lat, ref_lon, e, n);
                        return cv::Point2d(bx + (e * cr - n * sr) / m_per_px,
                                           by - (e * sr + n * cr) / m_per_px);
                    },
                    bunproject);

                // Night: knock the imagery back so the amber route and the green
                // ego read against it instead of competing with a sunlit city.
                if (mosaic.painted > 0 && view.dark)
                    g_basemap.img.convertTo(g_basemap.img, -1, 0.78, -6);
            }
            const bool over_imagery = mosaic.painted > 0;

            const double reach_m = std::hypot(BW, BH) * 0.5 * m_per_px + 100.0;
            const double dlat = reach_m / kEarthR * 180.0 / M_PI;
            const double dlon = dlat / std::max(0.2, std::cos(c_lat * M_PI / 180.0));

            const auto ids = map.ways_in_bbox(c_lat - dlat, c_lon - dlon,
                                              c_lat + dlat, c_lon + dlon);

            // Two passes so arterials are not cut by the streets crossing them:
            // casings first, then the fills.
            //
            // Over imagery the same network is drawn, at the same widths and in
            // the same order — that is the point of it: the picture shows where
            // the tarmac is, these lines show where the map thinks it is, and
            // the route, the map match and the speed limit all come from the
            // map. Seeing the two together is what makes the panel precise. The
            // only difference is that the whole overlay goes down into a copy
            // and is blended back, so the photograph is dimmed by it rather than
            // replaced.
            const bool blend_roads = over_imagery && view.sat_road_opacity < 0.99;
            cv::Mat road_layer;
            cv::Mat& road_dst = blend_roads ? (road_layer = g_basemap.img.clone())
                                            : g_basemap.img;
            const bool major_only = over_imagery && view.sat_roads == RoadOverlay::Major;
            const bool no_roads   = over_imagery && view.sat_roads == RoadOverlay::Off;

            std::vector<cv::Point> pts;
            for (int pass = 0; pass < 2 && !no_roads; ++pass) {
                for (const uint32_t wi : ids) {
                    const Way& w = map.ways()[wi];
                    if (major_only && w.road_class > kTertiary) continue;
                    const RoadStyle st = style_for(w.road_class, (w.flags & kLink) != 0);
                    pts.clear();
                    pts.reserve(w.nodes.size());
                    for (const uint32_t ni : w.nodes) {
                        const Node& n = map.nodes()[ni];
                        pts.push_back(bproject(n.lat(), n.lon()));
                    }
                    if (pts.size() < 2) continue;
                    if (pass == 0)
                        draw_polyline_clipped(road_dst, pts, kCasing,
                                              cvRound(st.width) + 3);
                    else
                        draw_polyline_clipped(road_dst, pts, st.color,
                                              std::max(1, cvRound(st.width)));
                }
            }
            if (blend_roads && !no_roads) {
                const double a = std::clamp(view.sat_road_opacity, 0.0, 1.0);
                cv::addWeighted(road_layer, a, g_basemap.img, 1.0 - a, 0.0, g_basemap.img);
            }

            if (view.route && view.route->valid) {
                pts.clear();
                pts.reserve(view.route->nodes.size());
                for (const uint32_t ni : view.route->nodes) {
                    const Node& n = map.nodes()[ni];
                    pts.push_back(bproject(n.lat(), n.lon()));
                }
                if (pts.size() >= 2) {
                    draw_polyline_clipped(g_basemap.img, pts, cv::Scalar(40, 60, 90), 11);
                    draw_polyline_clipped(g_basemap.img, pts, kRouteLine, 6);
                }
                const cv::Point d = bproject(view.route->dest_lat, view.route->dest_lon);
                if (cv::Rect(0, 0, BW, BH).contains(d)) {
                    cv::circle(g_basemap.img, d, 8, cv::Scalar(60, 60, 240), -1, cv::LINE_AA);
                    cv::circle(g_basemap.img, d, 8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                }
            }

            g_basemap.valid    = true;
            g_basemap.dark     = view.dark;
            g_basemap.w        = BW;
            g_basemap.h        = BH;
            g_basemap.ref_lat  = ref_lat;
            g_basemap.ref_lon  = ref_lon;
            g_basemap.rot      = rot;
            g_basemap.m_per_px = m_per_px;
            g_basemap.route_fp = fp;
            g_basemap.pan_x    = view.pan_x;
            g_basemap.pan_y    = view.pan_y;
            g_basemap.satellite     = view.satellite;
            g_basemap.sat_roads     = view.sat_roads;
            g_basemap.sat_road_opacity = view.sat_road_opacity;
            g_basemap.tiles_painted = mosaic.painted;
            g_basemap.tiles_pending = mosaic.pending;
            g_basemap.served_at     = view.tiles ? view.tiles->served() : -1;
            g_basemap.redrawn_at    = now_tick;
        }
        imagery_drawn = g_basemap.satellite && g_basemap.tiles_painted > 0;

        // Blit the window of the buffer this pan is looking at — the whole point
        // of the cache: a drag is a memcpy, not a redraw.
        const int sx = kBasemapMargin - static_cast<int>(view.pan_x - g_basemap.pan_x);
        const int sy = kBasemapMargin - static_cast<int>(view.pan_y - g_basemap.pan_y);
        const cv::Rect src(std::clamp(sx, 0, std::max(0, BW - width)),
                           std::clamp(sy, 0, std::max(0, BH - map_h)),
                           width, map_h);
        g_basemap.img(src).copyTo(panel(cv::Rect(0, map_top, width, map_h)));

        // The unfiltered receiver, when comparing. Drawn first so the fused
        // marker sits on top: the gap between the two *is* the measurement —
        // how far the raw fix jumps while the fused one stays on the road.
        if (view.raw_fix) {
            const double dn = (view.raw_fix->latitude - view.fix->latitude) *
                              111132.0;
            const double de = (view.raw_fix->longitude - view.fix->longitude) *
                              111320.0 * std::cos(view.fix->latitude * M_PI / 180.0);
            double rx = de, ry = dn;
            if (view.heading_up && view.fix->has_bearing) {
                const double a = -view.fix->bearing_deg * M_PI / 180.0;
                const double nx = rx * std::cos(a) - ry * std::sin(a);
                const double ny = rx * std::sin(a) + ry * std::cos(a);
                rx = nx; ry = ny;
            }
            const cv::Point raw_px(ego_px.x + static_cast<int>(rx / m_per_px),
                                   ego_px.y - static_cast<int>(ry / m_per_px));
            // Two markers of equal weight, so neither looks like the "real" one:
            // red is the bare receiver, green is the fused estimate. The line
            // between them is the disagreement, which is the whole measurement.
            // No label — at this size the text lands on top of both of them.
            cv::line(panel, ego_px, raw_px, cv::Scalar(110, 110, 180), 1, cv::LINE_AA);
            cv::circle(panel, raw_px, 11, cv::Scalar(70, 70, 190), -1, cv::LINE_AA);
            cv::circle(panel, raw_px, 11, cv::Scalar(235, 235, 245), 2, cv::LINE_AA);
            cv::circle(panel, raw_px, 4, cv::Scalar(235, 235, 245), -1, cv::LINE_AA);
        }

        {
            const bool known = view.fix->has_bearing;
            // Heading-up already rotated the world, so the marker points up the
            // screen; north-up leaves the world alone and turns the marker.
            const double ego_rot = (!known || view.heading_up)
                                       ? 0.0
                                       : view.fix->bearing_deg * M_PI / 180.0;
            draw_ego(panel, ego_px, ego_rot, known,
                     view.fix->accuracy_m > 0.0 ? view.fix->accuracy_m / m_per_px : 0.0);
        }

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

        // Whose imagery this is. Every provider requires the credit, and it is
        // also the fastest way to see which one a run actually used.
        if (imagery_drawn && view.tiles) {
            const std::string credit = view.tiles->attribution();
            int bl = 0;
            const cv::Size ts = cv::getTextSize(ascii(credit), kFont, 0.34, 1, &bl);
            text(panel, ascii(credit), cv::Point(width - ts.width - 10, map_bottom - 8),
                 0.34, kTextDim);
        } else if (view.satellite && view.tiles) {
            // Say it plainly rather than leaving a drawn map that looks like the
            // imagery was never asked for.
            text(panel, view.tiles->pending() > 0 ? "loading imagery..."
                                                  : "no imagery - drawn map",
                 cv::Point(14, map_bottom - 34), 0.38, kWarn);
        }
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
            text(panel, "Route paused - no position", cv::Point(16, banner_top + 40), 0.52, kWarn);
        } else if (idx >= 0 && idx < static_cast<int>(r.steps.size())) {
            const Step& s = r.steps[idx];
            draw_maneuver(panel, s.type, cv::Point(52, banner_top + kManeuverH / 2), 46, kTextMain);
            text(panel, format_distance(view.progress.distance_to_step_m),
                 cv::Point(104, banner_top + 40), 0.78, kTextMain, 2);
            const std::string what = s.road.empty()
                                         ? maneuver_text(s.type)
                                         : std::string(maneuver_text(s.type)) + " onto " + s.road;
            text(panel, elide(what, width - 118, 0.46), cv::Point(104, banner_top + 68), 0.46, kTextDim);
        }

        // Destination strip along the bottom of the banner.
        cv::line(panel, cv::Point(12, banner_top + kManeuverH),
                 cv::Point(width - 12, banner_top + kManeuverH),
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
    // inside the panel it would be a postage stamp.
    if (!view.input_active && view.map_loaded) {
        // Top-left, clear of the scale bar that lives in the bottom-left.
        text(panel, "click  set destination      d  search      wheel  zoom      t  light/dark",
             cv::Point(12, map_top + 20), 0.36, kTextDim);
        if (view.panned)
            text(panel, "r  recentre", cv::Point(12, map_top + 40), 0.4, kWarn);
    }

    cv::rectangle(panel, cv::Rect(0, 0, width, height), kPanelEdge, 1);
    return panel;
}

// ─── Destination box ─────────────────────────────────────────────────────────
// Drawn over the entire window rather than inside the map panel: while it is
// open it owns the keyboard, and it should own the screen to match.
void draw_destination_box(cv::Mat& canvas, const MapView& view)
{
    if (!view.input_active || canvas.empty()) return;
    set_theme(view.dark);

    const int W = canvas.cols, H = canvas.rows;
    fill_rect_alpha(canvas, cv::Rect(0, 0, W, H),
                    g_dark ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255), 0.62);

    const int rows   = std::max<int>(1, static_cast<int>(view.input_hits.size()));
    const int row_h  = 46;
    const int box_w  = std::min(860, W - 120);
    const int box_h  = std::min(H - 100, 168 + rows * row_h);
    const cv::Rect box((W - box_w) / 2, std::max(40, H / 2 - box_h / 2), box_w, box_h);

    cv::rectangle(canvas, box, kBanner, -1);
    cv::rectangle(canvas, box, kDest, 2, cv::LINE_AA);

    text(canvas, "Where to?", cv::Point(box.x + 22, box.y + 42), 0.78, kDest, 2);

    // Entry line with a blinking caret.
    const cv::Rect entry(box.x + 22, box.y + 60, box.width - 44, 46);
    cv::rectangle(canvas, entry, g_dark ? cv::Scalar(16, 15, 14)
                                        : cv::Scalar(255, 255, 255), -1);
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
