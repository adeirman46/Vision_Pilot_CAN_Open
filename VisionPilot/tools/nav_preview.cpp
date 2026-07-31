// nav_preview — render one navigation panel to a PNG.
//
// Checks a freshly built .vpmap without a phone, a camera or a GPU:
//
//   ./nav_preview --map ../maps/bandung-roads.vpmap --at -6.9175,107.6191 \
//                 --dest -6.9034,107.6181 -o /tmp/nav.png
//
// Prints the matched road and, with --dest, the route summary and its steps.
#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include <navigation/map_renderer.hpp>
#include <navigation/osm_map.hpp>
#include <navigation/router.hpp>

namespace nav = navigation;

namespace {

bool parse_lat_lon(const std::string& s, double& lat, double& lon)
{
    const auto comma = s.find(',');
    if (comma == std::string::npos) return false;
    lat = std::atof(s.substr(0, comma).c_str());
    lon = std::atof(s.substr(comma + 1).c_str());
    return true;
}

void usage()
{
    std::printf(
        "usage: nav_preview --map <file.vpmap> --at <lat,lon> [options]\n"
        "       nav_preview --map <file.vpmap> --find <text> [--at <lat,lon>]\n"
        "  --find <text>      search road names in the extract and exit. With --at,\n"
        "                     the nearest match is listed first. Machine-readable:\n"
        "                     \"<lat>,<lon>\\t<name>\\t<km>\" per line.\n"
        "  --dest <lat,lon>   route to here and draw it\n"
        "  --heading <deg>    ego heading, 0 = north (default 0)\n"
        "  --accuracy <m>     fix accuracy; widens the map match like Navigator does\n"
        "  --zoom <m>         metres across the panel (default 400)\n"
        "  --size <WxH>       panel size in px (default 512x512)\n"
        "  --north-up         north-up instead of heading-up\n"
        "  -o <file.png>      output image (default nav_preview.png)\n");
}

}  // namespace

int main(int argc, char** argv)
{
    std::string map_path, out = "nav_preview.png", find_query, dest_query;
    double lat = 0.0, lon = 0.0, dest_lat = 0.0, dest_lon = 0.0;
    double heading = 0.0, zoom = 400.0, accuracy = 60.0;
    int w = 512, h = 512;
    bool have_at = false, have_dest = false, heading_up = true;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--map")            map_path = next();
        else if (a == "--at")        have_at = parse_lat_lon(next(), lat, lon);
        else if (a == "--find")      find_query = next();
        else if (a == "--dest") {
            // A destination is either coordinates or a street name to look up.
            const std::string v = next();
            have_dest = parse_lat_lon(v, dest_lat, dest_lon);
            if (!have_dest) dest_query = v;
        }
        else if (a == "--heading")   heading = std::atof(next().c_str());
        else if (a == "--accuracy")  accuracy = std::atof(next().c_str());
        else if (a == "--zoom")      zoom = std::atof(next().c_str());
        else if (a == "--north-up")  heading_up = false;
        else if (a == "-o")          out = next();
        else if (a == "--size") {
            const std::string s = next();
            const auto x = s.find('x');
            if (x != std::string::npos) {
                w = std::atoi(s.substr(0, x).c_str());
                h = std::atoi(s.substr(x + 1).c_str());
            }
        } else { usage(); return 1; }
    }

    if (map_path.empty() || w < 160 || h < 160) { usage(); return 1; }
    if (find_query.empty() && !have_at) { usage(); return 1; }

    nav::OsmMap map;
    if (!map.load(map_path)) {
        std::fprintf(stderr, "error: %s\n", map.error().c_str());
        return 1;
    }

    // ── Search mode ──────────────────────────────────────────────────────────
    if (!find_query.empty()) {
        const auto hits = map.search(find_query, 15,
                                     have_at ? &lat : nullptr,
                                     have_at ? &lon : nullptr);
        if (hits.empty()) {
            std::fprintf(stderr, "no road matching \"%s\" in %s\n",
                         find_query.c_str(), map_path.c_str());
            return 1;
        }
        for (const auto& p : hits)
            std::printf("%.6f,%.6f\t%s\t%.1f\n", p.lat, p.lon, p.name.c_str(),
                        p.length_m / 1000.0);
        return 0;
    }

    // A named destination resolves through the same search, nearest first.
    if (!dest_query.empty()) {
        const auto hits = map.search(dest_query, 1, &lat, &lon);
        if (hits.empty()) {
            std::fprintf(stderr, "error: no road matching \"%s\"\n", dest_query.c_str());
            return 1;
        }
        dest_lat = hits[0].lat;
        dest_lon = hits[0].lon;
        have_dest = true;
        std::printf("destination: %s (%.6f, %.6f)\n",
                    hits[0].name.c_str(), dest_lat, dest_lon);
    }
    if (!map.contains(lat, lon))
        std::fprintf(stderr, "warning: %.5f,%.5f is outside the extract\n", lat, lon);

    nav::GpsFix fix;
    fix.latitude    = lat;
    fix.longitude   = lon;
    fix.bearing_deg = heading;
    fix.has_bearing = true;
    fix.speed_ms    = 10.0;
    fix.accuracy_m  = accuracy;

    nav::MapView view;
    view.fix        = fix;
    view.map_loaded = true;
    view.source_name = "preview";
    view.zoom_m     = zoom;
    view.heading_up = heading_up;

    // Same rule as Navigator::update(): never match tighter than the fix.
    const double reach = std::clamp(accuracy, 60.0, 200.0);
    view.match = map.match(lat, lon, reach, &heading);
    if (view.match.valid) {
        const nav::Way& way = map.ways()[view.match.way];
        view.road_name       = map.way_name(way);
        view.matched_class   = way.road_class;
        view.speed_limit_kph = way.speed_kph;
        view.speed_inferred  = (way.flags & nav::kSpeedInferred) != 0;
        std::printf("on: %s (class %u, %u km/h%s, %.1f m off centre)\n",
                    view.road_name.empty() ? "<unnamed>" : view.road_name.c_str(),
                    way.road_class, way.speed_kph,
                    view.speed_inferred ? ", inferred" : "", view.match.offset_m);
    } else {
        std::printf("on: no road within %.0f m\n", reach);
    }

    nav::Route route;
    if (have_dest) {
        route = nav::compute_route(map, lat, lon, dest_lat, dest_lon, &heading);
        if (!route.valid) {
            std::fprintf(stderr, "error: no route to %.5f,%.5f\n", dest_lat, dest_lon);
            return 1;
        }
        view.route    = &route;
        view.progress = nav::track_route(map, route, lat, lon, fix.speed_ms);
        std::printf("route: %.2f km, %.0f min, %zu steps\n",
                    route.length_m / 1000.0, route.duration_s / 60.0, route.steps.size());
        for (size_t i = 0; i < route.steps.size() && i < 12; ++i) {
            const nav::Step& s = route.steps[i];
            std::printf("  %6.0f m  %-12s %s\n", s.distance_from_start_m,
                        nav::maneuver_text(s.type), s.road.c_str());
        }
        if (route.steps.size() > 12) std::printf("  ... %zu more\n", route.steps.size() - 12);
    }

    const cv::Mat panel = nav::render_map_panel(map, view, w, h);
    if (!cv::imwrite(out, panel)) {
        std::fprintf(stderr, "error: cannot write %s\n", out.c_str());
        return 1;
    }
    std::printf("wrote %s (%dx%d)\n", out.c_str(), panel.cols, panel.rows);
    return 0;
}
