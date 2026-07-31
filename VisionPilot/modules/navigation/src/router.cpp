#include <navigation/router.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include <logging/logger.hpp>

namespace navigation {

namespace {

constexpr double kNoCost = std::numeric_limits<double>::infinity();

double norm180(double d)
{
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

Maneuver classify(double turn_deg)
{
    const double t = norm180(turn_deg);
    if (std::abs(t) > 160.0) return Maneuver::UTurn;
    if (t >  110.0) return Maneuver::SharpRight;
    if (t >   35.0) return Maneuver::Right;
    if (t >   15.0) return Maneuver::SlightRight;
    if (t < -110.0) return Maneuver::SharpLeft;
    if (t <  -35.0) return Maneuver::Left;
    if (t <  -15.0) return Maneuver::SlightLeft;
    return Maneuver::Straight;
}

}  // namespace

const char* maneuver_text(Maneuver m)
{
    switch (m) {
    case Maneuver::Depart:      return "Head";
    case Maneuver::Straight:    return "Continue";
    case Maneuver::SlightLeft:  return "Slight left";
    case Maneuver::Left:        return "Turn left";
    case Maneuver::SharpLeft:   return "Sharp left";
    case Maneuver::SlightRight: return "Slight right";
    case Maneuver::Right:       return "Turn right";
    case Maneuver::SharpRight:  return "Sharp right";
    case Maneuver::UTurn:       return "U-turn";
    case Maneuver::Arrive:      return "Arrive";
    }
    return "Continue";
}

// ─── Dijkstra ────────────────────────────────────────────────────────────────
// Travel time is the metric, so the route prefers a longer arterial over a
// shortcut through kampung lanes — which is what a driver wants and what the
// per-class speeds in the extract encode.
Route compute_route(const OsmMap& map,
                    double from_lat, double from_lon,
                    double to_lat, double to_lon,
                    const double* from_heading)
{
    Route route;
    if (!map.loaded()) return route;

    const MapMatch start = map.match(from_lat, from_lon, 200.0, from_heading);
    const MapMatch goal  = map.match(to_lat, to_lon, 500.0);
    if (!start.valid || !goal.valid) {
        VP_WARN("[Nav] route: %s not on the map extract",
                start.valid ? "destination" : "current position");
        return route;
    }

    const uint32_t src = start.nearest_node;
    const uint32_t dst = goal.nearest_node;

    const size_t n = map.nodes().size();
    std::vector<double>   dist(n, kNoCost);
    std::vector<uint32_t> prev(n, std::numeric_limits<uint32_t>::max());
    std::vector<uint32_t> prev_way(n, std::numeric_limits<uint32_t>::max());
    std::vector<double>   len(n, 0.0);

    using Item = std::pair<double, uint32_t>;   // (cost, node)
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;

    dist[src] = 0.0;
    pq.emplace(0.0, src);

    while (!pq.empty()) {
        const auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u] + 1e-9) continue;
        if (u == dst) break;

        for (const auto& e : map.edges(u)) {
            const double nd = d + e.cost_s;
            if (nd + 1e-9 >= dist[e.to]) continue;
            dist[e.to]     = nd;
            prev[e.to]     = u;
            prev_way[e.to] = e.way;
            len[e.to]      = len[u] + e.length_m;
            pq.emplace(nd, e.to);
        }
    }

    if (dist[dst] == kNoCost) {
        VP_WARN("[Nav] route: no drivable path to %.5f,%.5f", to_lat, to_lon);
        return route;
    }

    // ── Walk the parents back to the start ───────────────────────────────────
    std::vector<uint32_t> nodes, ways;
    for (uint32_t at = dst; at != src; at = prev[at]) {
        nodes.push_back(at);
        ways.push_back(prev_way[at]);
        if (prev[at] == std::numeric_limits<uint32_t>::max()) return route;   // broken chain
    }
    nodes.push_back(src);
    std::reverse(nodes.begin(), nodes.end());
    std::reverse(ways.begin(), ways.end());   // ways[i] connects nodes[i] → nodes[i+1]

    route.valid      = true;
    route.nodes      = std::move(nodes);
    route.length_m   = len[dst];
    route.duration_s = dist[dst];
    route.dest_lat   = to_lat;
    route.dest_lon   = to_lon;

    // ── Cumulative distance ──────────────────────────────────────────────────
    route.cumulative_m.assign(route.nodes.size(), 0.0);
    for (size_t i = 1; i < route.nodes.size(); ++i)
        route.cumulative_m[i] = route.cumulative_m[i - 1] +
                                map.node_distance_m(route.nodes[i - 1], route.nodes[i]);
    route.length_m = route.cumulative_m.back();

    // ── Turn-by-turn steps ───────────────────────────────────────────────────
    // A step is emitted where the road name changes or where the heading swings
    // by more than a lane-change's worth, which is close enough to what a driver
    // recognises as "a turn" without pulling in OSM junction relations.
    const auto& nodes_v = map.nodes();
    auto node_ll = [&](uint32_t idx, double& la, double& lo) {
        la = nodes_v[idx].lat();
        lo = nodes_v[idx].lon();
    };

    {
        Step s;
        s.type = Maneuver::Depart;
        s.node = route.nodes[0];
        node_ll(route.nodes[0], s.lat, s.lon);
        if (!ways.empty()) s.road = map.way_name(map.ways()[ways[0]]);
        route.steps.push_back(s);
    }

    for (size_t i = 1; i + 1 < route.nodes.size(); ++i) {
        const uint32_t way_in  = ways[i - 1];
        const uint32_t way_out = ways[i];
        const std::string& name_in  = map.way_name(map.ways()[way_in]);
        const std::string& name_out = map.way_name(map.ways()[way_out]);

        double la_p, lo_p, la_c, lo_c, la_n, lo_n;
        node_ll(route.nodes[i - 1], la_p, lo_p);
        node_ll(route.nodes[i],     la_c, lo_c);
        node_ll(route.nodes[i + 1], la_n, lo_n);

        const double turn = norm180(bearing_deg(la_c, lo_c, la_n, lo_n) -
                                    bearing_deg(la_p, lo_p, la_c, lo_c));
        const Maneuver m = classify(turn);

        const bool named_change = !name_out.empty() && name_out != name_in;
        if (m == Maneuver::Straight && !named_change) continue;
        if (m == Maneuver::Straight && named_change &&
            !route.steps.empty() && route.steps.back().road == name_out) continue;

        // Fold consecutive geometry nodes of the same bend into one step.
        if (!route.steps.empty() &&
            route.cumulative_m[i] - route.steps.back().distance_from_start_m < 25.0 &&
            route.steps.back().type == m)
            continue;

        Step s;
        s.type = m;
        s.node = route.nodes[i];
        s.lat  = la_c;
        s.lon  = lo_c;
        s.distance_from_start_m = route.cumulative_m[i];
        s.road = name_out;
        route.steps.push_back(s);
    }

    {
        double la, lo;
        node_ll(route.nodes.back(), la, lo);
        Step s;
        s.type = Maneuver::Arrive;
        s.node = route.nodes.back();
        s.lat  = la;
        s.lon  = lo;
        s.distance_from_start_m = route.length_m;
        route.steps.push_back(s);
    }

    VP_INFO("[Nav] route: %.1f km, %.0f min, %zu steps",
            route.length_m / 1000.0, route.duration_s / 60.0, route.steps.size());
    return route;
}

// ─── Progress ────────────────────────────────────────────────────────────────

RouteProgress track_route(const OsmMap& map, const Route& route,
                          double lat, double lon, double speed_ms)
{
    RouteProgress p;
    if (!route.valid || route.nodes.size() < 2) return p;

    const auto& nodes = map.nodes();

    // Closest route vertex, then refine onto the two segments touching it.
    size_t best_i = 0;
    double best_d = 1e18;
    for (size_t i = 0; i < route.nodes.size(); ++i) {
        const Node& n = nodes[route.nodes[i]];
        double e, nn;
        lat_lon_to_en(n.lat(), n.lon(), lat, lon, e, nn);
        const double d = e * e + nn * nn;
        if (d < best_d) { best_d = d; best_i = i; }
    }

    double travelled = route.cumulative_m[best_i];
    double offset    = std::sqrt(best_d);

    const size_t lo_i = best_i > 0 ? best_i - 1 : 0;
    const size_t hi_i = std::min(best_i + 1, route.nodes.size() - 1);
    for (size_t i = lo_i; i < hi_i; ++i) {
        const Node& a = nodes[route.nodes[i]];
        const Node& b = nodes[route.nodes[i + 1]];
        double ax, ay, bx, by;
        lat_lon_to_en(a.lat(), a.lon(), lat, lon, ax, ay);
        lat_lon_to_en(b.lat(), b.lon(), lat, lon, bx, by);

        const double dx = bx - ax, dy = by - ay;
        const double l2 = dx * dx + dy * dy;
        if (l2 < 1e-9) continue;
        double t = std::clamp((-ax * dx - ay * dy) / l2, 0.0, 1.0);
        const double d = std::hypot(ax + t * dx, ay + t * dy);
        if (d < offset) {
            offset    = d;
            travelled = route.cumulative_m[i] +
                        t * (route.cumulative_m[i + 1] - route.cumulative_m[i]);
        }
    }

    p.nearest_index = static_cast<uint32_t>(best_i);
    p.offset_m      = offset;
    p.travelled_m   = travelled;
    p.remaining_m   = std::max(0.0, route.length_m - travelled);
    p.on_route      = offset < 50.0;
    // Blend the measured speed with the route's own estimate so the ETA does not
    // run away while stopped at a light.
    {
        const double v = std::max(4.0, speed_ms);
        p.eta_s = p.remaining_m / v;
        if (route.length_m > 1.0)
            p.eta_s = 0.5 * p.eta_s +
                      0.5 * route.duration_s * (p.remaining_m / route.length_m);
    }

    for (size_t s = 0; s < route.steps.size(); ++s) {
        if (route.steps[s].distance_from_start_m > travelled + 5.0) {
            p.next_step          = static_cast<int>(s);
            p.distance_to_step_m = route.steps[s].distance_from_start_m - travelled;
            break;
        }
    }
    if (p.next_step < 0 && !route.steps.empty()) {
        p.next_step          = static_cast<int>(route.steps.size()) - 1;
        p.distance_to_step_m = p.remaining_m;
    }
    return p;
}

}  // namespace navigation
