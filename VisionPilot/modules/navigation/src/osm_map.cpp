#include <navigation/osm_map.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <unordered_map>

#include <logging/logger.hpp>

namespace navigation {

namespace {

// VPMAP02 appends the places table; VPMAP01 files still load, just with no
// searchable destinations in them.
constexpr char kMagic[8]   = {'V', 'P', 'M', 'A', 'P', '0', '2', '\n'};
constexpr char kMagicV1[8] = {'V', 'P', 'M', 'A', 'P', '0', '1', '\n'};
constexpr double kTargetCellM = 500.0;
constexpr int    kMaxGridDim  = 1024;

// Fallback travel speeds when the extract carries no maxspeed, km/h.
// Only used for routing cost, never shown as a limit.
constexpr double kClassSpeed[kRoadClassMax] = {
    40.0,   // unknown
    80.0,   // motorway
    60.0,   // trunk
    50.0,   // primary
    45.0,   // secondary
    40.0,   // tertiary
    30.0,   // residential
    35.0,   // unclassified
    20.0,   // living_street
    15.0,   // service — drivable, but never the fast way through
};

template <typename T>
bool read_pod(std::ifstream& f, T& out)
{
    f.read(reinterpret_cast<char*>(&out), sizeof(T));
    return static_cast<bool>(f);
}

double norm_deg(double d)
{
    d = std::fmod(d, 360.0);
    return d < 0.0 ? d + 360.0 : d;
}

double angle_diff(double a, double b)
{
    double d = std::abs(norm_deg(a) - norm_deg(b));
    return d > 180.0 ? 360.0 - d : d;
}

// Perpendicular distance from p to segment ab, all in metres, plus where along
// the segment the foot falls (0..1).
double point_segment_m(double px, double py, double ax, double ay,
                       double bx, double by, double& t)
{
    const double dx = bx - ax, dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-9) { t = 0.0; return std::hypot(px - ax, py - ay); }
    t = ((px - ax) * dx + (py - ay) * dy) / len2;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

}  // namespace

// ─── Geodesy ─────────────────────────────────────────────────────────────────

void lat_lon_to_en(double lat, double lon, double ref_lat, double ref_lon,
                   double& east_m, double& north_m)
{
    const double lat_r = ref_lat * M_PI / 180.0;
    north_m = (lat - ref_lat) * M_PI / 180.0 * kEarthR;
    east_m  = (lon - ref_lon) * M_PI / 180.0 * kEarthR * std::cos(lat_r);
}

double haversine_m(double lat1, double lon1, double lat2, double lon2)
{
    const double p = M_PI / 180.0;
    const double dlat = (lat2 - lat1) * p;
    const double dlon = (lon2 - lon1) * p;
    const double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                     std::cos(lat1 * p) * std::cos(lat2 * p) *
                     std::sin(dlon / 2) * std::sin(dlon / 2);
    return 2.0 * kEarthR * std::asin(std::min(1.0, std::sqrt(a)));
}

double bearing_deg(double lat1, double lon1, double lat2, double lon2)
{
    const double p = M_PI / 180.0;
    const double y = std::sin((lon2 - lon1) * p) * std::cos(lat2 * p);
    const double x = std::cos(lat1 * p) * std::sin(lat2 * p) -
                     std::sin(lat1 * p) * std::cos(lat2 * p) * std::cos((lon2 - lon1) * p);
    return norm_deg(std::atan2(y, x) * 180.0 / M_PI);
}

// ─── Loading ─────────────────────────────────────────────────────────────────

bool OsmMap::load(const std::string& path)
{
    loaded_ = false;
    error_.clear();
    path_ = path;

    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error_ = "cannot open map " + path + " — build it with scripts/build_osm_map.sh";
        return false;
    }

    char magic[8] = {};
    f.read(magic, sizeof(magic));
    const bool v2 = f && std::memcmp(magic, kMagic, sizeof(magic)) == 0;
    const bool v1 = f && std::memcmp(magic, kMagicV1, sizeof(magic)) == 0;
    if (!v1 && !v2) {
        error_ = path + " is not a VPMAP file (rebuild it with scripts/build_osm_map.sh)";
        return false;
    }

    uint32_t node_count = 0, way_count = 0, name_count = 0;
    if (!read_pod(f, min_lat_e7_) || !read_pod(f, min_lon_e7_) ||
        !read_pod(f, max_lat_e7_) || !read_pod(f, max_lon_e7_) ||
        !read_pod(f, node_count) || !read_pod(f, way_count) || !read_pod(f, name_count)) {
        error_ = path + ": truncated header";
        return false;
    }

    nodes_.resize(node_count);
    f.read(reinterpret_cast<char*>(nodes_.data()),
           static_cast<std::streamsize>(node_count) * sizeof(Node));
    if (!f) { error_ = path + ": truncated node table"; return false; }

    names_.resize(name_count);
    for (uint32_t i = 0; i < name_count; ++i) {
        uint16_t len = 0;
        if (!read_pod(f, len)) { error_ = path + ": truncated name table"; return false; }
        names_[i].resize(len);
        if (len) f.read(names_[i].data(), len);
        if (!f) { error_ = path + ": truncated name table"; return false; }
    }
    if (names_.empty()) names_.emplace_back();

    ways_.resize(way_count);
    for (uint32_t i = 0; i < way_count; ++i) {
        Way& w = ways_[i];
        uint32_t pts = 0;
        if (!read_pod(f, w.road_class) || !read_pod(f, w.flags) ||
            !read_pod(f, w.speed_kph) || !read_pod(f, w.name_idx) || !read_pod(f, pts)) {
            error_ = path + ": truncated way table";
            return false;
        }
        if (w.name_idx >= names_.size()) w.name_idx = 0;
        if (w.road_class >= kRoadClassMax) w.road_class = kUnknown;
        w.nodes.resize(pts);
        if (pts) f.read(reinterpret_cast<char*>(w.nodes.data()),
                        static_cast<std::streamsize>(pts) * sizeof(uint32_t));
        if (!f) { error_ = path + ": truncated way geometry"; return false; }
    }

    // Places: present from VPMAP02 on. A truncated table is not fatal — the map
    // still draws and routes, you just cannot search destinations by name.
    if (v2) {
        uint32_t place_count = 0;
        if (read_pod(f, place_count)) {
            pois_.reserve(place_count);
            for (uint32_t i = 0; i < place_count; ++i) {
                Poi p;
                uint16_t name_len = 0;
                uint8_t  kind_len = 0;
                if (!read_pod(f, p.lat_e7) || !read_pod(f, p.lon_e7) ||
                    !read_pod(f, name_len))
                    break;
                p.name.resize(name_len);
                if (name_len) f.read(p.name.data(), name_len);
                if (!read_pod(f, kind_len)) break;
                p.kind.resize(kind_len);
                if (kind_len) f.read(p.kind.data(), kind_len);
                if (!f) break;
                pois_.push_back(std::move(p));
            }
        }
    }

    build_index();
    build_graph();
    build_name_index();
    loaded_ = true;

    VP_INFO("[Nav] map %s: %u ways, %u nodes, %zu places, bbox %.4f,%.4f .. %.4f,%.4f",
            path.c_str(), way_count, node_count, pois_.size(),
            min_lat(), min_lon(), max_lat(), max_lon());
    return true;
}

bool OsmMap::contains(double lat, double lon) const
{
    return loaded_ && lat >= min_lat() && lat <= max_lat() &&
           lon >= min_lon() && lon <= max_lon();
}

void OsmMap::build_index()
{
    const double span_lat = std::max(1e-6, max_lat() - min_lat());
    const double span_lon = std::max(1e-6, max_lon() - min_lon());
    const double mid_lat  = (min_lat() + max_lat()) * 0.5;

    cell_lat_ = kTargetCellM / kEarthR * 180.0 / M_PI;
    cell_lon_ = cell_lat_ / std::max(0.2, std::cos(mid_lat * M_PI / 180.0));

    grid_h_ = std::clamp(static_cast<int>(span_lat / cell_lat_) + 1, 1, kMaxGridDim);
    grid_w_ = std::clamp(static_cast<int>(span_lon / cell_lon_) + 1, 1, kMaxGridDim);
    // Re-derive the cell size from the clamped dimensions so the grid still
    // covers the whole extract when it is much larger than Bandung.
    cell_lat_ = span_lat / grid_h_;
    cell_lon_ = span_lon / grid_w_;

    cells_.assign(static_cast<size_t>(grid_w_) * grid_h_, {});

    auto cell_of = [&](const Node& n, int& cx, int& cy) {
        cx = std::clamp(static_cast<int>((n.lon() - min_lon()) / cell_lon_), 0, grid_w_ - 1);
        cy = std::clamp(static_cast<int>((n.lat() - min_lat()) / cell_lat_), 0, grid_h_ - 1);
    };

    for (uint32_t wi = 0; wi < ways_.size(); ++wi) {
        const Way& w = ways_[wi];
        for (size_t i = 0; i + 1 < w.nodes.size(); ++i) {
            const Node& a = nodes_[w.nodes[i]];
            const Node& b = nodes_[w.nodes[i + 1]];
            int ax, ay, bx, by;
            cell_of(a, ax, ay);
            cell_of(b, bx, by);
            // Segments are short relative to a 500 m cell, so covering the
            // segment's cell bbox is enough to never miss a crossing.
            for (int cy = std::min(ay, by); cy <= std::max(ay, by); ++cy)
                for (int cx = std::min(ax, bx); cx <= std::max(ax, bx); ++cx) {
                    // Ways are indexed in order, so a repeat is always the tail.
                    auto& bucket = cells_[static_cast<size_t>(cy) * grid_w_ + cx];
                    if (bucket.empty() || bucket.back() != wi) bucket.push_back(wi);
                }
        }
        if (w.nodes.size() == 1) {
            int cx, cy;
            cell_of(nodes_[w.nodes[0]], cx, cy);
            cells_[static_cast<size_t>(cy) * grid_w_ + cx].push_back(wi);
        }
    }
}

void OsmMap::build_graph()
{
    std::vector<uint32_t> degree(nodes_.size() + 1, 0);

    auto for_each_edge = [&](auto&& emit) {
        for (uint32_t wi = 0; wi < ways_.size(); ++wi) {
            const Way& w = ways_[wi];
            const bool fwd_only = w.oneway_forward();
            const bool rev_only = w.oneway_reverse();
            for (size_t i = 0; i + 1 < w.nodes.size(); ++i) {
                const uint32_t a = w.nodes[i], b = w.nodes[i + 1];
                if (a == b || a >= nodes_.size() || b >= nodes_.size()) continue;
                if (!rev_only) emit(a, b, wi);
                if (!fwd_only) emit(b, a, wi);
            }
        }
    };

    for_each_edge([&](uint32_t a, uint32_t, uint32_t) { ++degree[a]; });

    edge_start_.assign(nodes_.size() + 1, 0);
    for (size_t i = 0; i < nodes_.size(); ++i)
        edge_start_[i + 1] = edge_start_[i] + degree[i];
    edges_.resize(edge_start_.back());

    std::vector<uint32_t> cursor(edge_start_.begin(), edge_start_.end() - 1);
    for_each_edge([&](uint32_t a, uint32_t b, uint32_t wi) {
        const Way& w = ways_[wi];
        const double len = node_distance_m(a, b);
        const double kph = w.speed_kph > 0 ? w.speed_kph : kClassSpeed[w.road_class];
        Edge& e = edges_[cursor[a]++];
        e.to       = b;
        e.way      = wi;
        e.length_m = static_cast<float>(len);
        e.cost_s   = static_cast<float>(len / std::max(5.0, kph) * 3.6);
    });
}

void OsmMap::build_name_index()
{
    name_agg_.assign(names_.size(), NameAgg{});
    for (uint32_t wi = 0; wi < ways_.size(); ++wi) {
        const Way& w = ways_[wi];
        if (w.name_idx == 0 || w.name_idx >= name_agg_.size()) continue;
        double len = 0.0;
        for (size_t i = 0; i + 1 < w.nodes.size(); ++i)
            len += node_distance_m(w.nodes[i], w.nodes[i + 1]);
        NameAgg& a = name_agg_[w.name_idx];
        a.total_m += len;
        // The longest single piece is the one worth driving to; a road is split
        // into dozens of ways and any one of them is a valid target.
        if (!a.has_way || len > 0.0) {
            if (!a.has_way) { a.best_way = wi; a.has_way = true; }
            else {
                double best_len = 0.0;
                const Way& b = ways_[a.best_way];
                for (size_t i = 0; i + 1 < b.nodes.size(); ++i)
                    best_len += node_distance_m(b.nodes[i], b.nodes[i + 1]);
                if (len > best_len) a.best_way = wi;
            }
        }
    }
}

OsmMap::EdgeRange OsmMap::edges(uint32_t node) const
{
    if (node + 1 >= edge_start_.size()) return {};
    return {edges_.data() + edge_start_[node], edges_.data() + edge_start_[node + 1]};
}

double OsmMap::node_distance_m(uint32_t a, uint32_t b) const
{
    if (a >= nodes_.size() || b >= nodes_.size()) return 0.0;
    const Node& na = nodes_[a];
    const Node& nb = nodes_[b];
    return haversine_m(na.lat(), na.lon(), nb.lat(), nb.lon());
}

// ─── Queries ─────────────────────────────────────────────────────────────────

std::vector<uint32_t> OsmMap::ways_in_bbox(double min_la, double min_lo,
                                           double max_la, double max_lo) const
{
    std::vector<uint32_t> out;
    if (!loaded_ || cells_.empty()) return out;

    const int x0 = std::clamp(static_cast<int>((min_lo - min_lon()) / cell_lon_), 0, grid_w_ - 1);
    const int x1 = std::clamp(static_cast<int>((max_lo - min_lon()) / cell_lon_), 0, grid_w_ - 1);
    const int y0 = std::clamp(static_cast<int>((min_la - min_lat()) / cell_lat_), 0, grid_h_ - 1);
    const int y1 = std::clamp(static_cast<int>((max_la - min_lat()) / cell_lat_), 0, grid_h_ - 1);

    for (int cy = y0; cy <= y1; ++cy)
        for (int cx = x0; cx <= x1; ++cx) {
            const auto& bucket = cells_[static_cast<size_t>(cy) * grid_w_ + cx];
            out.insert(out.end(), bucket.begin(), bucket.end());
        }

    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<OsmMap::Place> OsmMap::search(const std::string& query, size_t limit,
                                          const double* near_lat,
                                          const double* near_lon) const
{
    std::vector<Place> out;
    if (!loaded_ || query.empty()) return out;

    auto lower = [](std::string s) {
        for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };
    const std::string q = lower(query);

    // Rank by how the query hit the name: whole name, start of it, anywhere.
    // "braga" should find "Jalan Braga" before "Jalan Braga Pendek".
    std::unordered_map<uint32_t, int> rank;
    for (uint32_t i = 1; i < names_.size(); ++i) {
        const std::string n = lower(names_[i]);
        const auto pos = n.find(q);
        if (pos == std::string::npos) continue;
        rank[i] = (n == q) ? 0 : (pos == 0 ? 1 : 2);
    }
    // A query may match only places, only roads, or both.
    bool any_poi = false;
    for (const auto& poi : pois_)
        if (lower(poi.name).find(q) != std::string::npos) { any_poi = true; break; }
    if (rank.empty() && !any_poi) return out;

    // Named destinations first-class alongside roads: "Trans Studio Mall" is
    // what gets typed, "Jalan Gatot Subroto" is only where it happens to be.
    for (const auto& poi : pois_) {
        const std::string n = lower(poi.name);
        if (n.find(q) == std::string::npos) continue;
        Place p;
        p.name   = poi.name;
        p.lat    = poi.lat_e7 * 1e-7;
        p.lon    = poi.lon_e7 * 1e-7;
        p.is_poi = true;
        p.kind   = poi.kind;
        out.push_back(std::move(p));
    }

    // Roads: the per-name totals were computed once at load.
    for (const auto& [name_idx, hit_rank] : rank) {
        (void)hit_rank;
        if (name_idx >= name_agg_.size()) continue;
        const NameAgg& a = name_agg_[name_idx];
        if (!a.has_way) continue;
        const Way& w = ways_[a.best_way];
        if (w.nodes.empty()) continue;
        const Node& mid = nodes_[w.nodes[w.nodes.size() / 2]];

        Place p;
        p.name       = names_[name_idx];
        p.lat        = mid.lat();
        p.lon        = mid.lon();
        p.road_class = w.road_class;
        p.length_m   = a.total_m;
        out.push_back(std::move(p));
    }

    std::sort(out.begin(), out.end(), [&](const Place& a, const Place& b) {
        // Names are unique per entry here, so look the rank back up by string.
        auto rank_of = [&](const Place& p) {
            const std::string n = lower(p.name);
            const auto pos = n.find(q);
            return (n == q) ? 0 : (pos == 0 ? 1 : 2);
        };
        const int ra = rank_of(a), rb = rank_of(b);
        if (ra != rb) return ra < rb;
        // At equal rank a real destination beats a street of the same name.
        if (a.is_poi != b.is_poi) return a.is_poi;
        // "Jalan Merdeka" exists in half a dozen towns — prefer the near one.
        if (near_lat && near_lon) {
            const double da = haversine_m(*near_lat, *near_lon, a.lat, a.lon);
            const double db = haversine_m(*near_lat, *near_lon, b.lat, b.lon);
            if (std::abs(da - db) > 500.0) return da < db;
        }
        return a.length_m > b.length_m;
    });

    if (out.size() > limit) out.resize(limit);
    return out;
}

MapMatch OsmMap::match(double lat, double lon, double max_dist_m,
                       const double* heading) const
{
    MapMatch best;
    if (!loaded_) return best;

    // Grow the search box until something is found, so a position on the edge of
    // a cell still matches the road just outside it.
    for (double reach = std::max(60.0, max_dist_m); reach <= max_dist_m * 8.0 + 1.0;
         reach *= 2.0) {
        const double dlat = reach / kEarthR * 180.0 / M_PI;
        const double dlon = dlat / std::max(0.2, std::cos(lat * M_PI / 180.0));
        const auto candidates = ways_in_bbox(lat - dlat, lon - dlon, lat + dlat, lon + dlon);

        double best_score = 1e18;
        for (const uint32_t wi : candidates) {
            const Way& w = ways_[wi];
            for (size_t i = 0; i + 1 < w.nodes.size(); ++i) {
                const Node& na = nodes_[w.nodes[i]];
                const Node& nb = nodes_[w.nodes[i + 1]];

                double ax, ay, bx, by;
                lat_lon_to_en(na.lat(), na.lon(), lat, lon, ax, ay);
                lat_lon_to_en(nb.lat(), nb.lon(), lat, lon, bx, by);

                double t = 0.0;
                const double d = point_segment_m(0.0, 0.0, ax, ay, bx, by, t);
                if (d > max_dist_m) continue;

                double score = d;
                if (heading) {
                    double seg = bearing_deg(na.lat(), na.lon(), nb.lat(), nb.lon());
                    double delta = angle_diff(*heading, seg);
                    if (!w.oneway_forward() && !w.oneway_reverse())
                        delta = std::min(delta, 180.0 - delta);
                    score += 0.35 * delta;
                }
                if (score < best_score) {
                    best_score        = score;
                    best.valid        = true;
                    best.way          = wi;
                    best.segment      = static_cast<uint32_t>(i);
                    best.nearest_node = t < 0.5 ? w.nodes[i] : w.nodes[i + 1];
                    best.offset_m     = d;
                    best.lat = na.lat() + (nb.lat() - na.lat()) * t;
                    best.lon = na.lon() + (nb.lon() - na.lon()) * t;
                }
            }
        }
        if (best.valid) break;
    }
    return best;
}

}  // namespace navigation
