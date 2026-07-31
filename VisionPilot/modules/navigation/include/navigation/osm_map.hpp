#ifndef VISIONPILOT_OSM_MAP_HPP
#define VISIONPILOT_OSM_MAP_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace navigation {

// ─── Geodesy ─────────────────────────────────────────────────────────────────
constexpr double kEarthR = 6371000.0;   // m

// Local tangent plane around (ref_lat, ref_lon): east/north offsets in metres.
// Good to centimetres over the few kilometres a map panel ever shows.
void lat_lon_to_en(double lat, double lon, double ref_lat, double ref_lon,
                   double& east_m, double& north_m);
double haversine_m(double lat1, double lon1, double lat2, double lon2);
double bearing_deg(double lat1, double lon1, double lat2, double lon2);

// ─── Road classes (mirrors osm_to_vpmap.py) ──────────────────────────────────
enum RoadClass : uint8_t {
    kUnknown      = 0,
    kMotorway     = 1,
    kTrunk        = 2,
    kPrimary      = 3,
    kSecondary    = 4,
    kTertiary     = 5,
    kResidential  = 6,
    kUnclassified = 7,
    kLivingStreet = 8,
    // Drivable but not public through-roads: yard roads, complex access roads,
    // parking aisles. openpilot's mapd drops these because it only wants speed
    // limits off public roads — but they are exactly the streets inside a
    // factory site or a housing complex, and without them the map goes blank
    // right where the car actually is.
    kService      = 9,
    kRoadClassMax = 10,
};

enum WayFlags : uint8_t {
    kOnewayForward = 1 << 0,   // travel allowed only in node order
    kOnewayReverse = 1 << 1,   // oneway=-1, travel allowed only against node order
    kSpeedInferred = 1 << 2,   // maxspeed absent — value is a per-class default
    kLink          = 1 << 3,   // *_link: ramp / slip road
};

struct Node {
    int32_t lat_e7 = 0;
    int32_t lon_e7 = 0;

    double lat() const { return lat_e7 * 1e-7; }
    double lon() const { return lon_e7 * 1e-7; }
};

struct Way {
    uint8_t  road_class = kUnknown;
    uint8_t  flags      = 0;
    uint16_t speed_kph  = 0;          // 0 = unknown
    uint32_t name_idx   = 0;          // index into OsmMap::names(), 0 = unnamed
    std::vector<uint32_t> nodes;      // indices into OsmMap::nodes()

    bool oneway_forward() const { return flags & kOnewayForward; }
    bool oneway_reverse() const { return flags & kOnewayReverse; }
};

// Where on the road network a coordinate falls.
struct MapMatch {
    bool     valid       = false;
    uint32_t way         = 0;
    uint32_t segment     = 0;      // index of the segment start within the way
    uint32_t nearest_node = 0;     // node index, for routing
    double   offset_m    = 0.0;    // perpendicular distance from the way
    double   lat = 0.0, lon = 0.0; // the snapped point
};

// ─── The offline Bandung extract ─────────────────────────────────────────────
// Loads the compact .vpmap built by scripts/build_osm_map.sh, and answers the
// two questions the panel and the router ask: what is near this point, and what
// road am I on. Everything is read once into memory (~10 MB for Bandung) — there
// is no query thread and no network.
class OsmMap
{
public:
    OsmMap() = default;

    // Returns false and fills error() when the file is missing or malformed.
    bool load(const std::string& path);
    bool loaded() const { return loaded_; }
    const std::string& error() const { return error_; }
    const std::string& path() const { return path_; }

    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<Way>&  ways()  const { return ways_; }
    const std::string& way_name(const Way& w) const { return names_[w.name_idx]; }

    double min_lat() const { return min_lat_e7_ * 1e-7; }
    double min_lon() const { return min_lon_e7_ * 1e-7; }
    double max_lat() const { return max_lat_e7_ * 1e-7; }
    double max_lon() const { return max_lon_e7_ * 1e-7; }
    bool contains(double lat, double lon) const;

    // Way indices whose geometry may touch the bbox. Conservative: the caller
    // still clips. Returned in ascending order, no duplicates.
    std::vector<uint32_t> ways_in_bbox(double min_lat, double min_lon,
                                       double max_lat, double max_lon) const;

    // A road found by name. The extract carries 10k street names, which is
    // enough to pick a destination without a geocoder or an internet connection.
    struct Place {
        std::string name;
        double      lat = 0.0, lon = 0.0;   // midpoint of the longest way so named
        uint8_t     road_class = kUnknown;
        double      length_m = 0.0;
        // A named destination (mall, hospital, station) rather than a road.
        // These are what a driver actually asks for by name.
        bool        is_poi = false;
        std::string kind;                   // "mall", "hospital", ...
    };

    // Case-insensitive substring search over road names, best match first:
    // exact, then prefix, then longest road. `near_lat/near_lon`, when given,
    // prefers the nearest match — "Jalan Merdeka" exists in several places.
    std::vector<Place> search(const std::string& query, size_t limit = 10,
                              const double* near_lat = nullptr,
                              const double* near_lon = nullptr) const;

    // Closest drivable way within max_dist_m. When bearing is given (deg, 0 =
    // north) ways pointing the other way are penalised, which is what stops a
    // dual carriageway from matching the opposite side.
    MapMatch match(double lat, double lon, double max_dist_m = 60.0,
                   const double* heading_deg = nullptr) const;

    // Nodes adjacent to `node` along the ways that contain it, honouring oneway.
    // Used by the router; cheap because the adjacency is built once on load.
    struct Edge {
        uint32_t to;
        uint32_t way;
        float    length_m;
        float    cost_s;      // length / class speed — the routing metric
    };
    struct EdgeRange {
        const Edge* first = nullptr;
        const Edge* last  = nullptr;
        const Edge* begin() const { return first; }
        const Edge* end()   const { return last; }
    };
    EdgeRange edges(uint32_t node) const;

    double node_distance_m(uint32_t a, uint32_t b) const;

private:
    void build_index();
    void build_graph();
    // Totals each road name once at load, so search() is a scan of 10k names
    // rather than 126k ways — fast enough to run on every keystroke.
    void build_name_index();

    bool        loaded_ = false;
    std::string error_;
    std::string path_;

    std::vector<Node>        nodes_;
    std::vector<Way>         ways_;
    std::vector<std::string> names_;

    int32_t min_lat_e7_ = 0, min_lon_e7_ = 0, max_lat_e7_ = 0, max_lon_e7_ = 0;

    // Uniform grid over the bounds: cell → way indices.
    struct Poi {
        int32_t     lat_e7 = 0, lon_e7 = 0;
        std::string name;
        std::string kind;
    };
    std::vector<Poi> pois_;

    struct NameAgg {
        double   total_m  = 0.0;
        uint32_t best_way = 0;
        bool     has_way  = false;
    };
    std::vector<NameAgg> name_agg_;   // parallel to names_

    int    grid_w_ = 0, grid_h_ = 0;
    double cell_lat_ = 0.0, cell_lon_ = 0.0;
    std::vector<std::vector<uint32_t>> cells_;

    // CSR adjacency, node → edges.
    std::vector<uint32_t> edge_start_;
    std::vector<Edge>     edges_;
};

}  // namespace navigation

#endif  // VISIONPILOT_OSM_MAP_HPP
