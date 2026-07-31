#ifndef VISIONPILOT_ROUTER_HPP
#define VISIONPILOT_ROUTER_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <navigation/osm_map.hpp>

namespace navigation {

enum class Maneuver : uint8_t {
    Depart = 0,
    Straight,
    SlightLeft,
    Left,
    SharpLeft,
    SlightRight,
    Right,
    SharpRight,
    UTurn,
    Arrive,
};

const char* maneuver_text(Maneuver m);

struct Step {
    Maneuver    type = Maneuver::Straight;
    uint32_t    node = 0;             // where it happens
    double      lat = 0.0, lon = 0.0;
    double      distance_from_start_m = 0.0;
    std::string road;                 // the road being *entered*
};

struct Route {
    bool                  valid = false;
    std::vector<uint32_t> nodes;              // full geometry, start → destination
    std::vector<double>   cumulative_m;       // distance along the route per node
    std::vector<Step>     steps;
    double                length_m   = 0.0;
    double                duration_s = 0.0;
    double                dest_lat = 0.0, dest_lon = 0.0;
    std::string           dest_name;

    void clear() { *this = Route{}; }
};

// Where the ego sits along a computed route.
struct RouteProgress {
    bool     on_route          = false;
    double   offset_m          = 0.0;   // lateral distance to the route line
    double   travelled_m       = 0.0;
    double   remaining_m       = 0.0;
    double   eta_s             = 0.0;
    int      next_step         = -1;    // index into Route::steps, -1 = none
    double   distance_to_step_m = 0.0;
    uint32_t nearest_index     = 0;     // index into Route::nodes
};

// Dijkstra over the map graph, minimising travel time. One call is a few tens of
// milliseconds on the Bandung extract, which is why Navigator runs it off the
// inference thread.
Route compute_route(const OsmMap& map,
                    double from_lat, double from_lon,
                    double to_lat, double to_lon,
                    const double* from_heading_deg = nullptr);

// Project the current position onto the route. Cheap enough to run per frame.
RouteProgress track_route(const OsmMap& map, const Route& route,
                          double lat, double lon, double speed_ms);

}  // namespace navigation

#endif  // VISIONPILOT_ROUTER_HPP
