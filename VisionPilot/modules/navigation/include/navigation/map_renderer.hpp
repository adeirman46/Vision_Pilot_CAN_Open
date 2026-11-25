#ifndef VISIONPILOT_MAP_RENDERER_HPP
#define VISIONPILOT_MAP_RENDERER_HPP

#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include <navigation/gps_interface.hpp>
#include <navigation/nav_config.hpp>
#include <navigation/osm_map.hpp>
#include <navigation/router.hpp>
#include <navigation/tile_source.hpp>

namespace navigation {

// Everything the panel draws, gathered by Navigator so the renderer stays pure.
struct MapView {
    std::optional<GpsFix> fix;
    // The receiver's own answer, drawn beside the fused one when comparing.
    std::optional<GpsFix> raw_fix;
    // Pixels at the top of the panel the front-end has already claimed for its
    // own widgets. The manoeuvre banner starts below this, so a search field
    // floating over the map cannot bury the next turn.
    int top_inset = 0;
    bool        map_loaded   = false;
    std::string status;             // shown when there is no fix yet
    std::string source_name;

    MapMatch    match;              // current road
    std::string road_name;
    uint8_t     matched_class = kUnknown;   // labels the road when it has no name
    int         speed_limit_kph = 0;
    bool        speed_inferred  = false;

    const Route*  route    = nullptr;
    // The mission behind that route, so the intermediate points can be drawn
    // and numbered. The route alone cannot show them: once the legs are
    // stitched it is one line, and where the driver clicked is no longer
    // recoverable from it. Owned by the caller; may be null.
    const std::vector<Waypoint>* waypoints = nullptr;
    RouteProgress progress;
    bool          routing  = false;   // a route is being computed right now

    double zoom_m     = 400.0;
    bool   heading_up = true;
    bool   dark       = true;   // false = light / daytime palette

    // Aerial imagery under the roads instead of the flat drawn land. `tiles`
    // stays owned by the caller and may be null, in which case the flag does
    // nothing — as it also does while no tile has arrived yet, so a map with no
    // network is the vector one rather than an empty grey rectangle.
    bool        satellite = false;
    TileSource* tiles     = nullptr;
    // How much of the road network to trace on top of the imagery, and how
    // solidly. Photography shows the tarmac; these lines show where the map
    // believes it is — which is what the route and the map-matching are built
    // on, so seeing both at once is what makes a satellite panel precise rather
    // than merely pretty.
    RoadOverlay sat_roads       = RoadOverlay::All;
    double      sat_road_opacity = 0.85;

    // Map dragged away from the car, in panel pixels.
    double pan_x = 0.0, pan_y = 0.0;
    bool   panned = false;

    // Destination box, typed into the video window itself.
    bool                       input_active = false;
    std::string                input_text;
    std::vector<OsmMap::Place> input_hits;
    int                        input_sel  = 0;
    std::string                input_note;
};

// The projection the panel was last drawn with. Published because the mapping
// from a coordinate to a pixel is built from half a dozen things — the zoom, the
// heading, the pan, how tall the manoeuvre banner happens to be this frame — and
// a front-end that wants to turn a click back into a coordinate must invert
// exactly the transform that was used, not a reconstruction of it.
struct PanelProjection {
    bool   valid    = false;
    double ref_lat  = 0.0, ref_lon = 0.0;   // the fix the panel was centred on
    double m_per_px = 1.0;
    double cr = 1.0, sr = 0.0;              // rotation, heading-up
    double ego_x = 0.0, ego_y = 0.0;        // where that fix landed, panel px
    int    map_top = 0, map_bottom = 0;     // the band that is actually map
};

// Panel pixel → coordinate. False when the panel has not been drawn yet or the
// pixel is in the banner or the footer rather than on the map.
bool panel_px_to_lat_lon(const PanelProjection& p, double px, double py,
                         double& lat, double& lon);

// Draws the navigation panel: roads from the offline extract, the active route,
// the ego marker, and the next-manoeuvre banner. Returns a BGR image of exactly
// (width × height). `out_proj`, when given, receives the projection used.
cv::Mat render_map_panel(const OsmMap& map, const MapView& view,
                         int width, int height,
                         PanelProjection* out_proj = nullptr);

// Draws the destination search box over the whole window. No-op unless
// view.input_active. Called after the panel is composed onto the canvas.
void draw_destination_box(cv::Mat& canvas, const MapView& view);

}  // namespace navigation

#endif  // VISIONPILOT_MAP_RENDERER_HPP
