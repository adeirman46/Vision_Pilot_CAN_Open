#ifndef VISIONPILOT_MAP_RENDERER_HPP
#define VISIONPILOT_MAP_RENDERER_HPP

#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include <navigation/gps_interface.hpp>
#include <navigation/osm_map.hpp>
#include <navigation/router.hpp>

namespace navigation {

// Everything the panel draws, gathered by Navigator so the renderer stays pure.
struct MapView {
    std::optional<GpsFix> fix;
    bool        map_loaded   = false;
    std::string status;             // shown when there is no fix yet
    std::string source_name;

    MapMatch    match;              // current road
    std::string road_name;
    uint8_t     matched_class = kUnknown;   // labels the road when it has no name
    int         speed_limit_kph = 0;
    bool        speed_inferred  = false;

    const Route*  route    = nullptr;
    RouteProgress progress;
    bool          routing  = false;   // a route is being computed right now

    double zoom_m     = 400.0;
    bool   heading_up = true;

    // Destination box, typed into the video window itself.
    bool                       input_active = false;
    std::string                input_text;
    std::vector<OsmMap::Place> input_hits;
    int                        input_sel  = 0;
    std::string                input_note;
};

// Draws the navigation panel: roads from the offline extract, the active route,
// the ego marker, and the next-manoeuvre banner. Returns a BGR image of exactly
// (width × height).
cv::Mat render_map_panel(const OsmMap& map, const MapView& view,
                         int width, int height);

// Draws the destination search box over the whole window. No-op unless
// view.input_active. Called after the panel is composed onto the canvas.
void draw_destination_box(cv::Mat& canvas, const MapView& view);

}  // namespace navigation

#endif  // VISIONPILOT_MAP_RENDERER_HPP
