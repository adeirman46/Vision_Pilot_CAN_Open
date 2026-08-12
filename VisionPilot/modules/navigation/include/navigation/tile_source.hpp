#ifndef VISIONPILOT_TILE_SOURCE_HPP
#define VISIONPILOT_TILE_SOURCE_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/core.hpp>

namespace navigation {

// ─── Web Mercator ────────────────────────────────────────────────────────────
// The projection every slippy-tile server uses. Kept here rather than in
// osm_map because nothing but the imagery needs it: the panel itself works in a
// local tangent plane, which is both simpler and more accurate over the few
// kilometres it ever shows.
constexpr int kTilePx = 256;

void lat_lon_to_tile(double lat, double lon, int z, double& tx, double& ty);
void tile_to_lat_lon(double tx, double ty, int z, double& lat, double& lon);
// Metres per tile pixel at this zoom and latitude — how the renderer picks the
// zoom level that matches the panel's own scale.
double tile_resolution_m(int z, double lat);

// ─── Satellite / raster basemap ──────────────────────────────────────────────
// Serves 256×256 tiles to the renderer from a memory cache, backed by a disk
// cache, backed by the tile server. Everything the renderer touches is
// non-blocking: a miss returns an empty Mat and queues the fetch, so a slow or
// absent network costs frames nothing and the imagery fills in as it arrives.
//
// The disk cache is what makes this usable in a car: the first drive down a
// street downloads it, every drive after that needs no network at all. Point
// `cache_dir` at a directory that survives reboots and the map goes offline by
// itself.
class TileSource
{
public:
    struct Options {
        // Tile URL with {z}/{x}/{y} placeholders. Empty = disk cache only.
        std::string url =
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}";
        std::string cache_dir;        // empty = memory only
        // Printed in the corner of the panel. Every imagery provider requires
        // it, and it is also the quickest way to see which one is in use.
        std::string attribution = "Imagery (c) Esri";
        int  max_zoom     = 19;
        int  memory_tiles = 384;      // ~75 MB decoded at 256×256 BGR
        int  workers      = 4;        // parallel downloads
        int  timeout_s    = 8;
        bool offline      = false;    // never touch the network
    };

    explicit TileSource(Options opt);
    ~TileSource();

    TileSource(const TileSource&) = delete;
    TileSource& operator=(const TileSource&) = delete;

    // Why a tile is not here. "Pending" is worth redrawing for, "Missing" is
    // not: a square the server does not have will not appear by asking again,
    // and a renderer that cannot tell the two apart either gives up on tiles
    // still in flight or repaints for ever waiting on ones that will never come.
    enum class State { Ready, Pending, Missing };

    // The tile, or an empty Mat when it has not arrived yet. Never blocks and
    // never throws: the caller draws what it has and the rest lands later.
    cv::Mat get(int z, int x, int y, State* state = nullptr);

    int  max_zoom() const { return opt_.max_zoom; }
    // False until the first tile has ever been decoded — the honest way to say
    // "there is no imagery here" instead of drawing an empty grey panel.
    bool has_imagery() const { return served_ > 0; }
    // Tiles decoded since startup. A renderer watches this rather than the
    // queue: it changes exactly when there is something new to draw.
    int  served() const { return served_; }
    int  pending() const;
    // Tiles that failed and are waiting out their retry, for the status line.
    int  failed() const { return failed_; }
    const std::string& attribution() const { return opt_.attribution; }

private:
    struct Entry {
        cv::Mat  img;              // empty = known-missing
        uint64_t used_at = 0;      // LRU stamp
        uint64_t fail_at = 0;      // when the fetch last failed, 0 = never
    };

    static uint64_t key(int z, int x, int y)
    {
        return (static_cast<uint64_t>(z) << 58) |
               (static_cast<uint64_t>(x) << 29) | static_cast<uint64_t>(y);
    }

    std::string disk_path(int z, int x, int y) const;
    std::string tile_url(int z, int x, int y) const;
    void        worker_loop();
    void        evict_locked();

    Options opt_;

    mutable std::mutex                    mtx_;
    std::unordered_map<uint64_t, Entry>   cache_;
    std::deque<uint64_t>                  queue_;
    std::unordered_set<uint64_t>          inflight_;
    std::condition_variable               cv_;
    uint64_t                              clock_ = 0;

    std::vector<std::thread> workers_;
    std::atomic<bool>        shutdown_{false};
    std::atomic<int>         served_{0};
    std::atomic<int>         failed_{0};
};

}  // namespace navigation

#endif  // VISIONPILOT_TILE_SOURCE_HPP
