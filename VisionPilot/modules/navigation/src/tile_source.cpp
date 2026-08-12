#include <navigation/tile_source.hpp>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <opencv2/imgcodecs.hpp>

#include <logging/logger.hpp>

namespace navigation {

namespace {

uint64_t now_ms()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

// A tile the server does not have is not a transient failure — asking again
// every frame would be a denial of service on ourselves. Wait this long before
// retrying, which is short enough that driving back into a gap that was only a
// dropped connection still fills in.
constexpr uint64_t kRetryMs = 45000;
// Panning fast queues tiles faster than they can arrive. Beyond this the oldest
// requests are dropped: they are for a part of the map nobody is looking at any
// more, and the visible ones are always the newest.
constexpr size_t kMaxQueue = 96;

bool mkdir_p(const std::string& path)
{
    if (path.empty()) return false;
    std::string acc;
    acc.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        acc.push_back(path[i]);
        if (path[i] != '/' && i + 1 != path.size()) continue;
        if (acc == "/" || acc.empty()) continue;
        if (::mkdir(acc.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }
    return true;
}

bool file_exists(const std::string& path)
{
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0 && st.st_size > 0;
}

void replace_all(std::string& s, const std::string& from, const std::string& to)
{
    for (size_t at = s.find(from); at != std::string::npos; at = s.find(from, at + to.size()))
        s.replace(at, from.size(), to);
}

// Fetch with the curl binary rather than a socket of our own. The tile servers
// worth using are all HTTPS, and http_get.cpp speaks plain HTTP — linking a TLS
// stack into the navigation module to download map squares is not a trade worth
// making. execvp, not system(): the URL comes from a config file and must never
// reach a shell.
bool curl_to_file(const std::string& url, const std::string& out, int timeout_s)
{
    const std::string tmo = std::to_string(timeout_s);
    char* const argv[] = {
        const_cast<char*>("curl"),
        const_cast<char*>("-s"),            // no progress meter
        const_cast<char*>("-f"),            // HTTP errors are failures
        const_cast<char*>("-L"),            // follow the provider's redirects
        const_cast<char*>("--max-time"), const_cast<char*>(tmo.c_str()),
        const_cast<char*>("-A"), const_cast<char*>("VisionPilot/1.0"),
        const_cast<char*>("-o"), const_cast<char*>(out.c_str()),
        const_cast<char*>(url.c_str()),
        nullptr,
    };

    const pid_t pid = ::fork();
    if (pid < 0) return false;
    if (pid == 0) {
        // Child: nothing it prints belongs in the driving log.
        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            ::dup2(devnull, STDOUT_FILENO);
            ::dup2(devnull, STDERR_FILENO);
            if (devnull > STDERR_FILENO) ::close(devnull);
        }
        ::execvp("curl", argv);
        ::_exit(127);
    }

    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

}  // namespace

// ─── Web Mercator ────────────────────────────────────────────────────────────

void lat_lon_to_tile(double lat, double lon, int z, double& tx, double& ty)
{
    const double n = std::pow(2.0, z);
    // Clamped to the square the projection covers; past ±85.05° it diverges.
    const double clamped = std::clamp(lat, -85.05112878, 85.05112878);
    const double lat_r = clamped * M_PI / 180.0;
    tx = (lon + 180.0) / 360.0 * n;
    ty = (1.0 - std::log(std::tan(lat_r) + 1.0 / std::cos(lat_r)) / M_PI) / 2.0 * n;
}

void tile_to_lat_lon(double tx, double ty, int z, double& lat, double& lon)
{
    const double n = std::pow(2.0, z);
    lon = tx / n * 360.0 - 180.0;
    lat = std::atan(std::sinh(M_PI * (1.0 - 2.0 * ty / n))) * 180.0 / M_PI;
}

double tile_resolution_m(int z, double lat)
{
    constexpr double kEquatorM = 156543.03392804097;   // m per px at z=0
    return kEquatorM * std::cos(std::clamp(lat, -85.0, 85.0) * M_PI / 180.0) /
           std::pow(2.0, z);
}

// ─── TileSource ──────────────────────────────────────────────────────────────

TileSource::TileSource(Options opt) : opt_(std::move(opt))
{
    opt_.max_zoom     = std::clamp(opt_.max_zoom, 1, 22);
    opt_.memory_tiles = std::max(64, opt_.memory_tiles);
    opt_.workers      = std::clamp(opt_.workers, 1, 8);
    opt_.timeout_s    = std::clamp(opt_.timeout_s, 2, 60);

    if (!opt_.cache_dir.empty() && !mkdir_p(opt_.cache_dir)) {
        VP_WARN("[Tiles] cannot create cache dir %s — imagery will not be kept",
                opt_.cache_dir.c_str());
        opt_.cache_dir.clear();
    }
    if (opt_.url.empty()) opt_.offline = true;

    for (int i = 0; i < opt_.workers; ++i)
        workers_.emplace_back(&TileSource::worker_loop, this);

    VP_INFO("[Tiles] %s, cache %s, max z%d",
            opt_.offline ? "offline (cache only)" : opt_.url.c_str(),
            opt_.cache_dir.empty() ? "(memory only)" : opt_.cache_dir.c_str(),
            opt_.max_zoom);
}

TileSource::~TileSource()
{
    shutdown_ = true;
    cv_.notify_all();
    for (auto& t : workers_)
        if (t.joinable()) t.join();
}

std::string TileSource::disk_path(int z, int x, int y) const
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "/%d/%d/%d.jpg", z, x, y);
    return opt_.cache_dir + buf;
}

std::string TileSource::tile_url(int z, int x, int y) const
{
    std::string u = opt_.url;
    replace_all(u, "{z}", std::to_string(z));
    replace_all(u, "{x}", std::to_string(x));
    replace_all(u, "{y}", std::to_string(y));
    return u;
}

int TileSource::pending() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<int>(queue_.size() + inflight_.size());
}

void TileSource::evict_locked()
{
    if (cache_.size() <= static_cast<size_t>(opt_.memory_tiles)) return;
    // Drop the oldest quarter in one pass rather than one entry per insert:
    // a full-panel redraw touches dozens of tiles and would otherwise walk the
    // whole map that many times.
    std::vector<std::pair<uint64_t, uint64_t>> ages;   // (used_at, key)
    ages.reserve(cache_.size());
    for (const auto& [k, e] : cache_) ages.emplace_back(e.used_at, k);
    const size_t drop = cache_.size() - static_cast<size_t>(opt_.memory_tiles) +
                        cache_.size() / 4;
    std::partial_sort(ages.begin(), ages.begin() + std::min(drop, ages.size()),
                      ages.end());
    for (size_t i = 0; i < drop && i < ages.size(); ++i) cache_.erase(ages[i].second);
}

cv::Mat TileSource::get(int z, int x, int y, State* state)
{
    auto report = [state](State s) { if (state) *state = s; };

    const int n = 1 << z;
    if (z < 0 || z > opt_.max_zoom || y < 0 || y >= n) {
        report(State::Missing);
        return {};
    }
    // Wrap in x so a view straddling the antimeridian still resolves; y has no
    // wrap, the projection simply ends there.
    x = ((x % n) + n) % n;

    const uint64_t k = key(z, x, y);
    std::unique_lock<std::mutex> lock(mtx_);

    const auto it = cache_.find(k);
    if (it != cache_.end()) {
        it->second.used_at = ++clock_;
        if (!it->second.img.empty()) {
            report(State::Ready);
            return it->second.img;
        }
        // A known miss: hold off until the retry window has passed.
        if (now_ms() - it->second.fail_at < kRetryMs) {
            report(State::Missing);
            return {};
        }
        cache_.erase(it);
    }

    report(State::Pending);
    if (inflight_.count(k)) return {};
    inflight_.insert(k);
    queue_.push_back(k);
    while (queue_.size() > kMaxQueue) {
        inflight_.erase(queue_.front());
        queue_.pop_front();
    }
    lock.unlock();
    cv_.notify_one();
    return {};
}

void TileSource::worker_loop()
{
    while (!shutdown_) {
        uint64_t k = 0;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [this] { return shutdown_ || !queue_.empty(); });
            if (shutdown_) return;
            // Newest first: a queue that has backed up is full of tiles for
            // where the map used to be, and the driver is looking at where it
            // is now.
            k = queue_.back();
            queue_.pop_back();
        }

        const int z = static_cast<int>(k >> 58);
        const int x = static_cast<int>((k >> 29) & 0x1FFFFFFF);
        const int y = static_cast<int>(k & 0x1FFFFFFF);

        cv::Mat img;
        const std::string path = opt_.cache_dir.empty() ? std::string()
                                                        : disk_path(z, x, y);

        if (!path.empty() && file_exists(path)) img = cv::imread(path, cv::IMREAD_COLOR);

        if (img.empty() && !opt_.offline) {
            if (path.empty()) {
                // No disk cache: fetch to a temporary and read it back. Still
                // cheaper than linking a TLS stack in.
                char tmp[] = "/tmp/vp_tile_XXXXXX";
                const int fd = ::mkstemp(tmp);
                if (fd >= 0) {
                    ::close(fd);
                    if (curl_to_file(tile_url(z, x, y), tmp, opt_.timeout_s))
                        img = cv::imread(tmp, cv::IMREAD_COLOR);
                    ::unlink(tmp);
                }
            } else {
                mkdir_p(path.substr(0, path.rfind('/')));
                // Download beside the target and rename: a half-written tile
                // must never be left where the next run would read it as cached.
                const std::string tmp = path + ".part";
                if (curl_to_file(tile_url(z, x, y), tmp, opt_.timeout_s)) {
                    img = cv::imread(tmp, cv::IMREAD_COLOR);
                    if (!img.empty()) ::rename(tmp.c_str(), path.c_str());
                    else              ::unlink(tmp.c_str());
                } else {
                    ::unlink(tmp.c_str());
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(mtx_);
            Entry e;
            e.used_at = ++clock_;
            if (img.empty()) {
                e.fail_at = now_ms();
                ++failed_;
            } else {
                e.img = img;
                ++served_;
            }
            cache_[k] = std::move(e);
            inflight_.erase(k);
            evict_locked();
        }
    }
}

}  // namespace navigation
