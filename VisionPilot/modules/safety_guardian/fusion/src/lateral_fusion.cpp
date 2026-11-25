#include <fusion/lateral_fusion.hpp>
#include <logging/logger.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace visionpilot::fusion {

// ─── Construction / reset ────────────────────────────────────────────────────

LateralFusion::LateralFusion()
    : LateralFusion(Config{})
{}

LateralFusion::LateralFusion(Config cfg)
    : cfg_(cfg)
    , rng_(std::random_device{}())
{
    if (cfg_.n_particles < 10)
        throw std::invalid_argument("LateralFusion: n_particles must be >= 10");
    cy_.reserve(static_cast<std::size_t>(cfg_.n_particles));
    k_.reserve(static_cast<std::size_t>(cfg_.n_particles));
}

void LateralFusion::reset()
{
    cy_.clear();  cy_init_ = false;
    k_.clear();   k_init_  = false;
}

// ─── Public update ────────────────────────────────────────────────────────────

LateralFusionEstimate LateralFusion::update(
    const models::AutoSteerOutput& steer,
    const models::AutoDriveOutput& drive,
    float dt_s)
{
    LateralFusionEstimate est;
    const float dt = (dt_s > 1e-6f) ? dt_s : cfg_.dt_s;

    // ── Step 2: project AutoSteer waypoints → world points ───────────────────
    PathParams path;
    if (steer.valid) {
        auto pts = project_waypoints(steer);
        est.path_points = static_cast<int>(pts.size());

        // ── Step 2b: route corridor ──────────────────────────────────────────
        // Approaching a junction the waypoints stop describing one path and
        // start describing several. fit_ransac() below can only return one, and
        // it returns whichever branch its random sampling found most points on
        // — a different answer frame to frame, from the same road. Drop the
        // waypoints that are not near the branch the route takes and the
        // ambiguity is gone before the fit ever meets it.
        if (!route_ego_.empty() && cfg_.route_corridor_m > 0.f) {
            est.route_active     = true;
            est.route_ego        = route_ego_;
            est.route_corridor_m = cfg_.route_corridor_m;
            est.route_pts_in = static_cast<int>(pts.size());

            est.route_x_min_m = route_ego_.front().x;
            est.route_x_max_m = route_ego_.back().x;

            std::vector<WorldPt> kept;
            kept.reserve(pts.size());
            float nearest = std::numeric_limits<float>::max();
            for (const auto& p : pts) {
                const float d = dist_to_route(p);
                nearest = std::min(nearest, d);
                if (d <= cfg_.route_corridor_m) {
                    kept.push_back(p);
                    est.wp_kept.emplace_back(p.x, p.y);
                } else {
                    est.wp_dropped.emplace_back(p.x, p.y);
                }
            }
            est.route_min_dist_m = pts.empty() ? 0.f : nearest;
            est.route_pts_kept = static_cast<int>(kept.size());

            // A corridor is only ever allowed to be as bad as no corridor. When
            // too little survives — a stale heading, a fix 10 m off, a route
            // that has not caught up with a wrong turn — the filter stands down
            // and the frame is fitted exactly as it was before any of this.
            if (static_cast<int>(kept.size()) >= cfg_.route_min_pts)
                pts.swap(kept);
            else
                est.route_fallback = true;
        }

        if (!pts.empty())
            path = fit_ransac(pts);
    }

    // ── Step 2c: route fusion ────────────────────────────────────────────────
    // Where the corridor removed the branches the route does not take, this
    // supplies the branch it does. They are not alternatives: on a straight road
    // there is no turn, the weight is zero and the path is the camera's alone;
    // into a junction the weight rises and the turn comes from the map, because
    // there is nothing in the waypoints to take it from.
    //
    // Only the SHAPE is taken. The cross-track error stays the camera's, always:
    // the map knows which way the road goes and is wrong about where the car is
    // within it by however far the fix is out.
    if (cfg_.route_fuse && cfg_.route_authority > 0.f && man_valid_
        && !route_ego_.empty()) {
        est.route_bend_deg = man_bend_deg_;
        est.route_dist_m   = man_dist_m_;

        // Sharp enough to be a junction rather than a kink, and near enough to
        // be this junction rather than the next one.
        const float lo   = cfg_.route_bend_min_deg;
        const float hi   = std::max(lo + 1.f, cfg_.route_bend_max_deg);
        const float sharp = std::clamp((std::abs(man_bend_deg_) - lo) / (hi - lo), 0.f, 1.f);

        const float arm    = std::max(cfg_.route_arm_m, cfg_.route_engage_m + 1.f);
        const float engage = cfg_.route_engage_m;
        const float near_  = std::clamp((arm - man_dist_m_) / (arm - engage), 0.f, 1.f);

        const float w = cfg_.route_authority * sharp * near_;
        est.route_weight = w;

        // Aim past the corner, not at it: a lookahead that stops short of the
        // turn describes the straight before it and asks for no turn at all.
        //
        // The ceiling follows the arming distance rather than being a constant.
        // Fixing it independently lets the two disagree — arm at 110 m with a
        // 45 m reach and the map is given weight over a curvature it cannot yet
        // see, which shows up as authority that does nothing.
        const float reach = cfg_.route_arm_m + 15.f;
        const RouteShape rs = route_shape(std::clamp(man_dist_m_ + 12.f, 12.f, reach));
        if (rs.valid) est.route_kappa = rs.kappa;

        if (w > 0.f && rs.valid) {
            // With no usable fit there is nothing to blend against, so the route
            // becomes the path outright and the offset is held at whatever the
            // filter last believed. That case matters most: vision losing the
            // road mid-junction is exactly when the map has to carry it.
            const float k_vis = path.valid ? path.curvature : 0.f;
            const float wv    = path.valid ? (1.f - w) : 0.f;
            const float wm    = path.valid ? w          : 1.f;

            path.curvature = wv * k_vis + wm * rs.kappa;
            path.cte_m     = path.valid ? path.cte_m : last_cte_m_;
            // Heading stays the camera's. It is the one part of the shape that
            // is about where the car is pointing *now*, and the camera can see
            // that while the map can only infer it from a fix.
            if (!path.valid) path.yaw_rad = 0.f;

            // Rebuild the polynomial from the fused shape, so the corridor the
            // HUD draws is the path that is actually being followed. Drawing the
            // camera's straight line while the planner turns would make the two
            // impossible to tell apart exactly when they disagree.
            path.b = std::tan(path.yaw_rad);
            path.a = 0.5f * path.curvature * std::pow(1.f + path.b * path.b, 1.5f);
            path.c = path.cte_m;
            if (!path.valid) {
                path.valid   = true;
                path.inliers = 0;
                path.x_min_m = 0.f;
                path.x_max_m = 30.f;
            }
        }
    }

    // Publish the path as it now stands — after the corridor pruned it and the
    // route supplied whatever turn the camera had not got. Everything
    // downstream reads these: the planner, and the corridor the HUD draws.
    est.path_valid         = path.valid;
    est.raw_cte_m          = path.cte_m;
    est.raw_yaw_rad        = path.yaw_rad;
    est.raw_path_curvature = path.curvature;
    est.path_inliers       = path.inliers;
    est.path_a             = path.a;
    est.path_b             = path.b;
    est.path_c             = path.c;
    est.path_x_min_m       = path.x_min_m;
    est.path_x_max_m       = path.x_max_m;

    // ── Step 3: AutoDrive curvature ───────────────────────────────────────────
    float ad_curv = 0.f;
    bool  ad_curv_valid = false;
    if (drive.valid) {
        ad_curv       = drive.curvature_raw * cfg_.ad_curvature_scale;
        ad_curv_valid = true;
        est.raw_ad_curvature = ad_curv;
    }

    // ── Step 4: CTE / Yaw particle filter ────────────────────────────────────
    if (path.valid) {
        // Apply camera mounting offset correction before feeding the filter.
        const float corrected_cte = path.cte_m - cfg_.cte_bias_m;
        if (!cy_init_) {
            cy_init(corrected_cte, path.yaw_rad);
        } else {
            cy_predict(dt);
            cy_update(corrected_cte, cfg_.meas_noise_cte_m,
                      path.yaw_rad, cfg_.meas_noise_yaw_rad);
            if (cy_eff_n() < 0.5f * static_cast<float>(cfg_.n_particles))
                cy_resample();
        }
    }

    // ── Step 5: Curvature particle filter ────────────────────────────────────
    // Initialise from whichever source is available first.
    if (!k_init_) {
        if      (path.valid)    k_init(path.curvature);
        else if (ad_curv_valid) k_init(ad_curv);
    } else {
        k_predict(dt);
        if (path.valid)    k_update(path.curvature, cfg_.meas_noise_curv_path);
        if (ad_curv_valid) k_update(ad_curv,        cfg_.meas_noise_curv_ad);
        if (k_eff_n() < 0.5f * static_cast<float>(cfg_.n_particles))
            k_resample();
    }

    // ── Step 6: Extract posterior means ──────────────────────────────────────
    if (cy_init_ && !cy_.empty()) {
        const auto w = linear_weights(cy_);
        float mean_cte = 0.f, mean_yaw = 0.f;
        for (std::size_t i = 0; i < cy_.size(); ++i) {
            mean_cte += w[i] * cy_[i].cte;
            mean_yaw += w[i] * cy_[i].yaw;
        }
        float var_cte = 0.f, var_yaw = 0.f;
        for (std::size_t i = 0; i < cy_.size(); ++i) {
            const float dc = cy_[i].cte - mean_cte;
            const float dy = cy_[i].yaw - mean_yaw;
            var_cte += w[i] * dc * dc;
            var_yaw += w[i] * dy * dy;
        }
        est.cte_m          = mean_cte;
        // Kept for the next frame: when vision drops out mid-junction the fused
        // path has to be anchored somewhere, and the last offset the filter
        // believed is the only honest answer available.
        last_cte_m_        = mean_cte;
        est.yaw_rad        = mean_yaw;
        est.cte_stddev_m   = std::sqrt(std::max(0.f, var_cte));
        est.yaw_stddev_rad = std::sqrt(std::max(0.f, var_yaw));
        est.valid          = true;
    }

    if (k_init_ && !k_.empty()) {
        const auto w = linear_weights(k_);
        float mean_k = 0.f;
        for (std::size_t i = 0; i < k_.size(); ++i)
            mean_k += w[i] * k_[i].curv;
        float var_k = 0.f;
        for (std::size_t i = 0; i < k_.size(); ++i) {
            const float dk = k_[i].curv - mean_k;
            var_k += w[i] * dk * dk;
        }
        est.curvature   = mean_k;
        est.curv_stddev = std::sqrt(std::max(0.f, var_k));
        est.valid       = true;
    }

    // ── Step 7: Debug log ─────────────────────────────────────────────────────
    if (cfg_.debug) {
        char path_buf[96], ad_buf[32];
        if (path.valid)
            std::snprintf(path_buf, sizeof(path_buf),
                          "raw_CTE=%.2fm  bias=%.2fm  corr=%.2fm  yaw=%.3frad  κ=%.4f  (%d pts, xmin=%.1fm)",
                          path.cte_m, cfg_.cte_bias_m, path.cte_m - cfg_.cte_bias_m,
                          path.yaw_rad, path.curvature, path.inliers, path.x_min_m);
        else
            std::snprintf(path_buf, sizeof(path_buf), "(no path fit, %d pts)", est.path_points);

        if (ad_curv_valid)
            std::snprintf(ad_buf, sizeof(ad_buf), "%.4f (raw=%.4f)",
                          ad_curv, drive.curvature_raw);
        else
            std::snprintf(ad_buf, sizeof(ad_buf), "(none)");

        VP_INFO("[Lateral] Path: %s | AD-κ=%s | Fused CTE=%.2fm yaw=%.3frad κ=%.4f",
                path_buf, ad_buf,
                est.cte_m, est.yaw_rad, est.curvature);
    }

    return est;
}

// ─── Waypoint projection ──────────────────────────────────────────────────────
//
//  Matches Models/visualizations/AutoSteer/video_visualization.py:
//    yp[i] = linspace(0, NET_H-1, 64)
//    u = xp[i] * NET_W  (masked by h_vector[i] >= 0.5)
//
std::vector<LateralFusion::WorldPt>
LateralFusion::project_waypoints(const models::AutoSteerOutput& steer) const
{
    static constexpr int N_WP  = 64;
    static constexpr float NET_W = 1024.f;
    static constexpr float NET_H = 512.f;

    std::vector<WorldPt> out;
    out.reserve(N_WP);

    for (int i = 0; i < N_WP; ++i) {
        const float v_px = (N_WP <= 1) ? 0.f
            : static_cast<float>(i) * (NET_H - 1.f) / static_cast<float>(N_WP - 1);

        if (steer.h_vector[i] < 0.5f) continue;

        const float u_px = steer.xp[i] * NET_W;
        auto [xw, yw] = project_world(H_, u_px, v_px);
        // Keep points in a sensible forward range for the polynomial fit
        if (xw < 0.5f || xw > 120.f) continue;
        out.push_back({xw, yw});
    }
    return out;
}

// Project image (u,v) → world (x_forward [m], y_lateral [m, +left])
std::pair<float, float>
LateralFusion::project_world(const cv::Mat& H, float u, float v)
{
    std::vector<cv::Point2f> src = {cv::Point2f(u, v)}, dst;
    cv::perspectiveTransform(src, dst, H);
    return {dst[0].x, dst[0].y};
}

// ─── Route corridor ───────────────────────────────────────────────────────────
//
//  Distance from a waypoint to the route line. The route arrives resampled at
//  1 m, so nearest-sample would already be within half a metre — but the
//  segment distance is a handful of operations on 64 points and removes a
//  half-metre of slop from a threshold that is being tuned against GPS error.
//
float LateralFusion::point_segment_dist(float px, float py,
                                        float ax, float ay, float bx, float by)
{
    const float vx = bx - ax, vy = by - ay;
    const float len2 = vx * vx + vy * vy;
    if (len2 < 1e-9f) return std::hypot(px - ax, py - ay);
    float t = ((px - ax) * vx + (py - ay) * vy) / len2;
    t = std::max(0.f, std::min(1.f, t));
    return std::hypot(px - (ax + t * vx), py - (ay + t * vy));
}

float LateralFusion::dist_to_route(const WorldPt& p) const
{
    if (route_ego_.empty()) return std::numeric_limits<float>::max();
    if (route_ego_.size() == 1)
        return std::hypot(p.x - route_ego_[0].x, p.y - route_ego_[0].y);

    float best = std::numeric_limits<float>::max();
    for (size_t i = 0; i + 1 < route_ego_.size(); ++i) {
        best = std::min(best, point_segment_dist(p.x, p.y,
                                                 route_ego_[i].x,     route_ego_[i].y,
                                                 route_ego_[i + 1].x, route_ego_[i + 1].y));
    }
    return best;
}

// ─── Route shape ──────────────────────────────────────────────────────────────
//
//  The curvature that carries the car to a point `lookahead_m` along the route:
//  pure pursuit, the standard geometric path tracker (Coulter, CMU 1992), which
//  Autoware ships as its own baseline controller.
//
//  It is computed in a frame anchored to the route's near end and rotated onto
//  its near tangent. That is the whole trick and it is what makes this usable on
//  a phone fix: a position error slides the route sideways and a heading error
//  rotates it, and both are removed by construction. What is left is the shape
//  of the road ahead, which is the only thing a map is more reliable about than
//  a camera.
//
LateralFusion::RouteShape LateralFusion::route_shape(float lookahead_m) const
{
    RouteShape rs;
    if (route_ego_.size() < 6) return rs;

    std::vector<float> arc(route_ego_.size(), 0.f);
    for (size_t i = 1; i < route_ego_.size(); ++i)
        arc[i] = arc[i - 1] + static_cast<float>(
            std::hypot(route_ego_[i].x - route_ego_[i - 1].x,
                       route_ego_[i].y - route_ego_[i - 1].y));
    if (arc.back() < 8.f) return rs;

    // Index at or just past an arc length.
    auto at_arc = [&](float want) {
        size_t i = 0;
        while (i + 1 < arc.size() && arc[i] < want) ++i;
        return i;
    };

    // Anchor a couple of metres in, and take the near tangent over a few more,
    // so one jittery node cannot set the frame everything else is measured in.
    const size_t i_anchor = at_arc(1.5f);
    const size_t i_tan    = at_arc(6.0f);
    if (i_tan <= i_anchor) return rs;

    const cv::Point2f o = route_ego_[i_anchor];
    const float psi = std::atan2(route_ego_[i_tan].y - o.y,
                                 route_ego_[i_tan].x - o.x);
    const float cs = std::cos(-psi), sn = std::sin(-psi);

    const size_t i_look = at_arc(std::min(lookahead_m, arc.back()));
    if (i_look <= i_tan) return rs;

    const float dx = route_ego_[i_look].x - o.x;
    const float dy = route_ego_[i_look].y - o.y;
    const float x  = dx * cs - dy * sn;
    const float y  = dx * sn + dy * cs;

    const float l2 = x * x + y * y;
    if (l2 < 4.f) return rs;

    rs.valid = true;
    rs.kappa = 2.f * y / l2;
    return rs;
}

// ─── Polynomial RANSAC ────────────────────────────────────────────────────────
//
//  Fits: y_lateral = a·x_forward² + b·x_forward + c
//  RANSAC selects the best inlier set, then refits with least squares.
//
LateralFusion::PathParams
LateralFusion::fit_ransac(const std::vector<WorldPt>& pts) const
{
    if (static_cast<int>(pts.size()) < 3) return {};

    PathParams best;
    best.inliers = 0;
    std::vector<WorldPt> best_inliers_pts;

    // Fallback: all points
    best_inliers_pts = pts;

    if (static_cast<int>(pts.size()) > 3) {
        std::uniform_int_distribution<int> pick(0, static_cast<int>(pts.size()) - 1);

        for (int iter = 0; iter < cfg_.ransac_iters; ++iter) {
            // Sample 3 distinct points
            int ia = pick(const_cast<std::mt19937&>(rng_));
            int ib, ic;
            do { ib = pick(const_cast<std::mt19937&>(rng_)); } while (ib == ia);
            do { ic = pick(const_cast<std::mt19937&>(rng_)); } while (ic == ia || ic == ib);

            const std::vector<WorldPt> sample = {pts[ia], pts[ib], pts[ic]};
            const PathParams model = fit_quadratic(sample, cfg_.curv_x_min_m, cfg_.curv_x_max_m);
            if (!model.valid) continue;

            // Count inliers
            std::vector<WorldPt> inl;
            for (const auto& p : pts) {
                const float y_pred = eval_quad(model.a, model.b, model.c, p.x);
                if (std::abs(y_pred - p.y) < cfg_.ransac_thresh_m)
                    inl.push_back(p);
            }
            if (static_cast<int>(inl.size()) > best.inliers) {
                best.inliers     = static_cast<int>(inl.size());
                best_inliers_pts = inl;
            }
        }
    }

    // Final refit on inlier set
    if (static_cast<int>(best_inliers_pts.size()) >= 3) {
        PathParams final_fit = fit_quadratic(best_inliers_pts, cfg_.curv_x_min_m, cfg_.curv_x_max_m);
        final_fit.inliers    = static_cast<int>(best_inliers_pts.size());
        if (final_fit.inliers < cfg_.ransac_min_inliers) return {};
        if (std::abs(final_fit.cte_m) > cfg_.max_abs_cte_m) return {};
        return final_fit;
    }
    return {};
}

// κ(x) for y = a·x² + b·x + c  →  κ = 2a / (1 + (2ax + b)²)^(3/2)
float LateralFusion::curvature_at_x(float a, float b, float x)
{
    const float dydx = 2.f * a * x + b;
    const float denom = std::pow(1.f + dydx * dydx, 1.5f);
    if (denom < 1e-6f) return 0.f;
    return (2.f * a) / denom;
}

// Median κ(x) on the fitted curve at each waypoint x inside the lookahead window.
float LateralFusion::sample_path_curvature(const std::vector<WorldPt>& pts,
                                           float a, float b,
                                           float x_min_m, float x_max_m)
{
    if (x_min_m > x_max_m) std::swap(x_min_m, x_max_m);

    std::vector<float> kappas;
    kappas.reserve(pts.size());
    for (const auto& p : pts) {
        if (p.x < x_min_m || p.x > x_max_m) continue;
        kappas.push_back(curvature_at_x(a, b, p.x));
    }

    if (kappas.empty())
        return curvature_at_x(a, b, x_min_m);

    const auto mid = kappas.begin() + kappas.size() / 2;
    std::nth_element(kappas.begin(), mid, kappas.end());
    return *mid;
}

// Least-squares quadratic fit: y = a·x² + b·x + c
LateralFusion::PathParams
LateralFusion::fit_quadratic(const std::vector<WorldPt>& pts,
                             float curv_x_min_m, float curv_x_max_m)
{
    const int n = static_cast<int>(pts.size());
    if (n < 3) return {};

    cv::Mat A(n, 3, CV_32F);
    cv::Mat B(n, 1, CV_32F);
    for (int i = 0; i < n; ++i) {
        const float x = pts[i].x;
        A.at<float>(i, 0) = x * x;
        A.at<float>(i, 1) = x;
        A.at<float>(i, 2) = 1.f;
        B.at<float>(i, 0) = pts[i].y;
    }

    cv::Mat sol;
    if (!cv::solve(A, B, sol, cv::DECOMP_SVD)) return {};

    PathParams p;
    p.valid = true;
    p.a = sol.at<float>(0);
    p.b = sol.at<float>(1);
    p.c = sol.at<float>(2);

    // Find x range of actual data first so CTE is evaluated there, not extrapolated to x=0.
    p.x_min_m = pts[0].x;
    p.x_max_m = pts[0].x;
    for (const auto& pt : pts) {
        p.x_min_m = std::min(p.x_min_m, pt.x);
        p.x_max_m = std::max(p.x_max_m, pt.x);
    }

    // Evaluate CTE and yaw at x_min_m (closest observed waypoint) — avoids
    // extrapolating back to x=0 where there is no actual data.
    const float x_ref   = p.x_min_m;
    const float dydx    = 2.f * p.a * x_ref + p.b;   // polynomial slope at x_ref
    p.cte_m   = eval_quad(p.a, p.b, p.c, x_ref);
    p.yaw_rad = std::atan(dydx);
    p.curvature = sample_path_curvature(pts, p.a, p.b, curv_x_min_m, curv_x_max_m);

    return p;
}

float LateralFusion::eval_quad(float a, float b, float c, float x)
{
    return a * x * x + b * x + c;
}

// ─── CTE / Yaw particle filter ────────────────────────────────────────────────

void LateralFusion::cy_init(float cte, float yaw)
{
    const int N = cfg_.n_particles;
    cy_.resize(static_cast<std::size_t>(N));
    std::normal_distribution<float> nc(cte, cfg_.meas_noise_cte_m);
    std::normal_distribution<float> ny(yaw, cfg_.meas_noise_yaw_rad);
    for (auto& p : cy_) { p.cte = nc(rng_); p.yaw = ny(rng_); p.log_w = 0.f; }
    cy_init_ = true;
}

void LateralFusion::cy_predict(float dt)
{
    std::normal_distribution<float> nc(0.f, cfg_.proc_noise_cte_m  * std::sqrt(dt / cfg_.dt_s));
    std::normal_distribution<float> ny(0.f, cfg_.proc_noise_yaw_rad * std::sqrt(dt / cfg_.dt_s));
    for (auto& p : cy_) { p.cte += nc(rng_); p.yaw += ny(rng_); }
}

void LateralFusion::cy_update(float meas_cte, float noise_cte,
                               float meas_yaw, float noise_yaw)
{
    for (auto& p : cy_) {
        p.log_w += gauss_loglik(meas_cte, p.cte, noise_cte);
        p.log_w += gauss_loglik(meas_yaw, p.yaw, noise_yaw);
    }
}

float LateralFusion::cy_eff_n() const
{
    const auto w = linear_weights(cy_);
    float ss = 0.f;
    for (auto wi : w) ss += wi * wi;
    return 1.f / (ss + 1e-12f);
}

void LateralFusion::cy_resample() { systematic_resample(cy_); }

// ─── Curvature particle filter ────────────────────────────────────────────────

void LateralFusion::k_init(float curv)
{
    const int N = cfg_.n_particles;
    k_.resize(static_cast<std::size_t>(N));
    std::normal_distribution<float> nd(curv, cfg_.meas_noise_curv_path);
    for (auto& p : k_) { p.curv = nd(rng_); p.log_w = 0.f; }
    k_init_ = true;
}

void LateralFusion::k_predict(float dt)
{
    std::normal_distribution<float> nd(0.f, cfg_.proc_noise_curv * std::sqrt(dt / cfg_.dt_s));
    for (auto& p : k_) p.curv += nd(rng_);
}

void LateralFusion::k_update(float meas, float noise)
{
    for (auto& p : k_)
        p.log_w += gauss_loglik(meas, p.curv, noise);
}

float LateralFusion::k_eff_n() const
{
    const auto w = linear_weights(k_);
    float ss = 0.f;
    for (auto wi : w) ss += wi * wi;
    return 1.f / (ss + 1e-12f);
}

void LateralFusion::k_resample() { systematic_resample(k_); }

// ─── Shared helpers ────────────────────────────────────────────────────────────

float LateralFusion::gauss_loglik(float z, float mean, float sigma)
{
    const float d = z - mean;
    return -0.5f * (d / sigma) * (d / sigma);
}

// Log-sum-exp stable weight normalisation
template<typename P>
std::vector<float> LateralFusion::linear_weights(const std::vector<P>& ps) const
{
    const std::size_t N = ps.size();
    float max_lw = ps[0].log_w;
    for (const auto& p : ps) max_lw = std::max(max_lw, p.log_w);

    std::vector<float> w(N);
    float sum = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        w[i] = std::exp(ps[i].log_w - max_lw);
        sum  += w[i];
    }
    if (sum < 1e-12f) {
        const float w0 = 1.f / static_cast<float>(N);
        for (auto& wi : w) wi = w0;
    } else {
        for (auto& wi : w) wi /= sum;
    }
    return w;
}

// Systematic resampling — preserves diversity, O(N)
template<typename P>
void LateralFusion::systematic_resample(std::vector<P>& particles)
{
    const int N = static_cast<int>(particles.size());
    if (N == 0) return;

    const auto w = linear_weights(particles);
    std::vector<float> cs(static_cast<std::size_t>(N));
    cs[0] = w[0];
    for (int i = 1; i < N; ++i)
        cs[static_cast<std::size_t>(i)] = cs[static_cast<std::size_t>(i-1)] +
                                          w[static_cast<std::size_t>(i)];

    std::vector<P> np;
    np.reserve(static_cast<std::size_t>(N));
    std::uniform_real_distribution<float> u(0.f, 1.f / static_cast<float>(N));
    const float u0 = u(rng_);
    int j = 0;
    for (int i = 0; i < N; ++i) {
        const float thr = u0 + static_cast<float>(i) / static_cast<float>(N);
        while (j < N-1 && cs[static_cast<std::size_t>(j)] < thr) ++j;
        P pnew = particles[static_cast<std::size_t>(j)];
        pnew.log_w = 0.f;
        np.push_back(pnew);
    }
    particles = std::move(np);
}

}  // namespace visionpilot::fusion
