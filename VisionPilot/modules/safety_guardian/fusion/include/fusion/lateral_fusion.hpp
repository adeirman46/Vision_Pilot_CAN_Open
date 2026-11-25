#pragma once

#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <opencv2/core.hpp>
#include <random>
#include <string>
#include <vector>

namespace visionpilot::fusion {

// ─── Output struct ────────────────────────────────────────────────────────────
//
//  Axis convention throughout:
//    x = forward  [m]
//    y = lateral  [m, positive = left]
//
struct LateralFusionEstimate {
    bool valid = false;

    // ── Particle-filter tracked outputs (ready for planning) ──────────────────
    float cte_m          = 0.f;   // cross-track error [m]; +ve = ego right of path
    float yaw_rad        = 0.f;   // yaw error [rad];  +ve = path heading left
    float cte_stddev_m   = 0.f;
    float yaw_stddev_rad = 0.f;

    float curvature      = 0.f;   // fused curvature [1/m]; +ve = left turn
    float curv_stddev    = 0.f;

    // ── Raw intermediates for debug / downstream use ───────────────────────────
    bool  path_valid         = false;  // RANSAC polynomial fit succeeded
    float raw_cte_m          = 0.f;   // CTE direct from polynomial (= c-coeff)
    float raw_yaw_rad        = 0.f;   // yaw direct from polynomial (= atan(b))
    float raw_path_curvature = 0.f;   // κ sampled along fitted path (median)
    float raw_ad_curvature   = 0.f;   // curvature_raw from AutoDrive (scaled)
    int   path_inliers       = 0;     // RANSAC inlier count
    int   path_points        = 0;     // world points projected from waypoints
    // Fitted polynomial y = path_a·x² + path_b·x + path_c  (world frame)
    float path_a = 0.f, path_b = 0.f, path_c = 0.f;
    // Forward extent of RANSAC inliers [m] — cap path visualization / MPC samples
    float path_x_min_m = 0.f;
    float path_x_max_m = 0.f;

    // ── Route corridor ────────────────────────────────────────────────────────
    // Filled when a route was supplied via set_route(). At a junction the
    // waypoints fan out down every branch and the single-model RANSAC below
    // picks whichever branch happens to own the most of them — a coin toss that
    // is re-thrown every frame. The corridor removes the branches the route does
    // not take, so the fit has only one left to find.
    bool  route_active   = false;   // a route was supplied and the corridor ran
    int   route_pts_in   = 0;       // waypoints entering the corridor
    int   route_pts_kept = 0;       // waypoints inside it
    bool  route_fallback = false;   // too few survived → unfiltered fit used
    float route_corridor_m = 0.f;  // half-width in force, for the overlay
    // Closest any waypoint came to the route line [m]. The number that
    // separates "the corridor is a little tight" from "the corridor is on
    // the wrong road": a couple of metres over the half-width is the first,
    // tens of metres is the second, and they need completely different fixes.
    float route_min_dist_m = 0.f;
    // Forward span of the route in the ego frame [m]. A route that starts
    // well ahead of the car, or barely reaches it, is a heading problem.
    float route_x_min_m = 0.f, route_x_max_m = 0.f;

    // ── Route fusion ──────────────────────────────────────────────────────────
    // How far the route bends over the lookahead [deg], +ve = left, and how much
    // of the path's shape that bought the map, 0..1. Together they say why the
    // path is doing what it is doing: a bend with no weight is a threshold set
    // too high, weight with no bend is a heading gone wrong.
    float route_bend_deg   = 0.f;
    float route_weight     = 0.f;
    float route_kappa      = 0.f;   // curvature the route asked for [1/m]
    float route_dist_m     = 0.f;   // how far to that turn [m]
    // Ego-frame geometry for the overlay: x forward, y left, metres.
    std::vector<cv::Point2f> route_ego;    // the route line itself
    std::vector<cv::Point2f> wp_kept;      // waypoints the corridor accepted
    std::vector<cv::Point2f> wp_dropped;   // waypoints it threw away
};

// ─── LateralFusion ────────────────────────────────────────────────────────────
//
//  Per-frame lateral pipeline:
//    1. Project AutoSteer waypoints to world via H (matches video_visualization.py):
//       xp is (1,64): col = fixed image row (y = linspace).
//       u = xp[i] * 1024, v = image row i; mask with h_vector >= 0.5.
//    2. 2nd-order polynomial RANSAC on world points:
//         y_lateral = a·x² + b·x + c
//       → CTE = c, Yaw = atan(b), Curvature = median κ(x) at waypoint x in [curv_x_min, curv_x_max]
//    3. CTE/Yaw particle filter: state [cte, yaw], random-walk process,
//       Gaussian update from step-2 when path is valid.
//    4. Curvature particle filter: state [curv], fuses
//       a) path curvature from step-2  (when path_valid)
//       b) AutoDrive curvature_raw * ad_curvature_scale  (when drive.valid)
//
class LateralFusion {
public:
    struct Config {
        int   n_particles            = 300;
        float dt_s                   = 0.10f;   // nominal dt; overridden per-call

        // Process noise (random-walk, scaled by dt internally)
        float proc_noise_cte_m       = 0.05f;   // allows ~0.05m/s RMS drift for smooth tracking
        float proc_noise_yaw_rad     = 0.01f;
        float proc_noise_curv        = 0.002f;

        // Camera mounting offset compensation.  Set to the observed steady-state
        // CTE on a straight road (camera not at vehicle centreline).
        // Subtracted from raw polynomial CTE before it enters the particle filter.
        float cte_bias_m             = 0.0f;


        // Measurement 1-sigma
        float meas_noise_cte_m       = 0.15f;   // tighter now that CTE is at x_min (not extrapolated)
        float meas_noise_yaw_rad     = 0.05f;
        float meas_noise_curv_path   = 0.005f;  // AutoSteer polynomial curvature
        float meas_noise_curv_ad     = 0.010f;  // AutoDrive curvature (scaled)

        // RANSAC polynomial fit
        // thresh raised 0.20→0.25 m: AutoSteer waypoints spread more laterally
        // in close-following / occlusion scenarios; 20cm was too tight.
        // min_inliers lowered 20→15: requiring 70% consensus from ~28 pts was
        // too strict when a lead vehicle scatters some waypoints. 54% is still
        // majority agreement and keeps bad fits from passing.
        float ransac_thresh_m        = 0.20f;   // lateral inlier tolerance [m]
        int   ransac_iters           = 80;      // more iterations → better coverage in occlusion
        int   ransac_min_pts         = 5;       // min projected points to attempt fit

        // curvature_raw × scale → physical κ [1/m] (CURV_SCALE = 0.21 in training).
        float ad_curvature_scale     = 0.21f;
        int   ransac_min_inliers     = 12;      // reject path fit if fewer inliers
        float max_abs_cte_m          = 3.0f;    // reject RANSAC fits with |cte| > 3m (full lane width guard)
        float curv_x_min_m           = 3.f;     // κ sampled only at waypoint x ≥ this
        float curv_x_max_m           = 25.f;    // κ sampled only at waypoint x ≤ this

        // ── Route corridor ────────────────────────────────────────────────────
        // Half-width of the corridor around the route line, in metres. A
        // waypoint further than this from the route is not describing the
        // branch the route takes, so it is dropped before the fit sees it.
        // Sized against GPS error, not lane width: too tight and a fix that is
        // 5 m out throws the corridor off the road the car is actually on.
        // 0 disables the corridor.
        float route_corridor_m = 3.0f;
        // Below this many surviving waypoints the corridor is ignored and the
        // fit runs on everything. A corridor in the wrong place must never be
        // able to starve the fit — the filter is only ever allowed to be as bad
        // as no filter at all.
        int   route_min_pts    = 8;

        // ── Route fusion ──────────────────────────────────────────────────────
        // The corridor above can only remove waypoints. At a junction AutoSteer
        // does not produce any down the turn — it was trained to follow a lane,
        // so it predicts the lane it is in — and no filter can conjure a branch
        // out of waypoints that never described one. To make the path actually
        // turn, the shape of it has to come from the route.
        //
        // This is what Autoware does and it is not a compromise: mission_planner
        // produces the route, behavior_path_planner builds the path from the
        // centreline of the lanelets it names, and perception supplies obstacles
        // and drivable bounds rather than the geometry. Vision is better than a
        // map at where the lane is; a map is better than vision at which way the
        // road goes next. Each is used for what it knows.
        bool  route_fuse = true;

        // Ceiling on how much of the turn comes from the map, 0..1. Never 1:
        // the map's idea of a junction is a corner between two OSM ways, and
        // handing it the path outright would steer the corner rather than the
        // road. 0 disables fusion and leaves only the corridor.
        float route_authority = 0.7f;

        // The turn has to be sharper than this before the map is given any say,
        // and is worth full authority at the second. Below the first is a kink
        // in the road rather than a junction, and vision handles those better.
        float route_bend_min_deg = 12.f;
        float route_bend_max_deg = 50.f;

        // How near the turn has to be. Nothing beyond `arm`, full weight inside
        // `engage`, ramped between. Distance to the turn comes from the router,
        // which knows where its own turns are — measuring the road's bend over
        // a fixed window instead either misses a corner beyond the window or
        // averages it into the straights either side, and on a road whose nodes
        // are 75 m apart it does both.
        float route_arm_m        = 45.f;
        float route_engage_m     = 18.f;

        // Same YAML used by LongitudinalFusion (shared config field).
        bool debug = false;
    };

    LateralFusion();
    explicit LateralFusion(Config cfg);

    LateralFusionEstimate update(
        const models::AutoSteerOutput& steer,
        const models::AutoDriveOutput& drive,
        float dt_s = -1.f
    );

    void reset();

    // Override the internal H matrix used to project AutoSteer waypoints to
    // world space.  Call once after construction when the network runs on a
    // non-BEV image (e.g. plain-resized frame) that requires a different H.
    void set_H(const cv::Mat& H) { H_ = H.clone(); }

    // ── The route the vision path is asked to follow ──────────────────────────
    // Ego-frame waypoints, x forward, y left, metres — the same frame
    // project_waypoints() produces, which is what makes the comparison possible
    // at all. Navigator::route_ahead_ego() builds it. Call every frame: the
    // route moves with the car. An empty vector turns the corridor off, which is
    // what happens by itself whenever there is no fix, no route, or no heading.
    void set_route(const std::vector<cv::Point2f>& route_ego) { route_ego_ = route_ego; }
    const std::vector<cv::Point2f>& route() const { return route_ego_; }

    // The next turn the route asks for: how far, and how sharp (+ve = left).
    // From Navigator::next_maneuver(), which reads the router's own step list.
    // `valid` false — no route, or nothing but a straight ahead — leaves the
    // map with no say and the path entirely the camera's.
    void set_maneuver(bool valid, float dist_m, float bend_deg)
    { man_valid_ = valid; man_dist_m_ = dist_m; man_bend_deg_ = bend_deg; }

private:
    // ── Path processing ───────────────────────────────────────────────────────
    struct WorldPt { float x, y; };   // x = forward [m], y = lateral [m, +left]

    struct PathParams {
        bool  valid     = false;
        float a         = 0.f;        // y = a·x² + b·x + c
        float b         = 0.f;
        float c         = 0.f;
        float cte_m     = 0.f;        // = c
        float yaw_rad   = 0.f;        // = atan(b)
        float curvature = 0.f;        // median κ(x) along path
        int   inliers   = 0;
        float x_min_m   = 0.f;        // min forward x among fit points (inliers)
        float x_max_m   = 0.f;        // max forward x among fit points (inliers)
    };

    std::vector<WorldPt> project_waypoints(const models::AutoSteerOutput& steer) const;

    // ── Route corridor ────────────────────────────────────────────────────────
    // Shortest distance from a waypoint to the route line [m].
    float dist_to_route(const WorldPt& p) const;
    static float point_segment_dist(float px, float py,
                                    float ax, float ay, float bx, float by);

    // ── Route shape ───────────────────────────────────────────────────────────
    // The route's heading and curvature relative to the car, from its geometry
    // alone. Deliberately blind to where the route sits laterally: a GPS fix
    // five metres off moves the whole line sideways without bending it, so
    // shape survives the error that position does not.
    struct RouteShape {
        bool  valid = false;
        float kappa = 0.f;   // 1/m, +ve = left
    };
    // Curvature that carries the car to a point `lookahead_m` along the route,
    // pure-pursuit style. Measured in a frame anchored to the route's own near
    // end and rotated onto its near tangent, so both where the fix is and which
    // way it thinks it is pointing drop out — only the road's shape survives.
    RouteShape route_shape(float lookahead_m) const;
    PathParams           fit_ransac(const std::vector<WorldPt>& pts) const;
    static PathParams    fit_quadratic(const std::vector<WorldPt>& pts,
                                       float curv_x_min_m, float curv_x_max_m);
    static float         eval_quad(float a, float b, float c, float x);
    static float         curvature_at_x(float a, float b, float x);
    static float         sample_path_curvature(const std::vector<WorldPt>& pts,
                                               float a, float b,
                                               float x_min_m, float x_max_m);

    // Project a single image point (u,v) → (x_forward [m], y_lateral [m])
    static std::pair<float, float> project_world(const cv::Mat& H, float u, float v);

    // ── CTE / Yaw particle filter  [state: cte, yaw] ─────────────────────────
    struct CYParticle { float cte, yaw, log_w; };
    void  cy_predict(float dt);
    void  cy_update(float meas_cte, float noise_cte,
                    float meas_yaw, float noise_yaw);
    void  cy_resample();
    float cy_eff_n() const;
    void  cy_init(float cte, float yaw);

    // ── Curvature particle filter  [state: curvature] ────────────────────────
    struct KParticle { float curv, log_w; };
    void  k_predict(float dt);
    void  k_update(float meas, float noise);
    void  k_resample();
    float k_eff_n() const;
    void  k_init(float curv);

    // ── Shared helpers ────────────────────────────────────────────────────────
    static float gauss_loglik(float z, float mean, float sigma);

    template<typename P>
    std::vector<float> linear_weights(const std::vector<P>& ps) const;

    template<typename P>
    void systematic_resample(std::vector<P>& particles);

    Config cfg_;

    // Ego-frame route line, 1 m spacing. Empty = no corridor.
    std::vector<cv::Point2f> route_ego_;
    // The last cross-track error the filter published, to anchor a fused path
    // on a frame where vision produced none.
    float last_cte_m_ = 0.f;

    // The next turn, as the router reported it.
    bool  man_valid_    = false;
    float man_dist_m_   = 0.f;
    float man_bend_deg_ = 0.f;

    std::vector<CYParticle> cy_;
    bool                    cy_init_ = false;

    std::vector<KParticle>  k_;
    bool                    k_init_  = false;

    std::mt19937 rng_;
    // DO NOT MODIFY! VisionPilot model-view homography (1024x512 pixel -> world). Zenseact Open Dataset
    cv::Mat H_ = (cv::Mat_<double>(3, 3) <<
                      0.00209514907, -0.000941721466, -9.24906396,
                      0.00662758637, -0.000352940531, -3.33396502,
                      0.000120077371, -0.00411343505, 1.0
        );
};

}  // namespace visionpilot::fusion
