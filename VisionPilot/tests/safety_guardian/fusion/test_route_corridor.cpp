// Does the route actually decide which arm of a junction the path fit follows?
//
// This is the whole claim the route corridor makes, and both ways of getting it
// wrong are silent on a live camera: a corridor that filters nothing looks like
// a corridor that works until the first junction, and a corridor built in a
// mirrored frame confidently keeps the waypoints on the wrong arm. So the
// junction is synthesised here, where the right answer is known.
//
// The scene: waypoints that fan into two arms, one curving left and one curving
// right, in equal number — the case where the single-model RANSAC has no
// grounds to prefer either and returns whichever its sampling landed on.
//
//   build:  make test_route_corridor
//   run:    ./test_route_corridor        (exit 0 = pass)

#include <fusion/lateral_fusion.hpp>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>

#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

using visionpilot::fusion::LateralFusion;
using visionpilot::models::AutoDriveOutput;
using visionpilot::models::AutoSteerOutput;

namespace {

// Metres per image row / column, and the forward distance at the bottom row.
constexpr double kFwdPerRow = 0.1;
constexpr double kLatPerCol = 0.03;
constexpr double kX0        = 1.0;
// How hard each arm of the junction bends: y = ±kBend·x². At 50 m the arms are
// 20 m apart, at 5 m they are 20 cm apart — which is the point. Near the car
// both arms agree and there is nothing to choose between; the ambiguity only
// exists further out, and that is where the corridor has to do its work.
constexpr double kBend = 0.004;

// An H with the perspective row left as identity, so an image column maps to a
// fixed lateral offset and an image row to a fixed forward distance. The real
// homography is not this simple; the point here is a mapping whose inverse can
// be written down, so the waypoints can be placed exactly where the test wants
// them and the fit has a known right answer.
cv::Mat test_H()
{
    return (cv::Mat_<double>(3, 3) <<
            0.0,          -kFwdPerRow, kFwdPerRow * 511.0 + kX0,
            -kLatPerCol,   0.0,        kLatPerCol * 512.0,
            0.0,           0.0,        1.0);
}

// project_waypoints() samples fixed image rows: v = i·511/63, i = 0..63. Index 0
// is the far end of the path, index 63 the near end.
double wp_x(int i) { return kFwdPerRow * (511.0 - i * 511.0 / 63.0) + kX0; }

// The image column that lands a waypoint at lateral offset y.
float xp_for_y(double y) { return static_cast<float>((512.0 - y / kLatPerCol) / 1024.0); }

// Alternate waypoints down the two arms. Interleaving them rather than splitting
// the path in half is what makes both arms span the same forward range, so
// neither can win on extent alone.
AutoSteerOutput junction_waypoints()
{
    AutoSteerOutput s;
    s.valid = true;
    for (int i = 0; i < 64; ++i) {
        const double x    = wp_x(i);
        const double side = (i % 2 == 0) ? +1.0 : -1.0;
        s.xp[i]       = xp_for_y(side * kBend * x * x);
        s.h_vector[i] = 1.0f;
    }
    return s;
}

// One arm as a route: ego-frame waypoints at 1 m, the frame and the spacing
// Navigator::route_ahead_ego() hands over.
std::vector<cv::Point2f> arm(double side, double lateral_offset_m = 0.0)
{
    std::vector<cv::Point2f> r;
    for (double x = 0.0; x <= 55.0; x += 1.0)
        r.emplace_back(static_cast<float>(x),
                       static_cast<float>(side * kBend * x * x + lateral_offset_m));
    return r;
}

LateralFusion make_fusion(float corridor_m = 2.0f, bool fuse = false)
{
    LateralFusion::Config c;
    c.route_corridor_m = corridor_m;
    c.route_min_pts    = 8;
    // Off unless a case asks for it, so the corridor tests measure the corridor
    // and nothing else.
    c.route_fuse       = fuse;
    LateralFusion f(c);
    f.set_H(test_H());
    return f;
}

// Every waypoint straight ahead — a lane-following model doing exactly what it
// was trained to do while the road it is on is about to end in a T-junction.
// This is the case no filter can fix: there are no waypoints down either arm to
// keep, so the turn has to come from somewhere other than the camera.
AutoSteerOutput straight_waypoints()
{
    AutoSteerOutput s;
    s.valid = true;
    for (int i = 0; i < 64; ++i) {
        s.xp[i]       = xp_for_y(0.0);
        s.h_vector[i] = 1.0f;
    }
    return s;
}

// A route that runs straight for `straight_m` and then turns at a constant
// radius. Built by integrating a heading rather than written as a curve,
// because that is the one construction that stays correct past 90 degrees —
// where a function of x has already given up.
std::vector<cv::Point2f> turning_route(double turn_sign, double lateral_offset_m = 0.0)
{
    constexpr double kStraightM = 15.0;
    constexpr double kRadiusM   = 15.0;
    std::vector<cv::Point2f> r;
    double psi = 0.0, x = 0.0, y = lateral_offset_m;
    for (double s = 0.0; s <= 45.0; s += 1.0) {
        r.emplace_back(static_cast<float>(x), static_cast<float>(y));
        const double kappa = (s < kStraightM) ? 0.0 : turn_sign / kRadiusM;
        x   += std::cos(psi);
        y   += std::sin(psi);
        psi += kappa;              // ds = 1 m
    }
    return r;
}

int failures = 0;

void check(bool ok, const char* what)
{
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

}  // namespace

int main()
{
    const AutoSteerOutput steer = junction_waypoints();
    const AutoDriveOutput no_drive{};   // curvature comes from the path alone here

    // ── The claim: the route picks the arm ────────────────────────────────────
    std::printf("route decides the arm\n");
    {
        auto left = make_fusion();
        left.set_route(arm(+1.0));
        const auto e = left.update(steer, no_drive, 0.1f);

        check(e.route_active,             "corridor ran");
        check(e.path_valid,               "a path was fitted");
        check(e.route_pts_kept < e.route_pts_in, "waypoints were actually dropped");
        check(e.route_pts_kept >= 12,     "enough survived to fit");
        check(e.path_a > 0.002f,          "fit follows the LEFT arm");
        std::printf("       kept %d/%d, path_a = %+.5f\n",
                    e.route_pts_kept, e.route_pts_in, e.path_a);
    }
    {
        auto right = make_fusion();
        right.set_route(arm(-1.0));
        const auto e = right.update(steer, no_drive, 0.1f);

        check(e.path_valid,      "a path was fitted");
        check(e.path_a < -0.002f, "fit follows the RIGHT arm");
        std::printf("       kept %d/%d, path_a = %+.5f\n",
                    e.route_pts_kept, e.route_pts_in, e.path_a);
    }

    // ── Stability: same waypoints, many frames, one answer ────────────────────
    // The flicker this exists to remove is RANSAC re-sampling every frame, so
    // one frame proving the sign is not enough — the sign has to hold across
    // frames from a filter whose random state keeps advancing.
    std::printf("the answer holds across frames\n");
    {
        auto left = make_fusion();
        left.set_route(arm(+1.0));
        bool all_left = true;
        int  fitted   = 0;
        for (int frame = 0; frame < 24; ++frame) {
            const auto e = left.update(steer, no_drive, 0.1f);
            if (!e.path_valid) continue;
            ++fitted;
            if (e.path_a <= 0.002f) all_left = false;
        }
        check(fitted >= 20, "the fit succeeded on essentially every frame");
        check(all_left,     "every frame chose the same arm");

        // The same waypoints with no route, for contrast. Not asserted on: how
        // often the unfiltered fit changes its mind is a property of the RNG,
        // and a test that pins it would be testing the RNG. Printed because it
        // is the number the corridor exists to make zero.
        auto loose = make_fusion();
        int  flips = 0;
        float prev = 0.f;
        for (int frame = 0; frame < 24; ++frame) {
            const auto e = loose.update(steer, no_drive, 0.1f);
            if (!e.path_valid) continue;
            if (frame > 0 && ((prev > 0.f) != (e.path_a > 0.f))) ++flips;
            prev = e.path_a;
        }
        std::printf("       unfiltered, the same 24 frames changed arm %d time(s)\n", flips);
    }

    // ── The floor: a corridor in the wrong place must not starve the fit ──────
    std::printf("a misplaced corridor stands down\n");
    {
        auto lost = make_fusion();
        lost.set_route(arm(+1.0, 40.0));   // 40 m off to the left of everything
        const auto e = lost.update(steer, no_drive, 0.1f);

        check(e.route_active,   "corridor ran");
        check(e.route_pts_kept < 8, "it kept almost nothing");
        check(e.route_fallback, "so it stood down");
        check(e.path_valid,     "and the frame was still fitted");
    }

    // ── No route, no corridor ────────────────────────────────────────────────
    std::printf("no route means no corridor\n");
    {
        auto none = make_fusion();
        const auto e = none.update(steer, no_drive, 0.1f);
        check(!e.route_active, "corridor did not run");
        check(e.path_valid,    "the frame was fitted as it always was");
    }

    // ── The turn has to come from the route ──────────────────────────────────
    // The corridor cannot do this and was never able to: with every waypoint
    // pointing straight there is no branch to select. Either the shape comes
    // from the map or the path goes straight on into the junction.
    std::printf("a turn nobody saw\n");
    {
        const AutoSteerOutput straight = straight_waypoints();

        auto right = make_fusion(2.0f, /*fuse=*/true);
        right.set_route(turning_route(-1.0));           // right: y decreasing
        right.set_maneuver(true, 15.f, -60.f);          // as the router reports it
        const auto e = right.update(straight, no_drive, 0.1f);

        check(e.route_bend_deg < -20.f, "the route is seen bending RIGHT");
        check(e.route_weight > 0.f,     "which buys the map some authority");
        check(e.path_a < -1e-4f,        "and the fitted path now curves RIGHT");
        std::printf("       bend %+.0f deg, map %.0f%%, path_a %+.5f\n",
                    e.route_bend_deg, e.route_weight * 100.f, e.path_a);

        auto left = make_fusion(2.0f, true);
        left.set_route(turning_route(+1.0));
        left.set_maneuver(true, 15.f, +60.f);
        const auto el = left.update(straight, no_drive, 0.1f);
        check(el.route_bend_deg > 20.f, "a left route is seen bending LEFT");
        check(el.path_a > 1e-4f,        "and the path curves LEFT");
        std::printf("       bend %+.0f deg, map %.0f%%, path_a %+.5f\n",
                    el.route_bend_deg, el.route_weight * 100.f, el.path_a);
    }

    // ── A straight road is the camera's ──────────────────────────────────────
    std::printf("a straight route keeps its hands off\n");
    {
        auto f = make_fusion(2.0f, true);
        f.set_route(arm(0.0));                          // dead straight
        f.set_maneuver(false, 0.f, 0.f);                // the router reports no turn
        const auto e = f.update(junction_waypoints(), no_drive, 0.1f);
        check(e.route_weight == 0.f, "the map gets no say at all");
    }

    // ── A turn too far off is still not this turn ────────────────────────────
    std::printf("a distant turn waits its turn\n");
    {
        auto f = make_fusion(2.0f, true);
        f.set_route(turning_route(-1.0));
        f.set_maneuver(true, 200.f, -90.f);             // sharp, but 200 m away
        const auto e = f.update(straight_waypoints(), no_drive, 0.1f);
        check(e.route_weight == 0.f, "no weight while the turn is far off");
        check(std::abs(e.path_a) < 1e-4f, "and the path stays straight");
    }

    // ── The map never says where the car is ──────────────────────────────────
    // The whole design rests on this. A fix ten metres out moves the route ten
    // metres sideways without bending it, so taking shape from the map is safe
    // and taking position from it is not. If this check ever fails, the feature
    // is steering at a GPS error.
    std::printf("position stays the camera's\n");
    {
        auto near_ = make_fusion(2.0f, true);
        near_.set_route(turning_route(-1.0, 0.0));
        near_.set_maneuver(true, 15.f, -60.f);
        const auto a = near_.update(straight_waypoints(), no_drive, 0.1f);

        auto far_ = make_fusion(2.0f, true);
        far_.set_route(turning_route(-1.0, 10.0));      // same road, fix 10 m out
        far_.set_maneuver(true, 15.f, -60.f);
        const auto b = far_.update(straight_waypoints(), no_drive, 0.1f);

        check(std::abs(a.path_c - b.path_c) < 0.2f,
              "a 10 m lateral GPS error does not move the path");
        check(std::abs(a.path_a - b.path_a) < 1e-3f,
              "and does not change the turn either");
        std::printf("       offset 0 m: c=%+.2f a=%+.5f   offset 10 m: c=%+.2f a=%+.5f\n",
                    a.path_c, a.path_a, b.path_c, b.path_a);
    }

    std::printf("\n%s\n", failures == 0 ? "all checks passed"
                                        : "SOMETHING IS WRONG");
    return failures == 0 ? 0 : 1;
}
