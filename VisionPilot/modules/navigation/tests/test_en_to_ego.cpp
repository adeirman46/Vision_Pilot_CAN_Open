// The east/north → ego-frame conversion, checked against cases anyone can work
// out on their fingers.
//
// It is four multiplications, and it gets its own test because it is the one
// place where being wrong is invisible. Swap a sign and every route still
// converts, still has plausible magnitudes, still draws a line on the panel —
// mirrored about the car's axis. The corridor then sits on the wrong arm of
// every junction and rejects precisely the waypoints it was built to keep.
//
//   build:  make test_en_to_ego
//   run:    ./test_en_to_ego        (exit 0 = pass)

#include <navigation/osm_map.hpp>

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void check(const char* what, double heading_deg, double east, double north,
           double want_fwd, double want_left)
{
    double fwd = 0.0, left = 0.0;
    navigation::en_to_ego(east, north, heading_deg, fwd, left);

    const bool ok = std::abs(fwd - want_fwd) < 1e-6 && std::abs(left - want_left) < 1e-6;
    if (!ok) ++failures;
    std::printf("  %s  %-46s  fwd %+7.1f  left %+7.1f\n",
                ok ? "PASS" : "FAIL", what, fwd, left);
    if (!ok)
        std::printf("        expected fwd %+7.1f  left %+7.1f\n", want_fwd, want_left);
}

}  // namespace

int main()
{
    // Heading 0 is north. Facing north, east is on your right — and right is
    // negative, because y grows to the left.
    std::printf("facing north (heading 0)\n");
    check("100 m north is straight ahead",        0.0,   0.0, 100.0,  100.0,    0.0);
    check("100 m east is 100 m to the right",     0.0, 100.0,   0.0,    0.0, -100.0);
    check("100 m west is 100 m to the left",      0.0,-100.0,   0.0,    0.0, +100.0);
    check("100 m south is behind",                0.0,   0.0,-100.0, -100.0,    0.0);

    // Turn the car east and the same ground points have to rotate with it.
    std::printf("facing east (heading 90)\n");
    check("100 m east is now straight ahead",    90.0, 100.0,   0.0,  100.0,    0.0);
    check("100 m north is now 100 m to the left",90.0,   0.0, 100.0,    0.0, +100.0);
    check("100 m south is now to the right",     90.0,   0.0,-100.0,    0.0, -100.0);

    std::printf("facing south (heading 180)\n");
    check("100 m north is behind",              180.0,   0.0, 100.0, -100.0,    0.0);
    check("100 m east is 100 m to the left",    180.0, 100.0,   0.0,    0.0, +100.0);

    std::printf("facing west (heading 270)\n");
    check("100 m west is straight ahead",       270.0,-100.0,   0.0,  100.0,    0.0);
    check("100 m north is 100 m to the right",  270.0,   0.0, 100.0,    0.0, -100.0);

    // A bearing 45 deg off the point: the range splits evenly between the axes.
    std::printf("off-axis\n");
    {
        const double d = 100.0, r = d / std::sqrt(2.0);
        check("north-east at heading 45 is dead ahead", 45.0, r, r, d, 0.0);
        check("north-east at heading 135 is abeam left",135.0, r, r, 0.0, d);
    }

    std::printf("\n%s\n", failures == 0 ? "all checks passed"
                                        : "SOMETHING IS WRONG");
    return failures == 0 ? 0 : 1;
}
