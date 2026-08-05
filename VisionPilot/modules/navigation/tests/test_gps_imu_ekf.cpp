// Does the GPS/IMU filter actually help, and can it be trusted?
//
// A filter that runs is not a filter that works. Both failure modes are silent:
// one that leans too hard on GPS is no better than GPS alone, and one that leans
// too hard on the IMU drifts away between fixes while looking perfectly smooth.
// Neither shows up on a map — the dot moves either way.
//
// So this drives the filter along a route whose true position is known at every
// instant, feeds it GPS and IMU corrupted the way the real ones are, and
// measures the error against the truth. The comparison that matters is against
// raw GPS: fusion has to beat the receiver it is given, or it is not earning the
// IMU.
//
// The simulated drive: accelerate to 12 m/s, hold, turn 90 degrees at a
// junction, hold, stop at lights, pull away. Plus a 10 s GPS outage mid-turn —
// a tunnel, a bridge, an urban canyon — which is the case the IMU exists for.

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <navigation/gps_imu_ekf.hpp>

namespace {

int g_failures = 0;

void check(const std::string& what, double measured, double limit, const char* unit)
{
    const bool ok = measured <= limit;
    if (!ok) ++g_failures;
    std::printf("  %-46s %8.2f %-5s (limit %.2f)  %s\n", what.c_str(), measured, unit,
                limit, ok ? "ok" : "FAIL");
}

constexpr double kDt      = 1.0 / 25.0;   // IMU rate
constexpr double kGpsEvery = 25;          // one fix a second
constexpr double kDeg     = M_PI / 180.0;

// True state of the car, integrated in the same north/east plane the filter uses.
struct Truth
{
    double n = 0.0, e = 0.0;
    double speed = 0.0;
    double psi = 0.0;          // heading, rad clockwise from north

    void step(double accel, double yaw_rate, double dt)
    {
        speed += accel * dt;
        if (speed < 0.0) speed = 0.0;
        psi += yaw_rate * dt;
        n += speed * std::cos(psi) * dt;
        e += speed * std::sin(psi) * dt;
    }
};

struct Result
{
    double rms_fused = 0.0, max_fused = 0.0;
    double rms_gps   = 0.0;
    double max_outage = 0.0;      // worst error while GPS was gone
    double heading_err = 0.0;     // final, degrees
};

Result run(bool with_outage, unsigned seed = 3)
{
    std::mt19937                     rng(seed);
    std::normal_distribution<double> gps_noise(0.0, 3.5);    // m, per axis
    std::normal_distribution<double> acc_noise(0.0, 0.05);   // m/s^2
    std::normal_distribution<double> gyr_noise(0.0, 0.002);  // rad/s

    // Constant offsets the real sensors carry and the filter is not told about.
    const double acc_bias = 0.03;               // m/s^2 along the body x axis
    const double gyro_bias = 0.4 * kDeg;        // rad/s

    const double lat0 = -6.9350, lon0 = 107.6506;
    const double m_lat = 110574.0, m_lon = 111320.0 * std::cos(lat0 * kDeg);

    navigation::GpsImuEkf ekf;
    Truth                 truth;
    Result                r;

    // 120 s of driving.
    const int steps = static_cast<int>(120.0 / kDt);
    int       n_err = 0;
    double    sum_sq_fused = 0.0, sum_sq_gps = 0.0;

    for (int i = 0; i < steps; ++i) {
        const double t = i * kDt;

        // ── Command the drive ───────────────────────────────────────────────
        double accel = 0.0, yaw_rate = 0.0;
        if (t < 6.0)              accel = 2.0;                    // pull away
        else if (t < 30.0)        accel = 0.0;                    // cruise
        else if (t < 38.0)        yaw_rate = (90.0 / 8.0) * kDeg; // 90 deg turn
        else if (t < 60.0)        accel = 0.0;
        else if (t < 66.0)        accel = -2.0;                   // stop
        else if (t < 76.0)        accel = 0.0;                    // at the lights
        else if (t < 82.0)        accel = 2.0;                    // pull away again
        truth.step(accel, yaw_rate, kDt);

        // ── IMU ─────────────────────────────────────────────────────────────
        // Body frame: x forward, y left. Cornering pushes the car sideways, and
        // that centripetal term is real acceleration the filter must handle.
        const double ax = accel + acc_bias + acc_noise(rng);
        const double ay = -truth.speed * yaw_rate + acc_noise(rng);
        ekf.predict(ax, ay, yaw_rate + gyro_bias + gyr_noise(rng), kDt);

        // ── GPS, once a second ──────────────────────────────────────────────
        const bool outage = with_outage && t > 34.0 && t < 44.0;
        if (i % static_cast<int>(kGpsEvery) == 0 && !outage) {
            const double gn = truth.n + gps_noise(rng);
            const double ge = truth.e + gps_noise(rng);
            const double lat = lat0 + gn / m_lat;
            const double lon = lon0 + ge / m_lon;

            if (!ekf.initialised()) {
                ekf.initialise(lat, lon, truth.psi / kDeg, true);
            } else {
                ekf.update_position(lat, lon, 3.5);
                const double course = std::fmod(truth.psi / kDeg + 360.0, 360.0);
                ekf.update_velocity(truth.speed, course, truth.speed > 1.0);
            }
            // How good the raw receiver was at this instant, for comparison.
            if (ekf.initialised()) {
                const double eg = std::hypot(gn - truth.n, ge - truth.e);
                sum_sq_gps += eg * eg;
            }
        }

        if (!ekf.initialised()) continue;

        const double fn = (ekf.latitude() - lat0) * m_lat;
        const double fe = (ekf.longitude() - lon0) * m_lon;
        const double err = std::hypot(fn - truth.n, fe - truth.e);

        sum_sq_fused += err * err;
        ++n_err;
        r.max_fused = std::max(r.max_fused, err);
        if (outage) r.max_outage = std::max(r.max_outage, err);
    }

    r.rms_fused = std::sqrt(sum_sq_fused / std::max(1, n_err));
    r.rms_gps   = std::sqrt(sum_sq_gps / std::max(1, steps / int(kGpsEvery)));

    const double truth_hdg = std::fmod(truth.psi / kDeg + 360.0, 360.0);
    double dh = std::fmod(ekf.heading_deg() - truth_hdg + 540.0, 360.0) - 180.0;
    r.heading_err = std::fabs(dh);
    return r;
}

}  // namespace

int main()
{
    std::printf("120 s drive: pull away, cruise, 90 deg turn, stop at lights, pull away\n");
    std::printf("GPS 1 Hz with 3.5 m noise; IMU 25 Hz with bias and noise\n\n");

    // ── 1. Normal conditions: fusion must beat the receiver it is given ──────
    {
        std::printf("continuous GPS\n");
        const Result r = run(false);
        std::printf("  %-46s %8.2f m\n", "raw GPS error, rms", r.rms_gps);
        check("fused position error, rms", r.rms_fused, r.rms_gps, "m");
        check("fused position error, worst", r.max_fused, 12.0, "m");
        check("final heading error", r.heading_err, 5.0, "deg");
    }

    // ── 2. A 10 s outage mid-turn: the case the IMU exists for ───────────────
    {
        std::printf("\n10 s GPS outage during the turn (tunnel / urban canyon)\n");
        const Result r = run(true);
        check("error during the outage, worst", r.max_outage, 25.0, "m");
        check("position error over the drive, rms", r.rms_fused, 6.0, "m");
        check("final heading error", r.heading_err, 8.0, "deg");
    }

    // ── 3. Repeatability: no seed may diverge ────────────────────────────────
    {
        std::printf("\n20 different noise seeds\n");
        double worst_rms = 0.0, worst_max = 0.0;
        for (unsigned s = 1; s <= 20; ++s) {
            const Result r = run(true, s);
            worst_rms = std::max(worst_rms, r.rms_fused);
            worst_max = std::max(worst_max, r.max_fused);
        }
        check("worst rms error across seeds", worst_rms, 8.0, "m");
        check("worst peak error across seeds", worst_max, 40.0, "m");
    }

    std::printf("\n%s\n", g_failures == 0 ? "EKF checks passed" : "FAILURES — see above");
    return g_failures == 0 ? 0 : 1;
}
