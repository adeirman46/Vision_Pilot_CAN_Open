// Dead-reckoning regression tests.
//
// This is the part that cannot be checked by looking at the 3D view: a track
// that drifts and a track that is simply wrong look identical on screen. Each
// case drives the estimator with a motion profile whose answer is known in
// advance, using sensor readings shaped like the real MRA02's — 25 Hz, a steady
// gyro bias, an accelerometer offset, a small attitude error, and noise.
//
// The tolerances are what the GPS fusion can rely on between fixes.

#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include <imu/attitude.hpp>

namespace {

int g_failures = 0;

void check(const std::string& what, double measured, double limit, const char* unit)
{
    const bool ok = std::fabs(measured) <= limit;
    if (!ok) ++g_failures;
    std::printf("  %-44s %8.3f %-4s (limit %.3f)  %s\n", what.c_str(), measured, unit,
                limit, ok ? "ok" : "FAIL");
}

// One synthetic MRA02 sample: true world acceleration in, sensor reading out.
// The board is level, so body axes are world axes; the imperfections are the
// ones the real part has.
class Sensor
{
public:
    explicit Sensor(unsigned seed = 7) : rng_(seed) {}

    // Yaw the board at `rate` deg/s. The reported attitude lags the truth by a
    // couple of samples, as any real filter's does; that lag is precisely what
    // leaks gravity into the horizontal plane during a turn.
    void set_spin(double rate_dps) { spin_dps_ = rate_dps; }

    imu::ImuSample sample(double ax, double ay, double az, double dt)
    {
        std::normal_distribution<double> a_noise(0.0, 0.03);    // m/s^2 rms
        std::normal_distribution<double> g_noise(0.0, 0.05);    // deg/s rms

        imu::ImuSample s;
        s.has_accel = s.has_gyro = s.has_quat = true;

        // Accelerometer: true specific force, plus a fixed offset and noise.
        s.accel = {ax + 0.04 + a_noise(rng_),
                   ay - 0.03 + a_noise(rng_),
                   az + imu::kGravity + 0.06 + a_noise(rng_)};

        // Gyroscope: the steady bias this board actually shows, plus whatever
        // rotation is commanded. The bias is the reading that used to defeat the
        // stillness test and stall the whole estimator.
        s.gyro = {(-1.6 + g_noise(rng_)) * imu::kDeg2Rad,
                  (-1.7 + g_noise(rng_)) * imu::kDeg2Rad,
                  (spin_dps_ + g_noise(rng_)) * imu::kDeg2Rad};

        yaw_deg_ += spin_dps_ * dt;

        // Gravity as an accelerometer at this yaw actually sees it. Yaw about a
        // vertical axis does not change that, but the small tilt error does, and
        // the reported attitude lags — so the two disagree during a turn.
        const double reported_yaw = yaw_lag_;
        yaw_lag_ += (yaw_deg_ - yaw_lag_) * 0.35;      // ~2 samples of lag

        const imu::Quat truth = imu::Quat::from_euler_deg(0.23, -0.17, yaw_deg_);
        const imu::Vec3 g_body =
            imu::Quat{truth.w, -truth.x, -truth.y, -truth.z}.rotate(
                imu::Vec3{0.0, 0.0, imu::kGravity});
        s.accel.x += g_body.x - 0.0;
        s.accel.y += g_body.y - 0.0;
        s.accel.z += g_body.z - imu::kGravity;

        const imu::Quat rep = imu::Quat::from_euler_deg(0.23, -0.17, reported_yaw);
        s.qw = rep.w; s.qx = rep.x; s.qy = rep.y; s.qz = rep.z;

        stamp_ += static_cast<uint64_t>(dt * 1e6);
        s.stamp_us = stamp_;
        return s;
    }

private:
    std::mt19937 rng_;
    uint64_t     stamp_    = 1000000;
    double       spin_dps_ = 0.0;
    double       yaw_deg_  = 0.0;
    double       yaw_lag_  = 0.0;
};

constexpr double kDt = 1.0 / 25.0;

void rest(imu::AttitudeEstimator& est, Sensor& sensor, double seconds)
{
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i)
        est.update(sensor.sample(0, 0, 0, kDt));
}

// Move `distance` metres along x over `seconds`, accelerating for the first half
// and braking for the second — the shape of a hand movement, and of a car
// pulling away and stopping again.
void move_x(imu::AttitudeEstimator& est, Sensor& sensor, double distance,
            double seconds, double* peak = nullptr)
{
    const int    n = static_cast<int>(seconds / kDt);
    const double a = 4.0 * distance / (seconds * seconds);
    for (int i = 0; i < n; ++i) {
        est.update(sensor.sample(i < n / 2 ? a : -a, 0, 0, kDt));
        if (peak) *peak = std::max(*peak, std::fabs(est.position().x));
    }
}

}  // namespace

int main()
{
    // ── 1. Stationary, with the bias that used to deadlock the detector ──────
    {
        std::printf("stationary for 60 s (steady gyro bias, tilted mounting)\n");
        imu::AttitudeEstimator est;
        Sensor                 sensor;
        rest(est, sensor, 60.0);

        check("horizontal drift over 60 s",
              std::hypot(est.position().x, est.position().y), 0.05, "m");
        check("vertical drift over 60 s", est.position().z, 0.05, "m");
        check("residual speed", std::hypot(est.velocity().x, est.velocity().y), 0.02,
              "m/s");
        check("gyro bias learned, x error", est.gyro_bias_dps().x + 1.6, 0.15, "deg/s");
        check("gyro bias learned, y error", est.gyro_bias_dps().y + 1.7, 0.15, "deg/s");
        if (!est.still()) {
            ++g_failures;
            std::printf("  %-44s %19s  FAIL\n", "detected as still", "no");
        }
    }

    // ── 2. One metre out, known answer ──────────────────────────────────────
    {
        std::printf("\n1.00 m in 2 s, then 5 s at rest\n");
        imu::AttitudeEstimator est;
        Sensor                 sensor;
        rest(est, sensor, 3.0);
        move_x(est, sensor, 1.0, 2.0);
        const double travelled = est.position().x;
        rest(est, sensor, 5.0);

        check("distance travelled, error", travelled - 1.0, 0.10, "m");
        check("position held after stopping", est.position().x - travelled, 0.05, "m");
        check("lateral error", est.position().y, 0.10, "m");
    }

    // ── 3. 30 cm out and back, three times — the hand test ──────────────────
    {
        std::printf("\n0.30 m out and back, x3, 1.5 s pauses\n");
        imu::AttitudeEstimator est;
        Sensor                 sensor;
        rest(est, sensor, 3.0);

        double peak = 0.0;
        for (int i = 0; i < 3; ++i) {
            move_x(est, sensor, 0.30, 1.0, &peak);
            rest(est, sensor, 1.5);
            move_x(est, sensor, -0.30, 1.0);
            rest(est, sensor, 1.5);
        }

        check("furthest point reached, error vs 0.30", peak - 0.30, 0.08, "m");
        check("returned to start, error", est.position().x, 0.12, "m");
        check("lateral wander", est.position().y, 0.12, "m");
    }

    // ── 4. Turning on the spot — must not travel ────────────────────────────
    {
        std::printf("\n90 deg yaw, back and forth, x4 — no translation at all\n");
        imu::AttitudeEstimator est;
        Sensor                 sensor;
        rest(est, sensor, 3.0);

        for (int i = 0; i < 4; ++i) {
            sensor.set_spin(90.0);
            rest(est, sensor, 1.0);          // one second of turning
            sensor.set_spin(0.0);
            rest(est, sensor, 1.5);
            sensor.set_spin(-90.0);
            rest(est, sensor, 1.0);
            sensor.set_spin(0.0);
            rest(est, sensor, 1.5);
        }

        check("position after 8 turns",
              std::hypot(est.position().x, est.position().y), 0.15, "m");
        check("residual speed", std::hypot(est.velocity().x, est.velocity().y), 0.05,
              "m/s");
    }

    std::printf("\n%s\n",
                g_failures == 0 ? "all checks passed" : "FAILURES — see above");
    return g_failures == 0 ? 0 : 1;
}
