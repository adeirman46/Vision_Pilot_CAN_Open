#include <navigation/fused_gps.hpp>

#include <chrono>
#include <cmath>

#include <logging/logger.hpp>

namespace navigation {

namespace {

uint64_t now_ms()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

}  // namespace

FusedImuGps::FusedImuGps(std::unique_ptr<GpsInterface> gps, const std::string& imu_port,
                         int imu_baud)
    : gps_(std::move(gps))
{
    imu::ReaderConfig cfg;
    cfg.device = imu_port;
    cfg.baud   = imu_baud;
    // The module's own attitude solution is the one to use: its filter carries
    // the factory magnetometer calibration, which nothing here can reproduce.
    cfg.attitude.prefer_device = true;

    imu_ = std::make_unique<imu::ImuReader>(cfg);

    std::string err;
    if (imu_->start(err)) {
        imu_ok_ = true;
        VP_INFO("[Nav] IMU: %s", imu_->state().status.c_str());
    } else {
        imu_ok_ = false;
        VP_WARN("[Nav] IMU unavailable, falling back to GPS alone:\n%s", err.c_str());
    }

    running_ = true;
    thread_  = std::thread(&FusedImuGps::run, this);
}

FusedImuGps::~FusedImuGps()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (imu_) imu_->stop();
}

void FusedImuGps::run()
{
    while (running_) {
        // ── IMU: predict ────────────────────────────────────────────────────
        if (imu_ok_) {
            const imu::ImuState st = imu_->state();
            if (st.sample.stamp_us != last_imu_us_ && st.sample.stamp_us != 0) {
                double dt = 0.0;
                if (last_imu_us_ != 0 && st.sample.stamp_us > last_imu_us_)
                    dt = static_cast<double>(st.sample.stamp_us - last_imu_us_) * 1e-6;
                last_imu_us_ = st.sample.stamp_us;

                if (ekf_.initialised() && dt > 0.0) {
                    // The estimator has already rotated the accelerometer into
                    // world axes and taken gravity out; rotate that back into the
                    // vehicle's own heading so the filter's own psi is what turns
                    // it. Using the IMU's absolute yaw here instead would hand
                    // the filter two competing headings.
                    const double psi = ekf_.heading_deg() * M_PI / 180.0;
                    const double c = std::cos(psi), s = std::sin(psi);
                    const double an = st.linear_accel.x;   // world north
                    const double ae = st.linear_accel.y;   // world east
                    const double ax =  an * c + ae * s;    // body forward
                    const double ay = -an * s + ae * c;    // body left

                    ekf_.predict(ax, ay, st.sample.gyro.z, dt);
                }
            }
        }

        // ── GPS: correct ────────────────────────────────────────────────────
        if (auto fix = gps_->read()) {
            {
                std::lock_guard<std::mutex> lock(mtx_);
                raw_ = fix;
            }
            if (fix->stamp_ms != last_gps_ms_) {
                last_gps_ms_ = fix->stamp_ms;
                if (!ekf_.initialised()) {
                    ekf_.initialise(fix->latitude, fix->longitude, fix->bearing_deg,
                                    fix->has_bearing);
                    VP_INFO("[Nav] GPS+IMU: filter initialised at %.6f,%.6f",
                            fix->latitude, fix->longitude);
                } else {
                    ekf_.update_position(fix->latitude, fix->longitude,
                                         fix->accuracy_m);
                    ekf_.update_velocity(fix->speed_ms, fix->bearing_deg,
                                         fix->has_bearing);
                }
            }
        }

        // ── Publish ─────────────────────────────────────────────────────────
        if (ekf_.initialised()) {
            GpsFix out;
            out.latitude    = ekf_.latitude();
            out.longitude   = ekf_.longitude();
            out.speed_ms    = ekf_.speed_ms();
            out.bearing_deg = ekf_.heading_deg();
            out.has_bearing = true;
            out.accuracy_m  = ekf_.position_sigma_m();
            out.stamp_ms    = now_ms();

            std::lock_guard<std::mutex> lock(mtx_);
            fused_  = out;
            status_ = imu_ok_ ? "gps+imu" : "gps only (no IMU)";
        }

        // Faster than the IMU's 25 Hz so no sample waits a whole tick.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

std::optional<GpsFix> FusedImuGps::read()
{
    std::lock_guard<std::mutex> lock(mtx_);
    // Before the first fix there is nothing to anchor the filter to, so pass the
    // raw source through and let the panel say "no fix" exactly as it would.
    if (!fused_) return gps_->read();
    return fused_;
}

std::optional<GpsFix> FusedImuGps::raw_fix() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return raw_;
}

std::string FusedImuGps::status() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return status_;
}

double FusedImuGps::position_sigma_m() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return fused_ ? fused_->accuracy_m : 0.0;
}

}  // namespace navigation
