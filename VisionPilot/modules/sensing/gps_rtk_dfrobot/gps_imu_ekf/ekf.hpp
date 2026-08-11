#ifndef VP_STM32_GPS_IMU_EKF_HPP
#define VP_STM32_GPS_IMU_EKF_HPP

/*!
 * @file  ekf.hpp
 * @brief Loosely-coupled GPS/INS filter for a road vehicle, running on the MCU.
 *
 * A line-for-line port of the host's navigation::GpsImuEkf
 * (VisionPilot/modules/navigation/src/gps_imu_ekf.cpp), moved onto the Nucleo
 * because both sensors are now wired to it. Keeping the two identical is
 * deliberate: the host version has a test that drives it along a known route and
 * measures the error (modules/navigation/tests/test_gps_imu_ekf.cpp), and that
 * test is the only evidence either copy works. Port the maths, keep the test
 * meaningful; rewrite the maths and it is evidence about a different filter.
 *
 * The only changes are mechanical — Arduino has no <algorithm>, and the class
 * lives in `fusion` rather than `navigation`.
 *
 * State (6):
 *   0,1  position north, east  (metres from the origin fix)
 *   2,3  velocity north, east  (m/s)
 *   4    heading psi           (rad, 0 = north, CLOCKWISE positive)
 *   5    yaw-rate bias         (rad/s)
 *
 * Working in a local north/east tangent plane rather than in degrees keeps the
 * covariance in metres, where the tuning constants mean something. Over a city
 * the flat-earth error is millimetres.
 */

class GpsImuEkf
{
public:
    struct Tuning
    {
        // Process noise: how much the model is expected to be wrong per second.
        double accel_noise    = 0.6;     // m/s^2, unmodelled acceleration
        double yaw_rate_noise = 0.02;    // rad/s
        double bias_walk      = 0.0015;  // rad/s per sqrt(s), gyro bias drift

        // Measurement noise. The GPS figure is a floor: when a fix reports its
        // own accuracy that is used instead, because a fix that knows it is bad
        // should pull less. Standalone fixes report several metres, RTK ones
        // report centimetres, and the filter follows each accordingly with no
        // change here.
        double gps_pos_sigma    = 4.0;   // m
        double gps_speed_sigma  = 0.6;   // m/s
        double gps_course_sigma = 0.15;  // rad

        // Below this speed a GPS course is meaningless — a stationary receiver
        // reports a heading that wanders over the whole circle — so it is not
        // used as a heading measurement.
        double course_min_speed = 2.0;   // m/s
    };

    GpsImuEkf() = default;

    bool initialised() const { return init_; }

    // First fix defines the tangent-plane origin and seeds the state.
    void initialise(double lat, double lon, double heading_deg, bool have_heading);

    // IMU step. `ax`, `ay` are horizontal acceleration in the *body* frame
    // (x forward, y left) with gravity already removed; `yaw_rate` is the
    // vertical gyro in rad/s, positive clockwise (see kYawRateSign in main.cpp).
    void predict(double ax, double ay, double yaw_rate, double dt);

    // GPS position fix. `sigma_m` <= 0.5 means "use the configured default".
    void update_position(double lat, double lon, double sigma_m);

    // GPS speed and course. Course is ignored below course_min_speed.
    void update_velocity(double speed_ms, double course_deg, bool have_course);

    // ── Output ───────────────────────────────────────────────────────────────
    double latitude() const;
    double longitude() const;
    double speed_ms() const;
    double heading_deg() const;        // 0 = north, clockwise
    double yaw_rate_bias_dps() const;
    // Standard deviation of the horizontal position estimate — how much the
    // filter thinks it can be trusted, which the host draws rather than hiding.
    double position_sigma_m() const;

    double north() const { return x_[0]; }
    double east() const { return x_[1]; }

    Tuning& tuning() { return t_; }

private:
    void normalise_heading();
    // One scalar measurement, applied in sequence. Scalar updates avoid
    // inverting anything larger than a number and let each measurement be
    // rejected on its own merits.
    void fuse_scalar(const double H[6], double innovation, double R);

    static const int kN = 6;

    Tuning t_;
    bool   init_ = false;

    double x_[kN] = {0};
    double P_[kN][kN] = {{0}};

    double lat0_ = 0.0, lon0_ = 0.0;
    double m_per_deg_lat_ = 111132.0;
    double m_per_deg_lon_ = 111320.0;
};

#endif  // VP_STM32_GPS_IMU_EKF_HPP
