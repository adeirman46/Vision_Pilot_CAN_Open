#ifndef VISIONPILOT_GPS_IMU_EKF_HPP
#define VISIONPILOT_GPS_IMU_EKF_HPP

#include <cstdint>

namespace navigation {

// Loosely-coupled GPS/INS filter for a road vehicle.
//
// GPS is right on average but arrives once a second, jumps by metres between
// fixes, and says nothing at all about which way the car is pointing when it is
// stopped. The IMU is the opposite: 25 Hz, smooth, and honest about every small
// change in motion — but its errors integrate, so left alone it walks away.
// Fusing them keeps the strengths of both: GPS anchors the absolute position and
// bounds the drift, the IMU fills in the second between fixes and carries the
// heading through tunnels, under bridges and past tall buildings.
//
// State (6):
//   0,1  position north, east  (metres from the origin fix)
//   2,3  velocity north, east  (m/s)
//   4    heading psi           (rad, 0 = north, clockwise positive)
//   5    yaw-rate bias         (rad/s)
//
// Working in a local north/east tangent plane rather than in degrees keeps the
// covariance in metres, where the tuning constants mean something. Over a city
// the flat-earth error is millimetres.
class GpsImuEkf
{
public:
    struct Tuning
    {
        // Process noise: how much the model is expected to be wrong per second.
        double accel_noise    = 0.6;    // m/s^2, unmodelled acceleration
        double yaw_rate_noise = 0.02;   // rad/s
        double bias_walk      = 0.0015; // rad/s per sqrt(s), gyro bias drift

        // Measurement noise. The GPS figure is a floor: when a fix reports its
        // own accuracy, that is used instead, because a fix that knows it is bad
        // should pull less.
        double gps_pos_sigma   = 4.0;   // m
        double gps_speed_sigma = 0.6;   // m/s
        double gps_course_sigma = 0.15; // rad

        // Below this speed a GPS course is meaningless — a stationary receiver
        // reports a heading that wanders over the whole circle — so it is not
        // used as a heading measurement.
        double course_min_speed = 2.0;  // m/s
    };

    // Two constructors rather than a defaulted argument: Tuning is a nested
    // class, and its default member initialisers are not usable inside the
    // enclosing class's own declaration.
    GpsImuEkf();
    explicit GpsImuEkf(const Tuning& t);

    bool initialised() const { return init_; }

    // First fix defines the tangent-plane origin and seeds the state.
    void initialise(double lat, double lon, double heading_deg, bool have_heading);

    // IMU step. `ax`, `ay` are the horizontal acceleration in the *body* frame
    // (x forward, y left) with gravity already removed; `yaw_rate` is the z gyro
    // in rad/s. dt in seconds.
    void predict(double ax, double ay, double yaw_rate, double dt);

    // GPS position fix. `sigma_m` <= 0 means "use the configured default".
    void update_position(double lat, double lon, double sigma_m);

    // GPS speed and course. Course is ignored below course_min_speed.
    void update_velocity(double speed_ms, double course_deg, bool have_course);

    // ── Output ───────────────────────────────────────────────────────────────
    double latitude() const;
    double longitude() const;
    double speed_ms() const;
    double heading_deg() const;      // 0 = north, clockwise
    double yaw_rate_bias_dps() const;
    // Standard deviation of the horizontal position estimate — how much the
    // filter thinks it can be trusted, which the panel shows rather than hiding.
    double position_sigma_m() const;

    double north() const { return x_[0]; }
    double east() const { return x_[1]; }

private:
    void   normalise_heading();
    // One scalar measurement, applied in sequence. Scalar updates avoid
    // inverting anything larger than a number and let each measurement be
    // rejected on its own merits.
    void   fuse_scalar(const double H[6], double innovation, double R);

    static constexpr int kN = 6;

    Tuning t_;
    bool   init_ = false;

    double x_[kN]{};
    double P_[kN][kN]{};

    double lat0_ = 0.0, lon0_ = 0.0;
    double m_per_deg_lat_ = 111132.0;
    double m_per_deg_lon_ = 111320.0;
};

}  // namespace navigation

#endif  // VISIONPILOT_GPS_IMU_EKF_HPP
