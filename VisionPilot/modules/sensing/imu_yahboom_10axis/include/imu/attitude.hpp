#ifndef IMU_YAHBOOM_ATTITUDE_HPP
#define IMU_YAHBOOM_ATTITUDE_HPP

#include <imu/imu_types.hpp>

namespace imu {

struct Quat
{
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

    void   normalize();
    Vec3   to_euler_deg() const;                  // roll, pitch, yaw
    Vec3   rotate(const Vec3& v) const;           // body -> world
    static Quat from_euler_deg(double roll, double pitch, double yaw);
};

// Attitude and (best-effort) position from the raw channels.
//
// The device's own Euler output is used when it sends one, because its internal
// filter has the factory magnetometer calibration. When it only sends raw
// channels this runs a Mahony complementary filter instead: the gyro carries the
// short-term motion, gravity corrects roll and pitch, and the magnetometer — if
// present — corrects yaw. Without a magnetometer yaw is gyro-only and will
// slowly drift; the viewer says so rather than pretending otherwise.
class AttitudeEstimator
{
public:
    struct Options
    {
        double kp             = 2.0;    // gravity/mag correction gain
        double ki             = 0.05;   // gyro-bias learning gain
        bool   prefer_device  = true;   // trust the module's own Euler output
        bool   use_mag        = true;
        // "Not moving" is decided on how much the readings *vary* over a short
        // window, not on how large they are. A gyro sitting at a steady -1.7°/s
        // of bias is not rotating, and a threshold on magnitude would call it
        // motion forever — which is exactly what stops the bias being learned.
        double still_accel_std = 0.10;  // m/s^2, spread of |a| over the window
        double still_gyro_std  = 1.20;  // deg/s, spread of |w| over the window
        // Rest additionally needs the linear acceleration itself to be small,
        // since steady acceleration has no more spread than noise does.
        double still_lin_ms2   = 0.12;  // m/s^2, after gravity and offset
        // Above this rotation rate, velocity is not integrated at all. Gravity
        // is 9.8 m/s^2, so even a couple of degrees of lag between the reported
        // attitude and the real one leaks more phantom acceleration than any
        // hand movement produces. See integrate_position().
        double rotation_gate_dps = 20.0;
    };

    // Two constructors rather than a defaulted argument: Options is a nested
    // class, and its default member initialisers are not usable inside the
    // enclosing class's own declaration.
    AttitudeEstimator();
    explicit AttitudeEstimator(const Options& opt);

    void update(const ImuSample& s);
    void reset();

    // Level the sensor where it sits: the current attitude becomes 0/0/0. Same
    // idea as a spirit level's tare button — it makes a crooked mounting read
    // straight without touching the calibration in the module.
    void zero_here();

    Quat  orientation() const { return q_; }
    Vec3  euler_deg() const;             // roll, pitch, yaw after any tare
    Vec3  gyro_bias_dps() const;
    Vec3  linear_accel() const { return lin_accel_; }   // world frame, gravity removed
    Vec3  velocity() const { return vel_; }
    Vec3  position() const { return pos_; }

    bool   calibrating() const { return cal_samples_ < kCalSamples; }
    double calibration_progress() const;
    bool   still() const { return still_; }
    bool   has_mag_yaw() const { return mag_yaw_valid_; }
    double heading_deg() const;          // 0 = north, clockwise

    // Dead-reckoned position drifts by metres within a minute — this is here to
    // show the integration working, not to navigate with.
    void reset_position();

private:
    void integrate_position(const ImuSample& s, double dt);
    // World-frame acceleration with the body offset and gravity removed.
    Vec3 linear_from(const ImuSample& s) const;

    static constexpr int kCalSamples = 50;    // ~2 s of quiet at 25 Hz
    static constexpr int kWindow     = 12;    // ~0.5 s, the stillness window

    Options opt_;
    Quat    q_;
    Quat    tare_;                        // inverse of the attitude at zero_here()
    Vec3    bias_{};                      // rad/s
    // Accelerometer offset lives in the BODY frame, because that is where it
    // physically is — it belongs to the sensor, not to the room. Held in world
    // axes it would be correct at one attitude and wrong at every other, which
    // turns a pure rotation into metres of phantom travel.
    Vec3    accel_bias_{};                // m/s^2, body frame
    bool    accel_bias_valid_ = false;
    int     bias_seed_count_  = 0;
    int     not_still_count_  = 0;
    int     settle_countdown_ = 0;   // samples to wait after a rotation stops
    bool    quiet_var_        = false;   // window says nothing is disturbing it
    Vec3    integral_{};                  // Mahony integral term
    Vec3    lin_accel_{}, vel_{}, pos_{};
    Vec3    cal_sum_{};
    int     cal_samples_   = 0;
    uint64_t last_us_      = 0;
    bool    still_         = false;
    int     still_count_   = 0;
    bool    mag_yaw_valid_ = false;

    double  a_win_[kWindow]{};      // rolling |a| and |w|, for the variance test
    double  g_win_[kWindow]{};
    int     win_i_ = 0;
    int     win_n_ = 0;
};

}  // namespace imu

#endif  // IMU_YAHBOOM_ATTITUDE_HPP
