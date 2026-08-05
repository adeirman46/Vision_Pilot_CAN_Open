#include <imu/attitude.hpp>

#include <algorithm>
#include <cmath>

namespace imu {

namespace {

Quat mul(const Quat& a, const Quat& b)
{
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Quat conj(const Quat& q)
{
    return {q.w, -q.x, -q.y, -q.z};
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

bool normalize(Vec3& v)
{
    const double n = v.norm();
    if (n < 1e-9) return false;
    v.x /= n; v.y /= n; v.z /= n;
    return true;
}

}  // namespace

// ─── Quat ────────────────────────────────────────────────────────────────────

void Quat::normalize()
{
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n < 1e-12) { w = 1.0; x = y = z = 0.0; return; }
    w /= n; x /= n; y /= n; z /= n;
}

// Aerospace order: yaw about z, then pitch about y, then roll about x — the
// rotations the labels on the 3D view mean.
Vec3 Quat::to_euler_deg() const
{
    Vec3 e;
    const double sinr = 2.0 * (w * x + y * z);
    const double cosr = 1.0 - 2.0 * (x * x + y * y);
    e.x = std::atan2(sinr, cosr) * kRad2Deg;

    double sinp = 2.0 * (w * y - z * x);
    sinp = std::clamp(sinp, -1.0, 1.0);      // straight up/down: no gimbal blow-up
    e.y = std::asin(sinp) * kRad2Deg;

    const double siny = 2.0 * (w * z + x * y);
    const double cosy = 1.0 - 2.0 * (y * y + z * z);
    e.z = std::atan2(siny, cosy) * kRad2Deg;
    return e;
}

Vec3 Quat::rotate(const Vec3& v) const
{
    // v + 2 * q_vec x (q_vec x v + w * v)
    const Vec3 qv{x, y, z};
    const Vec3 t = cross(qv, {v.x, v.y, v.z});
    const Vec3 u{t.x + w * v.x, t.y + w * v.y, t.z + w * v.z};
    const Vec3 r = cross(qv, u);
    return {v.x + 2.0 * r.x, v.y + 2.0 * r.y, v.z + 2.0 * r.z};
}

Quat Quat::from_euler_deg(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll * kDeg2Rad * 0.5);
    const double sr = std::sin(roll * kDeg2Rad * 0.5);
    const double cp = std::cos(pitch * kDeg2Rad * 0.5);
    const double sp = std::sin(pitch * kDeg2Rad * 0.5);
    const double cy = std::cos(yaw * kDeg2Rad * 0.5);
    const double sy = std::sin(yaw * kDeg2Rad * 0.5);

    Quat q{cr * cp * cy + sr * sp * sy,
           sr * cp * cy - cr * sp * sy,
           cr * sp * cy + sr * cp * sy,
           cr * cp * sy - sr * sp * cy};
    q.normalize();
    return q;
}

// ─── AttitudeEstimator ───────────────────────────────────────────────────────

AttitudeEstimator::AttitudeEstimator() = default;

AttitudeEstimator::AttitudeEstimator(const Options& opt) : opt_(opt) {}

void AttitudeEstimator::reset()
{
    q_    = Quat{};
    tare_ = Quat{};
    bias_ = integral_ = accel_bias_ = Vec3{};
    accel_bias_valid_ = false;
    bias_seed_count_  = 0;
    not_still_count_  = 0;
    settle_countdown_ = 0;
    quiet_var_        = false;
    win_i_ = win_n_   = 0;
    lin_accel_ = vel_ = pos_ = Vec3{};
    cal_sum_       = Vec3{};
    cal_samples_   = 0;
    last_us_       = 0;
    still_         = false;
    still_count_   = 0;
    mag_yaw_valid_ = false;
}

void AttitudeEstimator::zero_here()
{
    tare_ = conj(q_);
}

void AttitudeEstimator::reset_position()
{
    pos_ = vel_ = Vec3{};
}

double AttitudeEstimator::calibration_progress() const
{
    return std::clamp(static_cast<double>(cal_samples_) / kCalSamples, 0.0, 1.0);
}

Vec3 AttitudeEstimator::euler_deg() const
{
    return mul(tare_, q_).to_euler_deg();
}

Vec3 AttitudeEstimator::gyro_bias_dps() const
{
    return {bias_.x * kRad2Deg, bias_.y * kRad2Deg, bias_.z * kRad2Deg};
}

double AttitudeEstimator::heading_deg() const
{
    // Yaw counts anticlockwise from the world x axis; a compass counts
    // clockwise from north, and with a magnetometer world x *is* north.
    const double yaw = euler_deg().z;
    return std::fmod(360.0 - yaw + 360.0, 360.0);
}

// Body-frame offset out, then into world axes, then gravity out. Doing the
// offset removal in the body frame is the whole point: it is a property of the
// sensor and rotates with it.
Vec3 AttitudeEstimator::linear_from(const ImuSample& s) const
{
    const Vec3 body{s.accel.x - accel_bias_.x, s.accel.y - accel_bias_.y,
                    s.accel.z - accel_bias_.z};
    const Vec3 w = q_.rotate(body);
    return {w.x, w.y, w.z - kGravity};
}

void AttitudeEstimator::update(const ImuSample& s)
{
    double dt = 0.0;
    if (last_us_ != 0 && s.stamp_us > last_us_)
        dt = static_cast<double>(s.stamp_us - last_us_) * 1e-6;
    last_us_ = s.stamp_us;
    // A stalled stream must not be integrated as one huge step, and the first
    // sample has no interval at all.
    dt = std::clamp(dt, 0.0, 0.1);

    // ── Is the unit sitting still? ───────────────────────────────────────────
    //
    // Everything downstream depends on getting this right: the zero-velocity
    // update is the only thing standing between dead reckoning and a track that
    // wanders off, and both bias estimates are learned here.
    //
    // The test is on *variance* over a short window, never on magnitude. Two
    // separate traps make the obvious tests fail:
    //
    //   A gyro at rest on this part reads a steady -1.7 deg/s of bias. Testing
    //   |w| against a threshold calls that rotation, so the unit is never still,
    //   so the bias is never learned, so it is never still — a deadlock that
    //   ends with velocity ramping and the position metres away.
    //
    //   Sliding the board without tilting it barely changes |a| — 1 m/s²
    //   sideways moves it by 0.05 — so testing |a| against g calls real
    //   translation "at rest" and the zero-velocity update deletes exactly the
    //   motion being measured.
    //
    // A constant offset has zero variance and a real movement does not, which
    // separates the two cleanly.
    a_win_[win_i_] = s.has_accel ? s.accel.norm() : 0.0;
    g_win_[win_i_] = s.has_gyro ? s.gyro.norm() * kRad2Deg : 0.0;
    win_i_ = (win_i_ + 1) % kWindow;
    if (win_n_ < kWindow) ++win_n_;

    bool quiet = false;
    if (win_n_ >= kWindow) {
        double a_mean = 0.0, g_mean = 0.0;
        for (int i = 0; i < kWindow; ++i) { a_mean += a_win_[i]; g_mean += g_win_[i]; }
        a_mean /= kWindow;
        g_mean /= kWindow;

        double a_var = 0.0, g_var = 0.0;
        for (int i = 0; i < kWindow; ++i) {
            a_var += (a_win_[i] - a_mean) * (a_win_[i] - a_mean);
            g_var += (g_win_[i] - g_mean) * (g_win_[i] - g_mean);
        }
        quiet = std::sqrt(a_var / kWindow) < opt_.still_accel_std &&
                std::sqrt(g_var / kWindow) < opt_.still_gyro_std;
    }

    quiet_var_ = quiet;

    // Variance alone is not enough to declare rest, because steady acceleration
    // is steady: pushing the board at a constant 1 m/s² moves |a| by 0.05 and
    // varies it no more than noise does. Taken on its own the test calls a real
    // movement "still", and the zero-velocity update then deletes all of it.
    //
    // So the two tests do different jobs. Low variance means "not being
    // disturbed", which is when an offset can be measured. Rest additionally
    // requires the linear acceleration itself to be near zero once that offset
    // is removed — which steady acceleration is not.
    // q_ and accel_bias_ are one sample old here; at 25 Hz that is close enough
    // to classify motion with.
    const double lin_mag = s.has_accel ? linear_from(s).norm() : 0.0;

    // Debounced, so the instant of zero acceleration as a movement reverses
    // cannot be mistaken for the end of it.
    const bool at_rest = quiet && (!s.has_accel || lin_mag < opt_.still_lin_ms2);
    still_count_ = at_rest ? std::min(still_count_ + 1, 400) : 0;
    still_       = still_count_ > 10;

    // Gyro bias: whatever the rate gyro reads while nothing is disturbing the
    // board is offset. This one keys off the variance test rather than full
    // rest, because linear acceleration does not affect a rate gyro — low gyro
    // variance already means it is not turning. Tracked continuously rather
    // than averaged once, since the offset moves with temperature.
    if (s.has_gyro && quiet_var_) {
        const double a = 0.05;
        bias_.x += a * (s.gyro.x - bias_.x);
        bias_.y += a * (s.gyro.y - bias_.y);
        bias_.z += a * (s.gyro.z - bias_.z);
        if (cal_samples_ < kCalSamples) ++cal_samples_;
    }

    // The module's own solution wins when it has one: its filter runs on the
    // factory magnetometer calibration, which nothing here can reproduce.
    if (opt_.prefer_device && s.has_quat) {
        Quat q{s.qw, s.qx, s.qy, s.qz};
        if (std::fabs(q.w) + std::fabs(q.x) + std::fabs(q.y) + std::fabs(q.z) > 1e-6) {
            q.normalize();
            q_ = q;
            mag_yaw_valid_ = s.has_mag;
            integrate_position(s, dt);
            return;
        }
    }
    if (opt_.prefer_device && s.has_euler) {
        q_ = Quat::from_euler_deg(s.roll_deg, s.pitch_deg, s.yaw_deg);
        mag_yaw_valid_ = s.has_mag;
        integrate_position(s, dt);
        return;
    }

    // ── Mahony complementary filter ──────────────────────────────────────────
    Vec3 omega{0.0, 0.0, 0.0};
    if (s.has_gyro) {
        omega = {s.gyro.x - bias_.x, s.gyro.y - bias_.y, s.gyro.z - bias_.z};
    }

    Vec3 error{0.0, 0.0, 0.0};

    if (s.has_accel) {
        Vec3 a = s.accel;
        if (normalize(a)) {
            // Where the filter currently thinks "up" lies, expressed in body
            // axes. The accelerometer says where it really is; the cross product
            // between the two is the tilt error to correct.
            const Vec3 up{2.0 * (q_.x * q_.z - q_.w * q_.y),
                          2.0 * (q_.w * q_.x + q_.y * q_.z),
                          q_.w * q_.w - q_.x * q_.x - q_.y * q_.y + q_.z * q_.z};
            const Vec3 e = cross(a, up);
            error.x += e.x; error.y += e.y; error.z += e.z;
        }
    }

    if (opt_.use_mag && s.has_mag) {
        Vec3 m = s.mag;
        if (normalize(m)) {
            // Rotate the field into world axes, flatten it onto the horizontal
            // plane, and rotate that reference back: the residual is pure
            // heading error, so tilt never leaks into yaw.
            const Vec3 h = q_.rotate(m);
            const double bx = std::hypot(h.x, h.y);
            const Vec3 ref  = conj(q_).rotate({bx, 0.0, h.z});
            const Vec3 e    = cross(m, ref);
            error.x += e.x; error.y += e.y; error.z += e.z;
            mag_yaw_valid_ = true;
        }
    } else {
        mag_yaw_valid_ = false;
    }

    if (dt > 0.0) {
        if (opt_.ki > 0.0) {
            integral_.x += error.x * opt_.ki * dt;
            integral_.y += error.y * opt_.ki * dt;
            integral_.z += error.z * opt_.ki * dt;
            omega.x += integral_.x;
            omega.y += integral_.y;
            omega.z += integral_.z;
        }
        omega.x += opt_.kp * error.x;
        omega.y += opt_.kp * error.y;
        omega.z += opt_.kp * error.z;

        const Quat dq{0.0, omega.x, omega.y, omega.z};
        const Quat rate = mul(q_, dq);
        q_.w += 0.5 * rate.w * dt;
        q_.x += 0.5 * rate.x * dt;
        q_.y += 0.5 * rate.y * dt;
        q_.z += 0.5 * rate.z * dt;
        q_.normalize();
    }

    integrate_position(s, dt);
}

// Dead reckoning. Double integration is unforgiving: the error that dominates
// is not noise but *offset*, because a constant residual acceleration grows as
// t². Two things matter far more than the integration itself.
//
// First, the residual has to be measured, not assumed. At rest the world-frame
// acceleration must be exactly (0, 0, g); whatever is left over is accelerometer
// bias plus the tilt error in the attitude, and a mere 1° of tilt error leaks
// 0.17 m/s² into the horizontal plane — 8 m of phantom travel in ten seconds.
// So the leftover is learned whenever the unit is demonstrably still, and
// subtracted from then on. That single term is the difference between a track
// that sits still on the desk and one that slides off it.
//
// Second, the zero-velocity update: standing still means velocity is zero, and
// saying so out loud stops the remaining error accumulating between movements.
//
// What is left is honest short-term motion — good for a few seconds, drifting
// after that. That is precisely the signal GPS is there to correct.
void AttitudeEstimator::integrate_position(const ImuSample& s, double dt)
{
    if (!s.has_accel) return;

    // At rest the sensor should read gravity rotated into its own axes, and
    // nothing else. Whatever is left over is the offset — measured, and stored,
    // in the body frame.
    const Vec3 expect = conj(q_).rotate(Vec3{0.0, 0.0, kGravity});
    const Vec3 resid{s.accel.x - expect.x, s.accel.y - expect.y, s.accel.z - expect.z};

    if (!accel_bias_valid_) {
        // Bootstrap. Rest cannot be declared yet, because deciding that needs
        // the very offset being measured here; low variance is the best
        // available evidence that nothing is disturbing the board. A second of
        // it is enough, after which the stricter test takes over.
        // Sanity bound as well as the variance test: a board being carried can
        // hold a steady acceleration long enough to look quiet, and seeding the
        // offset from that bakes the movement in permanently.
        if (quiet_var_ && resid.norm() < 1.0) {
            const double a = 0.10;
            accel_bias_.x += a * (resid.x - accel_bias_.x);
            accel_bias_.y += a * (resid.y - accel_bias_.y);
            accel_bias_.z += a * (resid.z - accel_bias_.z);
            if (++bias_seed_count_ > 25) accel_bias_valid_ = true;
        }
    } else if (still_) {
        // Refined only at genuine rest, and slowly: chasing it any faster would
        // absorb real acceleration into the offset and cancel the movement.
        const double a = 0.02;
        accel_bias_.x += a * (resid.x - accel_bias_.x);
        accel_bias_.y += a * (resid.y - accel_bias_.y);
        accel_bias_.z += a * (resid.z - accel_bias_.z);
    }

    // Self-healing. If the offset was seeded from something that was not
    // actually rest, every later sample is measured against a wrong reference
    // and the unit never looks still again — the error is permanent and silent.
    // Going this long without a single moment of rest is far more likely to mean
    // a bad reference than genuinely continuous motion, so throw it away and
    // measure again. Whatever velocity was integrated against it is worthless.
    if (still_) {
        not_still_count_ = 0;
    } else if (accel_bias_valid_ && ++not_still_count_ > 250) {   // 10 s at 25 Hz
        accel_bias_valid_ = false;
        bias_seed_count_  = 0;
        not_still_count_  = 0;
        vel_ = Vec3{};
    }

    lin_accel_ = linear_from(s);

    if (dt <= 0.0) return;

    if (still_) {
        vel_ = Vec3{};                     // zero-velocity update
        return;
    }

    // One threshold on the magnitude, not three on the axes: a per-axis cut
    // deletes the small components of a real diagonal movement and bends the
    // track towards whichever axis happened to be largest.
    //
    // The bleed only runs while coasting. Damping velocity *during* a real
    // acceleration is damping the measurement itself — it costs a fixed
    // fraction of every movement, which showed up as a metre of travel reading
    // 87 cm. Between movements there is no signal to protect, so that is where
    // the leftover bias gets bled off instead.
    // While the board is turning quickly, do not integrate at all. Gravity is
    // 9.8 m/s^2 and the attitude solution always lags the real motion a little;
    // two degrees of lag leaks 0.34 m/s^2 into the horizontal plane, which is
    // larger than most real movements and points wherever the rotation happens
    // to be facing. Integrating through a turn is what makes spinning the board
    // on the spot walk the position metres away. There is no useful translation
    // signal to lose here — a hand rotating an object is not carrying it far.
    //
    // The hold-off matters as much as the gate. The module's own attitude filter
    // does not settle the instant rotation stops — it keeps converging for a few
    // tenths of a second afterwards, and every degree it is still out by is
    // another 0.17 m/s² of gravity pointing the wrong way. Resuming integration
    // the moment the gyro goes quiet feeds exactly that error straight into
    // velocity.
    const double spin_dps = s.has_gyro ? s.gyro.norm() * kRad2Deg : 0.0;
    if (spin_dps > opt_.rotation_gate_dps) {
        settle_countdown_ = 12;                      // ~0.5 s at 25 Hz
    } else if (settle_countdown_ > 0) {
        --settle_countdown_;
    }

    if (settle_countdown_ > 0) {
        const double leak = std::exp(-dt / 1.0);
        vel_.x *= leak; vel_.y *= leak; vel_.z *= leak;
    } else if (lin_accel_.norm() > 0.08) {
        vel_.x += lin_accel_.x * dt;
        vel_.y += lin_accel_.y * dt;
        vel_.z += lin_accel_.z * dt;
    } else {
        const double leak = std::exp(-dt / 3.0);
        vel_.x *= leak; vel_.y *= leak; vel_.z *= leak;
    }

    pos_.x += vel_.x * dt;
    pos_.y += vel_.y * dt;
    pos_.z += vel_.z * dt;
}

}  // namespace imu
