#include "ekf.hpp"

#include <math.h>

namespace {

const double kPi  = 3.14159265358979323846;
const double kDeg = kPi / 180.0;

double wrap_pi(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

}  // namespace

void GpsImuEkf::initialise(double lat, double lon, double heading_deg, bool have_heading)
{
    lat0_ = lat;
    lon0_ = lon;

    // Metres per degree at this latitude. Computed once, at the origin: over a
    // city the change is far below GPS noise.
    const double phi = lat * kDeg;
    m_per_deg_lat_ = 111132.92 - 559.82 * cos(2 * phi) + 1.175 * cos(4 * phi);
    m_per_deg_lon_ = 111412.84 * cos(phi) - 93.5 * cos(3 * phi);

    for (int i = 0; i < kN; ++i) {
        x_[i] = 0.0;
        for (int j = 0; j < kN; ++j) P_[i][j] = 0.0;
    }
    x_[4] = have_heading ? heading_deg * kDeg : 0.0;

    // Initial uncertainty: position is as good as the first fix, velocity is
    // unknown, and heading is either the fix's course or completely unknown.
    P_[0][0] = P_[1][1] = t_.gps_pos_sigma * t_.gps_pos_sigma;
    P_[2][2] = P_[3][3] = 4.0;
    P_[4][4] = have_heading ? (0.3 * 0.3) : (kPi * kPi);
    P_[5][5] = 0.01 * 0.01;

    init_ = true;
}

void GpsImuEkf::normalise_heading()
{
    x_[4] = wrap_pi(x_[4]);
}

void GpsImuEkf::predict(double ax, double ay, double yaw_rate, double dt)
{
    if (!init_ || dt <= 0.0) return;
    if (dt > 0.5) dt = 0.5;      // a stalled IMU must not take one giant step

    const double psi = x_[4];
    const double wz  = yaw_rate - x_[5];      // measured rate less its bias
    const double c = cos(psi), s = sin(psi);

    /* Body acceleration into the north/east plane.
     *
     * Getting this wrong is not a small error. With psi clockwise from north the
     * body x axis lies along (cos psi, sin psi) in (north, east); the body y
     * axis points *left*, which is that vector turned anticlockwise a quarter
     * turn — (sin psi, -cos psi), not (-sin psi, cos psi). Taking the latter
     * sends every lateral acceleration the opposite way, and a car corners at
     * v*omega: at 12 m/s through a 90 degree turn that is 2.4 m/s^2 applied
     * backwards, so the error is twice that for the length of the bend. */
    const double an = ax * c + ay * s;
    const double ae = ax * s - ay * c;

    x_[0] += x_[2] * dt + 0.5 * an * dt * dt;
    x_[1] += x_[3] * dt + 0.5 * ae * dt * dt;
    x_[2] += an * dt;
    x_[3] += ae * dt;
    x_[4] += wz * dt;
    normalise_heading();

    // Jacobian. Only the entries that are not identity are written.
    double F[kN][kN] = {{0}};
    for (int i = 0; i < kN; ++i) F[i][i] = 1.0;
    F[0][2] = dt;
    F[1][3] = dt;
    // Heading error rotates the acceleration, so it feeds straight into velocity
    // — this is the term that lets a GPS course correction fix the heading.
    F[2][4] = (-ax * s + ay * c) * dt;
    F[3][4] = ( ax * c + ay * s) * dt;
    F[4][5] = -dt;

    // P = F P F' + Q
    double FP[kN][kN] = {{0}};
    for (int i = 0; i < kN; ++i)
        for (int j = 0; j < kN; ++j) {
            double sum = 0.0;
            for (int k = 0; k < kN; ++k) sum += F[i][k] * P_[k][j];
            FP[i][j] = sum;
        }
    for (int i = 0; i < kN; ++i)
        for (int j = 0; j < kN; ++j) {
            double sum = 0.0;
            for (int k = 0; k < kN; ++k) sum += FP[i][k] * F[j][k];
            P_[i][j] = sum;
        }

    const double qa = t_.accel_noise * t_.accel_noise * dt;
    const double qw = t_.yaw_rate_noise * t_.yaw_rate_noise * dt;
    const double qb = t_.bias_walk * t_.bias_walk * dt;
    // Acceleration noise reaches position through the same dt^2 that the model
    // does, which is what keeps a long gap between fixes from looking more
    // certain than it is.
    P_[0][0] += qa * dt * dt * 0.25;
    P_[1][1] += qa * dt * dt * 0.25;
    P_[2][2] += qa;
    P_[3][3] += qa;
    P_[4][4] += qw;
    P_[5][5] += qb;
}

// One scalar measurement: P is symmetric, so this is a rank-1 update with no
// matrix inverse anywhere.
void GpsImuEkf::fuse_scalar(const double H[kN], double innovation, double R)
{
    double PH[kN] = {0};
    for (int i = 0; i < kN; ++i) {
        double sum = 0.0;
        for (int j = 0; j < kN; ++j) sum += P_[i][j] * H[j];
        PH[i] = sum;
    }

    double S = R;
    for (int i = 0; i < kN; ++i) S += H[i] * PH[i];
    if (S < 1e-12) return;

    double K[kN];
    for (int i = 0; i < kN; ++i) K[i] = PH[i] / S;

    for (int i = 0; i < kN; ++i) x_[i] += K[i] * innovation;
    normalise_heading();

    // Joseph-free form is adequate here and much cheaper, but P must be forced
    // symmetric afterwards or rounding slowly makes it indefinite.
    double Pn[kN][kN];
    for (int i = 0; i < kN; ++i)
        for (int j = 0; j < kN; ++j) Pn[i][j] = P_[i][j] - K[i] * PH[j];
    for (int i = 0; i < kN; ++i)
        for (int j = 0; j < kN; ++j) P_[i][j] = 0.5 * (Pn[i][j] + Pn[j][i]);
}

void GpsImuEkf::update_position(double lat, double lon, double sigma_m)
{
    if (!init_) return;

    const double zn = (lat - lat0_) * m_per_deg_lat_;
    const double ze = (lon - lon0_) * m_per_deg_lon_;

    double sigma = sigma_m > 0.5 ? sigma_m : t_.gps_pos_sigma;
    // A fix that claims to be worse than 50 m is a fix that has lost the sky;
    // let it nudge rather than teleport.
    if (sigma > 50.0) sigma = 50.0;
    const double R = sigma * sigma;

    double H[kN] = {1, 0, 0, 0, 0, 0};
    fuse_scalar(H, zn - x_[0], R);

    double H2[kN] = {0, 1, 0, 0, 0, 0};
    fuse_scalar(H2, ze - x_[1], R);
}

void GpsImuEkf::update_velocity(double speed_ms, double course_deg, bool have_course)
{
    if (!init_) return;

    const double vn = x_[2], ve = x_[3];
    const double sp = sqrt(vn * vn + ve * ve);

    // Speed: d|v|/dvn = vn/|v|. Below a crawl the derivative is meaningless, so
    // the measurement is applied to the components directly instead.
    if (sp > 0.3) {
        double H[kN] = {0, 0, vn / sp, ve / sp, 0, 0};
        fuse_scalar(H, speed_ms - sp, t_.gps_speed_sigma * t_.gps_speed_sigma);
    } else if (speed_ms < 0.5) {
        // Standing still. Saying so pins the velocity, which is what stops the
        // IMU quietly integrating a phantom crawl while the car waits at lights.
        double H[kN] = {0, 0, 1, 0, 0, 0};
        fuse_scalar(H, 0.0 - vn, 0.25);
        double H2[kN] = {0, 0, 0, 1, 0, 0};
        fuse_scalar(H2, 0.0 - ve, 0.25);
    }

    // Course only while genuinely moving: a stopped receiver reports a heading
    // that wanders over the whole circle, and feeding that in would spin the
    // map. This is the measurement that keeps yaw honest without a magnetometer.
    if (have_course && speed_ms > t_.course_min_speed) {
        double H[kN] = {0, 0, 0, 0, 1, 0};
        const double innov = wrap_pi(course_deg * kDeg - x_[4]);
        fuse_scalar(H, innov, t_.gps_course_sigma * t_.gps_course_sigma);
    }
}

double GpsImuEkf::latitude() const  { return lat0_ + x_[0] / m_per_deg_lat_; }
double GpsImuEkf::longitude() const { return lon0_ + x_[1] / m_per_deg_lon_; }
double GpsImuEkf::speed_ms() const  { return sqrt(x_[2] * x_[2] + x_[3] * x_[3]); }

double GpsImuEkf::heading_deg() const
{
    return fmod(x_[4] / kDeg + 360.0, 360.0);
}

double GpsImuEkf::yaw_rate_bias_dps() const { return x_[5] / kDeg; }

double GpsImuEkf::position_sigma_m() const
{
    const double v = P_[0][0] + P_[1][1];
    return v > 0.0 ? sqrt(v) : 0.0;
}
