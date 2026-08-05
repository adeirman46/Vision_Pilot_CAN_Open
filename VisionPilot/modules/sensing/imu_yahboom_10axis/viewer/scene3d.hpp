#ifndef IMU_YAHBOOM_SCENE3D_HPP
#define IMU_YAHBOOM_SCENE3D_HPP

#include <vector>

#include <QColor>
#include <QPointF>
#include <QRectF>
#include <QString>

#include <imu/attitude.hpp>

class QPainter;

namespace imu::view {

struct V3
{
    double x = 0.0, y = 0.0, z = 0.0;
};

struct Face
{
    std::vector<int> idx;      // indices into Mesh::v, wound anticlockwise
    QColor           color;
};

struct Mesh
{
    std::vector<V3>   v;
    std::vector<Face> f;
};

// The model that stands in for the sensor board: a small aircraft, because roll,
// pitch and yaw are only unambiguous on a shape with a nose and a pair of wings.
Mesh make_aircraft();

// Software 3D: transform, sort back-to-front, fill with QPainter. A few hundred
// polygons at 60 Hz costs nothing and avoids depending on a GL context, which on
// a machine that is already running TensorRT is one less thing to fight over.
class Renderer
{
public:
    void set_viewport(const QRectF& r);
    void set_camera(double azimuth_deg, double elevation_deg, double distance);
    void orbit(double d_azimuth_deg, double d_elevation_deg);
    void zoom(double factor);
    void reset_camera();

    double azimuth() const { return azimuth_; }
    double elevation() const { return elevation_; }

    // True when the point is in front of the camera; `out` is then its pixel.
    bool project(const V3& p, QPointF& out) const;

    // `at` is where the model sits in the world, in metres — the dead-reckoned
    // position, so the aircraft translates as well as turns.
    void draw_grid(QPainter& p, const QColor& line, const QColor& accent) const;
    void draw_mesh(QPainter& p, const Mesh& m, const Quat& q, const V3& at) const;
    void draw_shadow(QPainter& p, const Mesh& m, const Quat& q, const V3& at) const;
    void draw_axes(QPainter& p, const Quat& q, const V3& at, bool labels) const;
    void draw_trail(QPainter& p, const std::vector<V3>& path, const QColor& c) const;
    void draw_world_arrow(QPainter& p, const V3& from, const V3& to,
                          const QColor& c, double width) const;

private:
    V3     to_camera(const V3& world) const;
    void   draw_line3(QPainter& p, const V3& a, const V3& b, double width,
                      const QColor& c) const;
    void   draw_spin_arrow(QPainter& p, const V3& axis, const V3& ref, double radius,
                           const V3& at, const QColor& c) const;
    QPointF label_at(const V3& p, bool& ok) const;

    QRectF viewport_{0, 0, 800, 600};
    double azimuth_   = 38.0;    // degrees around the world z axis
    double elevation_ = 22.0;    // degrees above the horizon
    double distance_  = 8.5;
    double focal_     = 700.0;
};

}  // namespace imu::view

#endif  // IMU_YAHBOOM_SCENE3D_HPP
