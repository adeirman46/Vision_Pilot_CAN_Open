#include "scene3d.hpp"

#include <algorithm>
#include <cmath>

#include <QFont>
#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPolygonF>

namespace imu::view {

namespace {

V3 sub(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

V3 cross(const V3& a, const V3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

V3 norm(const V3& a)
{
    const double n = std::sqrt(dot(a, a));
    return n < 1e-12 ? V3{0, 0, 1} : V3{a.x / n, a.y / n, a.z / n};
}

V3 rotate(const Quat& q, const V3& v)
{
    const Vec3 r = q.rotate(Vec3{v.x, v.y, v.z});
    return {r.x, r.y, r.z};
}

// Add a box as six quads. Everything in the airframe is a box, so this is the
// whole modelling toolkit.
void add_box(Mesh& m, const V3& c, const V3& half, const QColor& col)
{
    const int b = static_cast<int>(m.v.size());
    for (int i = 0; i < 8; ++i) {
        m.v.push_back({c.x + ((i & 1) ? half.x : -half.x),
                       c.y + ((i & 2) ? half.y : -half.y),
                       c.z + ((i & 4) ? half.z : -half.z)});
    }
    const int quads[6][4] = {
        {0, 1, 3, 2},  // -z
        {4, 6, 7, 5},  // +z
        {0, 4, 5, 1},  // -y
        {2, 3, 7, 6},  // +y
        {0, 2, 6, 4},  // -x
        {1, 5, 7, 3},  // +x
    };
    for (const auto& q : quads)
        m.f.push_back({{b + q[0], b + q[1], b + q[2], b + q[3]}, col});
}

// Nose cone: a ring pulled forward to a point, so the model has an unmistakable
// front end even when it is pointing straight at the camera.
void add_cone_x(Mesh& m, double base_x, double tip_x, double r, const QColor& col)
{
    const int b     = static_cast<int>(m.v.size());
    const int kSeg  = 10;
    for (int i = 0; i < kSeg; ++i) {
        const double a = 2.0 * M_PI * i / kSeg;
        m.v.push_back({base_x, r * std::cos(a), r * std::sin(a)});
    }
    m.v.push_back({tip_x, 0.0, 0.0});
    const int tip = b + kSeg;
    for (int i = 0; i < kSeg; ++i)
        m.f.push_back({{b + i, b + (i + 1) % kSeg, tip}, col});
}

const QColor kBody{236, 239, 244};
const QColor kWing{214, 222, 235};
const QColor kTrim{ 96, 165, 250};

const QColor kRoll { 239,  68,  68};   // x — nose axis
const QColor kPitch{ 34, 197,  94};    // y — wing axis
const QColor kYaw  { 59, 130, 246};    // z — vertical axis

}  // namespace

Mesh make_aircraft()
{
    Mesh m;
    // x forward (nose), y left, z up — the same axes the sensor's own datasheet
    // draws on the board.
    add_box(m, {0.05, 0.0, 0.0},   {1.35, 0.16, 0.17}, kBody);   // fuselage
    add_cone_x(m, 1.40, 1.95, 0.16, kTrim);                      // nose
    add_box(m, {0.10, 0.0, 0.10},  {0.42, 1.75, 0.045}, kWing);  // main wing
    add_box(m, {-1.15, 0.0, 0.06}, {0.26, 0.62, 0.04}, kWing);   // stabiliser
    add_box(m, {-1.15, 0.0, 0.42}, {0.24, 0.035, 0.32}, kTrim);  // fin
    add_box(m, {0.62, 0.0, 0.16},  {0.34, 0.12, 0.09}, kTrim);   // canopy
    return m;
}

// ─── Camera ──────────────────────────────────────────────────────────────────

void Renderer::set_viewport(const QRectF& r)
{
    viewport_ = r;
    // Keep the model the same apparent size whatever the window does.
    focal_ = std::max(240.0, std::min(r.width(), r.height() * 1.35) * 0.95);
}

void Renderer::set_camera(double azimuth_deg, double elevation_deg, double distance)
{
    azimuth_   = azimuth_deg;
    elevation_ = std::clamp(elevation_deg, -85.0, 85.0);
    distance_  = std::clamp(distance, 3.0, 30.0);
}

void Renderer::orbit(double d_az, double d_el)
{
    azimuth_   = std::fmod(azimuth_ + d_az + 360.0, 360.0);
    elevation_ = std::clamp(elevation_ + d_el, -85.0, 85.0);
}

void Renderer::zoom(double factor)
{
    distance_ = std::clamp(distance_ * factor, 3.0, 30.0);
}

void Renderer::reset_camera()
{
    azimuth_   = 38.0;
    elevation_ = 22.0;
    distance_  = 8.5;
}

V3 Renderer::to_camera(const V3& world) const
{
    const double az = azimuth_ * kDeg2Rad;
    const double el = elevation_ * kDeg2Rad;
    const V3 eye{distance_ * std::cos(el) * std::cos(az),
                 distance_ * std::cos(el) * std::sin(az),
                 distance_ * std::sin(el)};

    const V3 fwd   = norm(sub(V3{0, 0, 0}, eye));       // looking at the origin
    const V3 right = norm(cross(fwd, V3{0, 0, 1}));
    const V3 up    = cross(right, fwd);
    const V3 rel   = sub(world, eye);
    return {dot(rel, right), dot(rel, up), dot(rel, fwd)};
}

bool Renderer::project(const V3& p, QPointF& out) const
{
    const V3 c = to_camera(p);
    if (c.z < 0.05) return false;      // behind the camera, or on the lens
    out = QPointF(viewport_.center().x() + focal_ * c.x / c.z,
                  viewport_.center().y() - focal_ * c.y / c.z);
    return true;
}

// Clip against the near plane so a line running past the camera is trimmed
// instead of flipping to the far side of the screen.
void Renderer::draw_line3(QPainter& p, const V3& a, const V3& b, double width,
                          const QColor& c) const
{
    V3 ca = to_camera(a), cb = to_camera(b);
    const double kNear = 0.05;
    if (ca.z < kNear && cb.z < kNear) return;
    if (ca.z < kNear) {
        const double t = (kNear - ca.z) / (cb.z - ca.z);
        ca = {ca.x + (cb.x - ca.x) * t, ca.y + (cb.y - ca.y) * t, kNear};
    } else if (cb.z < kNear) {
        const double t = (kNear - cb.z) / (ca.z - cb.z);
        cb = {cb.x + (ca.x - cb.x) * t, cb.y + (ca.y - cb.y) * t, kNear};
    }

    const QPointF pa(viewport_.center().x() + focal_ * ca.x / ca.z,
                     viewport_.center().y() - focal_ * ca.y / ca.z);
    const QPointF pb(viewport_.center().x() + focal_ * cb.x / cb.z,
                     viewport_.center().y() - focal_ * cb.y / cb.z);

    p.setPen(QPen(c, width, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(pa, pb);
}

// ─── Scene elements ──────────────────────────────────────────────────────────

void Renderer::draw_grid(QPainter& p, const QColor& line, const QColor& accent) const
{
    constexpr double kExtent = 7.0;
    constexpr double kStep   = 1.0;
    constexpr double kFloor  = -1.6;

    for (double t = -kExtent; t <= kExtent + 1e-9; t += kStep) {
        // Fade the far lines out so the plane reads as a floor rather than a
        // wall of wires.
        const double fade = 1.0 - std::fabs(t) / (kExtent * 1.6);
        QColor c = (std::fabs(t) < 1e-6) ? accent : line;
        c.setAlphaF(std::clamp(c.alphaF() * fade, 0.05, 1.0));
        const double w = (std::fabs(t) < 1e-6) ? 1.6 : 1.0;
        draw_line3(p, {t, -kExtent, kFloor}, {t, kExtent, kFloor}, w, c);
        draw_line3(p, {-kExtent, t, kFloor}, {kExtent, t, kFloor}, w, c);
    }
}

void Renderer::draw_shadow(QPainter& p, const Mesh& m, const Quat& q, const V3& at) const
{
    // Flatten the hull onto the floor. It is not a real shadow, but it is what
    // makes yaw readable — the outline turns even when the silhouette does not.
    constexpr double kFloor = -1.59;
    QPolygonF poly;
    for (const V3& v : m.v) {
        const V3 r = rotate(q, v);
        QPointF pt;
        if (project({r.x + at.x, r.y + at.y, kFloor}, pt)) poly << pt;
    }
    if (poly.size() < 3) return;

    // Convex hull (gift wrap) — the projected points are unordered, and drawing
    // them raw gives a star instead of a silhouette.
    QPolygonF hull;
    int leftmost = 0;
    for (int i = 1; i < poly.size(); ++i)
        if (poly[i].x() < poly[leftmost].x()) leftmost = i;
    int cur = leftmost;
    do {
        hull << poly[cur];
        int next = (cur + 1) % poly.size();
        for (int i = 0; i < poly.size(); ++i) {
            const QPointF a = poly[cur], b = poly[next], c = poly[i];
            const double turn = (b.x() - a.x()) * (c.y() - a.y()) -
                                (b.y() - a.y()) * (c.x() - a.x());
            if (turn < 0) next = i;
        }
        cur = next;
    } while (cur != leftmost && hull.size() < poly.size());

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 70));
    p.drawPolygon(hull);
}

void Renderer::draw_mesh(QPainter& p, const Mesh& m, const Quat& q, const V3& at) const
{
    struct Poly { QPolygonF pts; QColor col; double depth; };
    std::vector<Poly> polys;
    polys.reserve(m.f.size());

    const V3 light = norm({0.4, 0.55, 0.75});

    for (const Face& f : m.f) {
        QPolygonF pts;
        double    depth = 0.0;
        bool      ok    = true;
        std::vector<V3> world;
        world.reserve(f.idx.size());

        for (int i : f.idx) {
            const V3 r = rotate(q, m.v[i]);
            const V3 w{r.x + at.x, r.y + at.y, r.z + at.z};
            world.push_back(w);
            QPointF pt;
            if (!project(w, pt)) { ok = false; break; }
            pts << pt;
            depth += to_camera(w).z;
        }
        if (!ok || pts.size() < 3) continue;
        depth /= f.idx.size();

        const V3 n = norm(cross(sub(world[1], world[0]), sub(world[2], world[0])));
        // Two-sided lighting: the model is a thin shell, so a face turned away
        // is still visible from inside and should not go black.
        const double lam = std::fabs(dot(n, light));
        const double s   = 0.42 + 0.58 * lam;

        QColor c = f.color;
        c.setRgbF(std::clamp(c.redF() * s, 0.0, 1.0),
                  std::clamp(c.greenF() * s, 0.0, 1.0),
                  std::clamp(c.blueF() * s, 0.0, 1.0));
        polys.push_back({pts, c, depth});
    }

    // Painter's algorithm: far faces first.
    std::sort(polys.begin(), polys.end(),
              [](const Poly& a, const Poly& b) { return a.depth > b.depth; });

    for (const Poly& poly : polys) {
        p.setBrush(poly.col);
        p.setPen(QPen(poly.col.darker(160), 1.0));
        p.drawPolygon(poly.pts);
    }
}

// Where the module has been since the last reset. Dead reckoning drifts, so the
// trail is as much a picture of the drift as of the motion — which is exactly
// what makes it worth showing before the GPS is folded in.
void Renderer::draw_trail(QPainter& p, const std::vector<V3>& path, const QColor& c) const
{
    if (path.size() < 2) return;

    constexpr double kFloor = -1.58;
    for (size_t i = 1; i < path.size(); ++i) {
        // Fade the oldest end out so the current direction of travel reads
        // clearly rather than getting lost in the history.
        const double age = static_cast<double>(i) / path.size();
        QColor       cc  = c;
        cc.setAlphaF(0.12 + 0.78 * age);
        draw_line3(p, path[i - 1], path[i], 1.0 + 1.6 * age, cc);

        QColor ground = cc;
        ground.setAlphaF(cc.alphaF() * 0.4);
        draw_line3(p, {path[i - 1].x, path[i - 1].y, kFloor},
                   {path[i].x, path[i].y, kFloor}, 1.0, ground);
    }
}

void Renderer::draw_world_arrow(QPainter& p, const V3& from, const V3& to,
                                const QColor& c, double width) const
{
    draw_line3(p, from, to, width, c);

    QPointF tip, tail;
    if (!project(to, tip) || !project(from, tail)) return;

    QPointF dir = tip - tail;
    const double len = std::hypot(dir.x(), dir.y());
    if (len < 1e-6) return;
    dir /= len;
    const QPointF nrm(-dir.y(), dir.x());
    const double  h = std::max(9.0, width * 4.5);

    QPolygonF head;
    head << tip << (tip - dir * h + nrm * h * 0.42)
         << (tip - dir * h - nrm * h * 0.42);
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    p.drawPolygon(head);
}

// The curved arrow that says "this axis is the one you turn about" — the same
// convention as the roll/pitch/yaw diagram in any flight-dynamics text.
void Renderer::draw_spin_arrow(QPainter& p, const V3& axis, const V3& ref,
                               double radius, const V3& at, const QColor& c) const
{
    const V3 a = norm(axis);
    V3       u = norm(sub(ref, V3{a.x * dot(ref, a), a.y * dot(ref, a), a.z * dot(ref, a)}));
    const V3 v = cross(a, u);

    constexpr int    kSeg  = 26;
    constexpr double kSpan = 250.0 * kDeg2Rad;

    std::vector<V3> pts;
    pts.reserve(kSeg + 1);
    for (int i = 0; i <= kSeg; ++i) {
        const double t = kSpan * i / kSeg;
        pts.push_back({at.x + radius * (u.x * std::cos(t) + v.x * std::sin(t)),
                       at.y + radius * (u.y * std::cos(t) + v.y * std::sin(t)),
                       at.z + radius * (u.z * std::cos(t) + v.z * std::sin(t))});
    }
    for (size_t i = 1; i < pts.size(); ++i)
        draw_line3(p, pts[i - 1], pts[i], 2.4, c);

    if (pts.size() >= 2) {
        const V3& tip  = pts.back();
        const V3& prev = pts[pts.size() - 2];
        QPointF ptip, pprev;
        if (project(tip, ptip) && project(prev, pprev)) {
            QPointF dir = ptip - pprev;
            const double len = std::hypot(dir.x(), dir.y());
            if (len > 1e-6) {
                dir /= len;
                const QPointF n(-dir.y(), dir.x());
                QPolygonF head;
                head << ptip << (ptip - dir * 11.0 + n * 4.6)
                     << (ptip - dir * 11.0 - n * 4.6);
                p.setPen(Qt::NoPen);
                p.setBrush(c);
                p.drawPolygon(head);
            }
        }
    }
}

QPointF Renderer::label_at(const V3& p, bool& ok) const
{
    QPointF out;
    ok = project(p, out);
    return out;
}

void Renderer::draw_axes(QPainter& p, const Quat& q, const V3& at, bool labels) const
{
    struct Axis { V3 dir; QColor col; const char* name; V3 ref; };
    const Axis axes[] = {
        {{1, 0, 0}, kRoll,  "Roll",  {0, 0, 1}},
        {{0, 1, 0}, kPitch, "Pitch", {1, 0, 0}},
        {{0, 0, 1}, kYaw,   "Yaw",   {1, 0, 0}},
    };

    for (const Axis& ax : axes) {
        const V3 d = rotate(q, ax.dir);
        const V3 r = rotate(q, ax.ref);
        const double reach = 2.55;

        draw_line3(p, {at.x - d.x * reach * 0.55, at.y - d.y * reach * 0.55,
                       at.z - d.z * reach * 0.55},
                   at, 2.0, QColor(ax.col.red(), ax.col.green(), ax.col.blue(), 90));
        draw_world_arrow(p, at,
                         {at.x + d.x * reach, at.y + d.y * reach, at.z + d.z * reach},
                         ax.col, 2.6);
        draw_spin_arrow(p, d, r, 0.72, at,
                        QColor(ax.col.red(), ax.col.green(), ax.col.blue(), 210));

        if (!labels) continue;

        bool ok = false;
        const QPointF pos = label_at({at.x + d.x * (reach + 0.42),
                                      at.y + d.y * (reach + 0.42),
                                      at.z + d.z * (reach + 0.42)}, ok);
        if (!ok) continue;

        QFont f = p.font();
        f.setPointSizeF(12.5);
        f.setBold(true);
        p.setFont(f);
        const QFontMetricsF fm(f);
        const QString      text = QString::fromLatin1(ax.name);
        const QRectF box = fm.boundingRect(text).adjusted(-7, -4, 7, 4)
                               .translated(pos.x() - fm.horizontalAdvance(text) / 2.0,
                                           pos.y() + fm.ascent() / 2.0);

        // A pill behind the text keeps it readable when the axis swings over
        // the bright wing.
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(15, 20, 30, 190));
        p.drawRoundedRect(box, 6, 6);
        p.setPen(ax.col);
        p.drawText(box, Qt::AlignCenter, text);
    }
}

}  // namespace imu::view
