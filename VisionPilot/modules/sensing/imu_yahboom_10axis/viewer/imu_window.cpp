#include "imu_window.hpp"

#include <cmath>

#include <QApplication>
#include <QFontMetricsF>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>
#include <QWheelEvent>

namespace imu::view {

namespace {

const QColor kBg     {17, 21, 29};
const QColor kPanel  {24, 29, 40};
const QColor kInk    {226, 232, 240};
const QColor kMuted  {129, 143, 165};
const QColor kGrid   {58, 70, 92};
const QColor kAxisX  {239, 68, 68};
const QColor kAxisY  {34, 197, 94};
const QColor kAxisZ  {59, 130, 246};
const QColor kGood   {74, 222, 128};
const QColor kWarn   {250, 204, 21};
const QColor kBad    {248, 113, 113};

QString deg(double v)
{
    return QString::number(v, 'f', 1) + QString::fromUtf8("°");
}

}  // namespace

// ─── SceneWidget ─────────────────────────────────────────────────────────────

SceneWidget::SceneWidget(QWidget* parent) : QWidget(parent)
{
    model_ = make_aircraft();
    setMinimumSize(420, 380);
    setMouseTracking(false);
    setAutoFillBackground(false);
}

void SceneWidget::setOrientation(const Quat& q)
{
    q_ = q;
    update();
}

void SceneWidget::resetView()
{
    renderer_.reset_camera();
    update();
}

void SceneWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    QLinearGradient bg(0, 0, 0, height());
    bg.setColorAt(0.0, QColor(23, 29, 41));
    bg.setColorAt(1.0, QColor(12, 15, 22));
    p.fillRect(rect(), bg);

    renderer_.set_viewport(QRectF(rect()));
    renderer_.draw_grid(p, kGrid, QColor(96, 116, 152));
    renderer_.draw_shadow(p, model_, q_, V3{});
    renderer_.draw_mesh(p, model_, q_, V3{});
    renderer_.draw_axes(p, q_, V3{}, true);

    // Corner hints, small and out of the way.
    QFont f = p.font();
    f.setPointSizeF(9.5);
    p.setFont(f);
    p.setPen(kMuted);
    p.drawText(rect().adjusted(12, 0, -12, -10), Qt::AlignBottom | Qt::AlignLeft,
               QStringLiteral("drag to orbit · wheel to zoom · V resets the view"));
    if (!live_) {
        p.setPen(kBad);
        p.drawText(rect().adjusted(12, 10, -12, 0), Qt::AlignTop | Qt::AlignLeft,
                   QStringLiteral("no data — the model is not moving"));
    }
}

void SceneWidget::mousePressEvent(QMouseEvent* e)
{
    drag_     = e->pos();
    dragging_ = true;
}

void SceneWidget::mouseMoveEvent(QMouseEvent* e)
{
    if (!dragging_) return;
    const QPoint d = e->pos() - drag_;
    drag_          = e->pos();
    renderer_.orbit(-d.x() * 0.4, d.y() * 0.35);
    update();
}

void SceneWidget::wheelEvent(QWheelEvent* e)
{
    renderer_.zoom(e->angleDelta().y() > 0 ? 0.9 : 1.111);
    update();
}

// ─── Instruments ─────────────────────────────────────────────────────────────

Instruments::Instruments(QWidget* parent) : QWidget(parent)
{
    setMinimumHeight(150);
}

void Instruments::setState(double roll_deg, double pitch_deg, double heading_deg,
                           bool mag)
{
    roll_    = roll_deg;
    pitch_   = pitch_deg;
    heading_ = heading_deg;
    mag_     = mag;
    update();
}

void Instruments::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    const double side = std::min(width() / 2.0 - 10.0, static_cast<double>(height()));
    const double top  = (height() - side) / 2.0;
    drawHorizon(p, QRectF(4, top, side, side));
    drawCompass(p, QRectF(width() - side - 4, top, side, side));
}

void Instruments::drawHorizon(QPainter& p, const QRectF& r) const
{
    p.save();
    QPainterPath clip;
    clip.addEllipse(r);
    p.setClipPath(clip);

    p.translate(r.center());
    p.rotate(-roll_);
    // 1 degree of pitch to this many pixels — the usual ADI scaling, so the
    // horizon leaves the dial at a believable attitude rather than instantly.
    const double px_per_deg = r.height() / 90.0;
    p.translate(0, pitch_ * px_per_deg);

    const double big = r.width() * 2.0;
    p.fillRect(QRectF(-big, -big, 2 * big, big), QColor(56, 116, 178));   // sky
    p.fillRect(QRectF(-big, 0, 2 * big, big), QColor(120, 88, 58));       // ground
    p.setPen(QPen(QColor(235, 240, 250), 1.6));
    p.drawLine(QPointF(-big, 0), QPointF(big, 0));

    QFont f = p.font();
    f.setPointSizeF(7.5);
    p.setFont(f);
    p.setPen(QPen(QColor(228, 236, 248), 1.2));
    for (int a = -60; a <= 60; a += 10) {
        if (a == 0) continue;
        const double y = -a * px_per_deg;
        const double w = (a % 20 == 0) ? r.width() * 0.22 : r.width() * 0.12;
        p.drawLine(QPointF(-w, y), QPointF(w, y));
        if (a % 20 == 0)
            p.drawText(QRectF(w + 3, y - 7, 28, 14), Qt::AlignVCenter | Qt::AlignLeft,
                       QString::number(std::abs(a)));
    }
    p.restore();

    // Fixed aircraft reference, drawn over the moving world.
    p.save();
    p.translate(r.center());
    p.setPen(QPen(QColor(250, 204, 21), 2.6, Qt::SolidLine, Qt::RoundCap));
    const double w = r.width() * 0.30;
    p.drawLine(QPointF(-w, 0), QPointF(-w * 0.3, 0));
    p.drawLine(QPointF(w * 0.3, 0), QPointF(w, 0));
    p.drawPoint(QPointF(0, 0));
    p.drawEllipse(QPointF(0, 0), 2.4, 2.4);
    p.restore();

    p.setPen(QPen(QColor(70, 84, 108), 2.0));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(r);

    QFont f2 = p.font();
    f2.setPointSizeF(8.5);
    p.setFont(f2);
    p.setPen(kMuted);
    p.drawText(QRectF(r.left(), r.bottom() - 2, r.width(), 16), Qt::AlignCenter,
               QStringLiteral("attitude"));
}

void Instruments::drawCompass(QPainter& p, const QRectF& r) const
{
    p.save();
    p.translate(r.center());
    const double rad = r.width() / 2.0;

    p.setBrush(QColor(18, 23, 33));
    p.setPen(QPen(QColor(70, 84, 108), 2.0));
    p.drawEllipse(QPointF(0, 0), rad - 1, rad - 1);

    p.save();
    p.rotate(-heading_);       // the card turns, the aircraft stays put
    QFont f = p.font();
    f.setPointSizeF(8.0);
    f.setBold(true);
    p.setFont(f);
    for (int a = 0; a < 360; a += 15) {
        p.save();
        p.rotate(a);
        const bool cardinal = (a % 90 == 0);
        p.setPen(QPen(cardinal ? kInk : QColor(110, 126, 150), cardinal ? 1.8 : 1.0));
        p.drawLine(QPointF(0, -rad + 4), QPointF(0, -rad + (cardinal ? 13 : 8)));
        if (cardinal) {
            static const char* names[] = {"N", "E", "S", "W"};
            p.setPen(a == 0 ? kBad : kInk);
            p.drawText(QRectF(-10, -rad + 14, 20, 14), Qt::AlignCenter,
                       QString::fromLatin1(names[a / 90]));
        }
        p.restore();
    }
    p.restore();

    p.setPen(QPen(kWarn, 2.4, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(0, 6), QPointF(0, -rad * 0.62));
    QPolygonF head;
    head << QPointF(0, -rad * 0.72) << QPointF(-5, -rad * 0.58)
         << QPointF(5, -rad * 0.58);
    p.setPen(Qt::NoPen);
    p.setBrush(kWarn);
    p.drawPolygon(head);

    QFont fv = p.font();
    fv.setPointSizeF(11.0);
    fv.setBold(true);
    p.setFont(fv);
    p.setPen(kInk);
    p.drawText(QRectF(-rad, rad * 0.30, 2 * rad, 18), Qt::AlignCenter, deg(heading_));
    p.restore();

    QFont f2 = p.font();
    f2.setPointSizeF(8.5);
    f2.setBold(false);
    p.setFont(f2);
    p.setPen(mag_ ? kMuted : kWarn);
    p.drawText(QRectF(r.left(), r.bottom() - 2, r.width(), 16), Qt::AlignCenter,
               mag_ ? QStringLiteral("heading · magnetometer")
                    : QStringLiteral("heading · gyro only, drifts"));
}

// ─── Bars ────────────────────────────────────────────────────────────────────

Bars::Bars(QWidget* parent) : QWidget(parent)
{
    // Three groups of a title plus three rows, at the sizes drawGroup() uses.
    setMinimumHeight(3 * (19 + 3 * 16 + 8) + 6);
}

void Bars::setSample(const ImuSample& s)
{
    s_ = s;
    update();
}

void Bars::drawGroup(QPainter& p, QRectF& r, const QString& title, const QString& unit,
                     const Vec3& v, double full_scale, bool present) const
{
    QFont f = p.font();
    f.setPointSizeF(9.0);
    f.setBold(true);
    p.setFont(f);
    p.setPen(present ? kInk : kMuted);
    p.drawText(QRectF(r.left(), r.top(), r.width(), 16), Qt::AlignVCenter | Qt::AlignLeft,
               title);
    p.setPen(kMuted);
    f.setBold(false);
    p.setFont(f);
    p.drawText(QRectF(r.left(), r.top(), r.width(), 16), Qt::AlignVCenter | Qt::AlignRight,
               present ? unit : QStringLiteral("not reported"));

    const double vals[3] = {v.x, v.y, v.z};
    const QColor cols[3] = {kAxisX, kAxisY, kAxisZ};
    const char*  names[3] = {"x", "y", "z"};

    double y = r.top() + 19;
    for (int i = 0; i < 3; ++i) {
        const QRectF row(r.left(), y, r.width(), 15);
        p.setPen(kMuted);
        f.setPointSizeF(8.5);
        p.setFont(f);
        p.drawText(QRectF(row.left(), row.top(), 12, row.height()),
                   Qt::AlignVCenter | Qt::AlignLeft, QString::fromLatin1(names[i]));

        const QRectF track(row.left() + 14, row.top() + 4, row.width() - 74, 7);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(38, 46, 62));
        p.drawRoundedRect(track, 3.5, 3.5);

        // Signed bars grow out from the middle, so the sign of each axis is
        // visible without reading the number.
        const double mid  = track.center().x();
        const double half = track.width() / 2.0;
        const double frac = present ? std::clamp(vals[i] / full_scale, -1.0, 1.0) : 0.0;
        QRectF fill(std::min(mid, mid + frac * half), track.top(),
                    std::fabs(frac) * half, track.height());
        p.setBrush(present ? cols[i] : QColor(60, 70, 90));
        p.drawRoundedRect(fill, 3.5, 3.5);
        p.setPen(QPen(QColor(90, 104, 128), 1.0));
        p.drawLine(QPointF(mid, track.top() - 1), QPointF(mid, track.bottom() + 1));

        p.setPen(present ? kInk : kMuted);
        p.drawText(QRectF(track.right() + 6, row.top(), 54, row.height()),
                   Qt::AlignVCenter | Qt::AlignRight,
                   present ? QString::number(vals[i], 'f', 2) : QStringLiteral("—"));
        y += 16;
    }
    r.setTop(y + 8);
}

void Bars::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    QRectF r = rect().adjusted(2, 2, -2, -2);
    drawGroup(p, r, QStringLiteral("accelerometer"), QStringLiteral("m/s²"), s_.accel,
              2.0 * kGravity, s_.has_accel);
    drawGroup(p, r, QStringLiteral("gyroscope"), QStringLiteral("°/s"),
              Vec3{s_.gyro.x * kRad2Deg, s_.gyro.y * kRad2Deg, s_.gyro.z * kRad2Deg},
              250.0, s_.has_gyro);
    // No published scale factor for this part's magnetometer, so the raw counts
    // are shown as they arrive; only the direction matters to the fusion.
    drawGroup(p, r, QStringLiteral("magnetometer"), QStringLiteral("raw LSB"), s_.mag,
              2000.0, s_.has_mag);
}

// ─── MainWindow ──────────────────────────────────────────────────────────────

MainWindow::MainWindow(const ReaderConfig& cfg, QWidget* parent) : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("Yahboom 10-axis IMU — live attitude"));
    reader_ = std::make_unique<ImuReader>(cfg);

    auto* central = new QWidget(this);
    auto* row     = new QHBoxLayout(central);
    row->setContentsMargins(0, 0, 0, 0);
    row->setSpacing(0);

    scene_ = new SceneWidget(central);
    row->addWidget(scene_, 1);
    row->addWidget(buildSidebar(), 0);
    setCentralWidget(central);

    // The port may not be there yet; the window still opens and says why, which
    // beats exiting to a terminal message the user has to go and find.
    std::string err;
    if (!reader_->start(err)) {
        status_->setText(QString::fromStdString(err));
        status_->setStyleSheet(QStringLiteral("color:#f87171;"));
    }

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::tick);
    timer_->start(16);   // 60 Hz repaint, independent of the sensor's rate

    resize(1280, 900);
}

MainWindow::~MainWindow()
{
    if (reader_) reader_->stop();
}

QLabel* MainWindow::bigValue(const QString& name, const QColor& c, QLabel** out_name)
{
    auto* caption = new QLabel(name);
    caption->setStyleSheet(QStringLiteral("color:%1; font-size:11px; font-weight:600;"
                                          " letter-spacing:1px;")
                               .arg(c.name()));
    *out_name = caption;

    auto* value = new QLabel(QStringLiteral("—"));
    value->setStyleSheet(QStringLiteral(
        "color:#e2e8f0; font-size:26px; font-weight:600; font-family:'DejaVu Sans Mono',"
        " monospace;"));
    return value;
}

QWidget* MainWindow::buildSidebar()
{
    auto* side = new QWidget(this);
    side->setFixedWidth(392);
    side->setStyleSheet(QStringLiteral("background:%1;").arg(kPanel.name()));

    auto* col = new QVBoxLayout(side);
    col->setContentsMargins(16, 14, 16, 14);
    col->setSpacing(12);

    status_ = new QLabel(QStringLiteral("connecting…"));
    status_->setWordWrap(true);
    status_->setStyleSheet(QStringLiteral("color:#94a3b8; font-size:11px;"));
    col->addWidget(status_);

    auto* grid = new QGridLayout();
    grid->setHorizontalSpacing(14);
    grid->setVerticalSpacing(0);
    QLabel* n0 = nullptr;
    QLabel* n1 = nullptr;
    QLabel* n2 = nullptr;
    roll_  = bigValue(QStringLiteral("ROLL"), kAxisX, &n0);
    pitch_ = bigValue(QStringLiteral("PITCH"), kAxisY, &n1);
    yaw_   = bigValue(QStringLiteral("YAW"), kAxisZ, &n2);
    grid->addWidget(n0, 0, 0);
    grid->addWidget(n1, 0, 1);
    grid->addWidget(n2, 0, 2);
    grid->addWidget(roll_, 1, 0);
    grid->addWidget(pitch_, 1, 1);
    grid->addWidget(yaw_, 1, 2);
    col->addLayout(grid);

    instr_ = new Instruments(side);
    col->addWidget(instr_);

    bars_ = new Bars(side);
    col->addWidget(bars_);

    detail_ = new QLabel();
    detail_->setWordWrap(true);
    detail_->setStyleSheet(QStringLiteral("color:#7f8ea6; font-size:10.5px;"));
    col->addWidget(detail_);

    col->addStretch(1);

    const QString btn = QStringLiteral(
        "QPushButton { background:#2b3244; color:#e2e8f0; border:1px solid #3b4457;"
        " border-radius:6px; padding:7px 6px; font-size:11px; }"
        "QPushButton:hover { background:#35405a; }"
        "QPushButton:pressed { background:#222937; }");

    auto* buttons = new QGridLayout();
    buttons->setSpacing(8);
    zero_  = new QPushButton(QStringLiteral("Zero here  (Space)"));
    recal_ = new QPushButton(QStringLiteral("Recalibrate  (C)"));
    view_  = new QPushButton(QStringLiteral("Reset view  (V)"));
    for (QPushButton* b : {zero_, recal_, view_}) b->setStyleSheet(btn);
    buttons->addWidget(zero_, 0, 0);
    buttons->addWidget(recal_, 0, 1);
    buttons->addWidget(view_, 1, 0);
    col->addLayout(buttons);

    connect(zero_, &QPushButton::clicked, this, [this] { reader_->zero_here(); });
    connect(recal_, &QPushButton::clicked, this, [this] { reader_->recalibrate(); });
    connect(view_, &QPushButton::clicked, this, [this] { scene_->resetView(); });

    return side;
}

void MainWindow::tick()
{
    const ImuState st = reader_->state();

    const bool live = st.connected && st.age_ms < 1500 && st.stats.samples > 0;
    scene_->setLive(live);
    scene_->setOrientation(st.orientation);

    roll_->setText(deg(st.euler_deg.x));
    pitch_->setText(deg(st.euler_deg.y));
    yaw_->setText(deg(st.euler_deg.z));

    instr_->setState(st.euler_deg.x, st.euler_deg.y, st.heading_deg, st.mag_yaw);
    bars_->setSample(st.sample);

    QString colour = QStringLiteral("#94a3b8");
    QString text   = QString::fromStdString(st.status);
    if (st.calibrating && live) {
        text = QStringLiteral("calibrating gyro bias — keep the module still  (%1%)")
                   .arg(static_cast<int>(st.cal_progress * 100));
        colour = kWarn.name();
    } else if (live) {
        colour = kGood.name();
    } else if (st.connected) {
        colour = kWarn.name();
    } else {
        colour = kBad.name();
    }
    status_->setText(text);
    status_->setStyleSheet(QStringLiteral("color:%1; font-size:11px;").arg(colour));

    detail_->setText(
        QStringLiteral("%1 @ %2 baud · %3 · %4 Hz · %5 samples · %6 bad frames\n"
                       "temp %7 °C · pressure %8 Pa · gyro bias %9 %10 %11 °/s")
            .arg(QString::fromStdString(st.device))
            .arg(st.baud)
            .arg(QString::fromLatin1(protocol_name(st.protocol)))
            .arg(st.stats.sample_hz, 0, 'f', 1)
            .arg(static_cast<qulonglong>(st.stats.samples))
            .arg(static_cast<qulonglong>(st.stats.frames_bad))
            .arg(st.sample.temperature_c, 0, 'f', 1)
            .arg(st.sample.pressure_pa, 0, 'f', 0)
            .arg(st.gyro_bias_dps.x, 0, 'f', 3)
            .arg(st.gyro_bias_dps.y, 0, 'f', 3)
            .arg(st.gyro_bias_dps.z, 0, 'f', 3));
}

void MainWindow::keyPressEvent(QKeyEvent* e)
{
    switch (e->key()) {
    case Qt::Key_Space:  reader_->zero_here(); break;
    case Qt::Key_C:      reader_->recalibrate(); break;
    case Qt::Key_V:      scene_->resetView(); break;
    case Qt::Key_Escape:
    case Qt::Key_Q:      close(); break;
    default:             QMainWindow::keyPressEvent(e); return;
    }
}

}  // namespace imu::view
