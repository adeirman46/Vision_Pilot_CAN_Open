#ifndef IMU_YAHBOOM_IMU_WINDOW_HPP
#define IMU_YAHBOOM_IMU_WINDOW_HPP

#include <memory>

#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <imu/imu_reader.hpp>

#include "scene3d.hpp"

namespace imu::view {

// The 3D half: floor grid, the aircraft turned by the current orientation, and
// the three labelled body axes. Drag to orbit, wheel to zoom.
class SceneWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SceneWidget(QWidget* parent = nullptr);

    // Attitude only. Position from an IMU alone is not a thing this sensor can
    // give — it belongs to the GPS/IMU filter, where GPS anchors it.
    void setOrientation(const Quat& q);
    void setLive(bool live) { live_ = live; }
    void resetView();

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    Renderer renderer_;
    Mesh     model_;
    Quat     q_;
    QPoint   drag_;
    bool     dragging_ = false;
    bool     live_     = false;
};

// Artificial horizon and compass rose. Both say the same thing as the numbers
// above them, but a tilted horizon is legible at a glance in a way that "roll
// -37.4" is not.
class Instruments : public QWidget
{
    Q_OBJECT

public:
    explicit Instruments(QWidget* parent = nullptr);
    void setState(double roll_deg, double pitch_deg, double heading_deg, bool mag);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void drawHorizon(QPainter& p, const QRectF& r) const;
    void drawCompass(QPainter& p, const QRectF& r) const;

    double roll_ = 0.0, pitch_ = 0.0, heading_ = 0.0;
    bool   mag_  = false;
};

// Signed bar meters for the raw channels — the quickest way to see that an axis
// is dead, stuck or wired the wrong way round.
class Bars : public QWidget
{
    Q_OBJECT

public:
    explicit Bars(QWidget* parent = nullptr);
    void setSample(const ImuSample& s);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void drawGroup(QPainter& p, QRectF& r, const QString& title, const QString& unit,
                   const Vec3& v, double full_scale, bool present) const;

    ImuSample s_;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(const ReaderConfig& cfg, QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void keyPressEvent(QKeyEvent*) override;

private slots:
    void tick();

private:
    QWidget* buildSidebar();
    QLabel*  bigValue(const QString& name, const QColor& c, QLabel** out_name);

    std::unique_ptr<ImuReader> reader_;

    SceneWidget* scene_   = nullptr;
    Instruments* instr_   = nullptr;
    Bars*        bars_    = nullptr;
    QLabel*      status_  = nullptr;
    QLabel*      roll_    = nullptr;
    QLabel*      pitch_   = nullptr;
    QLabel*      yaw_     = nullptr;
    QLabel*      detail_  = nullptr;
    QPushButton* zero_    = nullptr;
    QPushButton* recal_   = nullptr;
    QPushButton* view_    = nullptr;
    QTimer*      timer_   = nullptr;
};

}  // namespace imu::view

#endif  // IMU_YAHBOOM_IMU_WINDOW_HPP
