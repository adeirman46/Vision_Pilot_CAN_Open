#ifndef VISIONPILOT_QT_MAIN_WINDOW_HPP
#define VISIONPILOT_QT_MAIN_WINDOW_HPP

#include <memory>

#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QThread>
#include <QTimer>

#include <config/vision_pilot_config.hpp>
#include <navigation/navigation.hpp>

#include "pipeline_worker.hpp"

namespace visionpilot::ui {

// Map widget: the panel image plus the controls that float on top of it, the
// way a phone map app lays them out. Owns the pointer interaction because it is
// the only widget that knows the mapping from its own pixels to the panel's.
class MapView : public QWidget
{
    Q_OBJECT

public:
    explicit MapView(navigation::Navigator* nav, QWidget* parent = nullptr);

    void refresh();          // re-render the panel at the current widget size

signals:
    void panned();
    // A press and release in the same place, in the map's own pixels. Separate
    // from a drag on purpose: the same button both moves the map and picks a
    // destination, and letting go after shoving the map across must not be read
    // as "take me there".
    // The modifiers matter: a plain click retargets, a shift-click appends to
    // the mission. They have to travel with the position because the press
    // is what the driver meant, and by the time the release is handled the
    // key may already be up.
    void clicked(QPoint pos, Qt::KeyboardModifiers mods);
    // The children floating on the map belong to the window, but only the map
    // knows when its own size settled. Laying them out on the window's resize
    // used a size the map did not have yet, which is what threw the buttons off
    // the top edge.
    void resized();

protected:
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    navigation::Navigator* nav_;
    QImage                 image_;
    QPoint                 drag_last_;
    QPoint                 press_at_;
    Qt::KeyboardModifiers  press_mods_ = Qt::NoModifier;
    bool                   dragging_ = false;
    bool                   moved_    = false;   // travelled far enough to be a drag
    bool                   focus_click_ = false;   // the press that raised the window
};

// The window. Camera view on the left, map on the right, a search bar across
// the top of the map, and the theme toggle beside it.
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(Config cfg, bool debug_viz, QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onFrame();
    void onPipelineFailed(const QString& reason);
    void onSearchChanged(const QString& text);
    void onResultChosen(QListWidgetItem* item);
    void onSearchReturn();
    void onMapClicked(QPoint pos, Qt::KeyboardModifiers mods);
    void toggleTheme();
    void toggleHeading();
    void toggleSatellite();
    void clearRoute();
    void tick();

private:
    void applyTheme();
    void positionOverlay();
    void resizeEvent(QResizeEvent*) override;

    Config cfg_;

    std::unique_ptr<navigation::Navigator> nav_;

    QLabel*      camera_    = nullptr;
    MapView*     map_       = nullptr;
    QLineEdit*   search_    = nullptr;
    QListWidget* results_   = nullptr;
    QPushButton* theme_btn_ = nullptr;
    QPushButton* head_btn_  = nullptr;
    QPushButton* sat_btn_   = nullptr;
    QPushButton* clear_btn_ = nullptr;
    QPushButton* zoom_in_   = nullptr;
    QPushButton* zoom_out_  = nullptr;
    QPushButton* recenter_  = nullptr;
    QLabel*      status_    = nullptr;

    QThread*        thread_ = nullptr;
    PipelineWorker* worker_ = nullptr;
    QTimer*         timer_  = nullptr;
};

}  // namespace visionpilot::ui

#endif  // VISIONPILOT_QT_MAIN_WINDOW_HPP
