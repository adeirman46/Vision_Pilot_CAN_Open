#include "main_window.hpp"

#include <QGuiApplication>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QScreen>
#include <QStatusBar>
#include <QWheelEvent>

#include <logging/logger.hpp>

namespace visionpilot::ui {

namespace {

// Two flat palettes. Everything chrome-related reads from these, so the whole
// window switches with one call instead of each widget knowing about themes.
struct Palette {
    QString window, surface, text, dim, accent, border, field;
};

const Palette kDarkPal{
    "#131313", "#1e1e1e", "#f1f1f1", "#9a9a9a", "#ffb648", "#333333", "#262626"};
const Palette kLightPal{
    "#eceff1", "#ffffff", "#1f2124", "#5f6368", "#1a73e8", "#dadce0", "#ffffff"};

QString stylesheet(const Palette& p)
{
    return QString(
               "QMainWindow, QWidget#root { background: %1; }"
               "QLabel { color: %3; }"
               "QLabel#status { color: %4; padding: 0 10px; font-size: 12px; }"
               "QLineEdit {"
               "  background: %7; color: %3; border: 1px solid %6;"
               "  border-radius: 22px; padding: 10px 18px; font-size: 15px;"
               "}"
               "QLineEdit:focus { border: 2px solid %5; }"
               "QListWidget {"
               "  background: %2; color: %3; border: 1px solid %6;"
               "  border-radius: 10px; font-size: 14px; outline: none;"
               "}"
               "QListWidget::item { padding: 9px 14px; }"
               "QListWidget::item:selected { background: %5; color: %1; }"
               "QPushButton {"
               "  background: %2; color: %3; border: 1px solid %6;"
               "  border-radius: 18px; padding: 8px 14px; font-size: 13px;"
               "}"
               "QPushButton:hover { border: 1px solid %5; }"
               "QPushButton:pressed { background: %5; color: %1; }")
        .arg(p.window, p.surface, p.text, p.dim, p.accent, p.border, p.field);
}

}  // namespace

// ─── MapView ─────────────────────────────────────────────────────────────────

MapView::MapView(navigation::Navigator* nav, QWidget* parent)
    : QWidget(parent), nav_(nav)
{
    setMinimumWidth(360);
    setMouseTracking(true);
    setCursor(Qt::OpenHandCursor);
}

void MapView::refresh()
{
    if (!nav_) return;
    // Rendered at the widget's size *in real screen pixels*, not in logical
    // ones. On a scaled display the two differ, and rendering at the logical
    // size leaves Qt to stretch the result — which is what turned the road names
    // and the manoeuvre banner soft. Tagging the image with the ratio tells Qt
    // it is already at full resolution, so it is drawn one image pixel to one
    // screen pixel.
    const qreal dpr = devicePixelRatioF();
    const int   w   = std::max(160, static_cast<int>(width() * dpr));
    const int   h   = std::max(160, static_cast<int>(height() * dpr));
    const cv::Mat panel = nav_->panel(w, h);
    image_ = mat_to_qimage(panel);
    image_.setDevicePixelRatio(dpr);
    update();
}

void MapView::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    refresh();
    emit resized();
}

void MapView::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    if (image_.isNull()) {
        p.fillRect(rect(), QColor(20, 20, 20));
        return;
    }
    p.drawImage(0, 0, image_);
}

void MapView::mousePressEvent(QMouseEvent* e)
{
    if (e->button() != Qt::LeftButton) return;
    dragging_ = true;
    drag_last_ = e->pos();
    setCursor(Qt::ClosedHandCursor);
}

void MapView::mouseMoveEvent(QMouseEvent* e)
{
    if (!dragging_ || !nav_) return;
    const QPoint d = e->pos() - drag_last_;
    drag_last_ = e->pos();
    // Repaint immediately rather than waiting for the next timer tick: the map
    // renders on the GUI thread now, so a drag can follow the pointer directly.
    // The pointer moves in logical pixels and the panel is drawn in real ones,
    // so the delta has to be converted or the map slides slower than the hand.
    const qreal dpr = devicePixelRatioF();
    nav_->pan_by(d.x() * dpr, d.y() * dpr);
    refresh();
    emit panned();
}

void MapView::mouseReleaseEvent(QMouseEvent*)
{
    dragging_ = false;
    setCursor(Qt::OpenHandCursor);
}

void MapView::wheelEvent(QWheelEvent* e)
{
    if (!nav_) return;
    nav_->zoom_by(e->angleDelta().y() > 0 ? 0.85 : 1.18);
    refresh();
}

// ─── MainWindow ──────────────────────────────────────────────────────────────

MainWindow::MainWindow(Config cfg, bool debug_viz, QWidget* parent)
    : QMainWindow(parent), cfg_(std::move(cfg))
{
    setWindowTitle(QStringLiteral("VisionPilot"));

    nav_ = std::make_unique<navigation::Navigator>(cfg_.navigation);
    if (cfg_.navigation.enabled && !nav_->ready())
        VP_WARN("[Nav] degraded: %s", nav_->error().c_str());

    auto* root = new QWidget(this);
    root->setObjectName(QStringLiteral("root"));
    setCentralWidget(root);

    camera_ = new QLabel(root);
    camera_->setAlignment(Qt::AlignCenter);
    camera_->setMinimumSize(480, 270);
    camera_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    map_ = new MapView(nav_.get(), root);
    map_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    map_->setFixedWidth(std::max(360, cfg_.navigation.panel_width));

    auto* row = new QHBoxLayout(root);
    row->setContentsMargins(0, 0, 0, 0);
    row->setSpacing(0);
    row->addWidget(camera_, 1);
    row->addWidget(map_, 0);

    // ── Search bar: a permanent strip across the top of the map, the way a
    // phone map app does it. Not a modal — the map keeps running behind it.
    search_ = new QLineEdit(map_);
    search_->setPlaceholderText(QStringLiteral("Search a place or street"));
    search_->setClearButtonEnabled(true);

    results_ = new QListWidget(map_);
    results_->hide();

    theme_btn_ = new QPushButton(QStringLiteral("Light"), map_);
    theme_btn_->setToolTip(QStringLiteral("Switch between the light and dark map"));
    head_btn_  = new QPushButton(QStringLiteral("North up"), map_);
    clear_btn_ = new QPushButton(QStringLiteral("Clear route"), map_);
    zoom_in_   = new QPushButton(QStringLiteral("+"), map_);
    zoom_out_  = new QPushButton(QStringLiteral("−"), map_);
    recenter_  = new QPushButton(QStringLiteral("Recentre"), map_);

    status_ = new QLabel(root);
    status_->setObjectName(QStringLiteral("status"));
    statusBar()->addWidget(status_, 1);

    connect(search_, &QLineEdit::textEdited, this, &MainWindow::onSearchChanged);
    connect(search_, &QLineEdit::returnPressed, this, &MainWindow::onSearchReturn);
    connect(results_, &QListWidget::itemClicked, this, &MainWindow::onResultChosen);
    connect(theme_btn_, &QPushButton::clicked, this, &MainWindow::toggleTheme);
    connect(head_btn_, &QPushButton::clicked, this, &MainWindow::toggleHeading);
    connect(clear_btn_, &QPushButton::clicked, this, &MainWindow::clearRoute);
    connect(zoom_in_, &QPushButton::clicked, this, [this] {
        nav_->zoom_by(0.8);
        map_->refresh();
    });
    connect(zoom_out_, &QPushButton::clicked, this, [this] {
        nav_->zoom_by(1.25);
        map_->refresh();
    });
    connect(recenter_, &QPushButton::clicked, this, [this] {
        nav_->recenter();
        map_->refresh();
    });
    connect(map_, &MapView::resized, this, &MainWindow::positionOverlay);

    // ── Pipeline on its own thread ───────────────────────────────────────────
    thread_ = new QThread(this);
    worker_ = new PipelineWorker(cfg_, debug_viz);
    worker_->moveToThread(thread_);
    connect(thread_, &QThread::started, worker_, &PipelineWorker::run);
    connect(worker_, &PipelineWorker::frameReady, this, &MainWindow::onFrame);
    connect(worker_, &PipelineWorker::failed, this, &MainWindow::onPipelineFailed);
    connect(worker_, &PipelineWorker::finished, thread_, &QThread::quit);
    connect(thread_, &QThread::finished, worker_, &QObject::deleteLater);
    thread_->start();

    // Navigation runs on the GUI thread's own clock, deliberately: it is what
    // keeps the map fluid while the models grind away at 25 fps next door.
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::tick);
    timer_->start(33);

    applyTheme();
    resize(1600, 900);
}

MainWindow::~MainWindow()
{
    if (worker_) worker_->stop();
    if (thread_) {
        thread_->quit();
        thread_->wait(3000);
    }
}

void MainWindow::applyTheme()
{
    const bool dark = nav_->dark();
    setStyleSheet(stylesheet(dark ? kDarkPal : kLightPal));
    theme_btn_->setText(dark ? QStringLiteral("Light mode")
                             : QStringLiteral("Dark mode"));
    map_->refresh();
}

void MainWindow::toggleTheme()
{
    nav_->set_dark(!nav_->dark());
    applyTheme();
}

void MainWindow::toggleHeading()
{
    nav_->set_heading_up(!nav_->heading_up());
    head_btn_->setText(nav_->heading_up() ? QStringLiteral("North up")
                                          : QStringLiteral("Heading up"));
    map_->refresh();
}

void MainWindow::clearRoute()
{
    nav_->clear_destination();
    search_->clear();
    results_->hide();
    map_->refresh();
}

void MainWindow::onFrame()
{
    const QImage frame = worker_->takeFrame();
    if (frame.isNull()) return;

    // Same story as the map: scale to real screen pixels and label the pixmap
    // with the ratio, or the HUD text gets drawn small and stretched.
    const qreal dpr    = camera_->devicePixelRatioF();
    const QSize target = camera_->size() * dpr;

    // Resampling by a few percent is the worst of both worlds: it costs the
    // sharpness of every glyph the HUD drew and buys no extra size. A 1024-wide
    // frame shown in a 960-wide panel is a 0.94x scale — enough to smear the
    // lettering, not enough to be worth it. Inside that band, show the frame at
    // its own size and let the panel letterbox it instead.
    const double fit = std::min(target.width() / static_cast<double>(frame.width()),
                                target.height() / static_cast<double>(frame.height()));

    QPixmap pm = (fit > 0.88 && fit < 1.12)
                     ? QPixmap::fromImage(frame)
                     : QPixmap::fromImage(frame).scaled(target, Qt::KeepAspectRatio,
                                                        Qt::SmoothTransformation);
    pm.setDevicePixelRatio(dpr);
    camera_->setPixmap(pm);
}

void MainWindow::onPipelineFailed(const QString& reason)
{
    status_->setText(QStringLiteral("Camera: ") + reason);
}

void MainWindow::onSearchChanged(const QString& text)
{
    results_->clear();
    if (text.trimmed().size() < 3) {
        results_->hide();
        return;
    }

    const auto fix = nav_->fix();
    const double lat = fix ? fix->latitude : 0.0;
    const double lon = fix ? fix->longitude : 0.0;
    const auto hits = nav_->map().search(text.trimmed().toStdString(), 8,
                                         fix ? &lat : nullptr, fix ? &lon : nullptr);
    if (hits.empty()) {
        results_->addItem(QStringLiteral("No match in this map"));
        results_->item(0)->setFlags(Qt::NoItemFlags);
    }
    for (const auto& p : hits) {
        // Distance is what makes a list of identical street names usable.
        QString label = QString::fromStdString(p.name);
        if (fix) {
            const double d = navigation::haversine_m(lat, lon, p.lat, p.lon);
            label += d < 950 ? QString("   %1 m").arg(int(d / 10) * 10)
                             : QString("   %1 km").arg(d / 1000.0, 0, 'f', 1);
        }
        auto* item = new QListWidgetItem(label, results_);
        item->setData(Qt::UserRole, p.lat);
        item->setData(Qt::UserRole + 1, p.lon);
        item->setData(Qt::UserRole + 2, QString::fromStdString(p.name));
    }
    results_->show();
    results_->raise();
    positionOverlay();
}

void MainWindow::onResultChosen(QListWidgetItem* item)
{
    if (!item || !(item->flags() & Qt::ItemIsEnabled)) return;
    const double lat = item->data(Qt::UserRole).toDouble();
    const double lon = item->data(Qt::UserRole + 1).toDouble();
    const QString name = item->data(Qt::UserRole + 2).toString();

    nav_->set_destination(lat, lon, name.toStdString());
    search_->setText(name);
    results_->hide();
    search_->clearFocus();
    map_->refresh();
}

void MainWindow::onSearchReturn()
{
    if (results_->isVisible() && results_->count() > 0) {
        onResultChosen(results_->currentItem() ? results_->currentItem()
                                               : results_->item(0));
        return;
    }
    // Coordinates typed straight in still work, for anywhere the map has no
    // name for.
    const QStringList parts = search_->text().split(',');
    if (parts.size() == 2) {
        bool ok1 = false, ok2 = false;
        const double lat = parts[0].trimmed().toDouble(&ok1);
        const double lon = parts[1].trimmed().toDouble(&ok2);
        if (ok1 && ok2) {
            nav_->set_destination(lat, lon, "");
            results_->hide();
            map_->refresh();
        }
    }
}

void MainWindow::tick()
{
    nav_->update();
    worker_->set_speed_limit_kph(nav_->speed_limit_kph());
    map_->refresh();

    const auto fix = nav_->fix();
    QString s;
    if (!fix) {
        s = QStringLiteral("No GPS fix — ") + QString::fromStdString(nav_->error());
    } else {
        s = QString("%1, %2").arg(fix->latitude, 0, 'f', 5).arg(fix->longitude, 0, 'f', 5);
        if (!nav_->road_name().empty())
            s += QStringLiteral("   ·   ") + QString::fromStdString(nav_->road_name());
        if (nav_->has_route())
            s += QString("   ·   %1 km to %2")
                     .arg(nav_->progress().remaining_m / 1000.0, 0, 'f', 1)
                     .arg(QString::fromStdString(nav_->destination_name()));
    }
    status_->setText(s);
}

void MainWindow::positionOverlay()
{
    if (!map_) return;
    const int w = map_->width();
    const int h = map_->height();
    if (w < 80 || h < 80) return;          // not laid out yet

    const int pad = 12;
    const int bar_h = 44;

    // Search across the top, the way a phone map app puts it.
    search_->setGeometry(pad, pad, w - 2 * pad, bar_h);
    search_->raise();

    // Tell the renderer how much of its own top the search field is standing
    // on, in the panel's pixels rather than the window's, so the next-turn arrow
    // is drawn below it instead of behind it.
    if (nav_) nav_->set_top_inset(
        static_cast<int>((pad + bar_h + 6) * map_->devicePixelRatioF()));

    // Suggestions hang directly under the bar and never past the map's bottom.
    const int rows = std::max(1, results_->count());
    const int list_h = std::min(rows * 38 + 10, h - (pad + bar_h + 8) - 120);
    results_->setGeometry(pad, pad + bar_h + 6, w - 2 * pad, std::max(40, list_h));
    results_->raise();

    // Controls sit above the panel's own footer strip — the road name and route
    // summary are painted into the map image, and covering them would be worse
    // than having the buttons a little higher.
    constexpr int kFooter = 84;
    const int by = h - kFooter - 44;
    const int bw = 104, bh = 34, gap = 8;

    theme_btn_->setGeometry(pad, by, bw, bh);
    head_btn_->setGeometry(pad + bw + gap, by, 96, bh);
    clear_btn_->setGeometry(pad + bw + gap + 96 + gap, by, 104, bh);

    zoom_in_->setGeometry(w - pad - 40, by - 2 * (bh + gap), 40, bh);
    zoom_out_->setGeometry(w - pad - 40, by - (bh + gap), 40, bh);
    recenter_->setGeometry(w - pad - 96, by, 96, bh);

    const QList<QWidget*> controls{theme_btn_, head_btn_, clear_btn_,
                                   zoom_in_, zoom_out_, recenter_};
    for (QWidget* b : controls) b->raise();
}

void MainWindow::resizeEvent(QResizeEvent* e)
{
    QMainWindow::resizeEvent(e);
    // Keep the map a sane share of the window rather than a fixed slab. The
    // overlay follows from MapView::resized(), once the new size is real.
    map_->setFixedWidth(std::clamp(width() * 40 / 100, 380, 900));
}

}  // namespace visionpilot::ui
