// VisionPilot Qt front-end.
//
// The OpenCV window could only ever show pixels: no search field, no buttons,
// and one thread shared between inference and the interface, which is why the
// map moved in 40 ms steps. This runs the pipeline on a worker thread and the
// map on the GUI thread's own timer, so the interface stays responsive whatever
// the models are doing.
#include <QApplication>
#include <QFont>

#include <config/vision_pilot_config.hpp>
#include <logging/logger.hpp>

#include "main_window.hpp"

int main(int argc, char** argv)
{
    // Both of these must be set before the QApplication exists — afterwards
    // they do nothing. Without them Qt lays every widget out in logical pixels,
    // renders once at 1x, and lets the compositor stretch the result to the
    // real screen. Text scaled from a low-resolution bitmap is exactly the soft,
    // smeared lettering it looks like; asking for the true device pixel ratio
    // makes the glyphs be rasterised at the size they are actually shown.
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps, true);

    QApplication app(argc, argv);

    QFont f = app.font();
    f.setHintingPreference(QFont::PreferFullHinting);   // crisp stems at small sizes
    app.setFont(f);

    Config cfg;
    try {
        cfg = load_vision_pilot_config();
    } catch (const std::exception& e) {
        VP_ERROR("Config: %s", e.what());
        return 1;
    }

    bool debug_viz = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a(argv[i]);
        if (a == "--debug-viz")   debug_viz = true;
        else if (a == "--nav")    cfg.navigation.enabled = true;
        else if (a == "--no-nav") cfg.navigation.enabled = false;
        else if (a == "--guidance")    cfg.guidance.enabled = true;
        else if (a == "--no-guidance") cfg.guidance.enabled = false;
    }
    // The map panel is the point of this front-end; without it there is nothing
    // the plain OpenCV window does not already do.
    cfg.navigation.enabled = cfg.navigation.enabled || true;
    // The Qt widget owns the layout, so the engine must not also compose a
    // fixed canvas of its own.
    cfg.navigation.out_width = 0;
    cfg.navigation.out_height = 0;

    visionpilot::ui::MainWindow w(cfg, debug_viz);
    w.show();
    return app.exec();
}
