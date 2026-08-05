// Live 3D view of the Yahboom 10-axis IMU: turn the module, watch the model
// turn with it. This is the functional check — if roll, pitch and yaw on screen
// match what your hands are doing, the wiring, the axes and the scale factors
// are all right.
//
//   imu_view                       auto-detect the port, baud rate and format
//   imu_view --port /dev/ttyUSB0 --baud 115200
//   imu_view --protocol wit        skip detection, use the WitMotion parser
//   imu_view --raw-attitude        ignore the module's own solution and fuse
//                                  the raw channels here instead

#include <cstdio>
#include <string>

#include <QApplication>
#include <QFont>

#include <imu/imu_reader.hpp>

#include "imu_window.hpp"

namespace {

imu::Protocol parse_protocol(const std::string& v)
{
    if (v == "yahboom" || v == "mra02" || v == "mra")  return imu::Protocol::Yahboom;
    if (v == "wit" || v == "witmotion" || v == "wt901") return imu::Protocol::WitMotion;
    if (v == "yesense" || v == "yis")                   return imu::Protocol::Yesense;
    if (v == "ascii" || v == "text")                    return imu::Protocol::Ascii;
    return imu::Protocol::Unknown;
}

void usage()
{
    std::printf(
        "usage: imu_view [--port DEV] [--baud N] [--protocol wit|yesense|ascii]\n"
        "                [--raw-attitude] [--no-mag]\n\n"
        "  --raw-attitude  fuse the raw accel/gyro/mag here instead of trusting\n"
        "                  the module's own attitude output\n"
        "  --no-mag        leave the magnetometer out of the fusion\n");
}

}  // namespace

int main(int argc, char** argv)
{
    // Ask for the real device pixel ratio before the application exists —
    // afterwards it has no effect, and text ends up scaled from a low-resolution
    // bitmap, which is what makes it look soft on a HiDPI screen.
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps, true);

    imu::ReaderConfig cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "error: %s needs a value\n", what);
                std::exit(1);
            }
            return argv[++i];
        };

        if (a == "--port" || a == "-p")       cfg.device = next("--port");
        else if (a == "--baud" || a == "-b")  cfg.baud = std::stoi(next("--baud"));
        else if (a == "--protocol")           cfg.protocol = parse_protocol(next("--protocol"));
        else if (a == "--raw-attitude")       cfg.attitude.prefer_device = false;
        else if (a == "--no-mag")             cfg.attitude.use_mag = false;
        else if (a == "-h" || a == "--help")  { usage(); return 0; }
        else {
            std::fprintf(stderr, "error: unknown option '%s'\n", a.c_str());
            usage();
            return 1;
        }
    }

    QApplication app(argc, argv);

    QFont f = app.font();
    f.setFamily(QStringLiteral("DejaVu Sans"));
    f.setPointSizeF(10.0);
    f.setHintingPreference(QFont::PreferFullHinting);   // crisp stems at small sizes
    app.setFont(f);

    imu::view::MainWindow win(cfg);
    win.show();
    return app.exec();
}
