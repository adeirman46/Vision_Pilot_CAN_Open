#include <cstdlib>

#include <opencv2/highgui.hpp>
#include <visualization/local_display.hpp>
#include <visualization/visualization.hpp>

namespace visualization
{
    LocalDisplay::LocalDisplay(bool show_window)
        : show_window_(show_window)
    {
        if (show_window_)
        {
            cv::namedWindow("VisionPilot", cv::WINDOW_NORMAL);
            cv::setMouseCallback("VisionPilot", &LocalDisplay::on_mouse, this);
        }
    }

    LocalDisplay::~LocalDisplay()
    {
        cv::destroyAllWindows();
    }

    bool LocalDisplay::render_frame(const cv::Mat& display_frame)
    {
        if (!show_window_)
            return true;
        // Resize only when the frame size actually changes. Doing it every frame
        // fought the window manager, and any window left larger than the image —
        // which is what happens the moment the map panel widens the frame — is
        // padded by the toolkit with its own background, the white border.
        if (display_frame.cols != last_w_ || display_frame.rows != last_h_)
        {
            last_w_ = display_frame.cols;
            last_h_ = display_frame.rows;
            cv::resizeWindow("VisionPilot", last_w_, last_h_);
        }
        cv::imshow("VisionPilot", display_frame);
        // waitKey doubles as the window's event pump and its keyboard: the key
        // it returns here is how the destination box gets typed into.
        last_key_ = cv::waitKey(1);
        return true;
    }

    void LocalDisplay::on_mouse(int event, int x, int y, int flags, void* self)
    {
        auto* d = static_cast<LocalDisplay*>(self);
        std::lock_guard<std::mutex> lock(d->mouse_mtx_);
        d->mouse_.x = x;
        d->mouse_.y = y;
        d->mouse_.inside = true;
        if (event == cv::EVENT_LBUTTONDOWN) {
            d->mouse_.left_down = true;
            d->press_x_ = x;
            d->press_y_ = y;
        }
        if (event == cv::EVENT_LBUTTONUP) {
            d->mouse_.left_down = false;
            // A press and release in the same place is a click; one that
            // travelled was somebody moving the map, and must not be read as
            // "go here".
            if (std::abs(x - d->press_x_) <= 4 && std::abs(y - d->press_y_) <= 4)
                ++d->mouse_.clicks;
        }
        // Some builds report the button only in `flags`; trust that when it is
        // there, so a drag that started outside the window still tracks.
        if (event == cv::EVENT_MOUSEMOVE)
            d->mouse_.left_down = (flags & cv::EVENT_FLAG_LBUTTON) != 0;
        if (event == cv::EVENT_MOUSEWHEEL)
            d->mouse_.wheel += cv::getMouseWheelDelta(flags) > 0 ? 1 : -1;
    }

    VisualInterface::MouseState LocalDisplay::mouse()
    {
        std::lock_guard<std::mutex> lock(mouse_mtx_);
        MouseState out = mouse_;
        mouse_.wheel  = 0;     // wheel notches are consumed, position persists
        mouse_.clicks = 0;
        return out;
    }

    bool LocalDisplay::stop()
    {
        cv::destroyAllWindows();
        return true;
    }
}
