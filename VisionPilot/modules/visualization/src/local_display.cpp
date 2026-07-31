#include <opencv2/highgui.hpp>
#include <visualization/local_display.hpp>
#include <visualization/visualization.hpp>

namespace visualization
{
    LocalDisplay::LocalDisplay(bool show_window)
        : show_window_(show_window)
    {
        if (show_window_)
            cv::namedWindow("VisionPilot", cv::WINDOW_NORMAL);
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

    bool LocalDisplay::stop()
    {
        cv::destroyAllWindows();
        return true;
    }
}
