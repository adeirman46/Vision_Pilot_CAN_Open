#ifndef VISIONPILOT_DISPLAY_HPP
#define VISIONPILOT_DISPLAY_HPP
#include <mutex>

#include <opencv2/core/mat.hpp>
#include <visualization/visual_interface.hpp>

namespace visualization
{
    class LocalDisplay : public VisualInterface
    {
    public:
        // show_window == false runs headless: no OpenCV window is created and
        // render_frame() becomes a no-op. This keeps VisionPilot usable in
        // environments without a display (e.g. containers, CI).
        explicit LocalDisplay(bool show_window = true);
        ~LocalDisplay();

        bool render_frame(const cv::Mat& display_frame) override;
        bool stop() override;
        int  last_key() const override { return last_key_; }
        MouseState mouse() override;

    private:
        bool show_window_;
        int  last_w_ = 0;   // frame size the window was last fitted to
        int  last_h_ = 0;
        int  last_key_ = -1;

        // Written by the OpenCV mouse callback (its own thread), read once per
        // frame by the render loop.
        static void on_mouse(int event, int x, int y, int flags, void* self);
        std::mutex  mouse_mtx_;
        MouseState  mouse_;
        int         press_x_ = 0, press_y_ = 0;   // to tell a click from a drag
        bool        press_shift_ = false;        // shift held at the press
    };
}

#endif //VISIONPILOT_DISPLAY_HPP
