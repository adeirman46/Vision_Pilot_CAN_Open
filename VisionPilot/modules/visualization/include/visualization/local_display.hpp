#ifndef VISIONPILOT_DISPLAY_HPP
#define VISIONPILOT_DISPLAY_HPP
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

    private:
        bool show_window_;
        int  last_w_ = 0;   // frame size the window was last fitted to
        int  last_h_ = 0;
        int  last_key_ = -1;
    };
}

#endif //VISIONPILOT_DISPLAY_HPP
