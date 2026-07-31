#ifndef VISIONPILOT_VISUAL_INTERFACE_HPP
#define VISIONPILOT_VISUAL_INTERFACE_HPP
#include <opencv2/highgui.hpp>

class VisualInterface
{
public:
    VisualInterface();
    virtual ~VisualInterface() = default;

    virtual bool render_frame(const cv::Mat& display_frame) = 0;
    virtual bool stop() = 0;

    // Key typed into the window during the last render_frame(), or -1. Lets the
    // app take text input through the only window it has; a headless or remote
    // backend simply never reports one.
    virtual int last_key() const { return -1; }
};

#endif //VISIONPILOT_VISUAL_INTERFACE_HPP
