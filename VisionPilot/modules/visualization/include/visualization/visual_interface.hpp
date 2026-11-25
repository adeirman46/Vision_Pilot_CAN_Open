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

    // Pointer state inside the window, sampled once per frame. Enough to drag
    // and zoom the map: the consumer keeps its own previous position and works
    // in deltas, so a few dropped move events cost nothing.
    struct MouseState {
        int  x = 0, y = 0;
        bool left_down = false;
        int  wheel = 0;        // notches since the last read, +up / -down
        bool inside = false;
        // Left-clicks completed since the last read — pressed and released
        // without travelling, so a drag is not one. Where they were released is
        // (x, y), which for a click is also where they were pressed.
        int  clicks = 0;
        // Shift was held at the press. Carried from the press rather than read
        // at the release because the press is what the driver meant, and it is
        // what tells a "go here" apart from a "go here next".
        bool shift_click = false;
    };
    virtual MouseState mouse() { return {}; }
};

#endif //VISIONPILOT_VISUAL_INTERFACE_HPP
