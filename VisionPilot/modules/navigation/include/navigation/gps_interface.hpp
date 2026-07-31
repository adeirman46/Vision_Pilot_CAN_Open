#ifndef VISIONPILOT_GPS_INTERFACE_HPP
#define VISIONPILOT_GPS_INTERFACE_HPP

#include <cstdint>
#include <optional>

namespace navigation {

struct GpsFix {
    double   latitude    = 0.0;    // deg
    double   longitude   = 0.0;    // deg
    double   altitude_m  = 0.0;
    double   speed_ms    = 0.0;    // derived from successive fixes when the
    double   bearing_deg = 0.0;    // source reports position only
    double   accuracy_m  = 0.0;    // 0 = unknown
    uint64_t stamp_ms    = 0;      // wall clock of the fix
    bool     has_bearing = false;
};

// A position source. read() is non-blocking and returns the most recent fix, or
// nothing at all until one has arrived.
class GpsInterface
{
public:
    GpsInterface() = default;
    virtual ~GpsInterface() = default;

    virtual std::optional<GpsFix> read() = 0;

    // False once the source has permanently given up (track exhausted, socket
    // closed for good). A source that is merely waiting for a fix stays open.
    virtual bool is_open() const = 0;

    // Human-readable label for the panel status line.
    virtual const char* name() const = 0;
};

}  // namespace navigation

#endif  // VISIONPILOT_GPS_INTERFACE_HPP
