#ifndef VISIONPILOT_FUSED_GPS_HPP
#define VISIONPILOT_FUSED_GPS_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <imu/imu_reader.hpp>

#include <navigation/gps_imu_ekf.hpp>
#include <navigation/gps_interface.hpp>

namespace navigation {

// GPS and IMU fused into a single position source.
//
// It is a GpsInterface, deliberately: everything downstream — map matching, the
// route, the panel — keeps consuming fixes exactly as before and neither knows
// nor cares that an IMU is involved. The only difference is that the fixes now
// arrive at IMU rate, move smoothly, and carry a heading that survives a stop.
//
// What each sensor contributes:
//   GPS   absolute position, once or twice a second, jumping by metres
//   IMU   acceleration and yaw rate at 25 Hz, smooth, drifting
//
// The filter runs on its own thread. A dead IMU degrades to plain GPS, and a
// dead GPS coasts on the IMU with the uncertainty visibly growing — neither
// stops the drive.
class FusedImuGps : public GpsInterface
{
public:
    FusedImuGps(std::unique_ptr<GpsInterface> gps, const std::string& imu_port,
                int imu_baud);
    ~FusedImuGps() override;

    std::optional<GpsFix> read() override;
    bool                  is_open() const override { return true; }
    const char*           name() const override { return "gps+imu"; }
    std::optional<GpsFix> raw_fix() const override;

    // For the status line: whether the IMU actually turned up, and how good the
    // filter thinks its position is.
    bool        imu_ok() const { return imu_ok_; }
    std::string status() const;
    double      position_sigma_m() const;

private:
    void run();

    std::unique_ptr<GpsInterface> gps_;
    std::unique_ptr<imu::ImuReader> imu_;
    GpsImuEkf                     ekf_;

    mutable std::mutex    mtx_;
    std::optional<GpsFix> fused_;
    std::optional<GpsFix> raw_;
    std::string           status_;
    std::atomic<bool>     imu_ok_{false};
    std::atomic<bool>     running_{false};
    std::thread           thread_;

    uint64_t last_imu_us_ = 0;
    uint64_t last_gps_ms_ = 0;
};

}  // namespace navigation

#endif  // VISIONPILOT_FUSED_GPS_HPP
