#include "ydlidar_bridge.h"

#include "source/core/logging.h"
#include "source/core/math/angle.h"

#include <functional>

namespace mte {
namespace lidar {

constexpr int baud{115200};

// TODO(jacob): Get these from CAD/calibration
constexpr double kMinAngle = math::DegToRad(170);
constexpr double kMaxAngle = math::DegToRad(190);

ydlidar::LaserScan FilterScan(const ydlidar::LaserScan& scan,
                              const LidarFilterFunction& filter_fn) {
    ydlidar::LaserScan filtered_scan;
    filtered_scan.system_time_stamp = scan.system_time_stamp;
    filtered_scan.config = scan.config;

    filtered_scan.data.reserve(scan.data.size());
    std::copy_if(scan.data.begin(), scan.data.end(), std::back_inserter(filtered_scan.data),
                 filter_fn);

    return filtered_scan;
}

LidarBridge::LidarBridge() {
    const auto lidars = ydlidar::YDlidarDriver::lidarPortList();
    const auto port = lidars.begin()->second;

    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);
    laser.setFixedResolution(false);
    laser.setReversion(false);
    laser.setAutoReconnect(true);
    laser.setGlassNoise(true);
    laser.setSunNoise(true);
}

bool LidarBridge::Initialize() {
    return laser.initialize();
}

std::optional<ydlidar::LaserScan> LidarBridge::Scan() {
    bool is_hardware_error;
    ydlidar::LaserScan scan;

    if (laser.doProcessSimple(scan, is_hardware_error)) {
        const auto filter_fn = [](const ydlidar::LaserPoint& pt) {
            return pt.range != 0 && math::IsInAngularBounds(pt.angle, kMinAngle, kMaxAngle);
        };
        const auto filtered_scan = FilterScan(scan, filter_fn);
        if (filtered_scan.data.size() > 0) {
            return filtered_scan;
        } else {
            LOG_WARN("Filtered lidar scan had no points");
        }
    } else if (is_hardware_error) {
        LOG_WARN("Lidar experienced a hardware error");
    }
    return std::nullopt;
}

}  // namespace lidar
}  // namespace mte
