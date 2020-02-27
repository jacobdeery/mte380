#include "ydlidar_bridge.h"

namespace mte {
namespace lidar {

constexpr int baud{115200};

LidarBridge::LidarBridge() {
    const auto lidars = ydlidar::YDlidarDriver::lidarPortList();
    const auto port = lidars.begin()->second;

    // TODO(jacob): Get these from CAD/calibration
    constexpr double kAngleMinDeg{170};
    constexpr double kAngleMaxDeg{190};

    laser.setMinAngle(kAngleMinDeg);
    laser.setMaxAngle(kAngleMaxDeg);

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
        return scan;
    }
    return std::nullopt;
}

// TODO(jacob): Delete this function after construction check demo
bool IsThereAWall(const ydlidar::LaserScan& scan) {
    constexpr double kNumClosePointsRequired{20};
    constexpr double kWallDistM{0.2};

    if (scan.data.size() < kNumClosePointsRequired) {
        return false;
    }

    int num_close_points = 0;
    for (const auto& point : scan.data) {
        if (point.range <= kWallDistM) {
            ++num_close_points;
            if (num_close_points == kNumClosePointsRequired) {
                return true;
            }
        }
    }

    return false;
}

}  // namespace lidar
}  // namespace mte
