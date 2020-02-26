#include "ydlidar_bridge.h"

namespace mte {
namespace lidar {

LidarBridge::LidarBridge() {
    std::string port = "/dev/ttyUSB0";
    const int baud = 115200;

    std::map<std::string, std::string> lidars = ydlidar::YDlidarDriver::lidarPortList();
    port = lidars.begin()->second;

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
    bool hardError;
    ydlidar::LaserScan scan;

    if (laser.doProcessSimple(scan, hardError)) {
        return scan;
    }
    return std::nullopt;
}

}  // namespace lidar
}  // namespace mte
