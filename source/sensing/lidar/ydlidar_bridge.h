#pragma once

#include "CYdLidar.h"

#include <functional>
#include <optional>

namespace mte {
namespace lidar {

typedef std::function<bool(const ydlidar::LaserPoint&)> LidarFilterFunction;

ydlidar::LaserScan FilterScan(const ydlidar::LaserScan& scan, const LidarFilterFunction& filter_fn);

class LidarBridge {
   public:
    LidarBridge();
    bool Initialize();
    std::optional<ydlidar::LaserScan> Scan();

   private:
    ydlidar::CYdLidar laser;
};

}  // namespace lidar
}  // namespace mte
