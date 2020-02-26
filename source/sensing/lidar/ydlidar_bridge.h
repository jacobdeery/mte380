#pragma once

#include "lidar_types.h"

#include "CYdLidar.h"

#include <optional>

namespace mte {
namespace lidar {

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
