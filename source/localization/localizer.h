#pragma once

#include "pose.h"
#include "source/sensing/lidar/lidar_types.h"

namespace mte {
namespace localization {

constexpr double kWallXMin{0};
constexpr double kWallXMax{1.8};
constexpr double kWallYMin{0};
constexpr double kWallYMax{1.8};

Pose GetInitialPose();
lidar::PointCloud GetVisibleWallPoints(const Pose& pose);

}  // namespace localization
}  // namespace mte
