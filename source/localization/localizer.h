#pragma once

#include "pose.h"
#include "source/sensing/lidar/lidar_types.h"

namespace mte {
namespace localization {

Pose GetInitialPose();
lidar::PointCloud GetVisibleWallPoints(const Pose& pose);

}  // namespace localization
}  // namespace mte
