#pragma once

#include "pose.h"
#include "source/sensing/lidar/lidar_types.h"

namespace mte {
namespace localization {

namespace detail {

bool HasPoseConverged(const Pose& p1, const Pose& p2, double d_pos_max, double d_yaw_max);

}

lidar::PointCloud GetVisibleWallPoints(const Pose& pose);

Pose Localize(const lidar::PointCloud& pc, const Pose& old_pose);

}  // namespace localization
}  // namespace mte
