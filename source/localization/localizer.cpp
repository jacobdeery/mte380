#include "localizer.h"

#include "source/core/calibration.h"
#include "source/core/math/angle.h"

#include <cmath>
#include <iostream>

using namespace mte::calibration;

namespace mte {
namespace localization {

lidar::PointCloud GetVisibleWallPoints(const Pose& pose) {
    const double angle_min = pose.yaw - kLidarFOVRight;
    const double angle_max = pose.yaw + kLidarFOVLeft;

    const double angle_xmin_ymin = atan2(kWallYMin - pose.y, kWallXMin - pose.x);
    const double angle_xmin_ymax = atan2(kWallYMax - pose.y, kWallXMin - pose.x);
    const double angle_xmax_ymin = atan2(kWallYMin - pose.y, kWallXMax - pose.x);
    const double angle_xmax_ymax = atan2(kWallYMax - pose.y, kWallXMax - pose.x);

    std::vector<math::geometry::Point3d> triplets;

    for (double angle = angle_min; angle < angle_max;
         angle += calibration::kLidarAngularResolution) {
        if (math::IsInAngularBounds(angle, angle_xmax_ymax, angle_xmin_ymax)) {
            const double x_coord = (kWallYMax - pose.y) / tan(angle) + pose.x;
            triplets.emplace_back(x_coord, kWallYMax, 0);
        } else if (math::IsInAngularBounds(angle, angle_xmax_ymin, angle_xmax_ymax)) {
            const double y_coord = (kWallXMax - pose.x) * tan(angle) + pose.y;
            triplets.emplace_back(kWallXMax, y_coord, 0);
        } else if (math::IsInAngularBounds(angle, angle_xmin_ymin, angle_xmax_ymin)) {
            const double x_coord = (kWallYMin - pose.y) / tan(angle) + pose.x;
            triplets.emplace_back(x_coord, kWallYMin, 0);
        } else {
            const double y_coord = (kWallXMin - pose.x) * tan(angle) + pose.y;
            triplets.emplace_back(kWallXMin, y_coord, 0);
        }
    }

    return lidar::PointCloud{triplets};
}

}  // namespace localization
}  // namespace mte
