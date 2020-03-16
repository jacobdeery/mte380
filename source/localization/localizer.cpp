#include "localizer.h"

#include "icp.h"
#include "source/core/calibration.h"
#include "source/core/math/angle.h"

#include <cmath>
#include <limits>

using namespace mte::calibration;

namespace mte {
namespace localization {

// TODO(jacob): Set this based on experimental testing.
constexpr double kInlierDist{-1};

constexpr int kMaxIterations{5};
constexpr double kHalfMillimetre{0.5e-3};
constexpr double kHalfDegree{mte::math::DegToRad(0.5)};

namespace detail {

bool HasPoseConverged(const Pose& p1, const Pose& p2, double d_pos_max, double d_yaw_max) {
    if ((p2.Position() - p1.Position()).norm() > d_pos_max) {
        return false;
    } else if (fabs(p2.yaw - p1.yaw) > d_yaw_max) {
        return false;
    }
    return true;
}

}  // namespace detail

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

Pose Localize(const lidar::PointCloud& pc, const Pose& old_pose) {
    Pose last_estimate = old_pose;

    for (int i = 0; i < kMaxIterations; ++i) {
        const auto model_points = GetVisibleWallPoints(last_estimate);
        const Pose new_estimate{LocalizeToPointCloud(model_points, pc, last_estimate, kInlierDist)};
        if (detail::HasPoseConverged(last_estimate, new_estimate, kHalfMillimetre, kHalfDegree)) {
            return new_estimate;
        }
        last_estimate = new_estimate;
    }

    return last_estimate;
}

}  // namespace localization
}  // namespace mte
