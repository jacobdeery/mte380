#pragma once

#include "source/core/math/angle.h"
#include "source/core/math/geometry.h"
#include "source/localization/pose.h"

namespace mte {
namespace calibration {

constexpr double kYdLidarReference = math::DegToRad(180);
constexpr double kLidarFOVLeft = math::DegToRad(10);
constexpr double kLidarFOVRight = math::DegToRad(10);
constexpr double kLidarMinAngle = kYdLidarReference - kLidarFOVLeft;
constexpr double kLidarMaxAngle = kYdLidarReference + kLidarFOVRight;
constexpr double kLidarAngularResolution = math::DegToRad(1);

extern const math::geometry::Transform3d T_robot_lidar;

extern const localization::Pose initial_pose;

}  // namespace calibration
}  // namespace mte
