#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <tuple>

namespace mte {
namespace geometry {

using Eigen::Vector3d;
typedef Eigen::Vector3d Point3d;

typedef Eigen::Transform<double, 3, Eigen::Affine> Transform3d;
using Eigen::Translation3d;

// YPR angles are assumed to be intrinsic Tait-Bryan rotations applied in the order yaw (z) -->
// pitch (y') --> roll (x").

Transform3d TransformFromYPR(double yaw, double pitch, double roll);
std::tuple<double, double, double> YPRFromTransform(const Transform3d& tf);

}  // namespace geometry
}  // namespace mte
