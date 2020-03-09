#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <tuple>

namespace mte {
namespace geometry {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
typedef Eigen::Vector3d Point3d;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> PointSet;

typedef Eigen::Transform<double, 3, Eigen::Affine> Transform3d;
using Eigen::Translation3d;

// YPR angles are assumed to be intrinsic Tait-Bryan rotations applied in the order yaw (z) -->
// pitch (y') --> roll (x").

Transform3d TransformFromYPR(double yaw, double pitch, double roll);
std::tuple<double, double, double> YPRFromTransform(const Transform3d& tf);

template <class MatrixType>
std::vector<double> Unravel(const MatrixType& mat) {
    const Eigen::VectorXd v = mat.template reshaped<Eigen::RowMajor>();
    return std::vector<double>{v.begin(), v.end()};
}

}  // namespace geometry
}  // namespace mte
