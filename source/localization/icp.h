#pragma once

#include "source/core/math/geometry.h"
#include "source/sensing/lidar/lidar_types.h"

#include "third_party/libicp/icpPointToPlane.h"
#include "third_party/libicp/icpPointToPoint.h"
#include "third_party/libicp/matrix.h"

#include <optional>

namespace mte {
namespace localization {

math::geometry::Transform3d LocalizeToPointCloud(const lidar::PointCloud& model_points,
                                                 const lidar::PointCloud& template_points,
                                                 const math::geometry::Transform3d& initial_guess,
                                                 double inlier_dist);

namespace detail {

libicp::Matrix ToLibicpMatrix(const math::geometry::MatrixXd& geo_mat);
math::geometry::MatrixXd FromLibicpMatrix(const libicp::Matrix& libicp_mat);

libicp::Matrix PromoteTranslationTo3D(const libicp::Matrix& mat_2d);
libicp::Matrix PromoteRotationTo3D(const libicp::Matrix& mat_2d);
libicp::Matrix DemoteTranslationTo2D(const libicp::Matrix& mat_3d);
libicp::Matrix DemoteRotationTo2D(const libicp::Matrix& mat_3d);

std::pair<libicp::Matrix, libicp::Matrix> DecomposeTransform(const math::geometry::Transform3d& tf);
math::geometry::Transform3d ComposeTransform(const libicp::Matrix& rotation,
                                             const libicp::Matrix& translation);

}  // namespace detail

}  // namespace localization
}  // namespace mte
