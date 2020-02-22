#pragma once

#include "source/core/geometry/geometry.h"

#include "third_party/libicp/icpPointToPlane.h"
#include "third_party/libicp/icpPointToPoint.h"
#include "third_party/libicp/matrix.h"

#include <optional>

namespace mte {
namespace localization {

typedef std::vector<geometry::Point3d> PointCloud;

PointCloud TransformPoints(const PointCloud& points, const geometry::Transform3d& tf) {
    PointCloud transformed_points;
    transformed_points.reserve(points.size());
    for (const geometry::Point3d& p : points) {
        transformed_points.emplace_back(tf * p);
    }
    return transformed_points;
}

class ICPLocalizer {
   public:
    ICPLocalizer(const PointCloud& model_points, double inlier_dist);

    geometry::Transform3d Fit(const PointCloud& template_points,
                              const geometry::Transform3d& initial_guess);

   private:
    const double inlier_dist;
    std::vector<double> model_point_data;
    std::optional<libicp::IcpPointToPoint> icp_backend;
};

namespace detail {

libicp::Matrix ToLibicpMatrix(const geometry::MatrixXd& geo_mat);
geometry::MatrixXd FromLibicpMatrix(const libicp::Matrix& libicp_mat);
std::pair<libicp::Matrix, libicp::Matrix> DecomposeTransform(const geometry::Transform3d& tf);
geometry::Transform3d ComposeTransform(const libicp::Matrix& rotation,
                                       const libicp::Matrix& translation);

}  // namespace detail

}  // namespace localization
}  // namespace mte
