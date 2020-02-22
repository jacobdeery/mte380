#include "icp.h"

#include "source/core/logging.h"

namespace mte {
namespace localization {

namespace detail {

libicp::Matrix ToLibicpMatrix(const geometry::MatrixXd& geo_mat) {
    const auto mat_data = geometry::Unravel(geo_mat);
    return libicp::Matrix(geo_mat.rows(), geo_mat.cols(), mat_data.data());
}

geometry::MatrixXd FromLibicpMatrix(const libicp::Matrix& libicp_mat) {
    geometry::MatrixXd geo_mat(libicp_mat.m, libicp_mat.n);
    for (int i = 0; i < libicp_mat.m; ++i) {
        for (int j = 0; j < libicp_mat.n; ++j) {
            geo_mat(i, j) = libicp_mat.val[i][j];
        }
    }
    return geo_mat;
}

std::pair<libicp::Matrix, libicp::Matrix> DecomposeTransform(const geometry::Transform3d& tf) {
    const geometry::Matrix3d rotation{tf.rotation()};
    const geometry::Vector3d translation{tf.translation()};
    return {ToLibicpMatrix(rotation), ToLibicpMatrix(translation)};
}

geometry::Transform3d ComposeTransform(const libicp::Matrix& rotation,
                                       const libicp::Matrix& translation) {
    CHECK(translation.m == 3 && translation.n == 1, "Translation vector must be 3x1 (got " +
                                                        std::to_string(translation.m) + "x" +
                                                        std::to_string(translation.n) + ")");
    CHECK(rotation.m == 3 && rotation.n == 3, "Rotation matrix must be 3x3 (got " +
                                                  std::to_string(rotation.m) + "x" +
                                                  std::to_string(rotation.n) + ")");

    return geometry::Transform3d{geometry::Translation3d{FromLibicpMatrix(translation)}} *
           geometry::Transform3d{geometry::Matrix3d{FromLibicpMatrix(rotation)}};
}

}  // namespace detail

ICPLocalizer::ICPLocalizer(const PointCloud& model_points, double inlier_dist)
    : inlier_dist{inlier_dist} {
    // TODO(jacob): Investigate more efficient ways to get this data into ICP (possibly a PointCloud
    // type with contiguous memory instead of a vector of Point3ds).
    model_point_data.reserve(model_points.size() * 3);
    for (const geometry::Point3d& point : model_points) {
        model_point_data.emplace_back(point[0]);
        model_point_data.emplace_back(point[1]);
        model_point_data.emplace_back(point[2]);
    }
    icp_backend.emplace(model_point_data.data(), model_points.size(), 3);
    icp_backend.value().setMaxIterations(500);
    icp_backend.value().setMinDeltaParam(1e-6);
}

geometry::Transform3d ICPLocalizer::Fit(const PointCloud& template_points,
                                        const geometry::Transform3d& initial_guess) {
    std::vector<double> template_point_data;
    template_point_data.reserve(template_points.size() * 3);
    for (const geometry::Point3d& point : template_points) {
        template_point_data.emplace_back(point[0]);
        template_point_data.emplace_back(point[1]);
        template_point_data.emplace_back(point[2]);
    }
    auto [rotation, translation] = detail::DecomposeTransform(initial_guess);
    icp_backend.value().fit(template_point_data.data(), template_points.size(), rotation,
                            translation, inlier_dist);
    // NOTE: libicp returns the transform that gives M = R*T + t
    return detail::ComposeTransform(rotation, translation);
}

}  // namespace localization
}  // namespace mte
