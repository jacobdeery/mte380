#include "icp.h"

#include "source/core/logging.h"

namespace mte {
namespace localization {

namespace detail {

libicp::Matrix ToLibicpMatrix(const math::geometry::MatrixXd& geo_mat) {
    const auto mat_data = math::geometry::Unravel(geo_mat);
    return libicp::Matrix(geo_mat.rows(), geo_mat.cols(), mat_data.data());
}

math::geometry::MatrixXd FromLibicpMatrix(const libicp::Matrix& libicp_mat) {
    math::geometry::MatrixXd geo_mat(libicp_mat.m, libicp_mat.n);
    for (int i = 0; i < libicp_mat.m; ++i) {
        for (int j = 0; j < libicp_mat.n; ++j) {
            geo_mat(i, j) = libicp_mat.val[i][j];
        }
    }
    return geo_mat;
}

std::pair<libicp::Matrix, libicp::Matrix> DecomposeTransform(
    const math::geometry::Transform3d& tf) {
    const math::geometry::Matrix3d rotation{tf.rotation()};
    const math::geometry::Vector3d translation{tf.translation()};
    return {DemoteRotationTo2D(ToLibicpMatrix(rotation)),
            DemoteTranslationTo2D(ToLibicpMatrix(translation))};
}

math::geometry::Transform3d ComposeTransform(const libicp::Matrix& rotation,
                                             const libicp::Matrix& translation) {
    CHECK(translation.m == 2 && translation.n == 1, "Translation vector must be 2x1 (got " +
                                                        std::to_string(translation.m) + "x" +
                                                        std::to_string(translation.n) + ")");
    CHECK(rotation.m == 2 && rotation.n == 2, "Rotation matrix must be 2x2 (got " +
                                                  std::to_string(rotation.m) + "x" +
                                                  std::to_string(rotation.n) + ")");

    const math::geometry::Transform3d tf_trans{
        math::geometry::Translation3d{FromLibicpMatrix(PromoteTranslationTo3D(translation))}};
    const math::geometry::Transform3d tf_rot{
        math::geometry::Matrix3d{FromLibicpMatrix(PromoteRotationTo3D(rotation))}};

    return tf_trans * tf_rot;
}

libicp::Matrix PromoteTranslationTo3D(const libicp::Matrix& mat_2d) {
    CHECK(mat_2d.m == 2 && mat_2d.n == 1, "2D translation vector must be 2x1 (got " +
                                              std::to_string(mat_2d.m) + "x" +
                                              std::to_string(mat_2d.n) + ")");
    const std::vector<double> data{mat_2d.val[0][0], mat_2d.val[1][0], 0};
    return libicp::Matrix(3, 1, data.data());
}

libicp::Matrix PromoteRotationTo3D(const libicp::Matrix& mat_2d) {
    CHECK(mat_2d.m == 2 && mat_2d.n == 2, "2D rotation matrix must be 2x2 (got " +
                                              std::to_string(mat_2d.m) + "x" +
                                              std::to_string(mat_2d.n) + ")");
    // clang-format off
    const std::vector<double> data{
        mat_2d.val[0][0], mat_2d.val[0][1], 0,
        mat_2d.val[1][0], mat_2d.val[1][1], 0,
        0, 0, 1
    };
    // clang-format on
    return libicp::Matrix(3, 3, data.data());
}

libicp::Matrix DemoteTranslationTo2D(const libicp::Matrix& mat_3d) {
    CHECK(mat_3d.m == 3 && mat_3d.n == 1, "3D translation vector must be 3x1 (got " +
                                              std::to_string(mat_3d.m) + "x" +
                                              std::to_string(mat_3d.n) + ")");
    const std::vector<double> data{mat_3d.val[0][0], mat_3d.val[1][0]};
    return libicp::Matrix(2, 1, data.data());
}

libicp::Matrix DemoteRotationTo2D(const libicp::Matrix& mat_3d) {
    CHECK(mat_3d.m == 3 && mat_3d.n == 3, "3D rotation matrix must be 3x3 (got " +
                                              std::to_string(mat_3d.m) + "x" +
                                              std::to_string(mat_3d.n) + ")");
    // clang-format off
    const std::vector<double> data{
        mat_3d.val[0][0], mat_3d.val[0][1],
        mat_3d.val[1][0], mat_3d.val[1][1]
    };
    // clang-format on
    return libicp::Matrix(2, 2, data.data());
}

}  // namespace detail

ICPLocalizer::ICPLocalizer(const lidar::PointCloud& model_points, double inlier_dist)
    : inlier_dist{inlier_dist} {
    icp_backend.emplace(model_points.Flatpack2D().data(), model_points.NumPoints(), 2);
    icp_backend.value().setMaxIterations(500);
    icp_backend.value().setMinDeltaParam(1e-6);
}

math::geometry::Transform3d ICPLocalizer::Fit(const lidar::PointCloud& template_points,
                                              const math::geometry::Transform3d& initial_guess) {
    auto [rotation, translation] = detail::DecomposeTransform(initial_guess);
    icp_backend.value().fit(template_points.Flatpack2D().data(), template_points.NumPoints(),
                            rotation, translation, inlier_dist);
    // NOTE: libicp returns the transform that gives M = R*T + t. For localization, we want the
    // inverse (the transform required for us to have observed the points we did.)
    return detail::ComposeTransform(rotation, translation).inverse();
}

}  // namespace localization
}  // namespace mte
