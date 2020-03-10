#include "source/core/math/angle.h"
#include "source/core/testing/assertions.h"
#include "source/localization/icp.h"
#include "source/sensing/lidar/lidar_types.h"

#include "gtest/gtest.h"

using namespace mte::localization;
namespace mmg = mte::math::geometry;

constexpr double kSmallEps{1e-9};
constexpr double kBigEps{1e-6};
constexpr double k45Degrees{mte::math::DegToRad(45)};
constexpr double kInlierDist{-1};

void Expect_Libicp_Matrix_Equal(const libicp::Matrix& m1, const libicp::Matrix& m2) {
    ASSERT_EQ(m1.m, m2.m);
    ASSERT_EQ(m1.n, m2.n);
    for (int i = 0; i < m1.m; ++i) {
        for (int j = 0; j < m1.n; ++j) {
            EXPECT_DOUBLE_EQ(m1.val[i][j], m2.val[i][j])
                << "at i=" + std::to_string(i) + ", j=" + std::to_string(j);
        }
    }
}

void Expect_Libicp_Matrix_Near(const libicp::Matrix& m1, const libicp::Matrix& m2, double eps) {
    ASSERT_EQ(m1.m, m2.m);
    ASSERT_EQ(m1.n, m2.n);
    for (int i = 0; i < m1.m; ++i) {
        for (int j = 0; j < m1.n; ++j) {
            EXPECT_NEAR(m1.val[i][j], m2.val[i][j], eps)
                << "at i=" + std::to_string(i) + ", j=" + std::to_string(j);
        }
    }
}

mte::lidar::PointCloud MakeCorner(int points_per_wall = 100, double wall_length = 10) {
    double resolution = wall_length / points_per_wall;
    std::vector<mmg::Point3d> points;
    for (int i = 0; i < points_per_wall; ++i) {
        points.emplace_back(i * resolution, 0, 0);
    }
    for (int i = 1; i < points_per_wall; ++i) {
        points.emplace_back(wall_length, i * resolution, 0);
    }
    return mte::lidar::PointCloud{points};
}

TEST(LibicpIntegrationTests, LibicpMatrixConversionId) {
    const auto libicp_id = libicp::Matrix::eye(3);
    const mmg::Matrix3d geo_id = mmg::Matrix3d::Identity();

    Expect_Libicp_Matrix_Equal(libicp_id, detail::ToLibicpMatrix(geo_id));
    mte::ExpectMatrixEqual(geo_id, mmg::Matrix3d{detail::FromLibicpMatrix(libicp_id)});
}

TEST(LibicpIntegrationTests, LibicpMatrixConversionMoreValues) {
    const std::vector<double> matrix_vals{1, 2, 3, 4, 5, 6, 7, 8, 9};
    const libicp::Matrix libicp_mat{3, 3, matrix_vals.data()};

    // clang-format off
    const mmg::Matrix3d geo_mat{
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
    // clang-format on

    Expect_Libicp_Matrix_Equal(libicp_mat, detail::ToLibicpMatrix(geo_mat));
    mte::ExpectMatrixEqual(geo_mat, mmg::Matrix3d{detail::FromLibicpMatrix(libicp_mat)});
}

TEST(LibicpIntegrationTests, LibicpMatrixConversionRoundTrip) {
    // clang-format off
    const mmg::Matrix3d geo_mat{
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
    // clang-format on

    const mmg::Matrix3d round_trip_mat{detail::FromLibicpMatrix(detail::ToLibicpMatrix(geo_mat))};

    mte::ExpectMatrixEqual(geo_mat, round_trip_mat);
}

TEST(LibicpIntegrationTests, LibicpMatrixSingleRotation) {
    const double yaw = 0.5;

    const auto libicp_mat = libicp::Matrix::rotMatZ(yaw);
    const mmg::Matrix3d geo_mat{mmg::TransformFromYPR(yaw, 0, 0).rotation()};

    mte::ExpectMatrixNear(geo_mat, mmg::Matrix3d{detail::FromLibicpMatrix(libicp_mat)}, kSmallEps);
}

TEST(LibicpIntegrationTests, PromoteAndDemoteTranslations) {
    const std::vector<double> trans_vals_2d{1.0, 2.0};
    const std::vector<double> trans_vals_3d{1.0, 2.0, 0};

    const libicp::Matrix trans_2d{2, 1, trans_vals_2d.data()};
    const libicp::Matrix trans_3d{3, 1, trans_vals_3d.data()};

    Expect_Libicp_Matrix_Equal(trans_3d, detail::PromoteTranslationTo3D(trans_2d));
    Expect_Libicp_Matrix_Equal(trans_2d, detail::DemoteTranslationTo2D(trans_3d));
}

TEST(LibicpIntegrationTests, PromoteAndDemoteRotations) {
    // clang-format off
    const std::vector<double> rot_vals_2d{
        1.0, 2.0,
        3.0, 4.0
    };
    const std::vector<double> rot_vals_3d{
        1.0, 2.0, 0,
        3.0, 4.0, 0,
        0, 0, 1
    };
    // clang-format on

    const libicp::Matrix rot_2d{2, 2, rot_vals_2d.data()};
    const libicp::Matrix rot_3d{3, 3, rot_vals_3d.data()};

    Expect_Libicp_Matrix_Equal(rot_3d, detail::PromoteRotationTo3D(rot_2d));
    Expect_Libicp_Matrix_Equal(rot_2d, detail::DemoteRotationTo2D(rot_3d));
}

TEST(LibicpIntegrationTests, ComposeTransform) {
    const double yaw = M_PI - 0.1;
    const std::vector<double> translation{1.0, 2.0};

    const auto libicp_rot_mat = detail::DemoteRotationTo2D(libicp::Matrix::rotMatZ(yaw));
    const libicp::Matrix libicp_trans_mat{2, 1, translation.data()};

    const mmg::Transform3d geo_tf =
        mmg::Translation3d{1.0, 2.0, 0} * mmg::TransformFromYPR(yaw, 0, 0);
    const mmg::Transform3d new_geo_tf = detail::ComposeTransform(libicp_rot_mat, libicp_trans_mat);

    mte::ExpectMatrixNear(geo_tf.matrix(), new_geo_tf.matrix(), kSmallEps);
}

TEST(LibicpIntegrationTests, DecomposeTransform) {
    const double yaw = M_PI - 0.1;
    const std::vector<double> translation{1.0, 2.0};

    const auto libicp_rot_mat = detail::DemoteRotationTo2D(libicp::Matrix::rotMatZ(yaw));
    const libicp::Matrix libicp_trans_mat{2, 1, translation.data()};

    const mmg::Transform3d geo_tf =
        mmg::Translation3d{1.0, 2.0, 0} * mmg::TransformFromYPR(yaw, 0, 0);
    const auto [new_rot_mat, new_trans_mat] = detail::DecomposeTransform(geo_tf);

    Expect_Libicp_Matrix_Near(libicp_rot_mat, new_rot_mat, kSmallEps);
    Expect_Libicp_Matrix_Near(libicp_trans_mat, new_trans_mat, kSmallEps);
}

TEST(ICPTests, IdentityTransform) {
    const auto point_cloud = MakeCorner();

    ICPLocalizer localizer(point_cloud, kInlierDist);
    const mmg::Transform3d tf = localizer.Fit(point_cloud, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(mmg::Transform3d::Identity().matrix(), tf.matrix(), kBigEps);
}

TEST(ICPTests, TranslationOnly) {
    const auto model_points = MakeCorner();

    const mmg::Transform3d tf_real{mmg::Translation3d{1, 2, 0}};

    const auto template_points = tf_real * model_points;

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kBigEps);
}

TEST(ICPTests, YawOnly) {
    const auto model_points = MakeCorner();

    const double yaw = k45Degrees;
    const mmg::Transform3d tf_real = mmg::TransformFromYPR(yaw, 0, 0);

    const mte::lidar::PointCloud template_points = tf_real * model_points;

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kBigEps);
}

TEST(ICPTests, YawAndTranslation) {
    const auto model_points = MakeCorner();

    const double yaw = k45Degrees;
    const mmg::Transform3d tf_yaw = mmg::TransformFromYPR(yaw, 0, 0);
    const mmg::Transform3d tf_trans{mmg::Translation3d{1, 2, 0}};
    const mmg::Transform3d tf_real = tf_trans * tf_yaw;

    const auto template_points = tf_real * model_points;

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kBigEps);
}

TEST(ICPTests, DifferentResolution) {
    const auto model_points = MakeCorner(100, 10);

    const double yaw = k45Degrees;
    const mmg::Transform3d tf_yaw = mmg::TransformFromYPR(yaw, 0, 0);
    const mmg::Transform3d tf_trans{mmg::Translation3d{1, 2, 0}};
    const mmg::Transform3d tf_real = tf_trans * tf_yaw;

    const auto template_points = tf_real * MakeCorner(20, 10);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kBigEps);
}

// TODO(jacob): Enable this test once FOV filter is done.
TEST(ICPTests, DISABLED_PortionOfScene) {
    const auto model_points = MakeCorner(100, 10);

    const double yaw = k45Degrees;
    const mmg::Transform3d tf_yaw = mmg::TransformFromYPR(yaw, 0, 0);
    const mmg::Transform3d tf_trans{mmg::Translation3d{1, 2, 0}};
    const mmg::Transform3d tf_real = tf_trans * tf_yaw;

    const auto template_points = tf_real * MakeCorner(20, 8);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    std::cout << tf_real.matrix() << std::endl;
    std::cout << tf_guess.matrix() << std::endl;

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kBigEps);
}
