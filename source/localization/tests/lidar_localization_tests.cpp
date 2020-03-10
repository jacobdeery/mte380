#include "source/core/testing/assertions.h"
#include "source/localization/icp.h"

#include "gtest/gtest.h"

using namespace mte::localization;
namespace mmg = mte::math::geometry;

constexpr double kSmallEps{1e-9};
constexpr double kBigEps{1e-3};
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

PointCloud MakeCorner(int points_per_wall = 100, double wall_length = 10) {
    double resolution = wall_length / points_per_wall;
    PointCloud points;
    for (int i = 0; i < points_per_wall; ++i) {
        points.emplace_back(i * resolution, 0, 0);
    }
    for (int i = 1; i < points_per_wall; ++i) {
        points.emplace_back(wall_length, i * resolution, 0);
    }
    return points;
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

TEST(LibicpIntegrationTests, LibicpMatrixMultipleRotations) {
    const double yaw = M_PI - 0.1;
    const double pitch = M_PI_2 - 0.1;
    const double roll = M_PI - 0.1;

    const auto libicp_mat = libicp::Matrix::rotMatX(roll) * libicp::Matrix::rotMatY(pitch) *
                            libicp::Matrix::rotMatZ(yaw);
    const mmg::Matrix3d geo_mat{mmg::TransformFromYPR(yaw, pitch, roll).rotation()};

    mte::ExpectMatrixNear(geo_mat, mmg::Matrix3d{detail::FromLibicpMatrix(libicp_mat)}, kSmallEps);
}

TEST(LibicpIntegrationTests, ComposeTransform) {
    const double yaw = M_PI - 0.1;
    const double pitch = M_PI_2 - 0.1;
    const double roll = M_PI - 0.1;
    const std::vector<double> translation{1.0, 2.0, 3.0};

    const auto libicp_rot_mat = libicp::Matrix::rotMatX(roll) * libicp::Matrix::rotMatY(pitch) *
                                libicp::Matrix::rotMatZ(yaw);
    const libicp::Matrix libicp_trans_mat{3, 1, translation.data()};

    const mmg::Transform3d geo_tf =
        mmg::Translation3d{1.0, 2.0, 3.0} * mmg::TransformFromYPR(yaw, pitch, roll);
    const mmg::Transform3d new_geo_tf = detail::ComposeTransform(libicp_rot_mat, libicp_trans_mat);

    mte::ExpectMatrixNear(geo_tf.matrix(), new_geo_tf.matrix(), kSmallEps);
}

TEST(LibicpIntegrationTests, DecomposeTransform) {
    const double yaw = M_PI - 0.1;
    const double pitch = M_PI_2 - 0.1;
    const double roll = M_PI - 0.1;
    const std::vector<double> translation{1.0, 2.0, 3.0};

    const auto libicp_rot_mat = libicp::Matrix::rotMatX(roll) * libicp::Matrix::rotMatY(pitch) *
                                libicp::Matrix::rotMatZ(yaw);
    const libicp::Matrix libicp_trans_mat{3, 1, translation.data()};

    const mmg::Transform3d geo_tf =
        mmg::Translation3d{1.0, 2.0, 3.0} * mmg::TransformFromYPR(yaw, pitch, roll);
    const auto [new_rot_mat, new_trans_mat] = detail::DecomposeTransform(geo_tf);

    Expect_Libicp_Matrix_Near(libicp_rot_mat, new_rot_mat, kSmallEps);
    Expect_Libicp_Matrix_Near(libicp_trans_mat, new_trans_mat, kSmallEps);
}

TEST(ICPTests, IdentityTransform) {
    const auto point_cloud = MakeCorner();

    ICPLocalizer localizer(point_cloud, kInlierDist);
    const mmg::Transform3d tf = localizer.Fit(point_cloud, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(mmg::Transform3d::Identity().matrix(), tf.matrix(), kSmallEps);
}

TEST(ICPTests, TranslationOnly) {
    const auto model_points = MakeCorner();

    const mmg::Transform3d tf_real{mmg::Translation3d{1, 1, 1}};

    const auto template_points = TransformPoints(model_points, tf_real);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    std::cout << tf_real.matrix() << std::endl;
    std::cout << tf_guess.matrix() << std::endl;

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kSmallEps);
}

TEST(ICPTests, YawOnly) {
    const auto model_points = MakeCorner();

    const double yaw = M_PI_2;
    const mmg::Transform3d tf_real = mmg::TransformFromYPR(yaw, 0, 0);

    const PointCloud template_points = TransformPoints(model_points, tf_real);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    std::cout << tf_real.matrix() << std::endl;
    std::cout << tf_guess.matrix() << std::endl;

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kSmallEps);
}

TEST(ICPTests, YawPitchRoll) {
    const auto model_points = MakeCorner();

    const double yaw = M_PI_2;
    const double pitch = M_PI / 3.0;
    const double roll = -M_PI_2;
    const mmg::Transform3d tf_real = mmg::TransformFromYPR(yaw, pitch, roll);

    const PointCloud template_points = TransformPoints(model_points, tf_real);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kSmallEps);
}

TEST(ICPTests, TranslationAndRotation) {
    const auto model_points = MakeCorner();

    const double yaw = M_PI_2;
    const double pitch = M_PI / 3.0;
    const double roll = -M_PI_2;
    const mmg::Transform3d tf_real =
        mmg::Translation3d{4, 5, 6} * mmg::TransformFromYPR(yaw, pitch, roll);

    const PointCloud template_points = TransformPoints(model_points, tf_real);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kSmallEps);
}

TEST(ICPTests, DifferentNumberOfPoints) {
    const auto model_points = MakeCorner(20, 5);

    const double yaw = M_PI_2;
    const double pitch = M_PI / 3.0;
    const double roll = -M_PI_2;
    const mmg::Transform3d tf_real =
        mmg::Translation3d{4, 5, 6} * mmg::TransformFromYPR(yaw, pitch, roll);

    const PointCloud untransformed_template_points = MakeCorner(50, 5);
    const PointCloud template_points = TransformPoints(untransformed_template_points, tf_real);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kSmallEps);
}

TEST(ICPTests, SingleOutlier) {
    const auto model_points = MakeCorner();

    const double yaw = M_PI_2;
    const double pitch = M_PI / 3.0;
    const double roll = -M_PI_2;
    const mmg::Transform3d tf_real =
        mmg::Translation3d{4, 5, 6} * mmg::TransformFromYPR(yaw, pitch, roll);

    PointCloud untransformed_template_points = model_points;
    untransformed_template_points.emplace_back(50, 50, 50);
    const PointCloud template_points = TransformPoints(untransformed_template_points, tf_real);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kSmallEps);
}

TEST(ICPTests, SeveralOutliers) {
    const auto model_points = MakeCorner();

    const double yaw = M_PI_2;
    const double pitch = M_PI / 3.0;
    const double roll = -M_PI_2;
    const mmg::Transform3d tf_real =
        mmg::Translation3d{4, 5, 6} * mmg::TransformFromYPR(yaw, pitch, roll);

    PointCloud untransformed_template_points = model_points;
    const PointCloud outliers{{30, 0, 0}, {-30, 0, 0}, {0, 25, 0}, {1, 1, 20}};
    untransformed_template_points.insert(untransformed_template_points.end(), outliers.begin(),
                                         outliers.end());
    const PointCloud template_points = TransformPoints(untransformed_template_points, tf_real);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kSmallEps);
}

TEST(ICPTests, SubsetOfModelPointsObserved) {
    // clang-format off
    const PointCloud model_points{
        {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0},  // bottom wall
        {5, 1, 0}, {5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {5, 5, 0},             // right wall
        {4, 5, 0}, {3, 5, 0}, {2, 5, 0}, {1, 5, 0}, {0, 5, 0},             // top wall
        {0, 4, 0}, {0, 3, 0}, {0, 2, 0}, {0, 1, 0}                         // left wall
    };
    // clang-format on

    const double yaw = M_PI_2;
    const double pitch = M_PI / 3.0;
    const double roll = -M_PI_2;
    const mmg::Transform3d tf_real =
        mmg::Translation3d{4, 5, 6} * mmg::TransformFromYPR(yaw, pitch, roll);

    // clang-format off
    const PointCloud untransformed_template_points{
        {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0},  // bottom wall
        {5, 1, 0}, {5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {5, 5, 0}              // right wall
    };
    // clang-format on
    const PointCloud template_points = TransformPoints(untransformed_template_points, tf_real);

    ICPLocalizer localizer(model_points, kInlierDist);
    const mmg::Transform3d tf_guess = localizer.Fit(template_points, mmg::Transform3d::Identity());

    mte::ExpectMatrixNear(tf_real.matrix(), tf_guess.matrix(), kSmallEps);
}
