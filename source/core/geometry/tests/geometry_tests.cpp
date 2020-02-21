#include "source/core/geometry/geometry.h"
#include "source/core/testing/assertions.h"

#include <gtest/gtest.h>

#include <cmath>

using namespace mte::geometry;

constexpr double kEps{1e-9};
constexpr double kAngleEps{1.7453293e-4};  // 0.01 degrees

TEST(TransformTests, IdentityTransform) {
    const Point3d p1{1.0, 2.0, 3.0};
    const Transform3d id = Transform3d::Identity();

    mte::Expect_Matrix_Equal(p1, id * p1);
}

TEST(TransformTests, PureTranslation) {
    const Point3d p1{1.0, 2.0, 3.0};
    const Point3d transformed_p1{2.0, 3.0, 4.0};

    const Transform3d translation{Translation3d{1.0, 1.0, 1.0}};

    mte::Expect_Matrix_Near(transformed_p1, translation * p1, kEps);
}

TEST(TransformTests, ComposeTranslations) {
    const Point3d p1{1.0, 2.0, 3.0};
    const Point3d transformed_p1{4.0, 6.0, 8.0};

    const Transform3d translation_1{Translation3d{1.0, 1.0, 1.0}};
    const Transform3d translation_2{Translation3d{2.0, 3.0, 4.0}};

    const Transform3d composed = translation_1 * translation_2;

    mte::Expect_Matrix_Near(transformed_p1, composed * p1, kEps);
}

TEST(TransformTests, InvertTranslation) {
    const Point3d p1{1.0, 2.0, 3.0};

    const Transform3d translation{Translation3d{1.0, 1.0, 1.0}};
    const Transform3d inv_translation = translation.inverse();

    mte::Expect_Matrix_Near(p1, inv_translation * translation * p1, kEps);
}

TEST(TransformTests, RotationInOneAxis) {
    const Point3d p1{1.0, 2.0, 3.0};

    const Transform3d yaw_only = TransformFromYPR(M_PI_2, 0, 0);
    const Point3d yawed_p1{-2.0, 1.0, 3.0};

    const Transform3d pitch_only = TransformFromYPR(0, M_PI_2, 0);
    const Point3d pitched_p1{3.0, 2.0, -1.0};

    const Transform3d roll_only = TransformFromYPR(0, 0, M_PI_2);
    const Point3d rolled_p1{1.0, -3.0, 2.0};

    mte::Expect_Matrix_Near(yawed_p1, yaw_only * p1, kEps);
    mte::Expect_Matrix_Near(pitched_p1, pitch_only * p1, kEps);
    mte::Expect_Matrix_Near(rolled_p1, roll_only * p1, kEps);
}

TEST(TransformTests, RotationInMultipleAxes) {
    const Point3d p1{1.0, 2.0, 3.0};

    const Transform3d ypr = TransformFromYPR(M_PI_4, M_PI / 3.0, M_PI_2);

    const double final_x = 3.0 * sqrt(3.0) / 2.0 - sqrt(2) / 4.0;
    const double final_y = -sqrt(6.0) / 4.0 - 3.0 / 2.0;
    const double final_z = 3.0 * sqrt(2.0) / 2.0;
    const Point3d transformed_p1{final_x, final_y, final_z};

    mte::Expect_Matrix_Near(transformed_p1, ypr * p1, kEps);
}

TEST(TransformTests, InvalidAngleThrows) {
    EXPECT_ANY_THROW(TransformFromYPR(M_PI + kEps, 0, 0));
    EXPECT_ANY_THROW(TransformFromYPR(-M_PI - kEps, 0, 0));
    EXPECT_ANY_THROW(TransformFromYPR(0, M_PI_2 + kEps, 0));
    EXPECT_ANY_THROW(TransformFromYPR(0, -M_PI_2 - kEps, 0));
    EXPECT_ANY_THROW(TransformFromYPR(0, 0, M_PI + kEps));
    EXPECT_ANY_THROW(TransformFromYPR(0, 0, -M_PI - kEps));
}

TEST(TransformTests, RotationMatrixEquivalency) {
    const double yaw = M_PI - 0.1;
    const double pitch = M_PI_2 - 0.1;
    const double roll = M_PI / 3.0;

    const double cy = cos(yaw);
    const double cp = cos(pitch);
    const double cr = cos(roll);
    const double sy = sin(yaw);
    const double sp = sin(pitch);
    const double sr = sin(roll);

    const Transform3d ypr = TransformFromYPR(yaw, pitch, roll);

    const Transform3d tf_yaw{Eigen::AngleAxisd(yaw, Vector3d::UnitZ())};
    const Transform3d tf_pitch{Eigen::AngleAxisd(pitch, Vector3d::UnitY())};
    const Transform3d tf_roll{Eigen::AngleAxisd(roll, Vector3d::UnitX())};

    // clang-format off
    const Eigen::Matrix4d rotation_mat{
        { cp * cy,                 -sy * cp,                sp,        0},
        { sr * sp * cy + sy * cr,  cy * cr - sy * sr * sp,  -sr * cp,  0},
        { sy * sr - sp * cr * cy,  sy * sp * cr + cy * sr,  cr * cp,   0},
        { 0,                       0,                       0,         1}
    };
    // clang-format on

    mte::Expect_Matrix_Near(ypr.matrix(), (tf_roll * tf_pitch * tf_yaw).matrix(), kEps);
    mte::Expect_Matrix_Near(rotation_mat, ypr.matrix(), kEps);
}

TEST(TransformTests, RotationPlusTranslation) {
    const Point3d p1{1.0, 2.0, 3.0};

    const Transform3d translation{Translation3d{1.0, 2.0, 3.0}};
    const Transform3d rotation{TransformFromYPR(M_PI_4, M_PI / 3.0, M_PI_2)};
    const Transform3d tf = translation * rotation;

    const double final_x = 3.0 * sqrt(3.0) / 2.0 - sqrt(2) / 4.0 + 1.0;
    const double final_y = -sqrt(6.0) / 4.0 - 3.0 / 2.0 + 2.0;
    const double final_z = 3.0 * sqrt(2.0) / 2.0 + 3.0;
    const Point3d transformed_p1{final_x, final_y, final_z};

    mte::Expect_Matrix_Equal(translation.translation().matrix(), tf.translation().matrix());
    EXPECT_EQ(rotation.rotation().matrix(), tf.rotation().matrix());
    mte::Expect_Matrix_Near(transformed_p1, tf * p1, kEps);
}

TEST(TransformTests, YPRRoundTrip) {
    {
        const double yaw = M_PI - 0.1;
        const double pitch = M_PI_2 - 0.1;
        const double roll = M_PI - 0.1;

        const Transform3d tf = TransformFromYPR(yaw, pitch, roll);
        const auto [yaw2, pitch2, roll2] = YPRFromTransform(tf);

        EXPECT_NEAR(yaw, yaw2, kAngleEps);
        EXPECT_NEAR(pitch, pitch2, kAngleEps);
        EXPECT_NEAR(roll, roll2, kAngleEps);
    }
    {
        const double yaw = -M_PI + 0.1;
        const double pitch = -M_PI_2 + 0.1;
        const double roll = -M_PI + 0.1;

        const Transform3d tf = TransformFromYPR(yaw, pitch, roll);
        const auto [yaw2, pitch2, roll2] = YPRFromTransform(tf);

        EXPECT_NEAR(yaw, yaw2, kAngleEps);
        EXPECT_NEAR(pitch, pitch2, kAngleEps);
        EXPECT_NEAR(roll, roll2, kAngleEps);
    }
}

TEST(TransformTests, YPRRoundTripGimbalLock) {
    const double yaw = 1.0;
    const double roll = M_PI / 3.0;
    {
        const double pitch = M_PI_2;
        const Transform3d tf = TransformFromYPR(yaw, pitch, roll);
        const auto [yaw2, pitch2, roll2] = YPRFromTransform(tf);

        EXPECT_NEAR(0, yaw2, kAngleEps);
        EXPECT_NEAR(pitch, pitch2, kAngleEps);
        EXPECT_NEAR(roll + yaw, roll2, kAngleEps);
    }
    {
        const double pitch = -M_PI_2;
        const Transform3d tf = TransformFromYPR(yaw, pitch, roll);
        const auto [yaw2, pitch2, roll2] = YPRFromTransform(tf);

        EXPECT_NEAR(0, yaw2, kAngleEps);
        EXPECT_NEAR(pitch, pitch2, kAngleEps);
        EXPECT_NEAR(roll - yaw, roll2, kAngleEps);
    }
}
