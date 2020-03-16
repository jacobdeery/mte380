#include "source/core/calibration.h"
#include "source/core/math/angle.h"
#include "source/core/testing/assertions.h"
#include "source/localization/localizer.h"
#include "source/localization/pose.h"
#include "source/sensing/lidar/lidar_types.h"

#include "gtest/gtest.h"

using namespace mte::localization;
using namespace mte::calibration;
namespace mmg = mte::math::geometry;

constexpr double kHalfMillimetre{0.5e-3};
constexpr double kHalfDegree{mte::math::DegToRad(0.5)};

constexpr double kSmallEps{1e-9};
constexpr double kBigEps{1e-6};
constexpr double k45Degrees{mte::math::DegToRad(45)};

void ExpectNear(const Pose& p1, const Pose& p2, double eps_pos, double eps_ori) {
    EXPECT_NEAR(p1.x, p2.x, eps_pos);
    EXPECT_NEAR(p1.y, p2.y, eps_pos);
    EXPECT_NEAR(p1.z, p2.z, eps_pos);

    EXPECT_NEAR(p1.yaw, p2.yaw, eps_ori);
    EXPECT_NEAR(p1.pitch, p2.pitch, eps_ori);
    EXPECT_NEAR(p1.roll, p2.roll, eps_ori);
}

Pose MakePose(double x, double y, double yaw) {
    const mmg::Vector3d pos{x, y, 0};
    const mmg::Vector3d vel{0, 0, 0};
    const mmg::Vector3d ori{yaw, 0, 0};
    const mmg::Vector3d ang{0, 0, 0};

    return Pose{pos, vel, ori, ang};
}

TEST(HasPoseConvergedTests, SamePose) {
    const auto p = MakePose(1, 1, 1);

    EXPECT_TRUE(detail::HasPoseConverged(p, p, 0, 0));
}

TEST(HasPoseConvergedTests, TranslationOnly) {
    const auto p1 = MakePose(1, 1, 1);
    const auto p2 = MakePose(2, 2, 1);

    EXPECT_FALSE(detail::HasPoseConverged(p1, p2, sqrt(2) - 0.01, 0));
    EXPECT_TRUE(detail::HasPoseConverged(p1, p2, sqrt(2) + 0.01, 0));
}

TEST(HasPoseConvergedTests, YawOnly) {
    const auto p1 = MakePose(1, 1, 1);
    const auto p2 = MakePose(1, 1, 1.1);

    EXPECT_FALSE(detail::HasPoseConverged(p1, p2, 0, 0.099));
    EXPECT_TRUE(detail::HasPoseConverged(p1, p2, 0, 0.101));
}

TEST(HasPoseConvergedTests, TranslationAndYaw) {
    const auto p1 = MakePose(1, 1, 1);
    const auto p2 = MakePose(2, 2, 1.1);

    EXPECT_FALSE(detail::HasPoseConverged(p1, p2, sqrt(2) + 0.01, 0));
    EXPECT_FALSE(detail::HasPoseConverged(p1, p2, 0, 0.101));

    EXPECT_TRUE(detail::HasPoseConverged(p1, p2, sqrt(2) + 0.01, 0.101));
}

TEST(LocalizerTests, LocalizeWithExactPose) {
    const auto p = MakePose(kWallXMax - 0.3, kWallYMax - 0.3, k45Degrees);
    const auto pc = ExtractTransform(p).inverse() * GetVisibleWallPoints(p);

    const auto localized_p = Localize(pc, p);

    ExpectNear(p, localized_p, kSmallEps, kSmallEps);
}

TEST(LocalizerTests, LocalizeWithSlightlyTranslatedPose) {
    const auto actual_p = MakePose(kWallXMax - 0.3, kWallYMax - 0.3, k45Degrees);
    const auto last_p = MakePose(kWallXMax - 0.4, kWallYMax - 0.4, k45Degrees);

    const auto pc = ExtractTransform(actual_p).inverse() * GetVisibleWallPoints(actual_p);

    const auto localized_p = Localize(pc, last_p);

    ExpectNear(actual_p, localized_p, kHalfMillimetre, kHalfDegree);
}

TEST(LocalizerTests, LocalizeWithSlightlyRotatedPose) {
    const auto actual_p = MakePose(kWallXMax - 0.3, kWallYMax - 0.3, k45Degrees);
    const auto last_p =
        MakePose(kWallXMax - 0.3, kWallYMax - 0.3, k45Degrees + mte::math::DegToRad(1));

    const auto pc = ExtractTransform(actual_p).inverse() * GetVisibleWallPoints(actual_p);

    const auto localized_p = Localize(pc, last_p);

    ExpectNear(actual_p, localized_p, kHalfMillimetre, kHalfDegree);
}

TEST(LocalizerTests, LocalizeWithSignificantlyTranslatedPose) {
    const auto actual_p = MakePose(kWallXMax - 0.3, kWallYMax - 0.3, k45Degrees);
    const auto last_p = MakePose(kWallXMax - 0.6, kWallYMax - 0.6, k45Degrees);

    const auto pc = ExtractTransform(actual_p).inverse() * GetVisibleWallPoints(actual_p);

    const auto localized_p = Localize(pc, last_p);

    ExpectNear(actual_p, localized_p, kHalfMillimetre, kHalfDegree);
}

TEST(LocalizerTests, LocalizeWithSignificantlyRotatedPose) {
    const auto actual_p = MakePose(kWallXMax - 0.3, kWallYMax - 0.3, k45Degrees);
    const auto last_p =
        MakePose(kWallXMax - 0.3, kWallYMax - 0.3, k45Degrees + mte::math::DegToRad(5));

    const auto pc = ExtractTransform(actual_p).inverse() * GetVisibleWallPoints(actual_p);

    const auto localized_p = Localize(pc, last_p);

    ExpectNear(actual_p, localized_p, kHalfMillimetre, kHalfDegree);
}

TEST(LocalizerTests, LocalizeWithTranslatedAndRotatedPose) {
    const auto actual_p = MakePose(kWallXMax - 0.3, kWallYMax - 0.3, k45Degrees);
    const auto last_p =
        MakePose(kWallXMax - 0.4, kWallYMax - 0.4, k45Degrees + mte::math::DegToRad(5));

    const auto pc = ExtractTransform(actual_p).inverse() * GetVisibleWallPoints(actual_p);

    const auto localized_p = Localize(pc, last_p);

    ExpectNear(actual_p, localized_p, kHalfMillimetre, kHalfDegree);
}
