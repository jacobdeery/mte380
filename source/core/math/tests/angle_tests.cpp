#include "source/core/math/angle.h"
#include "source/core/testing/assertions.h"

#include <gtest/gtest.h>

#include <cmath>

using namespace mte::math;

TEST(AngleTests, NormalizeAngleTests) {
    EXPECT_DOUBLE_EQ(0, NormalizeAngle(0));
    EXPECT_DOUBLE_EQ(M_PI, NormalizeAngle(M_PI));

    EXPECT_DOUBLE_EQ(0, NormalizeAngle(2 * M_PI));
    EXPECT_DOUBLE_EQ(0, NormalizeAngle(-2 * M_PI));

    EXPECT_DOUBLE_EQ(-M_PI_2, NormalizeAngle(M_PI + M_PI_2));
    EXPECT_DOUBLE_EQ(M_PI_2, NormalizeAngle(-(M_PI + M_PI_2)));

    EXPECT_DOUBLE_EQ(M_PI_2, NormalizeAngle(2 * M_PI + M_PI_2));
    EXPECT_DOUBLE_EQ(-M_PI_2, NormalizeAngle(-(2 * M_PI + M_PI_2)));

    EXPECT_DOUBLE_EQ(-M_PI_2, NormalizeAngle(3 * M_PI + M_PI_2));
    EXPECT_DOUBLE_EQ(M_PI_2, NormalizeAngle(-(3 * M_PI + M_PI_2)));
}

TEST(AngleTests, RadToDegTests) {
    EXPECT_DOUBLE_EQ(0, RadToDeg(0));
    EXPECT_DOUBLE_EQ(45, RadToDeg(M_PI_4));
    EXPECT_DOUBLE_EQ(90, RadToDeg(M_PI_2));
    EXPECT_DOUBLE_EQ(180, RadToDeg(M_PI));

    EXPECT_DOUBLE_EQ(-45, RadToDeg(-M_PI_4));
    EXPECT_DOUBLE_EQ(-90, RadToDeg(-M_PI_2));
    EXPECT_DOUBLE_EQ(-180, RadToDeg(-M_PI));
}

TEST(AngleTests, DegToRadTests) {
    EXPECT_DOUBLE_EQ(0, DegToRad(0));
    EXPECT_DOUBLE_EQ(M_PI_4, DegToRad(45));
    EXPECT_DOUBLE_EQ(M_PI_2, DegToRad(90));
    EXPECT_DOUBLE_EQ(M_PI, DegToRad(180));

    EXPECT_DOUBLE_EQ(-M_PI_4, DegToRad(-45));
    EXPECT_DOUBLE_EQ(-M_PI_2, DegToRad(-90));
    EXPECT_DOUBLE_EQ(-M_PI, DegToRad(-180));
}
