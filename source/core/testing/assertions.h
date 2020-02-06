#pragma once

#include <gtest/gtest.h>

#include "source/localization/pose.h"

namespace mte {

void Expect_Pose_Eq(const localization::Pose& p1, const localization::Pose& p2) {
    EXPECT_DOUBLE_EQ(p1.x, p2.x);
    EXPECT_DOUBLE_EQ(p1.y, p2.y);
    EXPECT_DOUBLE_EQ(p1.z, p2.z);
    EXPECT_DOUBLE_EQ(p1.v_y, p2.v_y);
    EXPECT_DOUBLE_EQ(p1.v_z, p2.v_z);
    EXPECT_DOUBLE_EQ(p1.v_x, p2.v_x);
    EXPECT_DOUBLE_EQ(p1.roll, p2.roll);
    EXPECT_DOUBLE_EQ(p1.pitch, p2.pitch);
    EXPECT_DOUBLE_EQ(p1.yaw, p2.yaw);
    EXPECT_DOUBLE_EQ(p1.roll_rate, p2.roll_rate);
    EXPECT_DOUBLE_EQ(p1.pitch_rate, p2.pitch_rate);
    EXPECT_DOUBLE_EQ(p1.yaw_rate, p2.yaw_rate);
}

}  // namespace mte
