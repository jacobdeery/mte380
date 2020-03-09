#include "source/core/testing/assertions.h"
#include "source/localization/pose.h"

#include "gtest/gtest.h"

using namespace mte;

TEST(PoseTests, PoseFromVector3ds) {
    const math::geometry::Vector3d pos{1, 2, 3};
    const math::geometry::Vector3d vel{4, 5, 6};
    const math::geometry::Vector3d ori{7, 8, 9};
    const math::geometry::Vector3d ang{10, 11, 12};

    const localization::Pose p{pos, vel, ori, ang};

    EXPECT_DOUBLE_EQ(pos[0], p.x);
    EXPECT_DOUBLE_EQ(pos[1], p.y);
    EXPECT_DOUBLE_EQ(pos[2], p.z);

    EXPECT_DOUBLE_EQ(vel[1], p.v_y);
    EXPECT_DOUBLE_EQ(vel[2], p.v_z);
    EXPECT_DOUBLE_EQ(vel[0], p.v_x);

    EXPECT_DOUBLE_EQ(ori[0], p.roll);
    EXPECT_DOUBLE_EQ(ori[1], p.pitch);
    EXPECT_DOUBLE_EQ(ori[2], p.yaw);

    EXPECT_DOUBLE_EQ(ang[0], p.roll_rate);
    EXPECT_DOUBLE_EQ(ang[1], p.pitch_rate);
    EXPECT_DOUBLE_EQ(ang[2], p.yaw_rate);
}

TEST(PoseTests, PoseRoundTrip) {
    const math::geometry::Vector3d pos{1, 2, 3};
    const math::geometry::Vector3d vel{4, 5, 6};
    const math::geometry::Vector3d ori{7, 8, 9};
    const math::geometry::Vector3d ang{10, 11, 12};

    const localization::Pose p{pos, vel, ori, ang};

    const auto p_ser = bus::Serialize(p);
    const auto p_deser = bus::Deserialize<localization::Pose>(p_ser);

    Expect_Pose_Eq(p, p_deser);
}
