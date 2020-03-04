#include "source/planning/arduino_bridge.h"
#include "source/planning/planner.h"

#include "gtest/gtest.h"

using namespace mte::planning;

TEST(WheelSpeedPlanTests, IncorrectNumberOfPointsThrows) {
    const std::vector<double> right_num_points(kNumWheelSpeedPoints);
    const std::vector<double> wrong_num_points(kNumWheelSpeedPoints + 1);

    EXPECT_ANY_THROW(WheelSpeedPlan(right_num_points, wrong_num_points));
    EXPECT_ANY_THROW(WheelSpeedPlan(wrong_num_points, right_num_points));
    EXPECT_NO_THROW(WheelSpeedPlan(right_num_points, right_num_points));
}

TEST(SerializedPlanTests, SerializedPlanCreation) {
    std::vector<double> left;
    for (int i = 0; i < kNumWheelSpeedPoints; ++i) {
        left.push_back(i);
    }
    std::vector<double> right;
    for (int i = 0; i < kNumWheelSpeedPoints; ++i) {
        right.push_back(kNumWheelSpeedPoints + i);
    }
    const WheelSpeedPlan wsp{left, right};

    const mte::SerializedPlan sp{wsp};

    for (int i = 0; i < kNumWheelSpeedPoints; ++i) {
        float lval;
        memcpy(&lval, sp.data + i * sizeof(float), sizeof(float));
        EXPECT_EQ(static_cast<float>(left[i]), lval);

        float rval;
        memcpy(&rval, sp.data + (i + kNumWheelSpeedPoints) * sizeof(float), sizeof(float));
        EXPECT_EQ(static_cast<float>(right[i]), rval);
    }
}
