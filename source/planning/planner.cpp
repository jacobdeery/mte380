#include "planner.h"

#include "source/core/logging.h"

namespace mte {
namespace planning {

WheelSpeedPlan::WheelSpeedPlan(const std::vector<double>& v_l_rpm,
                               const std::vector<double>& v_r_rpm)
    : v_l_rpm{v_l_rpm}, v_r_rpm{v_r_rpm} {
    CHECK(v_l_rpm.size() == kNumWheelSpeedPoints,
          "Left wheel speed plan must have " + std::to_string(kNumWheelSpeedPoints) +
              " points (got" + std::to_string(v_l_rpm.size()) + ")");
    CHECK(v_r_rpm.size() == kNumWheelSpeedPoints,
          "Right wheel speed plan must have " + std::to_string(kNumWheelSpeedPoints) +
              " points (got" + std::to_string(v_r_rpm.size()) + ")");
}

WheelSpeedPlan MakeConstantSpeedPlan(double speed_left, double speed_right) {
    const std::vector<double> speeds_left(kNumWheelSpeedPoints, speed_left);
    const std::vector<double> speeds_right(kNumWheelSpeedPoints, speed_right);
    return WheelSpeedPlan(speeds_left, speeds_right);
}

}  // namespace planning
}  // namespace mte
