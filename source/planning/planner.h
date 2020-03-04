#pragma once

#include "source/core/bus/serialization.h"
#include "source/localization/pose.h"

namespace mte {
namespace planning {

// TODO(jacob/taylor): Tune these parameters based on controller performance.
constexpr int kPlanHorizonSeconds{5};
constexpr int kControllerFrequencyHz{10};
constexpr int kNumWheelSpeedPoints{kPlanHorizonSeconds * kControllerFrequencyHz};

class WheelSpeedPlan {
   public:
    WheelSpeedPlan(const std::vector<double>& v_l_rpm, const std::vector<double>& v_r_rpm);

    const std::vector<double> v_l_rpm;
    const std::vector<double> v_r_rpm;
};

WheelSpeedPlan MakeConstantSpeedPlan(double speed_left, double speed_right);

}  // namespace planning
}  // namespace mte
