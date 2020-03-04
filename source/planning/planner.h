#pragma once

#include "source/core/bus/serialization.h"
#include "source/localization/pose.h"

namespace mte {
namespace planning {

constexpr int kNumWheelSpeedPoints{50};

class WheelSpeedPlan {
   public:
    WheelSpeedPlan(const std::vector<double>& v_l_rpm, const std::vector<double>& v_r_rpm);

    const std::vector<double> v_l_rpm;
    const std::vector<double> v_r_rpm;
};

}  // namespace planning
}  // namespace mte
