#pragma once

#include "source/core/bus/serialization.h"
#include "source/localization/pose.h"

namespace mte {
namespace planning {

class WheelSpeedPlan {
   public:
    WheelSpeedPlan() = default;
    WheelSpeedPlan(const std::vector<double> v_l_rpm, const std::vector<double> v_r_rpm)
        : v_l_rpm{v_l_rpm}, v_r_rpm{v_r_rpm} {}

   private:
    std::vector<double> v_l_rpm;
    std::vector<double> v_r_rpm;

   public:
    MSGPACK_DEFINE(v_l_rpm, v_r_rpm);
};

}  // namespace planning
}  // namespace mte
