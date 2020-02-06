#pragma once

#include <Eigen/Core>

#include "source/core/bus/serialization.h"
#include "source/core/logging.h"

namespace mte {
namespace localization {

struct Pose {
    Pose() = default;
    Pose(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
         const Eigen::Vector3d& orientation, const Eigen::Vector3d& angular_velocity);

    Eigen::Vector3d Position() { return {x, y, z}; };

    double x = 0;
    double y = 0;
    double z = 0;

    double v_x = 0;
    double v_y = 0;
    double v_z = 0;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    double roll_rate = 0;
    double pitch_rate = 0;
    double yaw_rate = 0;

    MSGPACK_DEFINE(x, y, z, v_x, v_y, v_z, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate);
};

}  // namespace localization
}  // namespace mte
