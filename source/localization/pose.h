#pragma once

#include "source/core/bus/serialization.h"
#include "source/core/geometry.h"
#include "source/core/logging.h"

namespace mte {
namespace localization {

struct Pose {
    Pose() = default;
    Pose(const geometry::Vector3d& position, const geometry::Vector3d& velocity,
         const geometry::Vector3d& orientation, const geometry::Vector3d& angular_velocity);

    geometry::Vector3d Position() const { return {x, y, z}; };

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
