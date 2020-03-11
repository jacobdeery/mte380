#pragma once

#include "source/core/bus/serialization.h"
#include "source/core/logging.h"
#include "source/core/math/geometry.h"

namespace mte {
namespace localization {

struct Pose {
    Pose() = default;
    Pose(const math::geometry::Vector3d& position, const math::geometry::Vector3d& velocity,
         const math::geometry::Vector3d& orientation,
         const math::geometry::Vector3d& angular_velocity);
    Pose(const math::geometry::Transform3d& tf);

    math::geometry::Vector3d Position() const { return {x, y, z}; };

    double x = 0;
    double y = 0;
    double z = 0;

    double v_x = 0;
    double v_y = 0;
    double v_z = 0;

    double yaw = 0;
    double pitch = 0;
    double roll = 0;

    double yaw_rate = 0;
    double pitch_rate = 0;
    double roll_rate = 0;

    MSGPACK_DEFINE(x, y, z, v_x, v_y, v_z, yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate);
};

math::geometry::Transform3d ExtractTransform(const Pose& pose);

}  // namespace localization
}  // namespace mte
