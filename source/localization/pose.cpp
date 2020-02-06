#include "pose.h"

namespace mte {
namespace localization {

Pose::Pose(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
           const Eigen::Vector3d& orientation, const Eigen::Vector3d& angular_velocity) {
    x = position[0];
    y = position[1];
    z = position[2];

    v_x = velocity[0];
    v_y = velocity[1];
    v_z = velocity[2];

    roll = orientation[0];
    pitch = orientation[1];
    yaw = orientation[2];

    roll_rate = angular_velocity[0];
    pitch_rate = angular_velocity[1];
    yaw_rate = angular_velocity[2];
}

}  // namespace localization
}  // namespace mte
