#include "pose.h"

namespace mte {
namespace localization {

Pose::Pose(const math::geometry::Vector3d& position, const math::geometry::Vector3d& velocity,
           const math::geometry::Vector3d& orientation,
           const math::geometry::Vector3d& angular_velocity) {
    x = position[0];
    y = position[1];
    z = position[2];

    v_x = velocity[0];
    v_y = velocity[1];
    v_z = velocity[2];

    yaw = orientation[0];
    pitch = orientation[1];
    roll = orientation[2];

    yaw_rate = angular_velocity[0];
    pitch_rate = angular_velocity[1];
    roll_rate = angular_velocity[2];
}

Pose::Pose(const math::geometry::Transform3d& tf) {
    const math::geometry::Vector3d trans{tf.translation()};

    x = trans(0);
    y = trans(1);
    z = trans(2);

    std::tie(yaw, pitch, roll) = math::geometry::YPRFromTransform(tf);
}

math::geometry::Transform3d ExtractTransform(const Pose& pose) {
    const math::geometry::Transform3d tf_trans{
        math::geometry::Translation3d{pose.x, pose.y, pose.z}};
    const math::geometry::Transform3d tf_rot =
        math::geometry::TransformFromYPR(pose.yaw, pose.pitch, pose.roll);
    return tf_trans * tf_rot;
}

}  // namespace localization
}  // namespace mte
