#include "geometry.h"

#include "source/core/logging.h"

#include <cmath>

namespace mte {
namespace math {
namespace geometry {

constexpr double kAngleEps{1.7453293e-4};  // 0.01 degrees

Transform3d TransformFromYPR(double yaw, double pitch, double roll) {
    CHECK(-M_PI <= yaw && yaw <= M_PI,
          "Yaw value " + std::to_string(yaw) + " is outside of the range [-pi, pi]");
    CHECK(-M_PI_2 <= pitch && pitch <= M_PI_2,
          "Pitch value " + std::to_string(pitch) + " is outside of the range [-pi/2, pi/2]");
    CHECK(-M_PI <= roll && roll <= M_PI,
          "Roll value " + std::to_string(roll) + " is outside of the range [-pi, pi]");
    return Transform3d{Eigen::AngleAxisd(roll, Vector3d::UnitX()) *
                       Eigen::AngleAxisd(pitch, Vector3d::UnitY()) *
                       Eigen::AngleAxisd(yaw, Vector3d::UnitZ())};
}

std::tuple<double, double, double> YPRFromTransform(const Transform3d& tf) {
    const Eigen::Matrix3d rot = tf.rotation();
    const double pitch = asin(rot(0, 2));
    if (pitch >= M_PI_2 - kAngleEps) {
        const double roll = atan2(rot(2, 1), rot(1, 1));
        return std::tuple(0, pitch, roll);
    } else if (pitch <= -M_PI_2 + kAngleEps) {
        const double roll = -atan2(rot(1, 0), rot(1, 1));
        return std::tuple(0, pitch, roll);
    } else {
        const double yaw = atan2(-rot(0, 1), rot(0, 0));
        const double roll = atan2(-rot(1, 2), rot(2, 2));
        return std::tuple(yaw, pitch, roll);
    }
}

}  // namespace geometry
}  // namespace math
}  // namespace mte
