#include "calibration.h"

namespace mte {
namespace calibration {

extern const math::geometry::Transform3d T_robot_lidar =
    math::geometry::Translation3d{0, 0, 0} * math::geometry::TransformFromYPR(M_PI, 0, 0);

extern const localization::Pose initial_pose = localization::Pose{
    {0.9, 1.6, 0},   // x, y, z
    {0, 0, 0},       // v_x, v_y, v_z
    {M_PI_2, 0, 0},  // yaw, pitch, roll
    {0, 0, 0}        // d_yaw, d_pitch, d_roll
};

}  // namespace calibration
}  // namespace mte
