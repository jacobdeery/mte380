#include <eigen/core.hpp>

namespace mte {
namespace localization {

struct Pose {
    Eigen::Vector3d Position(){return {x, y, z}};

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
};

}  // namespace localization
}  // namespace mte
