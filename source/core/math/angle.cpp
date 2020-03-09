#include <cmath>

namespace mte {
namespace math {

double NormalizeAngle(double angle) {
    double bounded_angle = fmod(angle, (2 * M_PI));
    if (bounded_angle < -M_PI) {
        return bounded_angle + (2 * M_PI);
    } else if (bounded_angle > M_PI) {
        return bounded_angle - (2 * M_PI);
    }
    return bounded_angle;
}

}  // namespace math
}  // namespace mte
