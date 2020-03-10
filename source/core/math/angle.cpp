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

bool IsInAngularBounds(double angle, double min_angle, double max_angle) {
    const double angle_normalized = math::NormalizeAngle(angle);
    const double min_normalized = math::NormalizeAngle(min_angle);
    const double max_normalized = math::NormalizeAngle(max_angle);

    if (min_normalized < max_normalized) {
        return min_normalized <= angle_normalized && angle_normalized <= max_normalized;
    } else {
        return min_normalized <= angle_normalized || angle_normalized <= max_normalized;
    }
}

}  // namespace math
}  // namespace mte
