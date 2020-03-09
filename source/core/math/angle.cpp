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

double DegToRad(double angle_deg) {
    return NormalizeAngle(angle_deg * M_PI / 180.0);
}

double RadToDeg(double angle_rad) {
    return NormalizeAngle(angle_rad) * 180.0 / M_PI;
}

}  // namespace math
}  // namespace mte
