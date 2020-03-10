#pragma once

#include <cmath>

namespace mte {
namespace math {

double NormalizeAngle(double angle);

constexpr double DegToRad(double angle_deg) {
    return angle_deg * M_PI / 180.0;
}

constexpr double RadToDeg(double angle_rad) {
    return angle_rad * 180.0 / M_PI;
}

bool IsInAngularBounds(double angle, double min_angle, double max_angle);

}  // namespace math
}  // namespace mte
