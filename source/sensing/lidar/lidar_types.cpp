#include "lidar_types.h"

#include "source/core/bus/serialization.h"
#include "source/core/logging.h"

namespace mte {
namespace lidar {

PointCloud::PointCloud(const std::vector<double>& xyz_vals) {
    CHECK(xyz_vals.size() > 0, "Received no point values.");
    CHECK(xyz_vals.size() % 3 == 0, "Received number of values is not a multiple of 3.");
    points.resize(3, xyz_vals.size() / 3);
    for (size_t i = 0; i < xyz_vals.size() / 3; ++i) {
        points(0, i) = xyz_vals[3 * i];
        points(1, i) = xyz_vals[3 * i + 1];
        points(2, i) = xyz_vals[3 * i + 2];
    }
}

PointCloud::PointCloud(const geometry::PointSet& point_set) {
    points = point_set;
}

void PointCloud::Transform(const geometry::Transform3d& tf) {
    points = tf * points;
}

std::string PointCloud::Serialize() const {
    return bus::Serialize(geometry::Unravel(points.transpose()));
}

PointCloud PointCloud::Deserialize(const std::string& buf) {
    const auto xyz_vals = bus::Deserialize<std::vector<double>>(buf);
    return PointCloud(xyz_vals);
}

PointCloud operator*(const geometry::Transform3d& tf, const PointCloud& pc) {
    return PointCloud{tf * pc.Points()};
}

}  // namespace lidar
}  // namespace mte
