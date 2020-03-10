#pragma once

#include "source/core/math/geometry.h"

#include "CYdLidar.h"

namespace mte {
namespace lidar {

class PointCloud {
   public:
    explicit PointCloud(const std::vector<double>& vals);
    PointCloud(const ydlidar::LaserScan& scan);
    PointCloud(const math::geometry::PointSet& point_set);

    void Transform(const math::geometry::Transform3d& tf);
    math::geometry::PointSet Points() const { return points; };

   private:
    math::geometry::PointSet points;

   public:
    std::string Serialize() const;
    static PointCloud Deserialize(const std::string& buf);
};

PointCloud operator*(const math::geometry::Transform3d& tf, const PointCloud& pc);

}  // namespace lidar
}  // namespace mte
