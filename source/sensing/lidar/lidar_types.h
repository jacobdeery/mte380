#pragma once

#include "source/core/geometry/geometry.h"

namespace mte {
namespace lidar {

class PointCloud {
   public:
    explicit PointCloud(const std::vector<double>& vals);
    PointCloud(const geometry::PointSet& point_set);

    void Transform(const geometry::Transform3d& tf);
    geometry::PointSet Points() const { return points; };

   private:
    geometry::PointSet points;

   public:
    std::string Serialize() const;
    static PointCloud Deserialize(const std::string& buf);
};

PointCloud operator*(const geometry::Transform3d& tf, const PointCloud& pc);

}  // namespace lidar
}  // namespace mte
