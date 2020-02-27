#pragma once

#include "source/core/geometry/geometry.h"

namespace mte {
namespace lidar {

// TODO(jacob): Figure out complex type serialization and fix this class.
class PointCloud {
   public:
    PointCloud() = default;

   private:
    std::vector<geometry::Point3d> data;
};

PointCloud operator*(const geometry::Transform3d& tf, const PointCloud& pc);

}  // namespace lidar
}  // namespace mte
