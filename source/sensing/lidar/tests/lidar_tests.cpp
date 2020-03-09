#include "source/sensing/lidar/lidar_types.h"

#include "source/core/testing/assertions.h"

#include "gtest/gtest.h"

using namespace mte;

TEST(PointCloudTests, CreateFromVector) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};

    // clang-format off
    const geometry::PointSet expected_points {
        {1, 4},
        {2, 5},
        {3, 6},
    };
    // clang-format on

    const lidar::PointCloud pc{xyz_vals};

    mte::ExpectMatrixEqual(expected_points, pc.Points());
}

TEST(PointCloudTests, NoPointsThrows) {
    const std::vector<double> empty = {};

    EXPECT_ANY_THROW(lidar::PointCloud{empty});
}

TEST(PointCloudTests, NonMultipleOfThreeThrows) {
    const std::vector<double> vals = {1, 2, 3, 4, 5};

    EXPECT_ANY_THROW(lidar::PointCloud{vals});
}

TEST(PointCloudTests, TransformPointCloud) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};

    const geometry::Transform3d tf{geometry::Translation3d{10, 20, 30}};

    // clang-format off
    const geometry::PointSet expected_points {
        {11, 14},
        {22, 25},
        {33, 36},
    };
    // clang-format on

    lidar::PointCloud pc{xyz_vals};
    pc.Transform(tf);

    mte::ExpectMatrixEqual(expected_points, pc.Points());
}

TEST(PointCloudTests, TransformPointCloudOperator) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};

    const geometry::Transform3d tf{geometry::Translation3d{10, 20, 30}};

    // clang-format off
    const geometry::PointSet expected_points {
        {11, 14},
        {22, 25},
        {33, 36},
    };
    // clang-format on

    lidar::PointCloud pc{xyz_vals};

    mte::ExpectMatrixEqual(expected_points, (tf * pc).Points());
}

TEST(PointCloudTests, PointCloudRoundTrip) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};
    const lidar::PointCloud pc{xyz_vals};

    const auto round_trip_pc = bus::Deserialize<lidar::PointCloud>(bus::Serialize(pc));

    mte::ExpectMatrixEqual(pc.Points(), round_trip_pc.Points());
}
