#include "source/sensing/lidar/lidar_types.h"
#include "source/sensing/lidar/ydlidar_bridge.h"

#include "source/core/testing/assertions.h"

#include "gtest/gtest.h"

#include <cmath>

using namespace mte;

constexpr double kEps{1e-6};

TEST(PointCloudTests, CreateFromVector) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};

    // clang-format off
    const math::geometry::PointSet expected_points {
        {1, 4},
        {2, 5},
        {3, 6},
    };
    // clang-format on

    const lidar::PointCloud pc{xyz_vals};

    mte::ExpectMatrixEqual(expected_points, pc.PointSet());
}

TEST(PointCloudTests, CreateFromTriplets) {
    const std::vector<math::geometry::Point3d> triplets{{1, 2, 3}, {4, 5, 6}};

    // clang-format off
    const math::geometry::PointSet expected_points {
        {1, 4},
        {2, 5},
        {3, 6},
    };
    // clang-format on

    const lidar::PointCloud pc{triplets};

    mte::ExpectMatrixEqual(expected_points, pc.PointSet());
}

TEST(PointCloudTests, NoPointsThrows) {
    const std::vector<double> empty = {};

    EXPECT_ANY_THROW(lidar::PointCloud{empty});
}

TEST(PointCloudTests, NonMultipleOfThreeThrows) {
    const std::vector<double> vals = {1, 2, 3, 4, 5};

    EXPECT_ANY_THROW(lidar::PointCloud{vals});
}

TEST(PointCloudTests, Flatpack3DTests) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};
    const lidar::PointCloud pc{xyz_vals};

    const auto fp = pc.Flatpack3D();

    ASSERT_EQ(xyz_vals.size(), fp.size());
    for (size_t i = 0; i < xyz_vals.size(); ++i) {
        EXPECT_DOUBLE_EQ(xyz_vals[i], fp[i]);
    }
}

TEST(PointCloudTests, Flatpack2DTests) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};
    const lidar::PointCloud pc{xyz_vals};

    const auto fp = pc.Flatpack2D();

    ASSERT_EQ(4, fp.size());

    EXPECT_DOUBLE_EQ(1, fp[0]);
    EXPECT_DOUBLE_EQ(2, fp[1]);
    EXPECT_DOUBLE_EQ(4, fp[2]);
    EXPECT_DOUBLE_EQ(5, fp[3]);
}

TEST(PointCloudTests, TransformPointCloud) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};

    const math::geometry::Transform3d tf{math::geometry::Translation3d{10, 20, 30}};

    // clang-format off
    const math::geometry::PointSet expected_points {
        {11, 14},
        {22, 25},
        {33, 36},
    };
    // clang-format on

    lidar::PointCloud pc{xyz_vals};
    pc.Transform(tf);

    mte::ExpectMatrixEqual(expected_points, pc.PointSet());
}

TEST(PointCloudTests, TransformPointCloudOperator) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};

    const math::geometry::Transform3d tf{math::geometry::Translation3d{10, 20, 30}};

    // clang-format off
    const math::geometry::PointSet expected_points {
        {11, 14},
        {22, 25},
        {33, 36},
    };
    // clang-format on

    lidar::PointCloud pc{xyz_vals};

    mte::ExpectMatrixEqual(expected_points, (tf * pc).PointSet());
}

TEST(PointCloudTests, PointCloudRoundTrip) {
    const std::vector<double> xyz_vals{1, 2, 3, 4, 5, 6};
    const lidar::PointCloud pc{xyz_vals};

    const auto round_trip_pc = bus::Deserialize<lidar::PointCloud>(bus::Serialize(pc));

    mte::ExpectMatrixEqual(pc.PointSet(), round_trip_pc.PointSet());
}

TEST(YdLidarIntegrationTests, PointCloudFromLaserScan) {
    const double x1 = 1;
    const double y1 = 2;
    const double x2 = -3;
    const double y2 = -4;

    // clang-format off
    const math::geometry::PointSet true_points {
        {x1, x1, x2, x2},
        {y1, y2, y1, y2},
        {0,  0,  0,  0},
    };
    // clang-format on

    ydlidar::LaserPoint p1;
    p1.range = sqrt(x1 * x1 + y1 * y1);
    p1.angle = atan2(y1, x1);

    ydlidar::LaserPoint p2;
    p2.range = sqrt(x1 * x1 + y2 * y2);
    p2.angle = atan2(y2, x1);

    ydlidar::LaserPoint p3;
    p3.range = sqrt(x2 * x2 + y1 * y1);
    p3.angle = atan2(y1, x2);

    ydlidar::LaserPoint p4;
    p4.range = sqrt(x2 * x2 + y2 * y2);
    p4.angle = atan2(y2, x2);

    ydlidar::LaserScan scan;
    scan.data = {p1, p2, p3, p4};

    const lidar::PointCloud pc{scan};

    // NOTE: With sqrt, atan2, sin/cos, and the fact that LaserScan uses floats rather than doubles,
    // there's some inaccuracy converting to and from range/angle form. We want to make sure that
    // the error is "small enough", so we choose 1e-6 m as a very conservative bound.
    mte::ExpectMatrixNear(true_points, pc.PointSet(), kEps);
}

TEST(YdLidarIntegrationTests, FilterScan) {
    const std::vector ranges{1, 2, 3, 4, 5};

    ydlidar::LaserScan scan;
    for (size_t i = 0; i < ranges.size(); ++i) {
        ydlidar::LaserPoint pt;
        pt.range = ranges[i];
        scan.data.emplace_back(pt);
    }

    const auto pred = [](const ydlidar::LaserPoint& pt) { return pt.range > 2; };

    const auto filtered_scan = lidar::FilterScan(scan, pred);

    ASSERT_EQ(3, filtered_scan.data.size());

    EXPECT_NEAR(3, filtered_scan.data[0].range, kEps);
    EXPECT_NEAR(4, filtered_scan.data[1].range, kEps);
    EXPECT_NEAR(5, filtered_scan.data[2].range, kEps);
}
