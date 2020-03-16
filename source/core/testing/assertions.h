#pragma once

#include "source/core/math/geometry.h"
#include "source/localization/pose.h"

#include <gtest/gtest.h>

namespace mte {

void ExpectPoseEqual(const localization::Pose& p1, const localization::Pose& p2) {
    EXPECT_DOUBLE_EQ(p1.x, p2.x);
    EXPECT_DOUBLE_EQ(p1.y, p2.y);
    EXPECT_DOUBLE_EQ(p1.z, p2.z);
    EXPECT_DOUBLE_EQ(p1.v_y, p2.v_y);
    EXPECT_DOUBLE_EQ(p1.v_z, p2.v_z);
    EXPECT_DOUBLE_EQ(p1.v_x, p2.v_x);
    EXPECT_DOUBLE_EQ(p1.roll, p2.roll);
    EXPECT_DOUBLE_EQ(p1.pitch, p2.pitch);
    EXPECT_DOUBLE_EQ(p1.yaw, p2.yaw);
    EXPECT_DOUBLE_EQ(p1.roll_rate, p2.roll_rate);
    EXPECT_DOUBLE_EQ(p1.pitch_rate, p2.pitch_rate);
    EXPECT_DOUBLE_EQ(p1.yaw_rate, p2.yaw_rate);
}

template <class MatrixType>
void ExpectMatrixEqual(const MatrixType& m1, const MatrixType& m2) {
    ASSERT_EQ(m1.rows(), m2.rows());
    ASSERT_EQ(m1.cols(), m2.cols());
    for (int i = 0; i < m1.rows(); ++i) {
        for (int j = 0; j < m1.cols(); ++j) {
            EXPECT_DOUBLE_EQ(m1(i, j), m2(i, j))
                << "at i=" + std::to_string(i) + ", j=" + std::to_string(j);
        }
    }
}

template <class MatrixType>
void ExpectMatrixNear(const MatrixType& m1, const MatrixType& m2, double eps) {
    ASSERT_EQ(m1.rows(), m2.rows());
    ASSERT_EQ(m1.cols(), m2.cols());
    for (int i = 0; i < m1.rows(); ++i) {
        for (int j = 0; j < m1.cols(); ++j) {
            EXPECT_NEAR(m1(i, j), m2(i, j), eps)
                << "at i=" + std::to_string(i) + ", j=" + std::to_string(j);
        }
    }
}

}  // namespace mte
