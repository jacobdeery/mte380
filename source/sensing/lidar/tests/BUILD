package(default_visibility = ["//visibility:public"])

cc_test(
    name = "lidar_tests",
    srcs = ["lidar_tests.cpp"],
    deps = [
        "//source/core/math",
        "//source/core/testing:testing_utils",
        "//source/sensing/lidar:lidar_types",
        "//source/sensing/lidar:ydlidar_bridge",
        "@googletest//:gtest_main",
    ],
)
