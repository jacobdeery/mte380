package(default_visibility = ["//visibility:public"])

cc_test(
    name = "pose_tests",
    srcs = ["pose_tests.cpp"],
    deps = [
        "//source/core/testing:testing_utils",
        "//source/localization:pose",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "lidar_localization_tests",
    srcs = ["lidar_localization_tests.cpp"],
    deps = [
        "//source/core:calibration",
        "//source/core/math",
        "//source/core/testing:testing_utils",
        "//source/localization:icp",
        "//source/localization:localizer",
        "//source/localization:pose",
        "//source/sensing/lidar:lidar_types",
        "@googletest//:gtest_main",
    ],
)
