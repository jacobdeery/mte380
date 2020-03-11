package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pose",
    srcs = ["pose.cpp"],
    hdrs = ["pose.h"],
    deps = [
        "//source/core:logging",
        "//source/core/bus:serialization",
        "//source/core/math",
    ],
)

cc_library(
    name = "icp",
    srcs = ["icp.cpp"],
    hdrs = ["icp.h"],
    deps = [
        ":pose",
        "//source/core:logging",
        "//source/core/math",
        "//source/sensing/lidar:lidar_types",
        "//third_party/libicp",
    ],
)

cc_library(
    name = "localizer",
    srcs = ["localizer.cpp"],
    hdrs = ["localizer.h"],
    deps = [
        ":pose",
        "//source/core:calibration",
        "//source/core/math",
        "//source/sensing/lidar:lidar_types",
    ],
)

cc_binary(
    name = "localizer_main",
    srcs = ["localizer_main.cpp"],
    deps = [
        ":icp",
        ":localizer",
        ":pose",
        "//source/core:logging",
        "//source/core/bus",
        "//source/core/math",
        "//source/sensing/lidar:lidar_types",
    ],
)
