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
        "//source/core:logging",
        "//source/core/math",
        "//source/sensing/lidar:lidar_types",
        "//third_party/libicp",
    ],
)
