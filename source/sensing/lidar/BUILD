package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_types",
    srcs = ["lidar_types.cpp"],
    hdrs = ["lidar_types.h"],
    deps = [
        "//source/core:logging",
        "//source/core/bus:serialization",
        "//source/core/math",
        "@ydlidar",
    ],
)

cc_library(
    name = "ydlidar_bridge",
    srcs = ["ydlidar_bridge.cpp"],
    hdrs = ["ydlidar_bridge.h"],
    deps = [
        "//source/core:logging",
        "//source/core/bus:serialization",
        "//source/core/math",
        "@ydlidar",
    ],
)

cc_binary(
    name = "lidar_driver_main",
    srcs = [
        "lidar_driver.cpp",
    ],
    deps = [
        ":lidar_types",
        ":ydlidar_bridge",
        "//source/core:logging",
        "//source/core/bus",
        "@ydlidar",
    ],
)
