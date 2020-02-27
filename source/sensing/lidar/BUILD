package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_types",
    srcs = ["lidar_types.cpp"],
    hdrs = ["lidar_types.h"],
    deps = [
        "//source/core/geometry",
        "@eigen",
    ],
)

cc_binary(
    name = "lidar_driver_main",
    srcs = [
        "lidar_driver.cpp",
        "ydlidar_bridge.cpp",
        "ydlidar_bridge.h",
    ],
    deps = [
        ":lidar_types",
        "//source/core:logging",
        "//source/core/bus",
        "@ydlidar",
    ],
)
