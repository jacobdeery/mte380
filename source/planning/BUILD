package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "planner_main",
    srcs = [
        "planner_main.cpp",
    ],
    deps = [
        ":arduino_bridge",
        ":planner",
        "//source/core:logging",
        "//source/core/bus",
        "//source/localization:pose",
    ],
)

cc_library(
    name = "arduino_bridge",
    srcs = ["arduino_bridge.cpp"],
    hdrs = ["arduino_bridge.h"],
    deps = [
        ":planner",
    ],
)

cc_library(
    name = "planner",
    srcs = ["planner.cpp"],
    hdrs = ["planner.h"],
    deps = [
        "//source/core/bus:serialization",
        "//source/localization:pose",
    ],
)
