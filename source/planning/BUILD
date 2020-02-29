package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "planner_main",
    srcs = [
        "arduino_bridge.cpp",
        "arduino_bridge.h",
        "planner_main.cpp",
    ],
    deps = [
        ":planner",
        "//source/core:logging",
        "//source/core/bus",
        "//source/localization:pose",
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
