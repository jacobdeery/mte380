package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        ":logging",
        "//source/core/bus",
    ],
)

cc_library(
    name = "calibration",
    srcs = ["calibration.cpp"],
    hdrs = ["calibration.h"],
    deps = [
        "//source/core/math",
        "//source/localization:pose",
    ],
)

cc_library(
    name = "logging",
    srcs = ["logging.cpp"],
    hdrs = ["logging.h"],
    deps = [
        "@boost//:date_time",
        "@boost//:filesystem",
        "@boost//:log",
    ],
)
