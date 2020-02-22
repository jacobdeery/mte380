package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pose",
    srcs = ["pose.cpp"],
    hdrs = ["pose.h"],
    deps = [
        "//source/core:logging",
        "//source/core/bus:serialization",
        "//source/core/geometry",
    ],
)

cc_library(
    name = "icp",
    srcs = ["icp.cpp"],
    hdrs = ["icp.h"],
    deps = [
        "//source/core:logging",
        "//source/core/geometry",
        "//third_party/libicp",
    ],
)
