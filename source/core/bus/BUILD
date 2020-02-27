package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bus",
    srcs = ["bus.cpp"],
    hdrs = ["bus.h"],
    deps = [
        ":serialization",
        "//source/core:logging",
        "@zmqpp",
    ],
)

cc_library(
    name = "serialization",
    hdrs = ["serialization.h"],
    deps = ["@msgpack"],
)
