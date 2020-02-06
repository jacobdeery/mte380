package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bus",
    srcs = ["bus.cpp"],
    deps = [":serialization"],
)

cc_library(
    name = "serialization",
    hdrs = ["serialization.h"],
    deps = ["@msgpack"],
)
