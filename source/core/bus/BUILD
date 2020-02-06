package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bus",
    srcs = ["bus.cpp"],
    hdrs = ["bus.h"],
    # linkopts = ["-lzmq"],
    deps = [
        ":serialization",
        "@zmqpp",
    ],
)

cc_library(
    name = "serialization",
    hdrs = ["serialization.h"],
    deps = ["@msgpack"],
)
