cc_library(
    name = "zmqpp",
    srcs = glob([
        "src/zmqpp/*.cpp",
    ]),
    hdrs = glob([
        "src/zmqpp/*.hpp",
    ]),
    strip_include_prefix = "src/zmqpp",
    visibility = ["//visibility:public"],
    deps = ["@zeromq"],
)
