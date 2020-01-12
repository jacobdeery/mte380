cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [":logging"],
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
