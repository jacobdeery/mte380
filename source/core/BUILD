cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [":logging"],
)

cc_library(
    name = "logging",
    hdrs = ["logging.h"],
    srcs = ["logging.cpp"],
    deps = ["@boost//:date_time",
            "@boost//:filesystem",
            "@boost//:log"],
)
