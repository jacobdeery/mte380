cc_library(
    name = "ydlidar",
    srcs = glob([
        "src/*.cpp",
        "src/*.c",
        "src/impl/unix/*.cpp",
        "src/*.h",
        "src/impl/unix/*.h",
    ]),
    hdrs = glob([
        "include/*.h",
    ]),
    copts = [
        "-fpermissive",
    ],
    linkopts = [
        "-lpthread",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
