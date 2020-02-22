genrule(
    name = "platform_hpp",
    srcs = [
        "@//third_party/zeromq:platform_arm.hpp",
        "@//third_party/zeromq:platform_x86.hpp",
    ],
    outs = ["platform.hpp"],
    cmd = select({
        ":is_arm": "cat $(location @//third_party/zeromq:platform_arm.hpp) > $@",
        "//conditions:default": "cat $(location @//third_party/zeromq:platform_x86.hpp) > $@",
    }),
)

config_setting(
    name = "is_arm",
    constraint_values = [
        "@platforms//cpu:arm",
    ],
)

cc_library(
    name = "zeromq",
    srcs = glob([
        "src/tweetnacl.c",
        "src/*.cpp",
        "src/*.hpp",
    ]),
    hdrs = [
        "include/zmq.h",
        "include/zmq_utils.h",
        "src/tweetnacl.h",
        "src/zmq_draft.h",
        ":platform_hpp",
    ],
    includes = ["include"],
    linkopts = [
        "-lpthread",
    ],
    visibility = [
        "//visibility:public",
    ],
)
