# BUILD file adapted from https://github.com/iotaledger/rules_iota/blob/master/build/BUILD.libzmq

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

# NOTE: we make this a standalone library so we can compile it with -fpermissive (we don't want to
# use that flag for the rest of libzmq).
cc_library(
    name = "tweetnacl",
    srcs = [
        "src/tweetnacl.c",
    ],
    hdrs = [
        "src/tweetnacl.h",
        ":platform_hpp",
    ],
    copts = [
        "-fpermissive",
    ],
)

cc_library(
    name = "zeromq",
    srcs = glob([
        "src/*.cpp",
        "src/*.hpp",
    ]),
    hdrs = [
        "include/zmq.h",
        "include/zmq_utils.h",
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
    deps = [":tweetnacl"],
)
