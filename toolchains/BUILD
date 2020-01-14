package(default_visibility = ["//visibility:public"])

load(":arm_toolchain_config.bzl", "arm_toolchain_config")

arm_toolchain_config(name = "arm_toolchain_config")

filegroup(name = "empty")

filegroup(
    name = "all",
    srcs = [
        "archive.sh",
        "compile.sh",
        "@arm_toolchain//:all",
    ],
)

cc_toolchain(
    name = "arm_cc_toolchain",
    all_files = ":all",
    ar_files = ":all",
    compiler_files = ":all",
    dwp_files = ":empty",
    linker_files = ":all",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    toolchain_config = ":arm_toolchain_config",
    toolchain_identifier = "arm-cc-toolchain",
)

toolchain(
    name = "arm_toolchain",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:arm",
    ],
    toolchain = ":arm_cc_toolchain",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)
