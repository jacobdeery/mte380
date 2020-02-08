package(default_visibility = ["//visibility:public"])

cc_test(
    name = "bus_tests",
    srcs = ["bus_tests.cpp"],
    deps = [
        "//source/core/bus",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "serialization_tests",
    srcs = ["serialization_tests.cpp"],
    deps = [
        "//source/core/bus:serialization",
        "@googletest//:gtest_main",
    ],
)

cc_binary(
    name = "sender_demo",
    srcs = ["sender_demo.cpp"],
    deps = ["//source/core/bus"],
)

cc_binary(
    name = "receiver_demo",
    srcs = ["receiver_demo.cpp"],
    deps = ["//source/core/bus"],
)
