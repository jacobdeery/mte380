package(default_visibility = ["//visibility:public"])

cc_library(
    name = "libicp",
    srcs = glob(
        ["*.cpp"],
        ["demo.cpp"],
    ),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "@boost//:array",
        "@boost//:call_traits",
        "@boost//:concept_check",
        "@boost//:multi_array",
        "@boost//:type",
    ],
)

cc_binary(
    name = "libicp_demo",
    srcs = ["demo.cpp"],
    deps = [":libicp"],
)
