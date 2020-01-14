load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

http_archive(
    name = "arm_toolchain",
    build_file = "//toolchains:arm_toolchain.BUILD",
    sha256 = "51bbaf22a4d3e7a393264c4ef1e45566701c516274dde19c4892c911caa85617",
    strip_prefix = "gcc-arm-9.2-2019.12-x86_64-arm-none-linux-gnueabihf",
    url = "https://developer.arm.com/-/media/Files/downloads/gnu-a/9.2-2019.12/binrel/gcc-arm-9.2-2019.12-x86_64-arm-none-linux-gnueabihf.tar.xz",
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "ed844db5990d21b75dc3553c057069f324b3916b",
    remote = "https://github.com/nelhage/rules_boost",
    shallow_since = "1576879360 -0800",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")

boost_deps()

new_git_repository(
    name = "eigen",
    build_file = "@//third_party:eigen.BUILD",
    commit = "1d0c45122a5c4c5c1c4309f904120e551bacad02",
    remote = "https://gitlab.com/libeigen/eigen",
    shallow_since = "1578751349 +0100",
)
