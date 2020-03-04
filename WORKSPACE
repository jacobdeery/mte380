load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

http_archive(
    name = "arm_toolchain",
    build_file = "//toolchains:arm_toolchain.BUILD",
    sha256 = "d4f6480ecaa99e977e3833cc8a8e1263f9eecd1ce2d022bb548a24c4f32670f5",
    strip_prefix = "gcc-arm-8.3-2019.03-x86_64-arm-linux-gnueabihf",
    url = "https://developer.arm.com/-/media/Files/downloads/gnu-a/8.3-2019.03/binrel/gcc-arm-8.3-2019.03-x86_64-arm-linux-gnueabihf.tar.xz",
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

new_git_repository(
    name = "msgpack",
    build_file = "@//third_party:msgpack.BUILD",
    commit = "8085ab8721090a447cf98bb802d1406ad7afe420",
    remote = "https://github.com/msgpack/msgpack-c",
    shallow_since = "1575956176 +0900",
)

http_archive(
    name = "zeromq",
    build_file = "@//third_party/zeromq:zeromq.BUILD",
    sha256 = "02ecc88466ae38cf2c8d79f09cfd2675ba299a439680b64ade733e26a349edeb",
    strip_prefix = "libzmq-4.3.2",
    urls = ["https://github.com/zeromq/libzmq/archive/v4.3.2.tar.gz"],
)

new_git_repository(
    name = "zmqpp",
    build_file = "@//third_party:zmqpp.BUILD",
    commit = "7f099a8dba534661c69db32e31e13f06f34ad6bc",
    remote = "https://github.com/zeromq/zmqpp",
    shallow_since = "1578644481 +0000",
)

new_git_repository(
    name = "ydlidar",
    build_file = "@//third_party/ydlidar:ydlidar.BUILD",
    commit = "d7cc405b4845347b178def020c43573fcc8af2ec",
    patch_args = ["-p1"],  # needed for patches made with Git
    patches = ["@//third_party/ydlidar:ydlidar.patch"],
    remote = "https://github.com/YDLIDAR/sdk",
    shallow_since = "1574386903 +0800",
)

git_repository(
    name = "googletest",
    commit = "703bd9caab50b139428cea1aaff9974ebee5742e",
    remote = "https://github.com/google/googletest",
    shallow_since = "1570114335 -0400",
)
