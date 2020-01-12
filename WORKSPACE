load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "ed844db5990d21b75dc3553c057069f324b3916b",
    remote = "https://github.com/nelhage/rules_boost",
    shallow_since = "1576879360 -0800",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")

boost_deps()
