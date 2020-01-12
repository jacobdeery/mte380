# MTE380

This is the software repo for MTE380 W20 Group 4.

# Dependencies

- To build this project, [Bazel](https://www.bazel.build) is required. This workspace has been tested with Bazel v1.2.0; other versions may also work but this is not guaranteed.
- To run the `format.sh` formatting script, `clang-format` and [`buildifier`](https://github.com/bazelbuild/buildtools/tree/master/buildifier) must be installed and on the `PATH`.

# Building

This is a [Bazel](https://www.bazel.build) project. In order to build a specific target, run `bazel build <target>`. For example, to build the localization module, run `bazel build //source/localization:localization`. In order to build the entire stack, the `build.sh` script is provided for convenience.
