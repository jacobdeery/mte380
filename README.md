# MTE380

This is the software repo for MTE380 W20 Group 4.

# Building

This is a [Bazel](https://www.bazel.build) project. In order to build a specific target, run `bazel build <target>`. For example, to build the localization module, run `bazel build //source/localization:localization`. In order to build the entire stack, the `build.sh` script is provided for convenience.
