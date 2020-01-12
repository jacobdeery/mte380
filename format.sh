#!/usr/bin/bash

WORKSPACE_DIR=$(bazel info workspace)

cd $WORKSPACE_DIR

git status -s -u | grep '\.cpp$\|\.h$' | sed "s/^.\{3\}//" | xargs clang-format -i
