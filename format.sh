#!/usr/bin/bash

WORKSPACE_DIR=$(bazel info workspace)

cd $WORKSPACE_DIR

echo "Running clang-format..."
git status -s -u | grep '\.cpp$\|\.h$' | sed "s/^.\{3\}//" | xargs clang-format -i

echo "Running buildifier..."
git status -s -u | grep 'WORKSPACE\|BUILD\|\.bzl$' | sed "s/^.\{3\}//" | xargs buildifier

echo "Formatting completed."
