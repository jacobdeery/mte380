#!/bin/bash

WORKSPACE_DIR=$(git rev-parse --show-toplevel)
cd $WORKSPACE_DIR

echo "Running clang-format..."
git ls-files -oc | grep '\.cpp$\|\.h$' | xargs clang-format -i

echo "Running buildifier..."
git ls-files -oc | grep 'WORKSPACE\|BUILD\|\.bzl$' | xargs buildifier

echo "Formatting completed."
