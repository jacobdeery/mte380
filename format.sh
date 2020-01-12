#!/bin/bash

WORKSPACE_DIR=$(git rev-parse --show-toplevel)
cd $WORKSPACE_DIR

echo "Running clang-format..."
git ls-files -om --exclude-standard | grep '\.cpp$\|\.h$' | xargs clang-format -i

echo "Running buildifier..."
git ls-files -om --exclude-standard | grep 'WORKSPACE\|BUILD\|\.bzl$' | xargs buildifier

echo "Formatting completed."
