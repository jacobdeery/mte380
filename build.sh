#!/bin/bash

usage()
{
  echo "Usage: $0 [-c CONFIG_NAME] [-d]"
  exit 2
}

unset CONFIG DEPLOY

while getopts 'c:dh?' c
do
  case $c in
    c) CONFIG=$OPTARG ;;
    d) DEPLOY=1 ;;
    h|?) usage ;;
  esac
done

if [ -n "$CONFIG" ]; then
  bazel build //source/... --config=$CONFIG
else
  bazel build //source/...
fi

if [ -n "$DEPLOY" ]; then
  ./deploy.sh
fi
