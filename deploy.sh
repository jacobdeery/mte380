#!/bin/bash

PI_USERNAME=pi
PI_IP=192.168.0.113

if ping -c 1 -W 1 $PI_IP > /dev/null ; then
  rsync -av --files-from=file_manifest.txt --no-relative . $PI_USERNAME@$PI_IP:~/bin
else
  echo "ERROR: Could not connect to the Pi." >&2
  exit 1
fi
