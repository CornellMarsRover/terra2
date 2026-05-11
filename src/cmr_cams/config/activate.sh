#!/usr/bin/env bash
set -e

cd ~/cmr/terra2
source install/setup.bash

echo "Available video devices:"
v4l2-ctl --list-devices || true
echo

echo "Enter cam numbers to activate (ex: 0 2 4):"
read -r cam_nums

if [ -z "$cam_nums" ]; then
  echo "No cams entered."
  exit 1
fi

for num in $cam_nums; do
  cam="/cam$num"

  echo
  echo "=============================="
  echo "Starting $cam"
  echo "=============================="

  echo "Current lifecycle state:"
  ros2 lifecycle get "$cam" || {
    echo "Could not contact $cam. Skipping."
    continue
  }

  echo "Configuring $cam ..."
  timeout 15s ros2 lifecycle set "$cam" configure || {
    echo "Configure failed or timed out for $cam. Skipping activate."
    continue
  }

  sleep 1

  echo "Activating $cam ..."
  timeout 15s ros2 lifecycle set "$cam" activate || {
    echo "Activate failed or timed out for $cam."
    continue
  }

  echo "$cam done."
  sleep 1
done

echo
echo "All requested cameras processed."