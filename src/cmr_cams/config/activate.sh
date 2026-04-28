#!/usr/bin/env bash

set -e

cd ~/cmr/terra2
source install/setup.bash

echo “Available video devices:”
v4l2-ctl --list-devices || true
echo

echo “Enter video numbers to activate (ex: 0 2 4):”
read video_nums

if [ -z “$video_nums” ]; then
  echo “No video devices entered.”
  exit 1
fi

echo “Finding camera nodes...”
all_cams=$(ros2 node list | grep ‘^/cam’)

selected_cams=“”

for num in $video_nums; do
  dev=“/dev/video$num”

  for node in $all_cams; do
    value=$(ros2 param get “$node” video_device 2>/dev/null || true)
    if echo “$value” | grep -q “$dev”; then
      selected_cams=“$selected_cams $node”
    fi
  done
done

if [ -z “$selected_cams” ]; then
  echo “No matching camera nodes found.”
  exit 1
fi

echo
echo “Camera nodes to activate:”
echo “$selected_cams”
echo

for cam in $selected_cams; do
  echo “Configuring $cam ...”
  ros2 lifecycle set “$cam” configure
done

sleep 2

for cam in $selected_cams; do
  echo “Activating $cam ...”
  ros2 lifecycle set “$cam” activate
done

echo
echo “Done.”