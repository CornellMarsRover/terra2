#!/usr/bin/env bash

set -e

cd ~/cmr/terra2
source install/setup.bash

echo "Finding camera nodes..."
cams=$(ros2 node list | grep '^/cam' || true)

if [ -z "$cams" ]; then
  echo "No /cam nodes found."
  echo "Make sure Terminal 1 is already running:"
  echo "ros2 launch cmr_cams default.launch.py"
  exit 1
fi

echo "Camera nodes found:"
echo "$cams"
echo

for cam in $cams; do
  echo "Configuring $cam ..."
  ros2 lifecycle set "$cam" configure
done

sleep 2

for cam in $cams; do
  echo "Activating $cam ..."
  ros2 lifecycle set "$cam" activate
done

echo
echo "Done. Current states:"
for cam in $cams; do
  echo -n "$cam -> "
  ros2 lifecycle get "$cam"
done

echo
echo "Image topics:"
ros2 topic list | grep image_raw || true