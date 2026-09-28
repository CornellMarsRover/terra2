#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required. Run this on the Jetson or in the dev container." >&2
  exit 1
fi

if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: ./run build" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u

package_prefix="$(ros2 pkg prefix cmr_zed)"
config="$package_prefix/share/cmr_zed/config/human_gesture_detection.yaml"
model="$package_prefix/share/cmr_zed/config/yolo26n-pose.pt"

if [[ ! -f "$config" || ! -f "$model" ]]; then
  echo "Gesture configuration or model is missing." >&2
  echo "Run: ./run gesture-setup && ./run build" >&2
  exit 1
fi

exec ros2 run cmr_zed human_gesture_detection --ros-args \
  --params-file "$config" \
  -p "model_path:=$model" \
  "$@"
