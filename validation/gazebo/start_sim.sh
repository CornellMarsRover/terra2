#!/usr/bin/env bash
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
set -euo pipefail
export DISPLAY="${DISPLAY:-:99}"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
if ! pgrep -f "Xvfb $DISPLAY" >/dev/null; then
  Xvfb "$DISPLAY" -screen 0 1280x720x24 +extension GLX +render -noreset &
  pids+=("$!")
fi
WORLD="${WORLD:-$ROOT/validation/gazebo/assets/obstacle_course.world}"
gzserver "$WORLD" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
pids+=("$!")
sleep 6
ros2 run gazebo_ros spawn_entity.py -entity drives \
  -file "$ROOT/validation/gazebo/assets/rover_depth.urdf" -z 0.12
gzclient --verbose &
pids+=("$!")
wait "${pids[${#pids[@]}-2]}"
