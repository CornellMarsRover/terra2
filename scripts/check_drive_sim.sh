#!/usr/bin/env bash
set -euo pipefail

MODEL=${GAZEBO_MODEL:-drives}
pose() { gz model -m "$MODEL" -p; }
reset() {
  gz model -m "$MODEL" -x 0 -y 0 -z 0.12 -R 0 -P 0 -Y 0 >/dev/null
  sleep 0.5
}
exercise() {
  local topic=$1 type=$2 message=$3 start end
  reset
  start=$(pose)
  ros2 topic pub --rate 10 --times 20 "$topic" "$type" "$message" >/dev/null
  sleep 0.3
  end=$(pose)
  awk -v start="$start" -v end="$end" 'BEGIN {
    split(start, a, " "); split(end, b, " "); print b[1] - a[1]
  }'
}

auto_dx=$(exercise /cmd_vel_drives geometry_msgs/msg/Twist \
  '{linear: {x: 0.8}, angular: {z: 0.0}}')
teleop_dx=$(exercise /drives_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {y: -0.8}, angular: {z: 0.0}}}')
awk -v auto="$auto_dx" -v teleop="$teleop_dx" 'BEGIN {
  delta = auto - teleop; if (delta < 0) delta = -delta
  printf "autonomy dx=%.3f m, tele-op dx=%.3f m, delta=%.3f m\n", auto, teleop, delta
  if (auto < 0.4 || teleop < 0.4 || delta > 0.25) exit 1
}'
