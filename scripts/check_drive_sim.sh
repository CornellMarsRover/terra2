#!/usr/bin/env bash
set -euo pipefail

MODEL=${GAZEBO_MODEL:-drives}
retry_gz() {
  local output
  for _ in 1 2 3; do
    if output=$(timeout 5 gz "$@" 2>/dev/null); then
      printf '%s\n' "$output"
      return 0
    fi
  done
  return 1
}
pose() { retry_gz model -m "$MODEL" -p; }
reset() {
  retry_gz model -m "$MODEL" -x 0 -y 0 -z 0.12 \
    -R 0 -P 0 -Y 0 >/dev/null
  sleep 0.5
}
exercise() {
  local source=$1 topic=$2 message=$3 start end
  reset
  ros2 topic pub --once /cmd_vel/source std_msgs/msg/String \
    "{data: $source}" >/dev/null
  start=$(pose)
  ros2 topic pub --rate 10 --times 20 "$topic" cmr_msgs/msg/DriveCommand \
    "$message" >/dev/null
  sleep 0.3
  end=$(pose)
  awk -v start="$start" -v end="$end" 'BEGIN {
    split(start, a, " "); split(end, b, " "); print b[1] - a[1]
  }'
}

command='{vx: 0.8, vy: 0.0, omega: 0.0, speed_rps: 4.0}'
auto_dx=$(exercise autonomy /cmd_vel/autonomy "$command")
teleop_dx=$(exercise teleop /cmd_vel/teleop "$command")

reset
ros2 topic pub --once /cmd_vel/estop std_msgs/msg/Bool '{data: true}' >/dev/null
start=$(pose)
ros2 topic pub --rate 10 --times 10 /cmd_vel/teleop \
  cmr_msgs/msg/DriveCommand "$command" >/dev/null
sleep 0.3
end=$(pose)
ros2 topic pub --once /cmd_vel/estop std_msgs/msg/Bool '{data: false}' >/dev/null
estop_dx=$(awk -v start="$start" -v end="$end" 'BEGIN {
  split(start, a, " "); split(end, b, " "); print b[1] - a[1]
}')
awk -v auto="$auto_dx" -v teleop="$teleop_dx" -v estop="$estop_dx" 'BEGIN {
  delta = auto - teleop; if (delta < 0) delta = -delta
  if (estop < 0) estop = -estop
  printf "autonomy=%.3fm tele-op=%.3fm delta=%.3fm estop=%.3fm\n", auto, teleop, delta, estop
  if (auto < 0.4 || teleop < 0.4 || delta > 0.25 || estop > 0.05) exit 1
}'
