#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

if [[ "${1:-}" != "--pipeline-only" ]]; then
  colcon test --packages-select cmr_msgs astronaut_autonomy \
    --event-handlers console_direct+
  colcon test-result --verbose
fi

python3 - <<'PY'
from cmr_msgs.msg import AstronautCommand, AstronautVoiceCommand, DriveCommand

assert AstronautCommand is not None
assert AstronautVoiceCommand is not None
assert DriveCommand is not None
command = AstronautCommand()
assert hasattr(command, "source")
assert hasattr(command, "decision_reason")
assert hasattr(command, "safety_stop")
print("Astronaut command and drive message imports succeeded")
PY

rm -f /tmp/astronaut_sim/report.json /tmp/astronaut_sim/trajectory.csv
ros2 launch astronaut_autonomy astronaut_mission_sim.launch.py \
  > /tmp/astronaut_ros_pipeline.log 2>&1 &
launch_pid=$!
trap 'kill "$launch_pid" 2>/dev/null || true' EXIT

check_topic() {
  local topic="$1"
  local expected="$2"
  local actual=""
  for _ in $(seq 1 30); do
    actual="$(timeout 3 ros2 topic type "$topic" 2>/dev/null || true)"
    [[ "$actual" == "$expected" ]] && return 0
    sleep 0.5
  done
  echo "Topic $topic has type '$actual', expected '$expected'" >&2
  return 1
}

check_topic /autonomy/astronaut/perception cmr_msgs/msg/AstronautCommand
check_topic /autonomy/astronaut/gesture_perception cmr_msgs/msg/AstronautCommand
check_topic /autonomy/astronaut/voice_command cmr_msgs/msg/AstronautVoiceCommand
check_topic /autonomy/astronaut/command_source std_msgs/msg/String
check_topic /autonomy/astronaut/fusion_detail std_msgs/msg/String
check_topic /autonomy/astronaut/drive_command cmr_msgs/msg/DriveCommand
check_topic /autonomy/astronaut/mission_state std_msgs/msg/String
check_topic /autonomy/astronaut/manipulation/request std_msgs/msg/String
check_topic /autonomy/astronaut/sim/rover_pose geometry_msgs/msg/Pose2D
check_topic /cmd_vel/estop std_msgs/msg/Bool

if ros2 topic list | grep -Fxq /cmd_vel/autonomy; then
  echo "Safety failure: simulation exposed the physical autonomy mux topic" >&2
  exit 1
fi

for _ in $(seq 1 180); do
  [[ -s /tmp/astronaut_sim/report.json ]] && break
  sleep 0.5
done

if [[ ! -s /tmp/astronaut_sim/report.json ]]; then
  cat /tmp/astronaut_ros_pipeline.log >&2
  echo "Timed out waiting for the ROS simulation report" >&2
  exit 1
fi

python3 - <<'PY'
import json
from pathlib import Path

report = json.loads(Path("/tmp/astronaut_sim/report.json").read_text())
assert report["succeeded"], report
assert report["recognized_commands"] == [
    "FOLLOW", "STAY", "FETCH", "COME", "GIVE"
], report
assert report["follow_stop_distance_m"] <= 3.0, report
assert report["remote_command_distance_m"] >= 20.0, report
assert not report["safety_violations"], report
assert report["unexpected_commands_rejected"] == 1, report
print(json.dumps(report, indent=2, sort_keys=True))
PY

echo "ROS astronaut pipeline passed."
