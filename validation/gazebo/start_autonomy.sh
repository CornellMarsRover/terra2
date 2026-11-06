#!/usr/bin/env bash
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
set -euo pipefail
SESSION="${SESSION_DIR:-$ROOT/validation/gazebo/logs/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$SESSION"
pids=()
start() { local name="$1"; shift; "$@" >"$SESSION/$name.log" 2>&1 & pids+=("$!"); }
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
args=(--ros-args -p use_sim_time:=true)
start pose python3 "$ROOT/validation/gazebo/adapters/odom_to_autonomy_pose.py" "${args[@]}"
start drive python3 "$ROOT/validation/gazebo/adapters/drive_command_to_gazebo.py" "${args[@]}"
start mux "$ROOT/install/cmr_rovernet/lib/cmr_rovernet/drive_command_mux" \
  "${args[@]}" -p active_source:=autonomy
start state "$ROOT/install/autonomous_navigation/lib/autonomous_navigation/state_machine" \
  "${args[@]}" -p real:=false -p "waypoints_file:=$ROOT/validation/gazebo/assets/course_waypoints.yaml"
if [[ ${SIM_COORDINATE_ONLY:-false} != true ]]; then
  start detector "$ROOT/install/autonomous_navigation/lib/autonomous_navigation/object_detection" \
    "${args[@]}" -p real:=false
fi
sleep 2
start costmap "$ROOT/install/autonomous_navigation/lib/autonomous_navigation/costmap" \
  "${args[@]}" -p real:=false
sleep 3
start planner "$ROOT/install/autonomous_navigation/lib/autonomous_navigation/local_planner" \
  "${args[@]}" -p real:=false -p visualize:=false
sleep 2
start controller "$ROOT/install/autonomous_navigation/lib/autonomous_navigation/controller" \
  "${args[@]}" -p real:=false
printf 'Autonomy running; logs: %s\n' "$SESSION"
wait
