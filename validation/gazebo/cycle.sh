#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
ROOT=/cmr/terra2
cd "$ROOT"
if [[ $1 == check ]]; then
  bash scripts/test_autonomy.sh > "$SESSION_DIR/tests.log" 2>&1
  python3 -m pytest -q validation/gazebo/tests >> "$SESSION_DIR/tests.log" 2>&1
  colcon build --symlink-install --packages-select cmr_msgs cmr_rovernet autonomous_navigation > "$SESSION_DIR/build.log" 2>&1
  echo 'Tests and build passed.' | tee "$SESSION_DIR/summary.txt"
  exit
fi
source install/setup.bash
set -u
export ROS_DOMAIN_ID=96 DISPLAY=:99
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
world=stress_course; [[ $2 == basic ]] && world=obstacle_course
export WORLD="$ROOT/validation/gazebo/assets/$world.world"
cp "$WORLD" "$SESSION_DIR/world.sdf"
pids=()
cleanup() {
  [[ -z ${video_pid:-} ]] || { kill -INT "$video_pid" 2>/dev/null || true; wait "$video_pid" || true; }
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
setsid bash validation/gazebo/start_sim.sh > "$SESSION_DIR/gazebo.log" 2>&1 & pids+=("$!")
setsid ros2 topic echo /drives/odom nav_msgs/msg/Odometry --csv > "$SESSION_DIR/odom.csv" 2> "$SESSION_DIR/odom.log" & pids+=("$!")
ready=false
for ((i=0;i<90;i++)); do
  if [[ $(wc -l < "$SESSION_DIR/odom.csv") -gt 2 ]]; then ready=true; break; fi
  sleep 1
done
$ready || { echo 'FAIL: no odometry within 90 seconds' | tee "$SESSION_DIR/summary.txt"; exit 1; }
python3 validation/gazebo/resize_view.py
setsid python3 validation/gazebo/dashboard.py --ros-args -p use_sim_time:=true > "$SESSION_DIR/dashboard.log" 2>&1 & pids+=("$!")
sleep 2
ffmpeg -nostdin -y -f x11grab -framerate 12 -video_size 1280x720 -i :99 \
  -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$SESSION_DIR/demo.mp4" > "$SESSION_DIR/video.log" 2>&1 & video_pid=$!
SIM_COORDINATE_ONLY=true setsid bash validation/gazebo/start_autonomy.sh > "$SESSION_DIR/autonomy.log" 2>&1 & pids+=("$!")
deadline=$((SECONDS + $3))
while (( SECONDS < deadline )); do
  if grep -q 'All waypoints reached' "$SESSION_DIR/state.log" 2>/dev/null; then
    sleep 8
    break
  fi
  sleep 1
done
cleanup
video_pid=; pids=()
python3 validation/gazebo/analyze_course.py "$WORLD" "$SESSION_DIR/odom.csv" --goal 15,15 --output "$SESSION_DIR/report.json" > "$SESSION_DIR/analysis.log"
python3 validation/gazebo/summarize.py "$SESSION_DIR"
