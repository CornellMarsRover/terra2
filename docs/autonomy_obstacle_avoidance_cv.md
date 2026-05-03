# Autonomy Obstacle Avoidance CV (Simulation)

## Branch
- Base branch: `autonomy`
- Feature branch: `feature/autonomy-obstacle-avoidance-cv`

## What Was Implemented
This branch adds an initial obstacle-avoidance pipeline to the autonomy stack with a simulation-first perception path.

The implementation keeps the existing autonomy flow intact:
- global target -> global planner
- global planner -> local planner
- local planner -> controller
- controller -> autonomy motion topics

The new pieces added on top are:
- `sim_drive_bridge`: bridges autonomy motion outputs into Gazebo motion commands
- `odom_to_autonomy_pose`: converts Gazebo odometry into the autonomy pose topic
- `sim_vision_obstacle_detector`: publishes obstacle costmap data from simulated vision input
- `obstacle_guard`: adds a close-range safety layer that can stop or turn the rover when an obstacle is too close
- `obstacle_avoidance_sim.launch.py`: repeatable end-to-end Gazebo launch for obstacle avoidance testing

## How Gazebo Branch Was Referenced
The `gazebo_sim` branch was used only as a reference for:
- Gazebo launch structure
- the pattern of bridging autonomy motion to `/drives/cmd_vel`
- stable use of the Gazebo planar move plugin for repeatable simulation testing

No merge, rebase, or branch-base change from `gazebo_sim` was performed.

## Files Manually Copied From Gazebo Branch
To unify the rover asset across simulations, these shared sim assets were copied from `gazebo_sim` into `rover_gazebo`:
- `src/rover_gazebo/urdf/drives.urdf`
- `src/rover_gazebo/meshes/rover_26.stl`

No merge, rebase, or unrelated branch history from `gazebo_sim` was brought over.

## New / Updated Files
### New
- `src/autonomous_navigation/autonomous_navigation/sim_drive_bridge.py`
- `src/autonomous_navigation/autonomous_navigation/odom_to_autonomy_pose.py`
- `src/autonomous_navigation/autonomous_navigation/obstacle_guard.py`
- `src/autonomous_navigation/autonomous_navigation/obstacle_guard_core.py`
- `src/autonomous_navigation/autonomous_navigation/sim_goal_publisher.py`
- `src/autonomous_navigation/autonomous_navigation/sim_vision_obstacle_core.py`
- `src/autonomous_navigation/autonomous_navigation/sim_vision_obstacle_detector.py`
- `src/autonomous_navigation/launch/obstacle_avoidance_sim.launch.py`
- `src/rover_gazebo/urdf/drives.urdf`
- `src/rover_gazebo/meshes/rover_26.stl`
- `src/autonomous_navigation/models/obstacle_box.sdf`
- `src/autonomous_navigation/test/test_obstacle_guard.py`
- `src/autonomous_navigation/test/test_sim_vision_obstacle_detector.py`

### Updated
- `src/autonomous_navigation/autonomous_navigation/controller.py`
- `src/autonomous_navigation/autonomous_navigation/local_planner.py`
- `src/autonomous_navigation/autonomous_navigation/global_planner.py`
- `src/autonomous_navigation/autonomous_navigation/costmap.py`
- `src/autonomous_navigation/setup.py`
- `src/autonomous_navigation/package.xml`
- `src/rover_gazebo/setup.py`

## Control / Perception Flow
### Motion pipeline
- `sim_goal_publisher` publishes `/autonomy/target/global`
- `global_planner` publishes `/autonomy/target/local`
- `local_planner` publishes `/autonomy/path/next_waypoint`
- `controller` publishes:
  - `/autonomy/move/ackerman`
  - `/autonomy/move/point_turn`
- `sim_drive_bridge` converts those into `/drives/cmd_vel`
- Gazebo planar drive plugin moves the rover
- `odom_to_autonomy_pose` republishes `/drives/odom` as `/autonomy/pose/robot/global`

### Obstacle pipeline
- `sim_vision_obstacle_detector` publishes `/autonomy/costmap`
- `local_planner` uses `/autonomy/costmap` to invalidate blocked segments and replan
- `obstacle_guard` monitors the same costmap for close-range hazards
- `controller` subscribes to:
  - `/autonomy/obstacle_avoidance/active`
  - `/autonomy/obstacle_avoidance/override`

This gives two layers of behavior:
- medium-range: local planner reroutes around the obstacle
- close-range: guard can stop or point-turn if needed

## Simulated Vision Input
The detector supports an `auto` mode.

In `auto` mode:
- if ROS camera images are available, the detector can use image-based color segmentation
- if ROS camera images are not available, it falls back to synthetic obstacle visibility based on robot pose, obstacle pose, field of view, and range

In the current devcontainer/Gazebo setup, ROS camera image publishing from Gazebo was not reliable, so the repeatable simulation path uses the synthetic-vision fallback.

This still satisfies the initial feature goal of obstacle detection from camera or simulated vision input while keeping the autonomy branch independent from the Gazebo branch.

The autonomy simulation now uses the same shared rover mesh and URDF as the other Gazebo simulation work by spawning the canonical source asset at `src/rover_gazebo/urdf/drives.urdf`.

## How To Run In Simulation
From the ROS container:

```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select autonomous_navigation
source install/setup.bash
ros2 launch autonomous_navigation obstacle_avoidance_sim.launch.py gui:=false
```

Optional launch arguments:

```bash
ros2 launch autonomous_navigation obstacle_avoidance_sim.launch.py \
  gui:=true \
  goal_north:=6.0 \
  goal_west:=0.0 \
  obstacle_x:=2.5 \
  obstacle_y:=0.0
```

## Expected Simulation Behavior
Default scenario:
- rover starts near `(0, 0)`
- goal is placed at `(6.0, 0.0)`
- obstacle box is placed directly in the rover path near `(2.5, 0.0)`

Expected behavior:
1. rover initially drives toward the goal
2. simulated vision publishes obstacle cost cells
3. local planner invalidates the straight path segment
4. local planner computes a detour waypoint to the side of the obstacle
5. controller turns toward the detour waypoint
6. rover passes around the obstacle
7. rover resumes progress toward the goal

## Validation Performed
### Build
- `colcon build --symlink-install --packages-select autonomous_navigation`

### Unit tests
- `python3 -m pytest src/autonomous_navigation/test/test_obstacle_guard.py src/autonomous_navigation/test/test_sim_vision_obstacle_detector.py -q`

### End-to-end sim validation
Verified in Gazebo using the shared `src/rover_gazebo/urdf/drives.urdf` rover asset:
- rover starts from the launch origin
- obstacle is spawned in front of the rover
- detector publishes obstacle visibility logs
- local planner logs segment invalidation and replanning
- controller turns toward replanned detour waypoints
- rover moves around the obstacle and continues toward the goal

## Known Limitations
- The Gazebo ROS camera plugin path in this environment did not produce ROS image topics reliably, so the simulation currently uses synthetic vision fallback rather than true image-stream detection.
- The controller still oscillates near the final goal after the rover reaches the target vicinity. This appears to be an existing goal-completion/state-machine gap rather than an obstacle-avoidance-specific regression.
- The shared rover URDF includes visible wheel / steer joints, but chassis motion is still driven by Gazebo's planar move plugin rather than full wheel-contact dynamics.

## Next Steps
- Make the Gazebo camera ROS plugin path reliable so the detector can exercise true image-based CV in sim by default.
- Add a goal-reached state to suppress end-of-goal point-turn oscillation.
- Add a recorded validation artifact or scripted integration check for CI if the Gazebo environment becomes stable enough for it.
