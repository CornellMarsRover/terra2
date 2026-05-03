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
- `sim_obstacle_pointcloud`: publishes simulated `/camera/points` obstacle returns into the normal costmap pipeline
- `obstacle_guard`: adds a close-range safety layer that can stop or turn the rover when an obstacle is too close
- `obstacle_avoidance_sim.launch.py`: repeatable end-to-end Gazebo launch for obstacle avoidance testing
- `obstacle_avoidance_multi_obstacle_sim.launch.py`: multi-obstacle Gazebo scenario for testing visibility and rerouting behavior

## How Gazebo Branch Was Referenced
The `gazebo_sim` branch was used only as a reference for:
- Gazebo launch structure
- the pattern of bridging autonomy motion to `/drives/cmd_vel`
- stable use of the Gazebo planar move plugin for repeatable simulation testing

No merge, rebase, or branch-base change from `gazebo_sim` was performed.

## Files Manually Copied From Gazebo Branch
To keep the autonomy simulation on the exact same rover asset used in `gazebo_sim`, these files were copied directly from that branch into the workspace root:
- `drives.urdf`
- `meshes/rover_26.stl`

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
- `src/autonomous_navigation/autonomous_navigation/sim_obstacle_pointcloud.py`
- `src/autonomous_navigation/launch/obstacle_avoidance_sim.launch.py`
- `src/autonomous_navigation/worlds/obstacle_avoidance_demo.world`
- `drives.urdf`
- `meshes/rover_26.stl`
- `src/autonomous_navigation/models/obstacle_box.sdf`
- `src/autonomous_navigation/test/test_obstacle_guard.py`
- `src/autonomous_navigation/test/test_sim_vision_obstacle_detector.py`
- `scripts/log_autonomy_demo_topics.py`
- `scripts/render_autonomy_demo_map.py`

### Updated
- `src/autonomous_navigation/autonomous_navigation/controller.py`
- `src/autonomous_navigation/autonomous_navigation/local_planner.py`
- `src/autonomous_navigation/autonomous_navigation/global_planner.py`
- `src/autonomous_navigation/autonomous_navigation/costmap.py`
- `src/autonomous_navigation/setup.py`
- `src/autonomous_navigation/package.xml`

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
- `sim_obstacle_pointcloud` publishes synthetic obstacle returns on `/camera/points`
- `costmap` consumes `/camera/points` and produces `/autonomy/costmap`
- `sim_obstacle_pointcloud` also publishes debug topics:
  - `/autonomy/sim_obstacles/all`
  - `/autonomy/sim_obstacles/visible`
- `local_planner` uses `/autonomy/costmap` to invalidate blocked segments and replan
- `obstacle_guard` monitors the same costmap for close-range hazards
- `controller` subscribes to:
  - `/autonomy/obstacle_avoidance/active`
  - `/autonomy/obstacle_avoidance/override`

This gives two layers of behavior:
- medium-range: local planner reroutes around the obstacle
- close-range: guard can stop or point-turn if needed

## Simulated Sensor Input
The repeatable simulation path now feeds obstacle perception into the real autonomy costmap path instead of publishing a parallel sim-only costmap.

The setup is:
- `sim_obstacle_pointcloud` computes which configured blocks are currently visible
- it publishes those returns as a synthetic `PointCloud2` on `/camera/points`
- the existing `costmap` node converts that sensor data into `/autonomy/costmap`
- the rest of the autonomy stack consumes that costmap unchanged

This keeps Gazebo as a realistic harness for the rover autonomy graph:
- sim adapters are limited to odometry, drive bridging, and synthetic sensor publication
- planning, costmap generation, control, and emergency guard logic remain shared autonomy code

The autonomy simulation uses the exact same root-level rover asset as `gazebo_sim` by spawning `drives.urdf`, which references `meshes/rover_26.stl`.

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

Multi-obstacle scenario:

```bash
ros2 launch autonomous_navigation obstacle_avoidance_multi_obstacle_sim.launch.py
```

This scenario spawns:
- one center obstacle directly in the nominal path
- one right-side obstacle that is visible early in the run
- one farther-left obstacle that starts outside the tighter FOV and becomes relevant later as the rover turns

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
Verified in Gazebo using the shared root-level `drives.urdf` rover asset:
- rover starts from the launch origin
- obstacle is spawned in front of the rover
- detector publishes obstacle visibility logs
- local planner logs segment invalidation and replanning
- controller turns toward replanned detour waypoints
- rover moves around the obstacle and continues toward the goal

### Recorded demo artifact
Generated a side-by-side demo video showing:
- Gazebo simulation with the rover and obstacle in frame
- a synchronized autonomy map view with perceived obstacle cells, FOV, path trail, and targets

Artifacts:
- `logs/autonomy_demo/autonomy_obstacle_avoidance_demo.mp4`
- `logs/autonomy_demo/autonomy_obstacle_avoidance_frame.jpg`
- `logs/autonomy_multi_demo/autonomy_multi_obstacle_demo.mp4`

## Known Limitations
- The Gazebo ROS camera plugin path in this environment did not produce ROS image topics reliably, so the simulation currently uses synthetic vision fallback rather than true image-stream detection.
- The controller still oscillates near the final goal after the rover reaches the target vicinity. This appears to be an existing goal-completion/state-machine gap rather than an obstacle-avoidance-specific regression.
- The shared rover URDF includes visible wheel / steer joints, but chassis motion is still driven by Gazebo's planar move plugin rather than full wheel-contact dynamics.

## Next Steps
- Make the Gazebo camera ROS plugin path reliable so the detector can exercise true image-based CV in sim by default.
- Add a goal-reached state to suppress end-of-goal point-turn oscillation.
- Add a recorded validation artifact or scripted integration check for CI if the Gazebo environment becomes stable enough for it.

## Logging and Recording
The updated stack logs clean debug lines from the shared autonomy path:
- `sim_obstacle_pointcloud`: visible obstacle set and point count
- `costmap`: occupied cell count, nearest obstacle distance, rover pose, yaw
- `local_planner`: target changes, blocked segments, replans, path previews
- `controller`: waypoint error, heading error, and final motion commands
- `obstacle_guard`: close-range corridor summary and emergency intervention status

Useful topic recording command:

```bash
ros2 bag record \
  /camera/points \
  /autonomy/costmap \
  /autonomy/path/next_waypoint \
  /autonomy/move/ackerman \
  /autonomy/move/point_turn \
  /autonomy/pose/robot/global \
  /autonomy/sim_obstacles/all \
  /autonomy/sim_obstacles/visible
```
