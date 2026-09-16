# Gazebo obstacle-avoidance harness

This directory is the repeatable ROS/Gazebo validation harness for rover driving
and autonomy. New developers should begin with the
[Gazebo autonomy onboarding guide](../../docs/gazebo-onboarding.md).

## Quick reference

From the repository root:

```bash
./sim setup                 # Build the public-base Docker image once
./sim doctor                # Verify ROS, Gazebo, Xvfb, ffmpeg, and Python tools
./sim check                 # Run tests and build the three driving packages
./sim demo basic 300        # Fast regression and video
./sim demo stress 420       # Full obstacle-course acceptance and video
./sim report                # Print the newest summary
```

`SECONDS` is elapsed wall time, not simulated time. Generated evidence is kept
under `validation/gazebo/logs/` and ignored by Git. A failed acceptance returns a
nonzero status but still retains available evidence.

## What is production and what is simulated

Production code used unchanged:

- `autonomous_navigation` state machine, costmap, local planner, and controller
- `cmr_msgs/DriveCommand`
- `cmr_rovernet` command mux, source timeout, and estop behavior

Simulation boundaries:

- `rover_depth.urdf`: rover model and RGB/depth/point-cloud sensor
- `odom_to_autonomy_pose.py`: `/drives/odom` to autonomy pose
- `drive_command_to_gazebo.py`: selected `DriveCommand` to Gazebo `Twist`
- Gazebo planar-motion plugin: commanded motion and odometry

The harness validates the shared ROS behavior but not wheel contact dynamics,
swerve module timing, traction, Moteus transport, or CAN watchdog behavior.

## ROS control flow

```text
Gazebo blocks
  -> /camera/points
  -> costmap -> /autonomy/costmap
  -> local planner -> /autonomy/path/plan
                   -> /autonomy/path/next_waypoint
  -> controller -> /cmd_vel/autonomy (cmr_msgs/DriveCommand)
  -> command mux -> /cmd_vel (cmr_msgs/DriveCommand)
  -> drive adapter -> /drives/cmd_vel (geometry_msgs/Twist)
  -> Gazebo planar backend -> /drives/odom
  -> pose adapter -> /autonomy/pose/robot/global
```

The state machine reads `course_waypoints.yaml` and publishes
`/autonomy/target/local`. The dashboard subscribes only; it displays
`/camera/image_raw`, `/autonomy/costmap`, `/autonomy/path/plan`, the global
target, and `/drives/odom`. It does not create obstacles or paths.

The one-command course sets `SIM_COORDINATE_ONLY=true`, so ArUco mission-target
detection is omitted. Collision avoidance still uses the physical simulated
depth point cloud and production costmap.

## Commands and ownership

| Command | Runs | Output |

```bash
# Terminal 1
./validation/gazebo/start_sim.sh
# Terminal 2
./validation/gazebo/start_autonomy.sh
```

The second script prints its per-session log directory. Stop each script with
Ctrl-C. Generated logs and videos are ignored by Git.

## Shared control flow

```text
physical Gazebo blocks
  -> rover depth camera -> /camera/points
  -> production costmap -> /autonomy/costmap
  -> production local planner -> /autonomy/path/next_waypoint
  -> production controller -> /cmd_vel/autonomy (cmr_msgs/DriveCommand)
  -> production command mux -> /cmd_vel
  -> simulation drive adapter -> /drives/cmd_vel
  -> Gazebo planar-motion plugin -> /drives/odom
  -> simulation pose adapter -> /autonomy/pose/robot/global
```

`object_detection` also receives the simulated RGB/depth camera topics, but it
identifies requested ArUco mission targets. Collision avoidance itself uses the
physical depth point cloud and production costmap shown above.

## Acceptance result

The run reached two coordinate goals and detected all three blocks in the costmap.
Across 1,044 poses it had zero footprint intersections; center, right, and left
clearances were 0.601 m, 0.432 m, and 0.833 m.
