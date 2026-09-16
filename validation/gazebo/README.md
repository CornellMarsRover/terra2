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

Use two container terminals with the same optional `ROS_DOMAIN_ID` (default 96):

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
