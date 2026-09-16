---
name: terra-gazebo
description: Develop, debug, validate, and record terra2 rover driving or autonomy in the repository's Docker/Gazebo harness. Use for simulator setup, ROS control-flow failures, obstacle courses, costmaps, planning, acceptance evidence, and Gazebo videos; not for arm work.
---
# Terra2 Gazebo workflow

Work from the repository root and preserve the production path:
`/camera/points -> costmap -> planner -> controller -> command mux -> /cmd_vel`.
Simulation code belongs only at sensor, pose, visualization, or drive-backend
boundaries. Do not change arm code or bypass the mux, production planner, or
production controller to make a demo pass.

Read `docs/gazebo-onboarding.md` for a new environment, course editing, handoff,
or failure routing. Read `validation/gazebo/README.md` for topic contracts,
artifacts, and acceptance rules. Do not rediscover information already recorded
there.

## Select the smallest valid mode

- Environment problem: `./sim setup`, then `./sim doctor`.
- Source change: `./sim check` before running Gazebo.
- Fast behavioral regression: `./sim demo basic 300`.
- Costmap/planner/course acceptance: `./sim demo stress 420`.
- Existing run review: `./sim report`, then its `report.json` and video.

A demo timeout is wall time. On an ARM Mac, increase the limit rather than
reducing camera fidelity, obstacle inflation, or acceptance thresholds.

## Evidence discipline

Read `summary.txt` first. A run passes only when it says `PASS`; a generated MP4,
node log, or `All waypoints reached` line alone is insufficient. Confirm goal
distances, final distance, mission flag, contacts, sample count, and video.

Use `demo_short.mp4` for review and `demo.mp4` for timing. Check `playback.txt`
before discussing speed. Sample the raw video at multiple times before claiming
smoothness. The costmap panel is subscribed ROS data, not manually drawn ground
truth. Footprint analysis does not establish wheel/contact dynamics.

## Debug upstream to downstream

1. `gazebo.log` and `/drives/odom`: world, spawn, sensor, backend.
2. Camera age and `costmap.log`: perception input and cost generation.
3. `state.log` and `planner.log`: target, path, and replan decisions.
4. `controller.log`: autonomy `DriveCommand` output.
5. `mux.log`: active source, timeout, estop, selected `/cmd_vel`.
6. `drive.log` and odometry: Gazebo translation and resulting movement.

Fix the owner of the first broken boundary. Do not compensate downstream.
Shutdown-only `ExternalShutdownException` and duplicate-shutdown traces are
