# Gazebo autonomy onboarding

This is the handoff guide for developing and validating rover driving and
obstacle avoidance without hardware. It covers driving only; do not modify arm
packages while following this guide.

## What you are responsible for

The simulator is useful only when it exercises the same autonomy path used by
the rover. Preserve this shared path:

```text
camera point cloud -> costmap -> local planner -> controller
-> drive command mux -> selected /cmd_vel
```

Gazebo may replace sensors, pose, and the motor backend. It must not replace the
production costmap, planner, controller, drive-command message, or mux with a
second simulation-only implementation.

The current Gazebo backend applies planar motion. It validates ROS integration,
obstacle perception, path planning, arbitration, and collision-free trajectories.
It does not validate steering-joint timing, wheel slip, traction, suspension, or
Moteus/CAN behavior.

## First run

Prerequisites: Git, Docker Desktop, and at least 15 GB of free disk space. On an
ARM Mac, Gazebo Classic runs through `linux/amd64` emulation and will be slower
than real time.

From the repository root:

```bash
./sim setup
./sim doctor
./sim check
./sim demo basic 600
./sim report
```

What to expect:

1. `setup` builds `terra-dev:cycle` from the public OSRF ROS Humble image.
2. `doctor` prints paths for `gzserver`, `gzclient`, `Xvfb`, `ffmpeg`, and `ros2`.
3. `check` runs autonomy tests, Gazebo harness tests, and a focused ROS build.
4. `demo` prints a session folder and a live `http://localhost:PORT` dashboard.
5. `report` prints `PASS` only if every goal was reached, the final pose is within
   1 m, no footprint contacts were recorded, mission completion was logged, and
   a valid video exists.
