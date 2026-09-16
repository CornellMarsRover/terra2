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

A nonzero `demo` exit is an acceptance failure, not necessarily a crashed tool.
The run still keeps its logs and video for diagnosis.

## Normal development loop

Use this order after each focused change:

```bash
./sim check
./sim demo basic 300
./sim report
./sim demo stress 420
./sim report
```

The basic course is the fast regression. The stress course adds a cylinder, low
curb, angled wall, staggered gate, concave pocket, and sphere. Run stress only
after basic passes. If the machine is slow, increase the wall-clock limit rather
than weakening sensors, collision margins, or acceptance thresholds.

Follow one-change-at-a-time testing:

1. Save the baseline `summary.txt`, `report.json`, and video.
2. State one hypothesis and change the owning layer only.
3. Run unit tests before Gazebo.
4. Run basic before stress.
5. Compare goal distances, contacts, path length, and simulated duration.
6. Inspect the video and dashboard; metrics alone do not prove visual behavior.
7. Keep a change only when evidence improves and no earlier check regresses.

## Where to make changes

| Need | Primary location |
| --- | --- |
| Cost decay, projection, obstacle cells | `src/autonomous_navigation/autonomous_navigation/costmap*.py` |
| A*, inflation, path validation | `src/autonomous_navigation/autonomous_navigation/local_planner.py`, `planner_core.py` |
| Waypoint mission progression | `state_machine.py`, `state_machine_core.py` |
| Rover command generation | `controller.py`, `drive_command.py` |
| Source selection, timeout, estop | `src/cmr_rovernet/cmr_rovernet/command_mux*.py` |
| Course geometry | `validation/gazebo/assets/*.world` |
| Sim mission coordinates | `validation/gazebo/assets/course_waypoints.yaml` |
| Rover mesh, depth camera | `validation/gazebo/assets/rover_depth.urdf` |
| Gazebo pose/drive boundaries | `validation/gazebo/adapters/` |
| Run orchestration and evidence | `sim`, `validation/gazebo/cycle.sh` |

Do not put navigation decisions in an adapter. Adapters translate interfaces;
production nodes decide where and how the rover moves.

## Read evidence in this order
