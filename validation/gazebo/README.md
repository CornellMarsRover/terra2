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
| --- | --- | --- |
| `./sim setup` | Docker build from `osrf/ros:humble-desktop-jammy` | `terra-dev:cycle` image |
| `./sim doctor` | Dependency probe | executable paths or nonzero exit |
| `./sim check` | 120 autonomy tests, harness tests, focused `colcon` build | session test/build logs |
| `./sim demo basic N` | Three-block world, complete ROS stack, recorder | session evidence |
| `./sim demo stress N` | Mixed-shape course, complete ROS stack, recorder | session evidence |
| `./sim report` | Newest `summary.txt` | concise acceptance result |

The test count may increase; the command result, not this table, is authoritative.
Only `setup` downloads packages. The remaining commands use the local image.
`SIM_IMAGE=name` selects another compatible image.

## Courses and mission

- `obstacle_course.world`: small three-block regression.
- `stress_course.world`: cylinder, curb, angled wall, gate, concave pocket, and
  sphere.
- `course_waypoints.yaml`: start plus local targets near `(5,0)`, `(10,10)`, and
  `(15,15)`, each with a 1 m threshold.

The stress goal lies beyond the obstacle field so the rover must interact with
the course. `analyze_course.py` evaluates the recorded footprint against the
world geometry and computes goal distances and path length.

## Acceptance contract

`summarize.py` returns success only when all of these are true:

- every configured goal came within 1 m;
- final pose is within 1 m of `(15,15)`;
- `state.log` contains `All waypoints reached.`;
- obstacle footprint intersection samples equal zero;
- more than ten odometry samples exist; and
- `demo.mp4` is valid and longer than one second.

The summary explicitly says `PASS` or `INCOMPLETE/FAIL`. A video by itself is not
a passing test.

## Session contents

| File | Meaning |
| --- | --- |
| `summary.txt` | Human-readable acceptance decision |
| `report.json` | Trajectory, goals, obstacle contacts, and clearances |
| `demo.mp4` | Unaltered 1280x720 Gazebo/dashboard recording |
| `demo_short.mp4` | Accelerated review copy capped near one minute |
| `playback.txt` | Raw duration, short duration, and speed factor |
| `telemetry.jsonl` | Costmap, planned path, and target messages |
| `odom.csv` | Recorded Gazebo odometry |
| `state.log` | Mission progression and completion |
| `costmap.log` | Perception-to-costmap node output |
| `planner.log` | Planning, validation, and replan decisions |
| `controller.log` | Autonomy command generation |
| `mux.log` | Source selection, timeout, and estop status |
| `drive.log` | Selected command translated for Gazebo |
| `gazebo.log` | World, model, plugins, camera, and backend output |
| `world.sdf` | Exact world copied into the run |
| `revision.txt`, `changes.txt` | Source provenance at run start |

Read `summary.txt` first. Use the owning log from the table instead of searching
all output blindly.

## Live view and video

A demo prints a localhost URL, starting at port 8765 and selecting the next free
port. The dashboard shows camera, cost cells, planned path, actual trajectory,
and topic ages. `WAIT` means that topic has not arrived.

`demo_short.mp4` changes playback speed only. Use `demo.mp4` to reason about
pauses or timing. On Apple Silicon, the required Gazebo Classic amd64 container
is emulated and can run far below real time.

## Manual two-terminal mode

Use this only when interactively inspecting ROS topics inside an already built
ROS/Gazebo development container:

```bash
# Terminal 1
./validation/gazebo/start_sim.sh

# Terminal 2
./validation/gazebo/start_autonomy.sh
```

Both terminals must use the same `ROS_DOMAIN_ID` (default 96) and sourced
workspace. This mode does not automatically record or score a run; prefer
`./sim demo` for repeatable evidence.

## GitHub-hosted run

Maintainers can use
[Run Gazebo demo](https://github.com/CornellMarsRover/terra2/actions/workflows/gazebo-demo.yaml),
choose a course and wall-time limit, then download the seven-day artifact. The
workflow runs `setup`, `check`, and `demo` through the same `./sim` interface.
GitHub requires the workflow on the default branch and write access for the Run
button. Other developers can fork the repository and enable Actions in the fork.

## Known limitations

## Acceptance result

The run reached two coordinate goals and detected all three blocks in the costmap.
Across 1,044 poses it had zero footprint intersections; center, right, and left
clearances were 0.601 m, 0.432 m, and 0.833 m.
