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

Each run is under `validation/gazebo/logs/run_<timestamp>_<pid>/`.

1. `summary.txt`: acceptance result and final metrics.
2. `report.json`: per-obstacle contacts/clearances and recorded trajectory data.
3. `demo_short.mp4`: approximately one-minute accelerated visual review.
4. `demo.mp4`: unaltered wall-clock recording for timing and pauses.
5. `playback.txt`: acceleration applied to the short video.
6. `state.log`: mission target selection and completion.
7. `planner.log`: path generation, blocked segments, and replans.
8. `costmap.log`: sensor ingestion and obstacle-map failures.
9. `controller.log`, `mux.log`, `drive.log`: command generation and selection.
10. `gazebo.log`, `odom.csv`: spawn/sensor/backend status and ground-truth motion.
11. `telemetry.jsonl`: costmap, path, and target messages shown by the dashboard.
12. `revision.txt`, `changes.txt`, `world.sdf`: exact source/world provenance.

`demo_short.mp4` is presentation material. Never use its apparent speed to make
a real-time control claim. Shutdown-only `ExternalShutdownException` or
`rcl_shutdown already called` traces are noisy cleanup behavior; a traceback
before motion stops is a runtime failure and must be investigated.

## Failure routing

| Symptom | Check first | Likely layer |
| --- | --- | --- |
| `Start Docker Desktop, then retry` | `docker info` | Host setup |
| Missing image | `./sim setup`, then `./sim doctor` | Container setup |
| No odometry in 90 seconds | `gazebo.log`, spawn lines, `/drives/odom` | Gazebo/model/backend |
| Camera says `WAIT` | camera plugin lines in `gazebo.log` | URDF sensor/plugin |
| Camera live, costs absent | `costmap.log`, `/camera/points` | Costmap/input contract |
| Costs live, path absent | `planner.log`, target entries in `state.log` | Planner/mission target |
| Path live, rover stationary | `controller.log`, `mux.log`, `drive.log` | Control/arbitration/backend |
| Rover moves through blocks | `report.json`, costmap view, footprint assumptions | Perception/costmap/model |
| Rover loops or stalls | repeated replans in `planner.log`, path overlay | Planner or stale costs |
| Video exists but summary fails | goal distances, contacts, mission flag | Navigation acceptance |
| Demo looks frozen on ARM Mac | compare sim seconds with wall time | amd64 GUI emulation |

Do not fix a downstream symptom by bypassing an upstream contract. Trace one
message boundary at a time from sensor to `/drives/odom`.

## Changing a course safely

1. Copy an existing `.world` file and give every obstacle a unique model name.
2. Keep visible and collision geometry aligned.
3. Update `course_waypoints.yaml` only when the mission must change.
4. Extend `analyze_course.py` if the new geometry is not represented in its
   collision/clearance calculations.
5. Run `xmllint --noout` on XML assets and `./sim check`.
6. Record a baseline with the old course and an acceptance run with the new one.
7. Document obstacle shapes, goal coordinates, and known blind spots.

A visually difficult course is not useful unless the goal forces interaction
with its obstacles and the analyzer measures those obstacles.

## Definition of done

A Gazebo change is ready for review when:

- `./sim check` passes.
- Basic regression passes, or the failure is explicitly explained.
- Stress acceptance passes for planner/costmap changes.
- `summary.txt` and `report.json` agree with the visual result.
- The live dashboard showed current camera, costmap, and path data.
- The raw video was sampled for pauses or misleading frames.
- Simulation-only code remains at a sensor, pose, or drive boundary.
- Known hardware gaps are stated rather than implied to be tested.
- Generated logs and videos remain ignored by Git.
- The PR describes the exact course, command, runtime limit, and result folder.

## Handoff template

Leave this in a PR or issue when another developer continues the work:

```text
Branch/commit:
Hypothesis/change:
Commands run:
Basic result:
Stress result:
Session folder(s):
Observed regression or limitation:
Next smallest experiment:
```

For harness internals and the topic table, read
[`validation/gazebo/README.md`](../validation/gazebo/README.md). For the complete
production control flow, read [`docs/ros-structure.md`](ros-structure.md).
