# Terra
[![Build and test](https://github.com/CornellMarsRover/terra2/actions/workflows/build-test.yaml/badge.svg?branch=main)](https://github.com/CornellMarsRover/terra2/actions/workflows/build-test.yaml)

Terra contains all the code that is part of a ROS codebase and implements the brains of our devices. This main branch contains the current fully integrated and tested functionality for the rover.

## Current Functionality

### RoverNet: [cmr_rovernet](./src/cmr_rovernet/)
- **Description:** CMR controls stack (Jetson ↔ ECE) for arm control, drives control, and ECE feedback.

### Controller Input: [cmr_controller_remote](./src/cmr_controller_remote/)
- **Description:** PS5 controller input reception for data sent by team laptop over UDP.
- **Flow:** Controller → Laptop → Jetson → `cmr_controller_remote` → `cmr_rovernet`.

### Arm URDF Data: [cmr_arm_sim](./src/cmr_arm_sim/)
- **Description:** Hosts the Rover Arm's URDF for our Inverse Kinematics algorithms.
- **Additional:** Includes a launch file to visualize the arm: [display.launch.py](./src/cmr_arm_sim/launch).

### Arm Inverse Kinematics: [cmr_arm_simulator](./src/cmr_arm_simulator/)
- **Description:** Contains the ROS2 MoveIt configs and basic launch files to implement IK for the rover's arm.

### Arm PS5 Controller Control: [moveit_servo](./src/cmr_arm_simulator/)
- **Description:** Initializes the IK algorithm, takes controller data from `cmr_controller_remote`, and outputs arm joint positions/velocities to `cmr_rovernet`.

### Fabric Nodes: [cmr_fabric](./src/cmr_fabric/)
- **Description:** Custom nodes with better fault handling and configuration management.
- **Features:**
  - Launch ROS nodes using a config folder with TOML files.
  - Avoids manually launching each node.
  - Example: `rovernet.launch.py` and the config folder in `cmr_rovernet`.

### IMU Input: [cmr_imu](./src/cmr_imu)
- **Description:** Pulls data from WitMotion's HWT905 IMU and feeds it into an IMU topic for other packages.

### Custom Messages: [cmr_msgs](./src/cmr_msgs)
- **Description:** Contains and initializes custom message types for CMR-specific node-to-node communication.

### C++ Utility Functions: [cmr_utils](./src/cmr_utils)
- **Description:** Custom functions to simplify C++ ROS2 development (not used much anymore).

### Camera Integration: [cmr_cams](./src/cmr_cams)
- **Description:** Contains the functiaonlity for using the Rover's cameras

### GCS Bridge (Foxglove): [launch/gcs_bridge.launch.py](./launch/gcs_bridge.launch.py)
- **Description:** Starts the Foxglove WebSocket bridge (`foxglove_bridge`) so
  Foxglove Studio can connect to the rover. Used in place of `rosbridge_server`
  — lower latency and speaks Foxglove's native WebSocket protocol.
- **Install dep (Humble):** `sudo apt install ros-humble-foxglove-bridge`
- **Launch:** `ros2 launch launch/gcs_bridge.launch.py`
  - Optional args: `port:=8765 address:=0.0.0.0 tls:=false`
- **Connection URL from Foxglove Studio:** `ws://<rover_ip>:8765`
  (defaults; change with the `port` arg above).
- **QoS overrides:** camera / image / point-cloud / IMU / pose topics are
  forwarded with `BEST_EFFORT` and queue depth 1 so a stalled RF link cannot
  back-pressure the bridge. Command and service topics stay on reliable QoS.
  See the `BEST_EFFORT_TOPIC_REGEXES` list in the launch file.

### Astrotech rover node: [src/astrotech_rover](./src/astrotech_rover)
- **Description:** ROS 2 node for the Astrotech science payload. Runs the
  **real hardware drivers by default** (auger moteus stack, CMR mixing
  servo, TCD1340 Raman, SCD-30 environment); each can be swapped for a
  simulation driver via a `URC_*_MOCK=1` env var for GUI development
  without hardware. Interface contract:
  [`src/astrotech_rover/config/astrotech_interfaces.yaml`](./src/astrotech_rover/config/astrotech_interfaces.yaml).
- **Foxglove GUI** (panels + layouts) and the full operator/dev guide live
  under [`gui/`](./gui/) — start with [`gui/README.md`](./gui/README.md).

## Running / developing the Astrotech GUI

One-time setup:

```bash
colcon build --symlink-install --packages-select cmr_msgs astrotech_rover
source install/setup.bash
```

Real hardware (default — the normal rover command):

```bash
ros2 launch astrotech_rover astrotech.launch.py
```

No hardware (GUI dev / demo — mock any or all features):

```bash
URC_AUGER_MOCK=1 URC_MIXING_SERVO_MOCK=1 URC_RAMAN_MOCK=1 URC_ENV_MOCK=1 \
  ros2 launch astrotech_rover astrotech.launch.py
```

Both bring up `astrotech_node` plus the Foxglove bridge on
`ws://localhost:8765`. Then connect Foxglove Studio and import a layout
from [`gui/layouts/`](./gui/layouts/). First time, install the custom
panels extension (`cd gui/extensions/urc-astrotech-panels && npm install
&& npm run build && npm run local-install`, then relaunch Foxglove).

**Full launch / debug / camera / sensor-wiring guide:**
[`gui/operator_guide.md`](./gui/operator_guide.md). **System overview and
what's wired:** [`gui/README.md`](./gui/README.md).

> Camera *publishers* aren't part of `astrotech_rover`; they're launched
> rover-side from [`src/cmr_cams/`](./src/cmr_cams/) (cmr_cv nodes) plus the
> stereolabs `zed_wrapper`. Against a hardware-free dev launch the Foxglove
> Image panels render but show "no messages on topic" — expected. See
> the camera section of [`gui/operator_guide.md`](./gui/operator_guide.md).

A smoke test (build, launch, check topics/rates, call a service) is at
[`src/astrotech_rover/scripts/smoke_test.sh`](./src/astrotech_rover/scripts/smoke_test.sh); run from the repo root
on a ROS 2 Humble machine.

## Driving Quick Start

Run these short commands from the repository root. Use `./run help` at any time.

| Command | Run it on | Purpose |
| --- | --- | --- |
| `./run build` | Jetson or dev container | Build drive and autonomy packages |
| `./run teleop` | Jetson | Receive UDP input and drive the rover |
| `./run controller` | Controller laptop | Send DualSense input to the rover |
| `./run auto` | Jetson | Start hardware autonomy and shared rover driving |
| `./run sim` | Dev container | Start autonomy in simulated-input mode |
| `./run test` | Laptop or dev container | Run focused autonomy tests |

### First Build

Run `./run build` after cloning, switching branches, or changing ROS packages.
The launcher sources ROS 2 Humble and the workspace automatically.

### Hardware Tele-op

1. On the Jetson, run `./run teleop`.
2. Connect the DualSense to the laptop.
3. On the laptop, run `./run controller`.

Use `./run controller --list-controllers` to list controller IDs, then select one
with `./run controller --controller-serial SERIAL`. The tele-op shortcut
explicitly disables arm startup; it does not change any controller mappings.

### Hardware Autonomy

Run `./run auto` on the Jetson. This starts localization, perception, planning,
the controller, and the same RoverNet motor path used by tele-op. The controller
starts after the launch file's existing 30-second safety delay.

### Simulation

Run `./run sim` in the dev container. This starts the autonomy nodes in their
simulated-input mode, but it does not start Gazebo. Full Gazebo tooling remains
isolated on the `gazebo_sim` branch.

## Autonomy Testing Status

The `autonomy-fall2026` branch routes waypoint-following commands through
`/cmd_vel_drives`, the same RoverNet swerve and Moteus path used by teleop.
Teleop remains the manual input source and is not modified by autonomy.

Run the focused tests from the repository root in the CMR development image:

```bash
bash scripts/test_autonomy.sh
```

CI enforces 100% line and branch coverage for the extracted drive-command
mapping. This is not yet 100% coverage of every ROS autonomy node.

### Autonomy Test Roadmap

- [x] Unit-test normalized forward, steering, point-turn, and stop commands.
- [x] Build `cmr_msgs`, `cmr_rovernet`, and `autonomous_navigation` on Humble.
- [ ] Extract planner, costmap, and state-machine decisions into pure modules.
- [ ] Add deterministic tests for waypoint completion and replanning failures.
- [ ] Add recorded camera, point-cloud, GPS, and IMU fixture datasets.
- [ ] Add Gazebo collision, blocked-path, sensor-dropout, and timeout tests.
- [ ] Verify manual override, autonomy timeout, estop, and Moteus watchdog behavior.
- [ ] Run repeatable Jetson hardware-in-the-loop tests with wheels lifted first.
- [ ] Expand the coverage gate as each ROS node gains a testable core.

## TODO Functionality

- **Autonomous Navigation:** [autonomous_navigation](./src/autonomous_navigation/)
- **GPS Functionality** (inside of cmr_navigation in old main, gps.py)
- **Arm Autonomy**
- **ArUco Tag Detection**
- **Drives Simulation**
- **AruCo Tag Navigation (Spiral Algorithm)**
- **Bird's Eye View**
- **Obstacle Detection:** [cmr_obstacle_data](./src/cmr_obstacle_data/)
- **Drives GUI:** [cmr_param_gui](./src/cmr_param_gui/)
- **Improved camera integration**
