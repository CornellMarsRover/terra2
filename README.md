# Terra
[![Build and test](https://github.com/CornellMarsRover/terra2/actions/workflows/build-test.yaml/badge.svg?branch=main)](https://github.com/CornellMarsRover/terra2/actions/workflows/build-test.yaml)

Terra contains all the code that is part of a ROS codebase and implements the brains of our devices. This main branch contains the current fully integrated and tested functionality for the rover.

## Branch Status for Contributors and AI Agents

New work should normally branch from the latest `origin/main`. Do not use an inactive branch as a base unless a maintainer explicitly asks you to resume that work. Do not delete, merge, rebase onto, or broadly cherry-pick an inactive branch without explicit maintainer approval; these branches are retained for recovery and historical reference.

Status was reviewed on **2026-09-06**. A non-default branch is considered inactive when it has had no commits in the previous 90 days. GitHub does not provide a manual inactive state, so this section is the repository's source of truth. Before relying on this list after the review date, check the remote branch and open pull requests again.

### Current Branches

| Branch | Last commit | Status |
| --- | --- | --- |
| `main` | 2026-05-30 | Canonical base for new work; exempt from inactivity based on age because it is the default branch. |
| `anant/sim-bugfix-2026-07-07` | 2026-07-07 | Active at the time of review. |

### Inactive Branches

| Branch | Last commit | Notes |
| --- | --- | --- |
| `Ishaan-Combine-IK-Keyboard` | 2026-04-28 | Inactive. |
| `anant-test-driving` | 2026-04-24 | Inactive. |
| `astrotech-gui` | 2026-05-27 | Inactive. |
| `astrotech-twocan` | 2026-05-30 | Inactive. |
| `autonomy_tools` | 2026-04-12 | Inactive. |
| `autonomy_usama_testing` | 2026-04-16 | Inactive. |
| `autonomy` | 2026-05-27 | Inactive, but retained by open PR #2 into `main`. |
| `backup_comp_5_26` | 2026-05-27 | Inactive. |
| `competition` | 2026-05-28 | Inactive, but retained by open PR #24 into `main`. |
| `devon/IK_2026` | 2026-05-07 | Inactive. |
| `drive_controll_debug_usama` | 2026-04-18 | Inactive. |
| `feature/autonomy-obstacle-avoidance-cv` | 2026-05-04 | Inactive. |
| `gazebo_sim` | 2026-04-22 | Inactive. |
| `ik-mostly-working-2026-04-28` | 2026-05-27 | Inactive. |
| `integrate-ik-arm-controller` | 2026-05-27 | Inactive. |
| `local_controller_changes` | 2026-05-29 | Inactive. |
| `object-detection-search-detect` | 2026-05-27 | Inactive, but retained by open PR #23 into `autonomy`. |
| `testing-aruco` | 2026-04-28 | Inactive, but retained by open PR #4 into `autonomy`. |
| `usama-drive-auto` | 2026-04-27 | Inactive. |

If an inactive branch must be resumed, first compare it with the latest `origin/main`, identify the smallest changes still needed, and create a new feature branch from `main` unless a maintainer specifies another base.

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

## TODO Functionality

- **Autonomous Navigation:** [cmr_navigation](./src/cmr_navigation/)
- **GPS Functionality** (inside of cmr_navigation in old main, gps.py)
- **Arm Autonomy**
- **ArUco Tag Detection**
- **Drives Simulation**
- **AruCo Tag Navigation (Spiral Algorithm)**
- **Bird's Eye View**
- **Obstacle Detection:** [cmr_obstacle_data](./src/cmr_obstacle_data/)
- **Drives GUI:** [cmr_param_gui](./src/cmr_param_gui/)
- **Improved camera integration**
