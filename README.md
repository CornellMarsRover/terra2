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

### Astrotech Mock Rover: [src/urc_mock_rover](./src/urc_mock_rover)
- **Description:** Fake rover node that publishes every Astrotech GCS
  topic/service/action described in
  [`src/urc_mock_rover/config/astrotech_interfaces.yaml`](./src/urc_mock_rover/config/astrotech_interfaces.yaml).
  Lets GCS development happen with no real hardware.
- **Assumptions and Q-number map:** [docs/phase2a_assumptions.md](./docs/phase2a_assumptions.md).

## Developing the Astrotech GCS without the rover

One-time setup:

```bash
# 1. Build the workspace (only the GCS-relevant packages are strictly needed).
colcon build --symlink-install --packages-select cmr_msgs urc_mock_rover

# 2. Generate the sample H.264 video the mock cameras loop over.
#    Requires ffmpeg on PATH.
./scripts/fetch_sample_video.sh
```

Then, each dev session:

```bash
source install/setup.bash
ros2 launch urc_mock_rover mock.launch.py
```

That single command starts the mock rover plus the Foxglove bridge on
`ws://localhost:8765`. In Foxglove Studio:

1. **Open Connection** → `ws://localhost:8765`.
2. **Layouts → Import from file** →
   [`gcs/layouts/urc_astrotech_dashboard.json`](./gcs/layouts/urc_astrotech_dashboard.json).
3. (First time only) install the custom panels extension:

```bash
cd gcs/extensions/urc-astrotech-panels
npm install
npm run build
npm run local-install
# Fully quit + relaunch Foxglove Studio for panels to appear.
```

A smoke test that builds, launches, checks expected topics and rates, and
calls a service is at [`scripts/smoke_test.sh`](./scripts/smoke_test.sh).
Run from the repo root on a ROS 2 Humble machine.

### When `astrotech-q-N` is answered — files to change

Every assumption currently baked into the mock is tagged in source with
`TODO(astrotech-q-N)`. When the team gives an answer, grep the tag and
update these files in lockstep. (`docs/phase2a_assumptions.md` holds the
question text; repeated here only as a lookup table.)

| Q | What to change |
|---|---|
| Q1 — auger controller | `docs/phase2a_assumptions.md`; `src/urc_mock_rover/config/astrotech_interfaces.yaml` (`auger:`); `src/urc_mock_rover/urc_mock_rover/drivers/auger.py`; `src/cmr_msgs/msg/AugerState.msg`; `gcs/extensions/urc-astrotech-panels/src/interfaces.ts` (`Auger`); `gcs/extensions/urc-astrotech-panels/src/panels/AugerControl.tsx` |
| Q2 — mixing servo controller | `astrotech_interfaces.yaml` (`mixing_servo:`); `drivers/mixing_servo.py`; `interfaces.ts` (`MixingServo`); `panels/MixingServo.tsx` |
| Q3 — BDC sequence controller, duration, step names | `astrotech_interfaces.yaml` (`analysis.mock_duration_sec`); `drivers/analysis_sequencer.py` (`_PHASES`) |
| Q4 — `sequence_id` vs. `site_num` naming | `src/cmr_msgs/action/RunAnalysisSequence.action`; `drivers/analysis_sequencer.py`; `interfaces.ts` (`Analysis`); `panels/AnalysisSequence.tsx` (button labels) |
| Q5 — Raman driver message type | `astrotech_interfaces.yaml` (`raman.type`); `src/cmr_msgs/msg/RamanSpectrum.msg` (delete if replaced); `drivers/raman.py`; `interfaces.ts` (`Raman`); `panels/RamanSpectrum.tsx` |
| Q6 — CO2/humidity driver message type | same shape as Q5: `env:` YAML block, `EnvSample.msg`, `drivers/env.py`, `Env` in `interfaces.ts` |
| Q9 — camera id → logical role mapping | `astrotech_interfaces.yaml` (`cameras.feeds`); `interfaces.ts` (`Cameras`); `gcs/layouts/urc_astrotech_dashboard.json` (Image panel `imageTopic` fields) |

Q7, Q8, Q11, Q12, Q13 are intentionally deferred per Phase 1 — see
`docs/open_questions.md`.

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
