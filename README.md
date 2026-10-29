# Terra2
[![Build and test](https://github.com/CornellMarsRover/terra2/actions/workflows/build-test.yaml/badge.svg?branch=main)](https://github.com/CornellMarsRover/terra2/actions/workflows/build-test.yaml)

Terra2 contains all the code that is part of a ROS codebase and implements the brains of our devices. This main branch contains the current fully integrated and tested functionality for the rover.

## Astronaut Mission Simulation

The astronaut autonomy package includes a hardware-independent closed-loop
mission simulator. It models rover and astronaut poses, derives range and
bearing from those poses, applies the FOLLOW/COME velocity output to the mock
rover, and runs the `FOLLOW -> STAY -> FETCH -> COME -> GIVE` sequence. Physical
motor output stays disabled.

Run it directly on macOS without ROS:

```bash
./run astronaut-pure-sim
```

Results are written to `artifacts/astronaut_sim/report.json` and
`artifacts/astronaut_sim/trajectory.csv`.

For a real ROS 2 Humble build and topic-level integration test, install and
start Docker Desktop, then run:

```bash
./scripts/astronaut_docker.sh build
./scripts/astronaut_docker.sh test
```

The image is based on Ubuntu 22.04 and ROS 2 Humble. It builds only `cmr_msgs`
and `astronaut_autonomy`, verifies the generated messages import, starts the
mission and mock-rover nodes, checks topic types, and validates the final
simulation report. See
[`src/astronaut_autonomy/README.md`](./src/astronaut_autonomy/README.md) for
architecture, topic inspection, configuration, and remaining hardware tests.

## Mac/ZED Astronaut Gesture Test

[`gesture_test.py`](./gesture_test.py) is a standalone, perception-only
prototype. It uses OpenCV and MediaPipe Pose for gestures and can optionally use
offline Vosk speech recognition for the same five commands. It can also use the
official Stereolabs ZED SDK for synchronized metric depth. It does not require
ROS, networking, motor control, or other rover hardware.

### Recognized gestures

| Command | Pose |
| --- | --- |
| `FOLLOW` | Stand upright with both hands on the corresponding hips and elbows out |
| `STAY` | Extend both arms horizontally in a T-pose |
| `FETCH` | Keep the left hand on the neck and point with the extended right arm |
| `COME` | Hold both hands together above the face with relaxed, roof-shaped arms |
| `GIVE` | Cross both forearms over the chest in an X |

The camera window shows the skeleton, stable command, astronaut presence,
distance, camera-relative direction, X/Y/Z position, and depth status. FETCH
also shows whether the right arm points left, right, or forward in the mirrored
display. The terminal prints a tracking report at 2 Hz by default.
When voice input is enabled, the window shows the gesture, spoken command, and
selected command source. A recent accepted voice command takes priority for two
seconds, then control returns to the stable gesture.

### Requirements

- Mac development: Python 3.11
- macOS webcam development: macOS 11 or newer and camera permission for the
  application that starts Python
- Real depth deployment: a supported NVIDIA Jetson/Linux host, a Python version
  supported by its JetPack/ZED SDK combination, the SDK's `pyzed.sl` API, and a
  connected ZED camera

The direct Python dependencies are pinned in
[`requirements-gesture-test.txt`](./requirements-gesture-test.txt):

- MediaPipe 0.10.21
- OpenCV Contrib Python 4.11.0.86
- NumPy 1.26.4
- PyObjC AVFoundation 12.2.2
- SoundDevice 0.5.6
- Vosk 0.3.44

`pyzed.sl` is intentionally not a pip requirement. It is supplied by the ZED
SDK on supported systems and is imported only when `--zed` is requested.

### Architecture

- [`gesture_test.py`](./gesture_test.py) owns camera-mode selection, the existing
  MediaPipe gesture rules, visualization, and rate-limited terminal reports.
- [`zed_depth.py`](./zed_depth.py) lazily wraps official `pyzed.sl` synchronized
  capture and provides the explicitly simulated mock source.
- [`astronaut_tracker.py`](./astronaut_tracker.py) maps torso landmarks into the
  aligned XYZ image, filters depth, calculates 3D position/direction, smooths
  valid measurements, and handles stale tracking.
- [`voice_recognizer.py`](./voice_recognizer.py) captures microphone audio in a
  background thread and uses an offline, five-command Vosk grammar.

### Install

From the repository root:

```bash
brew install python@3.11
python3.11 -m venv .venv-gesture
source .venv-gesture/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-gesture-test.txt
```

When macOS prompts for camera or microphone access, allow it. These settings can
also be changed under **System Settings > Privacy & Security > Camera** and
**Microphone**.

### Add offline voice commands

Download and extract the small English Vosk model once. The model is kept out
of Git because it is a large generated/runtime asset:

```bash
mkdir -p models
curl -LO https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip -d models
```

List available microphones:

```bash
python gesture_test.py --list-audio-devices
```

Run the Mac camera with gestures and offline voice recognition together:

```bash
python gesture_test.py --no-zed --camera 0 --voice \
  --voice-model models/vosk-model-small-en-us-0.15
```

Say `follow`, `stay`, `fetch`, `come`, or `give`. A successful spoken command
prints output such as `Detected: COME (VOICE)`. Speech recognition runs locally;
no cloud service or network connection is used after the model is downloaded.
Use `--audio-device INDEX` to select a microphone shown by the listing command.
If the model, dependency, or microphone cannot open, `--voice` exits with a
clear error instead of silently continuing without voice input.

### Run with the Mac camera (gesture only)

This preserves the built-in Mac camera workflow and does not report distance:

```bash
source .venv-gesture/bin/activate
python gesture_test.py --no-zed
```

The program auto-detects the FaceTime HD Camera and prints the verified OpenCV
index, resolution, and frame rate. Depth status is `DISABLED`, and distance is
always `UNAVAILABLE` rather than estimated from body size.

### Test the depth pipeline on a Mac (simulated)

Mock mode uses the selected webcam and exercises torso mapping, smoothing,
display, stale tracking, and terminal output with an explicitly simulated map:

```bash
source .venv-gesture/bin/activate
python gesture_test.py --mock-depth --mock-distance 3.0
```

The overlay and terminal use `Depth Status: SIMULATED`. Mock measurements are
never labeled `VALID` and are never selected silently.

### Run with real ZED depth on a Jetson/Linux host

Install the official ZED SDK and its Python API for the Jetson, connect the ZED,
and install a compatible MediaPipe 0.10.x ARM64 build that exposes
`mediapipe.solutions.pose`. The standard MediaPipe PyPI release does not provide
the required Jetson/Linux ARM64 wheel, so that package is skipped by this
repository's requirements file on `aarch64`. Then verify all three native
bindings before launching:

```bash
sudo apt install libportaudio2
python3 -m venv --system-site-packages .venv-gesture
source .venv-gesture/bin/activate
python -m pip install -r requirements-gesture-test.txt
python -c "import cv2, mediapipe, pyzed.sl; print('Perception APIs ready')"
python gesture_test.py --zed
```

Add `--voice --voice-model models/vosk-model-small-en-us-0.15` to the Jetson
command to listen for spoken commands through its selected input microphone.

If `import mediapipe` fails, install or build a JetPack/Python-compatible ARM64
MediaPipe wheel before running this application. Do not substitute mock depth
for hardware validation.

`--zed` opens `sl.Camera`, retrieves `sl.VIEW.LEFT`, `sl.MEASURE.XYZ`, and
`sl.MEASURE.CONFIDENCE` from each synchronized grab, and configures image
coordinates in meters. MediaPipe torso landmarks are mapped into the aligned
XYZ image; the displayed position is the filtered median of valid torso
samples. The default camera setup is HD720 at 30 FPS with NEURAL depth.

If `pyzed.sl` is absent, the camera cannot open, or the host is macOS, `--zed`
exits with a clear error. It does not fall back to a webcam or mock values.

List cameras and their advertised formats without opening one:

```bash
python gesture_test.py --list-cameras
```

Preview the SDK camera without starting MediaPipe:

```bash
python gesture_test.py --zed --preview
```

To explicitly open an OpenCV index as a normal single-image camera:

```bash
python gesture_test.py --no-zed --camera 0
```

Press `Q` in the camera window to quit. Keep the shoulders, arms, and hips
visible for the most reliable results.

### Tuning

The default landmark/gesture confidence is `0.65`:

```bash
python gesture_test.py --confidence 0.70
```

FETCH has three adjustable normalized thresholds:

```bash
python gesture_test.py \
  --fetch-min-arm-angle 105 \
  --fetch-min-extension 0.75 \
  --fetch-neck-radius 0.55
```

- `--fetch-min-arm-angle` controls how straight the right elbow must be.
- `--fetch-min-extension` is the shoulder-to-wrist distance measured in
  shoulder widths.
- `--fetch-neck-radius` is the allowed normalized distance between the left
  wrist and the estimated neck position.

COME's hand separation and elbow-height tolerances are also adjustable:

```bash
python gesture_test.py \
  --come-hand-gap 0.35 \
  --come-elbow-tolerance 0.25
```

- `--come-hand-gap` is the maximum wrist-to-wrist distance in shoulder widths.
- `--come-elbow-tolerance` is how far either elbow may sit below its shoulder,
  measured in torso heights.

Use `python gesture_test.py --help` to see every option.

Voice controls include:

```bash
python gesture_test.py --no-zed --voice \
  --voice-confidence 0.50 \
  --voice-input-gain 1.75 \
  --voice-hold-seconds 2.0 \
  --voice-cooldown-seconds 1.0
```

The constrained vocabulary accepts the five command words and a few natural
variants such as `follow me`, `stay there`, `go fetch`, `come here`, and
`give it`. It ignores partial recognition and only acts on finalized speech
above the confidence threshold. Audio is processed in smaller low-latency
blocks, and the default 1.75x software gain helps with quiet microphones.

The live `Voice Status` includes the microphone level in dBFS. If speech stays
very quiet, try `--voice-input-gain 2.5`. If clear speech is heard but rejected,
try `--voice-confidence 0.40`. Higher gain and lower confidence increase
sensitivity but can also increase false detections in a noisy room.

Depth and reporting controls include:

```bash
python gesture_test.py --zed \
  --output-rate 2 \
  --depth-min 0.2 \
  --depth-max 20 \
  --depth-confidence 80 \
  --depth-smoothing 0.35 \
  --depth-stale-seconds 1.0
```

Run the standalone tests on the Mac without a ZED SDK installation:

```bash
source .venv-gesture/bin/activate
python -m unittest discover -s tests -v
```

### Notes and troubleshooting

- The final video is mirrored for an intuitive selfie view, but pose inference
  runs on the unmirrored frame so anatomical left/right landmarks stay correct.
- `FOLLOW` requires both wrists near their corresponding hips, both elbows
  pointing outward, and an approximately upright torso. It uses MediaPipe world
  landmarks for the upright check when available and falls back to the 2-D
  torso axis otherwise.
- If the camera cannot open, close other programs using it, verify macOS camera
  permission, and inspect `--list-cameras` plus the automatic probe output.
- OpenCV's AVFoundation indices may not match AVFoundation's native list order.
  Trust the verified `Selected OpenCV[...]` line rather than assuming index 0
  or 1 belongs to a particular camera.
- A ZED opened through OpenCV/UVC provides images, not calibrated SDK metric
  depth. This application therefore reserves `--zed` for `pyzed.sl` and never
  presents UVC capture as real distance.
- `VALID` means filtered SDK XYZ samples. `SIMULATED` means explicit mock mode,
  `STALE` means the last valid estimate is being held briefly, and
  `UNAVAILABLE` means no reliable measurement exists.
- Bright, even lighting and a full-body view reduce landmark jitter.

### Run the ROS 2 ZED integration on the rover

The ROS implementation is in
[`src/astronaut_autonomy`](./src/astronaut_autonomy/README.md). It deliberately
does not enable the microphone. Its mission controller publishes safe drive
proposals, while the adapter to the rover's shared drive topic remains disabled
by default. On the Jetson, build the workspace, start the one ZED owner, and
start astronaut perception:

```bash
./run setup
./run gesture-setup
./run build
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cmr_zed threaded
```

In a second terminal:

```bash
./run gesture
```

In a third terminal, start the mission state machine without physical drive
output:

```bash
./run astronaut-mission
```

This launch also starts the gesture/voice command-fusion node. Offline Vosk
voice input is opt-in and can use either a microphone or prerecorded PCM16 WAV:

```bash
./run voice-setup
./run astronaut-mission enable_voice:=true \
  voice_model_path:=/absolute/path/to/vosk-model-small-en-us-0.15
```

Gesture FOLLOW is primary when voice is unavailable, STAY has safety priority,
and any confident cross-modal conflict produces a tagged immediate safety hold.
FOLLOW and COME still require fresh ZED target localization. FETCH recognition
is intentionally independent of astronaut depth because the arm must localize
the nearby hammer separately. See `src/astronaut_autonomy/README.md` for WAV
mode, configuration, and tests.

In another sourced ROS terminal, inspect the combined output:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /autonomy/astronaut/perception
ros2 topic echo /autonomy/astronaut/mission_state
ros2 topic echo /autonomy/astronaut/drive_command
```

The node synchronizes `/zed/image` with `/camera/points`, recognizes all five
gestures, and publishes ZED-frame position, range, and bearing. FOLLOW is
emitted in the message's safe `command` field only while current torso depth is
valid; COME is checked again by the mission controller. The mission node provides debounced IDLE,
FOLLOWING, STAYING, FETCHING, COMING, GIVING, and ERROR behavior. See the
package README for hardware-free simulation, tests, topics, safety behavior,
and remaining rover integration work.

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
| `./run setup` | Jetson or dev container | Install ROS dependencies once |
| `./run build` | Jetson or dev container | Build drive and autonomy packages |
| `./run teleop` | Jetson | Receive UDP input and drive the rover |
| `./run controller` | Controller laptop | Send DualSense input to the rover |
| `./run auto` | Jetson | Start hardware autonomy and shared rover driving |
| `./run astronaut-mission` | Jetson or dev container | Start safe-by-default astronaut mission logic |
| `./run astronaut-sim` | ROS dev container | Simulate all astronaut commands without hardware |
| `./run sim` | Dev container | Start autonomy in simulated-input mode |
| `./run test` | Laptop or dev container | Run focused autonomy tests |

### First Build

Run `./run setup` once after cloning. Run `./run build` after setup, switching branches, or changing ROS packages.
The launcher sources ROS 2 Humble and the workspace automatically. For tests,
install missing tools with `python3 -m pip install coverage pytest`.

### Hardware Tele-op

On the controller laptop, run `./run controller setup` once.

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
simulated-input mode, but it does not start Gazebo. Use `./sim` for the full Docker/Gazebo harness; see the
[Gazebo onboarding guide](docs/gazebo-onboarding.md).

## Architecture Maps

- [Development workflow](docs/development-workflow.md)
- [Codebase map](docs/codebase-map.md)
- [ROS node and topic structure](docs/ros-structure.md)
- [Autonomy architecture](docs/autonomy-architecture.md)

## Autonomy Testing Status

The `autonomy_fall2026` drive path uses these topics:

- `/controller/drives/axes` and `/controller/drives/buttons`: raw controller data
- `/cmd_vel/teleop`: converted manual `cmr_msgs/DriveCommand`
- `/cmd_vel/autonomy`: waypoint controller `cmr_msgs/DriveCommand`
- `/cmd_vel`: the only selected command accepted by the drive backend

`/cmd_vel` is reserved for drive commands. The legacy arm IK keyboard utilities
must be remapped before running in the same ROS domain because they use `Twist`.

The command mux enforces a 0.5-second input timeout and `/cmd_vel/estop`. Launch
mode selects the initial source; switch a running mux with:

```bash
ros2 topic pub --once /cmd_vel/source std_msgs/msg/String '{data: autonomy}'
```

Run the focused tests from the repository root in the CMR development image:

```bash
bash scripts/test_autonomy.sh
```

CI enforces 100% line and branch coverage for the drive-command mapping and
the extracted planner, costmap, and state-machine decisions. ROS adapter nodes
still require integration coverage.

Software-only safeguards covered by this gate include deterministic selection
of a clear local goal, consecutive-frame path invalidation, and stopping inside
the configurable `waypoint_tolerance`. Static analysis is incremental: only C++
files changed by a commit or pull request are formatted and checked with
`clang-tidy`.

### Autonomy Test Roadmap

With Gazebo, the drive mux, and its bridge running, use
`scripts/check_drive_sim.sh` to compare both sources and verify estop motion.

- [x] Unit-test normalized forward, steering, point-turn, and stop commands.
- [x] Build `cmr_msgs`, `cmr_rovernet`, and `autonomous_navigation` on Humble.
- [x] Extract planner, costmap, and state-machine decisions into pure modules.
- [x] Add deterministic tests for waypoint completion and replanning failures.
- [x] Run incremental C++ formatting and static analysis in CI.
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
- **Obstacle Detection and Avoidance:** [autonomous_navigation](./src/autonomous_navigation/)
- **Drives GUI:** [cmr_param_gui](./src/cmr_param_gui/)
- **Improved camera integration**

### Docker and Gazebo development

Start with the [Gazebo autonomy onboarding guide](docs/gazebo-onboarding.md).
It gives the first-run commands, shared ROS boundary, evidence contract,
debugging table, normal iteration loop, and handoff checklist. Harness internals
and generated-file meanings are in the [Gazebo reference](validation/gazebo/README.md).

[Run a Gazebo demo with GitHub Actions](https://github.com/CornellMarsRover/terra2/actions/workflows/gazebo-demo.yaml)
(builds its own Docker container and uploads the video; see the Gazebo guide for access and setup).
