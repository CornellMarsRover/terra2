# Astronaut Autonomy

This ROS 2 package contains hardware-separated astronaut perception, offline
voice fallback, command fusion, and mission logic. Gesture perception consumes
synchronized images and organized XYZ point clouds from `cmr_zed`. A modular
Vosk source recognizes the same five commands from a microphone or PCM16 WAV.
The fusion node validates source, confidence, freshness, duplicates, and
cross-modal conflicts before the existing mission node applies state
transitions and calculates bounded FOLLOW/COME drive proposals.

## System overview

The implemented ROS data path is:

```text
ZED camera node (`threaded`)
        |
        | /zed/image + /camera/points
        v
Human gesture detection node
        | /autonomy/astronaut/gesture_perception
        |
        +--------------------+
                             v
Offline voice node ----> Command fusion node
 /voice_command          | gesture FOLLOW primary
                         | STAY safety priority
                         | conflicts -> tagged safety STAY
                         v
              /autonomy/astronaut/perception
        v
Astronaut mission node <--------- /cmd_vel/estop
        |
        +-- /autonomy/astronaut/drive_command (always; proposal only)
        +-- /autonomy/astronaut/mission_state
        +-- expected command + mission complete status
        +-- FETCH/GIVE request/status abstraction
        |
        +-- /cmd_vel/autonomy (disabled by default)
                     |
                     v
              existing drive mux
```

The mission node reuses `cmr_msgs/DriveCommand`, the same normalized navigation
interface used by the existing waypoint controller. Its adapter to
`/cmd_vel/autonomy` is intentionally disabled by default, so the new code does
not drive motors during development. Do not enable the adapter while the
existing waypoint controller is also publishing to that topic.

The existing bottle/mallet detector is separate and remains available through
the `test_detection` executable. Both detectors can subscribe to `/zed/image`
simultaneously, although this increases GPU usage.

## Requirements

Before running the nodes, make sure the rover computer has:

- ROS 2 Humble
- A supported ZED camera connected and recognized
- The ZED SDK and its Python API
- A working ROS `cv_bridge` installation
- Python 3 and `pip`
- Vosk plus a downloaded English model for voice input
- PortAudio and an input device for live microphone use (not needed for WAV)
- A Jetson-compatible PyTorch installation when running on a Jetson
- Internet access during the first model setup
- Enough GPU memory and compute capacity for real-time inference

The camera node must publish `sensor_msgs/Image` on `/zed/image` and an
organized `sensor_msgs/PointCloud2` on `/camera/points`. The modified
`cmr_zed threaded` node stamps both products from the same camera grab.

## First-time setup

Run these commands from the root of the `terra2` repository:

```bash
./run setup
./run gesture-setup
./run voice-setup      # only when voice input is needed
./run build
```

The commands perform these tasks:

- `./run setup` installs ROS package dependencies declared in `package.xml`.
- `./run gesture-setup` installs `ultralytics` from `requirements.txt` and
  downloads `yolo26n-pose.pt` into `src/astronaut_autonomy/config`.
- `./run voice-setup` installs Vosk and SoundDevice from
  `requirements-voice.txt`. The acoustic model is downloaded separately.
- `./run build` builds the ROS workspace and copies the configuration and pose
  weights into the installed `astronaut_autonomy` package.

`requirements.txt` contains the Ultralytics package dependency. It does not
contain the model weights themselves. The weights are downloaded by
`gesture-setup` and are intentionally not committed to Git.

On a Jetson, install the NVIDIA-supported PyTorch build before running
`gesture-setup`. A general desktop PyTorch wheel may not support the Jetson GPU.

## Running gesture detection

Open a terminal at the repository root, source the workspace, and start the ZED
publisher:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cmr_zed threaded
```

Leave that process running. In a second terminal at the repository root, run:

```bash
./run gesture
```

`./run gesture` loads the installed configuration and `yolo26n-pose.pt`, then
starts the `human_gesture_detection` node from the `astronaut_autonomy`
package.

Start the mission state machine in a third terminal. This leaves physical drive
output disabled and starts command fusion, but leaves voice disabled:

```bash
./run astronaut-mission
```

## Offline voice fallback

Download and extract a small English Vosk model, such as
[`vosk-model-small-en-us-0.15`](https://alphacephei.com/vosk/models/). Small
models are intended for constrained/mobile deployments and are the starting
point for Jetson evaluation. The model is not stored in Git.

Start microphone recognition with fusion and the mission controller:

```bash
./run astronaut-mission \
  enable_voice:=true \
  voice_model_path:=/absolute/path/to/vosk-model-small-en-us-0.15
```

For a prerecorded mono, uncompressed, 16-bit PCM WAV file:

```bash
./run astronaut-mission \
  enable_voice:=true \
  voice_input_mode:=wav \
  voice_model_path:=/absolute/path/to/vosk-model-small-en-us-0.15 \
  voice_wav_file:=/absolute/path/to/command.wav
```

Vosk is constrained to FOLLOW, STAY, FETCH, COME, GIVE, supported short
phrases, and `[unk]`. `minimum_confidence` and `ambiguity_margin` reject weak or
closely competing results. The recognition engine is isolated behind
`OfflineVoiceEngine`, so it can be replaced without changing ROS fusion.

Fusion rules are intentionally conservative:

- a fresh gesture FOLLOW is primary when voice is absent, weak, or stale;
- STAY from either valid modality has safety priority;
- matching commands become one `FUSED` command;
- any confident disagreement publishes an immediate tagged safety `STAY`;
- a conflict-generated stop does not advance the competition sequence;
- voice is used when gesture command quality is low, neutral, or stale;
- stale, low-confidence, duplicate, out-of-order, or invalid-source voice
  observations are rejected; and
- FOLLOW and COME require fresh ZED target localization before movement;
  FETCH command recognition does not require astronaut depth.

The current thresholds and time windows are in `config/voice_command.yaml` and
`config/command_fusion.yaml`.

## Verifying the system

Confirm that the camera is publishing at a steady rate:

```bash
ros2 topic hz /zed/image
```

Read the recognized gesture and its confidence:

```bash
ros2 topic echo /autonomy/human_gesture
ros2 topic echo /autonomy/human_gesture/confidence
ros2 topic echo /autonomy/astronaut/gesture_perception
ros2 topic echo /autonomy/astronaut/voice_command
ros2 topic echo /autonomy/astronaut/command_source
ros2 topic echo /autonomy/astronaut/fusion_detail
ros2 topic echo /autonomy/astronaut/perception
ros2 topic echo /autonomy/astronaut/distance
ros2 topic echo /autonomy/astronaut/mission_state
ros2 topic echo /autonomy/astronaut/drive_command
```

View the annotated image if `rqt_image_view` is installed:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/autonomy/human_gesture/debug_image` in the image viewer.

The gesture labels are:

- `FOLLOW` — both hands on hips with elbows out
- `STAY` — both arms extended in a T-pose
- `FETCH` — left hand at the neck and right arm pointing
- `COME` — hands together above the head
- `GIVE` — forearms crossed over the chest
- `NONE`

`/autonomy/astronaut/gesture_perception` reports the raw gesture command and
3-D target. `/autonomy/astronaut/perception` is the fused mission input and
also reports the selected command with the same fresh target position in the
ZED camera frame (`x=forward`, `y=left`, `z=up`), Euclidean distance, horizontal
bearing, sample count, and `VALID`, `STALE`, `UNAVAILABLE`, or `NO_POSE` depth
status. Gesture perception gates FOLLOW on `VALID` astronaut depth. The mission
layer also requires valid target localization for COME before it can move.
FETCH remains a valid command without astronaut depth.

## Mission state machine

The hardware-independent core has these states:

| State | Behavior |
| --- | --- |
| `IDLE` | Outputs zero while waiting for a command |
| `FOLLOWING` | Tracks the astronaut at `follow_distance_m` |
| `STAYING` | Outputs zero and maintains the stop condition |
| `FETCHING` | Outputs zero and starts the FETCH manipulation abstraction |
| `COMING` | Approaches and stops at `come_distance_m` |
| `GIVING` | Outputs zero and starts the GIVE manipulation abstraction |
| `ERROR` | Outputs zero until STAY is held or reset is requested |

Commands must remain stable for `command_debounce_s` and are latched until a
neutral `NONE` observation, preventing a held FETCH or GIVE pose from repeatedly
starting an operation. STAY bypasses the mission debounce so a valid explicit
stop or fusion conflict produces an immediate zero proposal. FOLLOW and COME
stop immediately on invalid or stale target data and enter ERROR if valid
tracking does not recover before `perception_timeout_s`. FETCH can start its
stationary manipulation handshake without astronaut depth; its arm coordinator
must independently localize the nearby hammer. The shared emergency stop
cancels an active manipulation, enters STAYING, and always produces zero drive.

Safety-input semantics are explicit:

| Input/event | Mission and drive effect |
| --- | --- |
| `NONE` with valid target data | No new transition; active FOLLOW/COME may continue |
| `NONE` with missing/invalid target data | Immediate zero proposal; tracking enters ERROR after the configured timeout |
| Explicit `STAY` | Immediate STAYING state and zero proposal; may advance the expected sequence |
| Fusion conflict | Immediate STAYING state and zero proposal; never advances the sequence |
| Stale command | Rejected by fusion and cannot refresh a command |
| Communication loss | Last safe target expires after `perception_timeout_s`; output becomes zero and ERROR latches |
| Emergency stop | Independently enforced by the drive mux and mission callback; immediate zero and manipulation cancel |

The FOLLOW/COME controller rotates in place for large bearing errors, combines
bounded forward and angular demand for smaller errors, and stops at or inside
the configured standoff. It does not reverse toward a nearby astronaut.
With `enforce_competition_sequence=true`, the expected sequence is FOLLOW,
STAY, FETCH, COME, GIVE. Out-of-order commands are ignored, except STAY remains
available at any time as a safety stop.

## FETCH/GIVE abstraction

No mission-level arm action exists in the current manipulation packages; they
expose teleoperation and low-level joint trajectories. The mission node
therefore uses this isolated handshake:

- Publishes `START_FETCH`, `START_GIVE`, or `CANCEL` on
  `/autonomy/astronaut/manipulation/request` (`std_msgs/String`).
- Consumes `RUNNING`, `SUCCEEDED`, `FAILED`, or `CANCELED` from
  `/autonomy/astronaut/manipulation/status` (`std_msgs/String`).
- Stops the rover throughout manipulation and transitions to ERROR on failure
  or `manipulation_timeout_s`.

The simulator implements this handshake. A future reviewed arm coordinator can
replace it without changing the mission core. That coordinator must perform
hammer detection, range validation, grasp planning, and collision checks; the
astronaut torso depth is not a substitute for hammer localization.

## Hardware-free development

The state machine, controller, depth logic, gesture rules, voice parsing, WAV
input, fusion rules, and scripted scenario are pure Python and can be tested
on macOS without ROS, Vosk, a microphone, or a ZED:

```bash
source .venv-gesture/bin/activate
PYTHONPATH=src/astronaut_autonomy \
  python -m unittest discover -s src/astronaut_autonomy/test -v
python -m unittest discover -s tests -v
```

Run the complete closed loop without ROS:

```bash
./run astronaut-pure-sim
```

Exercise each command-input mode with separate reports:

```bash
for mode in gesture voice matching conflict; do
  ./run astronaut-pure-sim \
    --command-input-mode "$mode" \
    --output "artifacts/astronaut_sim/$mode"
done
```

This simulation has a 2-D rover pose and a moving astronaut pose. Range and
Normalized `DriveCommand` values are limited and converted into mock linear and
angular motion. The scenario and its regression matrix check:

- an astronaut walking away while FOLLOW converges inside 3 m;
- a stationary rover during STAY;
- FETCH and COME issued beyond 20 m;
- FETCH recognized while astronaut depth is unavailable;
- gesture-only, voice-only, matching, and conflicting modality paths;
- held/repeated gestures and an out-of-order GIVE;
- temporary tracking loss and stale depth;
- an emergency stop during COME; and
- successful mock FETCH/GIVE handshakes.

The JSON report lists recognized commands, state transitions, start/final and
milestone distances, elapsed time, rejected unexpected commands, injected
failures, safety violations, and overall success. The CSV contains every rover
and astronaut pose, measurement, state, controller output, and applied velocity.
Configure noise from the command line:

```bash
./run astronaut-pure-sim \
  --range-noise 0.02 \
  --bearing-noise 0.25 \
  --output artifacts/astronaut_sim
```

The competition ordering and distance assertions follow the published
[URC requirements](https://urc.marssociety.org/home/requirements-guidelines)
and [official Q&A](https://urc.marssociety.org/home/requirements-guidelines/qa).

## ROS 2 Humble Docker environment

Docker Desktop must be installed and running. From the repository root:

```bash
./scripts/astronaut_docker.sh build
./scripts/astronaut_docker.sh test
```

`build` creates `terra2-astronaut-humble:local` from Ubuntu 22.04 / ROS 2
Humble, builds `cmr_msgs` and `astronaut_autonomy` with `colcon`, and imports
the generated `AstronautCommand`, `AstronautVoiceCommand`, and `DriveCommand`
Python classes. `test` runs the package tests and the complete topic pipeline:

```text
simulated astronaut -> gesture_perception -> command fusion
                    -> AstronautCommand -> mission state machine
                    -> DriveCommand -> closed-loop mock rover
```

The repository also contains
`.github/workflows/astronaut-autonomy-humble.yaml`, which performs the same
Docker build and ROS graph test on an Ubuntu 22.04 GitHub runner. A newly added
workflow must be treated as unverified until a GitHub Actions run completes.

Run only the pipeline or open a sourced shell with:

```bash
./scripts/astronaut_docker.sh sim
./scripts/astronaut_docker.sh shell
```

The Docker results are mounted back into `artifacts/astronaut_sim`. The same
environment can be started through Compose:

```bash
docker compose -f docker/astronaut-humble/compose.yaml up --build \
  --abort-on-container-exit
```

On a ROS 2 Humble development container, run the mock mission system without a
camera, Jetson, or rover:

```bash
./run build
./run astronaut-sim
```

The closed-loop ROS scenario generates all five commands, calculates changing
range and bearing from simulated positions, applies drive proposals to the mock
rover, and completes mock FETCH/GIVE requests. In another sourced terminal
inspect:

```bash
ros2 topic echo /autonomy/astronaut/perception
ros2 topic echo /autonomy/astronaut/mission_state
ros2 topic echo /autonomy/astronaut/drive_command
ros2 topic echo /autonomy/astronaut/manipulation/request
ros2 topic echo /autonomy/astronaut/sim/rover_pose
ros2 topic echo /autonomy/astronaut/mission_complete
```

The simulation launch explicitly keeps `enable_drive_adapter=false`.

## Topics and services

| Name | Type | Direction | Purpose |
| --- | --- | --- | --- |
| `/autonomy/astronaut/gesture_perception` | `cmr_msgs/AstronautCommand` | fusion input | Gesture command and ZED target |
| `/autonomy/astronaut/voice_command` | `cmr_msgs/AstronautVoiceCommand` | fusion input | Timestamped offline voice result |
| `/autonomy/astronaut/voice_status` | `std_msgs/String` | diagnostic | Listener/WAV/acceptance status |
| `/autonomy/astronaut/command_source` | `std_msgs/String` | diagnostic | `GESTURE`, `VOICE`, `FUSED`, or `FUSION` |
| `/autonomy/astronaut/fusion_detail` | `std_msgs/String` | diagnostic | Arbitration decision reason |
| `/autonomy/astronaut/perception` | `cmr_msgs/AstronautCommand` | mission input | Fused command and fresh 3-D target |
| `/cmd_vel/estop` | `std_msgs/Bool` | mission input | Existing shared emergency stop |
| `/autonomy/astronaut/drive_command` | `cmr_msgs/DriveCommand` | output | Always-published proposed velocity |
| `/cmd_vel/autonomy` | `cmr_msgs/DriveCommand` | optional output | Existing mux input; disabled by default |
| `/autonomy/astronaut/mission_state` | `std_msgs/String` | output | Current state |
| `/autonomy/astronaut/mission_detail` | `std_msgs/String` | output | State reason/status |
| `/autonomy/astronaut/expected_command` | `std_msgs/String` | output | Next command in the competition sequence |
| `/autonomy/astronaut/mission_complete` | `std_msgs/Bool` | output | Entire ordered mission succeeded |
| `/autonomy/astronaut/manipulation/request` | `std_msgs/String` | output | Minimal FETCH/GIVE request |
| `/autonomy/astronaut/manipulation/status` | `std_msgs/String` | input | Manipulation result |
| `/autonomy/astronaut/sim/rover_pose` | `geometry_msgs/Pose2D` | simulator output | Mock rover ground-truth pose |
| `/autonomy/astronaut/reset_error` | `std_srvs/Trigger` | service | Explicit ERROR recovery |

## Remaining rover integration

- Run the Docker build and ROS graph check on a Docker-capable host. This
  validates ROS middleware and generated messages, but not Jetson behavior.
- Tune standoff distances, gains, speed limits, and timeouts on a safe test
  course using recorded/simulated perception before enabling drive output.
- Review ownership of `/cmd_vel/autonomy`; only one autonomy controller should
  publish it during a mission.
- Connect and test the existing drive mux with its source set to `autonomy`,
  then validate emergency-stop latency before any person-following test. Only
  after those reviews should the mission be launched with
  `enable_drive_adapter:=true`.
- Implement and review a real arm coordinator behind the FETCH/GIVE handshake.
- Add hammer perception and manipulation-local depth checks to that coordinator;
  FETCH recognition alone never authorizes arm motion.
- Test the complete ZED-to-mission path on the Jetson and record CPU/GPU load,
  rate, latency, depth dropouts, and thermal behavior.
- Record multiple speakers saying all commands outdoors at competition-like
  distances and noise levels. Tune confidence/ambiguity thresholds from those
  recordings; the deterministic WAV tests do not establish real-world speech
  accuracy.
- Validate the selected Vosk wheel, small English model, USB microphone,
  PortAudio device selection, CPU load, latency, and acoustic performance on
  the actual Jetson.
- Validate the mock-to-real drive adapter and mux ownership on a secured rover
  test stand before enabling `enable_drive_adapter`; simulation never publishes
  `/cmd_vel/autonomy`.
- Purple-clothing detection has not been implemented. The node may process any
  visible person rather than only the astronaut.
- The current gestures use programmed body-keypoint rules; a custom trained
  gesture classifier has not been added yet.
- The body-pose model locates wrists but not individual fingers, so detailed
  signs such as a fist or thumbs-up require a hand-landmark model.
- Running gesture and bottle/mallet detection together may reduce frame rate or
  raise Jetson temperatures; this has not been measured on hardware.

More implementation details are available in
[`GESTURE_DETECTION.md`](GESTURE_DETECTION.md).

## Common problems

**No gesture messages appear**

Check that `/zed/image` exists and is receiving messages:

```bash
ros2 topic list | grep zed
ros2 topic hz /zed/image
ros2 topic hz /camera/points
```

If the image publishes but the synchronized detector remains quiet, rebuild
and restart `cmr_zed threaded`; both messages need valid, closely matching
timestamps. Confirm that `/camera/points` is organized (`height > 1`).

**The model or configuration is missing**

Run the setup again and rebuild so the downloaded model is copied into the
installed package:

```bash
./run gesture-setup
./run build
```

**Ultralytics or PyTorch cannot use the GPU**

Verify that the installed PyTorch build is compatible with the Jetson's JetPack
and CUDA versions before reinstalling Ultralytics.

**Voice node cannot start**

Run `./run voice-setup`, verify the extracted model directory passed through
`voice_model_path`, and confirm PortAudio can see the microphone. Use WAV mode
first to separate model/configuration problems from microphone-driver issues.
