# ROS2 Interface Inventory

Generated for the `astrotech-gui` branch as part of URC 2026 GCS Phase 1 discovery.
Based on a static read of `src/` — the rover is not running, so rates below
reflect code-level timer settings, not measured traffic.

## 1. ROS2 distro

**Humble.** Confirmed by:

- `.github/workflows/build-test.yaml` specifies `rosdistro: humble` for both
  the colcon build and the `cornellmarsrover/ros2-test-action` test step, and
  installs `ros-humble-*` packages.
- The dev container image `cornellmarsrover/dev:latest` is the image referenced
  by `.devcontainer/devcontainer.json` and by CI.
- No `package.xml` in this workspace declares an alternate distro; there are
  no ROS1 (`catkin`) packages.

## 2. Packages under `src/`

| Package | Build type | One-line purpose |
|---|---|---|
| `autonomous_navigation` | ament_python | Full autonomy stack: state machine, Kalman localization, global/local planners, costmap, controller, object detection, LED signaling. |
| `autonomous_typing_package` | ament_python | Experimental keyboard-typing autonomy (unrelated to Astrotech); out of scope. |
| `cmr_arm_sim` | ament_python | Hosts the rover arm URDF + `display.launch.py` for RViz preview. |
| `cmr_arm_simulator` | ament_cmake | MoveIt2 config / Xacro for the arm (URDF, ros2_controllers.yaml, SRDF). |
| `cmr_aruco` | *(empty)* | Placeholder; no source. |
| `cmr_cams` | ament_cmake | Bootstrap package for the camera/Foxglove stack. Holds TOML fabric configs and `gui_server.launch.py` (which today starts `foxglove_bridge`). |
| `cmr_controller_remote` | ament_python | UDP receiver for PS5 controller; converts laptop-side joystick packets into ROS2 command topics (drives, arm, mini-arm). |
| `cmr_controls` | ament_python | Swerve kinematics node + arm/keyboard helpers; consumes teleop and autonomy move commands. |
| `cmr_fabric` | ament_cmake | Custom lifecycle / fault-handler framework (fabric_node, lifecycle_manager, fault_handler, dependency_manager). |
| `cmr_fabric_wrappers` | ament_cmake | Fabric wrapper that runs an external command as a managed subprocess node (used for `micro_ros_agent`, etc.). |
| `cmr_imu` | ament_python | Driver for WitMotion HWT905 IMU; publishes `/imu` (custom IMU msg). |
| `cmr_msgs` | ament_cmake | Custom CMR messages / services / actions (motor data, joystick, site-analyze, etc.). |
| `cmr_param_gui` | ament_python | Tk-based parameter/debug GUI that subscribes to `/gui/display`. |
| `cmr_rovernet` | ament_python | Jetson ↔ ECE bridge: translates ROS2 drive/arm commands into moteus-CAN / serial frames and publishes motor telemetry. |
| `cmr_rtkgps` | ament_python | RTK GNSS base + rover nodes; publishes `/rtk/navsatfix_data`. |
| `cmr_utils` | ament_cmake | C++ utility headers (logging, service client helpers, string utilities). |
| `cmr_zed` | ament_python | ZED2 SDK wrappers: point cloud, ground plane, pose, image, experimental GNSS fusion. |
| `moveit_servo` | ament_cmake | Vendored MoveIt Servo; converts joystick twists into `/main_arm_controller/joint_trajectory`. |
| `ompl` | *(empty)* | Placeholder. |
| `temp_python_files` | n/a | Loose scratch Python; not a ROS package. |
| `usb_camera_publisher` | ament_python | Multi-USB-camera publisher: raw `Image`, rectified, stitched, birdseye, **and `foxglove_msgs/CompressedVideo` (H.264)** for Foxglove. |
| `zed-ros2-wrapper` | *(empty)* | Placeholder (expected submodule not initialized). |

## 3. Nodes

Format: **Node** *(package/executable)* → publishes / subscribes / services / clients / params.
Rates are from `create_timer(period, …)` in source, not measurements. Camera device paths and indices are copied from source defaults.

### 3.1 Teleop & controller input

#### `connect_node` (cmr_controller_remote)
- **Publishes**
  - `/drives_controller/cmd_vel` — `geometry_msgs/TwistStamped` — on UDP packet (~50 Hz typical PS5).
  - `/drives_controller/cmd_buttons` — `cmr_msgs/ControllerReading` — event-driven.
  - `/arm_controller/cmd_vel` — `sensor_msgs/Joy` — on packet (~50 Hz).
  - `/arm_controller/cmd_buttons` — `cmr_msgs/ControllerReading` — event-driven.
  - `/mini_arm_controller/cmd_pos` — `cmr_msgs/MiniArmDegree` — event-driven.
- **Subscribes** — none (pure UDP ingress).
- **Listens on UDP** ports `5010`, `5020`, `5030` (from codebase index).

### 3.2 Drives

#### `drivesnet_node` (cmr_rovernet/drivesnet.py)
- **Subscribes**
  - `/drives_controller/cmd_vel` — `geometry_msgs/TwistStamped`.
  - `/drives_controller/cmd_buttons` — `cmr_msgs/ControllerReading`.
  - `/autonomy_move` — `cmr_msgs/AutonomyDrive` *(legacy alternate topic — see gaps)*.
- **Publishes** — none; writes moteus-CAN frames.
- **Note** there are four sibling files (`drivesnet.py`, `drivesnet_og.py`, `drivesnet_new_devon.py`, `drivesnet(old).py`) — only `drivesnet.py` is the current entrypoint per `setup.py`.

#### `swerve_controller_node` (cmr_controls)
- **Subscribes**
  - `/cmd_vel_drives` — `geometry_msgs/Twist`.
  - `/autonomy/move/ackerman` — `cmr_msgs/AutonomyDrive`.
  - `/autonomy/move/move_type` — `std_msgs/String`.
  - `/autonomy/move/point_turn` — `geometry_msgs/Twist`.
- **Publishes** — none observed above the direct-motor layer.

#### `keyboard_listener` (cmr_controls)
- **Publishes** `/cmd_vel_drives` — `geometry_msgs/Twist` (UDP-triggered keyboard teleop).

#### `arm_controller_node` / `ik_node` / `keyboard_controller_node` (cmr_controls)

A parallel arm-control stack lives in `cmr_controls` and is registered in
`src/cmr_controls/setup.py` entry points. It **conflicts** with the
`moveit_servo → armnet` path above — only one stack should run at a time.
Flagging here for completeness; not used by the GCS bridge.

- **`arm_controller_node`** (`cmr_controls/arm_controller_node.py`)
  - Subscribes `/joint_angles/desired` — `std_msgs/Float32MultiArray`.
  - Subscribes `/joint_angles/offsets` — `std_msgs/Float32MultiArray`.
- **`ik_node`** (`cmr_controls/ik_node.py`)
  - Publishes `/arm/end_effector/pose` — `std_msgs/Float32MultiArray`.
  - Publishes `/joint_angles/desired` — `std_msgs/Float32MultiArray`.
  - Subscribes `/cmd_vel` — `geometry_msgs/Twist`.
  - Subscribes `/arm/joint_increment` — `std_msgs/Float32MultiArray`.
- **`keyboard_controller_node`** (`cmr_controls/keyboard_controller_node.py`)
  - Publishes `/cmd_vel` — `geometry_msgs/Twist` — 10 Hz timer.
  - Publishes `/arm/joint_increment` — `std_msgs/Float32MultiArray`.

### 3.3 Arm

#### `JoyToServoPub` (moveit_servo)
- **Subscribes** `/arm_controller/cmd_vel` — `sensor_msgs/Joy`.
- **Publishes**
  - `/servo_node/delta_twist_cmds` — `geometry_msgs/TwistStamped`.
  - `/servo_node/delta_joint_cmds` — `control_msgs/JointJog`.
  - `/planning_scene` — `moveit_msgs/PlanningScene`.
- **Clients** `/servo_node/start_servo` — `std_srvs/Trigger`.

#### `servo_node` (moveit_servo)
- **Subscribes** `/servo_node/delta_twist_cmds`, `/servo_node/delta_joint_cmds`, `~/collision_velocity_scale`.
- **Publishes**
  - `/main_arm_controller/joint_trajectory` — `trajectory_msgs/JointTrajectory` — rate = `publish_period` (default 0.034 s ≈ 30 Hz from `panda_simulated_config.yaml`; our config may differ).
  - `~/status` — `std_msgs/Int8`.
- **Services** `~/start_servo`, `~/stop_servo`, `~/pause_servo`, `~/unpause_servo` (all `std_srvs/Trigger`), `~/change_drift_dimensions`, `~/change_control_dimensions` (moveit_msgs), `~/reset_servo_status` (`std_srvs/Empty`).

#### `armnet_node` (cmr_rovernet/armnet.py)
- **Subscribes**
  - `/main_arm_controller/joint_trajectory` — `trajectory_msgs/JointTrajectory`.
  - `/mini_arm_controller/cmd_pos` — `cmr_msgs/MiniArmDegree`.
  - `/arm_controller/cmd_buttons` — `cmr_msgs/ControllerReading`.
- **Publishes** — none; writes moteus-CAN to IDs 9–14 (base, shoulder, elbow, wrist_rotate_1, wrist_tilt, wrist_rotate_2).

### 3.4 ECE / telemetry

#### `cmr_read_node` (cmr_rovernet/ccb_read.py)
- **Publishes** `/ccb/read` — `cmr_msgs/MotorReadData` (14-motor batch) — timer is set to **1000 Hz** (`create_timer(0.001, self.publish_msg)`) but each tick performs 14 sequential blocking `serial.read(20)` calls at 115200 baud with `timeout=1 s`, so the effective rate is serial-bound (measure on hardware).

#### `debug_node` (cmr_rovernet/debug.py)
- **Subscribes** `/ccb/read` — logs only; no publish.

### 3.5 IMU & GPS

#### `imu_node` (cmr_imu/imu.py)
- **Publishes** `/imu` — `cmr_msgs/IMUSensorData` — 10 Hz (`timer_period = 0.1`).
- **Note** this is a **custom** message, not `sensor_msgs/Imu`. Foxglove will need the schema.

#### `gps_rover` (cmr_rtkgps/rover.py)
- **Publishes** `/rtk/navsatfix_data` — `sensor_msgs/NavSatFix` — 10 Hz (`create_timer(0.1, …)`).

#### `gps_basestation` / `basestation_known` (cmr_rtkgps)
- **Subscribes** `/rtk/rover_ready` — `std_msgs/String`.
- No ROS publishes; corrections go out over sockets.

#### `gps_logger` / `displacement_logger` (cmr_rtkgps)
- **Subscribes** `NavSatFix` and `TwistStamped`; file sinks only.
- `displacement_logger` subscribes `/autonomy/pose/global/robot` — **typo bug**, the publisher uses `/autonomy/pose/robot/global` (flagged in codebase_index/04).

### 3.6 Vision (ZED)

Multiple overlapping implementations ship in `cmr_zed/`; exactly one should run at a time.

#### `zed_autonomy` / `pose` / `threaded` (cmr_zed)
- **Publishes** (union across variants)
  - `/camera/points` — `sensor_msgs/PointCloud2` — 10 Hz.
  - `/camera/ground_plane` — `cmr_msgs/GroundPlaneStamped` — 10 Hz (in variants that publish it).
  - `/zed/pose` — `geometry_msgs/TwistStamped` — 10 Hz.
  - `/zed/image` — `sensor_msgs/Image` — rate ~= ZED grab.
- **Notes** `sensor_msgs/Image` (raw) is used — see bandwidth audit.

#### `zed_publisher_node` (cmr_zed/zed_camera_publisher.py)
- **Publishes**
  - `/zed/image_left`, `/zed/image_right` — `sensor_msgs/Image` — 10 Hz.
  - `/zed/plane/equation` — `std_msgs/Float32MultiArray`.
  - `/zed/point/coordinate` — `std_msgs/Float32MultiArray`.
- **Subscribes** `/zed/plane/request`, `/zed/point/request` — `std_msgs/Int32MultiArray`.

#### `zed_gnss_fusion` (cmr_zed)
- **Subscribes** `/rtk/navsatfix_data`.
- **Publishes** `/camera/points`, `/autonomy/pose/robot/global` (conflicts with `new_kalman` if both run).

### 3.7 Vision (USB cameras)

#### `x264_multi_camera_publisher` (usb_camera_publisher/publisher.py)
- **Camera IDs hardcoded**: `[0, 2, 4, 6, 8, 10]`.
- **Publishes**, per camera `<id>`:
  - `camera_<id>/h264` — `foxglove_msgs/CompressedVideo` — tied to V4L2 capture, pipeline is 640×480 MJPEG → decoded → `x264enc` at `bitrate=2000` kbps, `tune=zerolatency`, `speed-preset=ultrafast`. Frame rate comes from the camera (no explicit v4l2src `framerate=` cap — flagged in gaps).
  - `camera_<id>/camera_info` — `sensor_msgs/CameraInfo`.
- **Calibration path** hardcoded `/home/cmr/cmr/terra2/calib_dir` (portability bug).

#### `multi_camera_image_publisher` (usb_camera_publisher/publish_images.py)
- **Publishes** `/camera_<id>/image_raw` — `sensor_msgs/Image` (bgr8) — 10 Hz for cams `[0,2,4,6]` at 320×240 (main) / entry point default.

#### `multi_camera_stitched_publisher` (usb_camera_publisher/stitched.py)
- **Subscribes** `/node_management/shutdown` — `std_msgs/String`.
- **Publishes** `/camera/stitched_image` — `sensor_msgs/Image` — 10 Hz (2×2 320×240 mosaic).

#### `birdseye_publisher` (usb_camera_publisher/bev.py)
- **Publishes**
  - `/camera_<id>/image_rect` — `sensor_msgs/Image` — 10 Hz, per cam in `[0,2,4,6]`.
  - `/birdseye/image_raw` — `sensor_msgs/Image` — 10 Hz (300×300 mosaic).

### 3.8 Autonomy

#### `state_machine` (autonomous_navigation/state_machine.py)
- **Subscribes** `/autonomy/pose/robot/global`.
- **Publishes**
  - `/autonomy/target/global` — `std_msgs/Float32MultiArray`.
  - `/autonomy/target/local` — `std_msgs/Float32MultiArray`.
  - `/autonomy/target_object/name` — `std_msgs/String`.
  - `/autonomy/led` — `std_msgs/String`.

#### `global_planner` / `local_planner` / `costmap` / `controller` / `object_detection` / `led_node` / `new_kalman` / `kalman_localization` (legacy) / `localization_sim`
- Both `new_kalman` and `kalman_localization` publish
  `/autonomy/pose/robot/global` (`geometry_msgs/TwistStamped`). Only one of
  them should run at a time — the launch files prefer `new_kalman`.
- Full topic/service table is covered in `codebase_index/01_runtime_graph.txt`
  and `codebase_index/02_topic_catalog.txt`. Reproducing the key autonomy topic
  set here:
  - `/autonomy/pose/robot/global` — `geometry_msgs/TwistStamped` — from `new_kalman`.
  - `/autonomy/target/{global,local}` — `std_msgs/Float32MultiArray`.
  - `/autonomy/path/next_waypoint` — `std_msgs/Float32MultiArray`.
  - `/autonomy/costmap` — `std_msgs/Float32MultiArray`.
  - `/autonomy/target_object/{name,position}` — `String` / `geometry_msgs/Twist`.
  - `/autonomy/move/{ackerman,point_turn,move_type}` — as above.
  - `/autonomy/led` — `std_msgs/String`.

### 3.9 Fabric lifecycle plumbing

Each fabric-managed node exposes, under its namespace:

- `<node>/acquire` — `cmr_msgs/AcquireDependency`.
- `<node>/release` — `cmr_msgs/ReleaseDependency`.
- `<node>/notify_deactivate` — `cmr_msgs/NotifyDeactivate`.
- `<ns>/lifecycle/activate` — `cmr_msgs/ActivateNode`.
- `<ns>/lifecycle/deactivate` — `cmr_msgs/DeactivateNode`.
- `<ns>/lifecycle/cleanup` — `cmr_msgs/DeactivateNode`.
- `<ns>/recover_fault` — `cmr_msgs/RecoverFault`.

The default composition namespace is `rover` (see `cmr_cams/launch/default.launch.py`).

## 4. Custom message / service / action types (from `cmr_msgs`)

Foxglove must be given the schema for every type in this list. `foxglove_bridge`
auto-serves them from the running ROS graph — listing here so Foxglove panel
authors know what to import.

**Messages**

- `cmr_msgs/ArmJointEffort`
- `cmr_msgs/ArucoMarkers`
- `cmr_msgs/AutonomyDrive`
- `cmr_msgs/ControllerReading`
- `cmr_msgs/Error`
- `cmr_msgs/GroundPlaneStamped`
- `cmr_msgs/IMUSensorData`
- `cmr_msgs/Int16ArrayStamped`
- `cmr_msgs/JoystickReading`
- `cmr_msgs/MiniArmDegree`
- `cmr_msgs/MotorData`
- `cmr_msgs/MotorReadData`
- `cmr_msgs/MotorWriteBatch`
- `cmr_msgs/Power`
- `cmr_msgs/SensorReadBatch`
- `cmr_msgs/State`

**Services**

- `cmr_msgs/AcquireDependency`
- `cmr_msgs/ActivateNode`
- `cmr_msgs/DeactivateNode`
- `cmr_msgs/NotifyDeactivate`
- `cmr_msgs/RecoverFault`
- `cmr_msgs/ReleaseDependency`
- `cmr_msgs/SiteAnalyze` — *defined but unused* (no server, no client in repo).

**Actions**

- `cmr_msgs/TestTargetPosition` — placeholder-only, not wired into autonomy.

## 5. Topic-rate summary (source-declared)

| Topic | Type | Source rate | Notes |
|---|---|---|---|
| `/imu` | `cmr_msgs/IMUSensorData` | 10 Hz | custom msg |
| `/rtk/navsatfix_data` | `sensor_msgs/NavSatFix` | 10 Hz | |
| `/camera/points` | `sensor_msgs/PointCloud2` | 10 Hz | ZED |
| `/camera/ground_plane` | `cmr_msgs/GroundPlaneStamped` | 10 Hz | |
| `/zed/pose` | `geometry_msgs/TwistStamped` | 10 Hz | |
| `/zed/image`, `/zed/image_left`, `/zed/image_right` | `sensor_msgs/Image` | 10 Hz | **raw Image** |
| `/main_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | ≈30 Hz | servo `publish_period` |
| `/drives_controller/cmd_vel` | `geometry_msgs/TwistStamped` | ≤50 Hz | UDP driven |
| `/arm_controller/cmd_vel` | `sensor_msgs/Joy` | ≤50 Hz | UDP driven |
| `camera_<id>/h264` | `foxglove_msgs/CompressedVideo` | camera native | 6 cams, 2 Mbps each |
| `/camera_<id>/image_raw` | `sensor_msgs/Image` | 10 Hz | 4 cams, 320×240 bgr8 |
| `/camera_<id>/image_rect` | `sensor_msgs/Image` | 10 Hz | 4 cams, 640×480 bgr8 |
| `/camera/stitched_image` | `sensor_msgs/Image` | 10 Hz | 640×480 bgr8 |
| `/birdseye/image_raw` | `sensor_msgs/Image` | 10 Hz | 300×300 bgr8 |
| `/ccb/read` | `cmr_msgs/MotorReadData` | 1000 Hz timer, serial-bound | 14-motor batch |

## 6. Gaps — hardware features with no corresponding node/topic/service

These are flagged as **MISSING** and not invented. Phase 1 deliverables do not
create any of them; Phase 2 will need them before Foxglove panels can actually
drive the payload.

1. **Auger linear (up/down) control — MISSING.**
   `cmr_rovernet/rovernet_utils.py` defines `ASTROTECH = 0x03` as a subteam ID
   in the serial protocol, but no node publishes or subscribes any auger topic,
   no moteus auger controller is instantiated, and no Maxon driver exists in
   the repo. Which controller family the auger uses is unknown — see Open
   Questions.
2. **Auger spin (forward/reverse) control — MISSING.** Same reasoning as (1).
3. **Analysis Sequence 1 / Sequence 2 — PARTIAL.**
   `cmr_msgs/srv/SiteAnalyze.srv` exists (`int8 site_num → bool success`) and
   has a slot in `cmr_msgs/CMakeLists.txt`, but no server advertises it and no
   client calls it. The BDC controller node that would execute the sequences is
   not in this workspace.
4. **Mixing servo presets (S1, S2, CO2_1, CO2_2, Retract) — MISSING.**
   No service, no topic, no enum, no driver node. The custom-msg side has no
   preset type.
5. **Auger / site / analysis cameras — PARTIAL.**
   `usb_camera_publisher` can enumerate `/dev/videoN` and publishes
   `camera_<id>/h264`, but there is no mapping from a logical name
   (`auger_cam`, `site_cam`, `analysis_cam`) to a specific `/dev/videoN`. Must
   be resolved by config, not code changes (see feature map).
6. **Raman spectrometer driver — MISSING.**
   No node ingests a spectrum. `std_msgs/Float32MultiArray` is used elsewhere
   but not for spectra. No wavelength/intensity message type exists in
   `cmr_msgs`.
7. **CO2 sensor driver — MISSING.** No node, no topic, no message.
8. **Humidity sensor driver — MISSING.** Same.
9. **Temperature sensor driver — MISSING.** Same (flagging proactively — URC
   science payloads typically require temperature alongside CO2/humidity).
10. **Snapshot service (save-frame with label) — MISSING.** No service
    advertises a capture-and-save action for mission-critical photos.
11. **Unified `/power`/`/estop` topics — PARTIAL.** `cmr_msgs/Power` exists
    (`bool on`) but is not published by any node in the repo.
12. **Bandwidth coordination.** No node throttles or compresses for RF link
    conditions; nothing drops raw `sensor_msgs/Image` off the wire when the
    rover is on RF vs tether.

## 7. Known runtime inconsistencies the code already has (keep out of GCS scope but aware of)

- `displacement_logger` subscribes `/autonomy/pose/global/robot`, publisher
  uses `/autonomy/pose/robot/global`. Listener starves.
- `cmr_cams/launch/default.launch.py` hardcodes
  `/home/cmr/cmr/terra2/src/cmr_cams/config`.
- Cam `toml`s reference packages `cmr_cv`, `cmr_arm`, `cmr_control`, `cmr_demo`
  that are not in this workspace — the cam fabric launch will fail to bring
  those nodes up unless those packages are in a parallel overlay.
- Multiple overlapping ZED node implementations exist; only one should run.

## 8. Source-of-truth references

- `src/cmr_msgs/{msg,srv,action}/*` — ground truth for custom types.
- `src/usb_camera_publisher/usb_camera_publisher/publisher.py` — USB H.264 publisher.
- `src/cmr_cams/launch/gui_server.launch.py` — existing foxglove_bridge launch stub.
- `src/cmr_rovernet/cmr_rovernet/rovernet_utils.py` — motor IDs, serial encoding, `ASTROTECH` subteam constant.
- `codebase_index/` — previously-authored summary; cross-checked against source for this document.
