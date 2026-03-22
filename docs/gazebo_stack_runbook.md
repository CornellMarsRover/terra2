# Gazebo Integration Runbook (terra2)

## TL;DR
This repo does **not** currently contain a complete Gazebo runtime stack (world + robot spawn + ROS bridge + simulated sensors + simulated drives).

What it has today:
- A Gazebo install helper script.
- A root-level `drives.urdf` that contains a Gazebo Classic camera plugin.
- Autonomy nodes with a `real=False` mode.
- Arm simulation with MoveIt + `ros2_control` **FakeSystem** (not Gazebo).

The most realistic way to "launch Gazebo" from this repo as-is is:
- use **Gazebo Classic**, not `gz sim` / Ignition;
- launch Gazebo manually through `gazebo_ros`;
- spawn `/Users/agupta/Desktop/terra2/drives.urdf`.

## What "Gazebo stack" means in this repo today

### 1) Install-time Gazebo hooks
- `scripts/install_gazebo.sh` installs Ignition Fortress and ROS bridge packages.
- This is installation-only; it does not launch a simulator or bridge by itself.
- The repo's actual robot file uses `libgazebo_ros_camera.so`, which is a **Gazebo Classic** plugin.
- So the install script and the URDF are slightly mixed: it installs Ignition packages, but the concrete robot artifact is written for Classic Gazebo.

### 2) Root-level drive robot that can be spawned in Gazebo Classic
- `/Users/agupta/Desktop/terra2/drives.urdf` is the only concrete Gazebo-targeted robot artifact in this repo.
- It includes:
  - a simple six-wheel base model;
  - a `gazebo` camera sensor block;
  - the Classic plugin `libgazebo_ros_camera.so`.
- It also includes a `ros2_control` block, but there is no matching Gazebo controller bringup in this repo.

### 3) Autonomy "sim" pipeline (not Gazebo-specific)
- `autonomous_navigation/launch/sim_autonomy.launch.py` launches:
  - `state_machine`
  - `object_detection`
  - `led_node`
  - `localization_sim`
  - `costmap`
  - `local_planner`
  - `global_planner`
  - `controller`

Expected data flow:
- `localization_sim` expects sensor inputs:
  - `/gps_exact` (`sensor_msgs/NavSatFix`)
  - `/navsatfix` (`sensor_msgs/NavSatFix`)
  - `/imu` (`sensor_msgs/Imu`)
- It publishes `/autonomy/pose/robot/global`.
- `costmap` expects `/camera/points` and pose.
- `local_planner` publishes `/autonomy/path/next_waypoint`.
- `controller` publishes drive commands:
  - `/autonomy/move/ackerman`
  - `/autonomy/move/point_turn`

### 4) Arm simulation stack (MoveIt fake hardware, not Gazebo)
- `moveit_servo/launch/servo_example.launch.py` starts RViz + MoveIt Servo + `ros2_control_node`.
- `cmr_arm_simulator/config/Arm.ros2_control.xacro` uses:
  - `mock_components/GenericSystem`
- `moveit_servo/config/panda_simulated_config.yaml` has:
  - `use_gazebo: false`

## Why it may not work (current blockers)

1. No Gazebo world/spawn/bridge launch in repo
- No `.world`/`.sdf` files or `ros_gz`/`ros_ign` bridge launch wiring were found.
- No `spawn_entity`/`gz sim` runtime integration found.

2. The only Gazebo-ready robot file is partial
- `drives.urdf` can be spawned in Classic Gazebo, but it is not wired into the autonomy stack.
- It only gives you a simple robot model plus camera sensor plugin.

3. The URDF references pieces that are not present here
- `drives.urdf` references `cmr_control/DrivesSystemHardware`, but there is no `cmr_control` package in this checkout.
- The Gazebo camera plugin uses `frameName=camera_link`, but `camera_link` is not defined in the URDF.
- That means Gazebo spawn may still work, but the control side and some frame assumptions are incomplete.

4. Drives node in autonomy path is hardware-only
- `cmr_controls/swerve_controller_node.py` initializes `moteus.Fdcanusb()` unconditionally.
- If no CAN-FD hardware/moteus is present, this node will fail.

5. IMU type mismatch in sim pipeline
- `localization_sim` subscribes to `/imu` as `sensor_msgs/Imu`.
- `cmr_imu/imu.py` publishes `/imu` as `cmr_msgs/IMUSensorData`.
- Those message types are incompatible.

6. Object detection defaults to real-camera path
- `object_detection.py` hard-codes `self.real = True`.
- It subscribes to `/zed/image` (not a typical Gazebo camera topic by default).

7. Hardcoded absolute path launch usage exists in some package launch files
- Several Fabric launch files rely on hardcoded absolute config paths.
- This reduces portability across machines/containers.

## How to run what is runnable today

## A) macOS reality check
On macOS, do **not** plan on running this Gazebo/ROS stack natively.

Reasons:
- this repo is built around Ubuntu package names like `ros-humble-*`;
- `scripts/install_gazebo.sh` uses `apt-get`;
- the Gazebo path here depends on `gazebo_ros` and a Classic Gazebo plugin;
- the repo already includes a Linux devcontainer setup in [/Users/agupta/Desktop/terra2/.devcontainer/devcontainer.json](/Users/agupta/Desktop/terra2/.devcontainer/devcontainer.json).

Recommended macOS approach:
- run the stack inside the repo's Linux devcontainer;
- use the devcontainer as the ROS/Gazebo runtime;
- use macOS only as the host for VS Code, Docker Desktop, and the GUI viewer.

## B) macOS setup path
Recommended:
1. Install Docker Desktop.
2. Install VS Code plus the `Dev Containers` extension.
3. Install XQuartz.
4. Open the repo in VS Code.
5. Use `Dev Containers: Rebuild and Reopen in Container`.

Important detail:
- this repo needed the devcontainer to run as `linux/amd64`, not `arm64`, in order for the Gazebo Classic path to work cleanly on Apple Silicon.
- the current repo devcontainer has already been updated to include `--platform=linux/amd64`.

Verification inside the container:
```bash
uname -m
```

Expected:
```bash
x86_64
```

## C) XQuartz note
XQuartz is still useful to have installed on macOS, and basic X11 tests can help with debugging:

On the Mac host:
```bash
defaults write org.xquartz.X11 nolisten_tcp -bool false
open -a XQuartz
sleep 2
/opt/X11/bin/xhost +localhost
```

However, in the working setup we debugged for this repo, the actual Gazebo GUI was **not** ultimately shown through XQuartz. The container is running an internal Xpra/Xvfb session, and Gazebo's GUI windows are attached to that session instead.

So:
- use XQuartz only as an optional debug tool;
- use the Xpra web UI as the main Gazebo GUI path.

## D) Fix ROS apt sources inside the devcontainer
After a fresh devcontainer rebuild, `gazebo_ros` may be missing even if ROS Humble is installed.

We hit two separate issues:
- the container initially came up as `arm64`;
- after switching to `amd64`, the ROS apt metadata inside the container was stale and needed to be refreshed.

From inside the devcontainer:
```bash
sudo apt-get update
sudo apt-get install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo rm -f /var/lib/apt/lists/packages.ros.org_ros2_ubuntu_dists_jammy_*
sudo apt-get update
```

## E) Install Gazebo dependencies in the devcontainer
From inside the devcontainer:
```bash
sudo apt-get install -y gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins x11-apps mesa-utils
```

Verify that `gazebo_ros` is now visible to ROS:
```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix gazebo_ros
```

Expected:
- a path under `/opt/ros/humble`

## F) Build workspace
From inside the devcontainer:
```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Note:
- for the first Gazebo smoke test, a full build is not strictly required because the robot is spawned directly from the root-level URDF;
- but it is still the cleanest default workflow.

## G) Launch Gazebo Classic headless first
Use Gazebo Classic, not `gz sim`:

```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros gazebo.launch.py gui:=false
```

This is the most reliable first step on macOS because it proves the Gazebo server is healthy before dealing with the GUI.

## H) Spawn the repo's drive robot into Gazebo
In a second devcontainer terminal:
```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py \
  -entity drives \
  -file /cmr/terra2/drives.urdf \
  -x 0 -y 0 -z 0.2
```

This is the concrete robot bringup path that worked during debugging.

## I) Open the Gazebo GUI through Xpra
This was the key discovery from the macOS debugging session.

Even though X11 tools like `xclock`, `xeyes`, and `gzclient` could connect, the devcontainer image was actually running its own Xpra/Xvfb display server internally. Gazebo windows were created there, not directly in XQuartz.

So the correct GUI workflow is:
1. Leave `gzserver` running from the headless launch.
2. Start `gzclient` inside the devcontainer.
3. Open the Xpra web UI in a browser on the Mac host.

In a third devcontainer terminal:
```bash
/cmr/terra2/scripts/gazebo_gui.sh --mode xpra
```

Then on the Mac host, open:
- [http://localhost:14500](http://localhost:14500)

That Xpra session is the viewer that should show the Gazebo desktop/window.

Important notes:
- `gzclient` staying alive is a good sign;
- `xwininfo` confirmed the Gazebo window existed even when XQuartz did not show it;
- the browser/Xpra view is the reliable GUI path for this image.

## J) Optional: try direct XQuartz instead
The repo now includes a switchable Gazebo GUI launcher:
- [/Users/agupta/Desktop/terra2/scripts/gazebo_gui.sh](/Users/agupta/Desktop/terra2/scripts/gazebo_gui.sh)

It supports:
- `--mode xpra`
- `--mode xquartz`
- `--mode custom`

On the Mac host, make sure XQuartz is running and network clients are allowed:
```bash
defaults write org.xquartz.X11 nolisten_tcp -bool false
open -a XQuartz
sleep 2
/opt/X11/bin/xhost +localhost
```

Then in the devcontainer:
```bash
/cmr/terra2/scripts/gazebo_gui.sh --mode xquartz
```

If XQuartz is listening on a different display target, you can override it:
```bash
/cmr/terra2/scripts/gazebo_gui.sh --mode xquartz --display host.docker.internal:0
```

Notes:
- `xquartz` is still more fragile than `xpra` in this setup;
- `xpra` should be treated as the default path;
- the launcher is flaggable by design because a single GUI process normally targets one display at a time, not both simultaneously.

## K) Verify the Gazebo bringup
In another devcontainer terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | rg 'image_raw|camera_info|clock'
```

Expected:
- `/clock`

Observed during debugging:
- Gazebo server startup and `spawn_entity.py` worked;
- the `drives` model spawn worked;
- `/clock` was the main topic we expected to confirm first;
- camera topics from the `libgazebo_ros_camera.so` plugin were **not** fully confirmed in this session, so treat camera ROS output as still needing verification.

## L) What this Gazebo launch actually gives you
- A Classic Gazebo server.
- The root-level `drives.urdf` robot spawned into the world.
- A GUI path through the devcontainer's Xpra session at `http://localhost:14500`.

It does **not** give you:
- IMU, GPS, or point cloud topics expected by `autonomous_navigation`;
- a wired drive controller;
- a full rover sim;
- an arm Gazebo sim.

## M) Run autonomy sim pipeline only (without hardware drives)
```bash
ros2 launch autonomous_navigation sim_autonomy.launch.py
```

Notes:
- This starts the autonomy graph, but behavior depends on sensor topics being available.
- It does **not** start Gazebo.

## N) Optional: run arm simulation (MoveIt + fake ros2_control)
```bash
ros2 launch moveit_servo servo_example.launch.py
```

Notes:
- This is RViz/FakeSystem simulation for the arm.
- It is not Gazebo physics simulation.

## O) If you insist on testing autonomy loop end-to-end without Gazebo
You can provide fake sensor topics from terminals (basic smoke test only):

```bash
# Terminal 1: GPS source
ros2 topic pub -r 10 /gps_exact sensor_msgs/msg/NavSatFix "{latitude: 40.0, longitude: -74.0, altitude: 0.0}"
```

```bash
# Terminal 2: Map GPS source
ros2 topic pub -r 10 /navsatfix sensor_msgs/msg/NavSatFix "{latitude: 40.0, longitude: -74.0, altitude: 0.0}"
```

```bash
# Terminal 3: IMU source expected by localization_sim
ros2 topic pub -r 30 /imu sensor_msgs/msg/Imu "{orientation: {w: 1.0}, linear_acceleration: {x: 0.0, y: 0.0, z: 9.81}}"
```

```bash
# Terminal 4: Empty point cloud for costmap (minimal shape)
ros2 topic pub -r 2 /camera/points sensor_msgs/msg/PointCloud2 "{header: {frame_id: map}, height: 1, width: 0, fields: [], is_bigendian: false, point_step: 0, row_step: 0, data: [], is_dense: false}"
```

This only validates node bringup and topic wiring, not realistic navigation.

## Why `gz sim` is the wrong command here
- The only concrete simulator plugin in the repo is `libgazebo_ros_camera.so`.
- That plugin is for **Gazebo Classic**.
- `gz sim` / Ignition Fortress would need different model/plugin integration than what this repo currently contains.

## What to add for a real Gazebo stack

1. Add a Gazebo launch package that starts world + robot model.
2. Decide on one simulator family and make the repo consistent:
- either Gazebo Classic with `gazebo_ros`
- or modern Gazebo with `ros_gz`
3. Add sensor and control wiring for the rover:
- IMU
- GPS/NavSat
- camera image/camera_info
- point cloud
- clock (`/clock`)
4. Add a simulated drives backend replacing moteus in sim mode.
5. Parameterize `object_detection` real/sim topic selection.
6. Standardize sensor message types (especially `/imu`) across sim and real pipelines.
