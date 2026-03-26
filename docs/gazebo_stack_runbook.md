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
- the devcontainer now publishes the Gazebo/Xpra and controller UDP ports explicitly instead of relying on Docker's Linux-side `host` network mode. That makes the Mac host able to use `127.0.0.1` for controller traffic.

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
colcon build --symlink-install --packages-up-to cmr_msgs cmr_controller_remote cmr_controls autonomous_navigation
source install/setup.bash
```

Note:
- this narrower build is recommended here because the full workspace can fail on unrelated packages;
- `cmr_msgs`, `cmr_controller_remote`, `cmr_controls`, and `autonomous_navigation` are the packages needed for the Gazebo controller + drive bridge path.

If you previously built this workspace in an `arm64` container and are now in `amd64`, clear stale caches first:
```bash
cd /cmr/terra2
rm -rf build install log
```

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

## I) Start the Gazebo drive bridge
The repo now includes a Gazebo bridge node that listens to the rover's normal drive topics and mirrors them into Gazebo wheel efforts.

Standalone launch:
```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cmr_controls gazebo_drives.launch.py
```

Controller-enabled launch:
```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cmr_controls gazebo_drives.launch.py with_remote:=true
```

What it listens to:
- `/cmd_vel_drives`
- `/cmd_vel`
- `/autonomy/move/point_turn`
- `/autonomy/move/ackerman`
- `/drives_controller/cmd_vel`

What it does:
- converts those commands into left/right wheel efforts;
- calls Gazebo's `/apply_joint_effort` service repeatedly while commands are active;
- clears wheel efforts when commands go stale.

Architecture note:
- the Gazebo bridge now reuses the repo's existing `cmr_controller_remote/connect.py` UDP ingress path instead of maintaining a separate container-side teleop receiver;
- the only new controller helper is the host-side Sony controller reader script, because the controller is attached to macOS while ROS runs inside the Linux devcontainer.

Important limitation:
- the spawned `drives.urdf` model is still not a true swerve/ackermann Gazebo model;
- the bridge mirrors **driving intent**, not exact real rover steering geometry.

## J) Open the Gazebo GUI through Xpra
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

## K) Optional: try direct XQuartz instead
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

## L) Verify the Gazebo bringup
In another devcontainer terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E 'image_raw|camera_info|clock|camera1'
```

Expected:
- `/clock`

Observed during debugging:
- Gazebo server startup and `spawn_entity.py` worked;
- the `drives` model spawn worked;
- `/clock` was confirmed;
- Gazebo logged camera info publishing under `/camera1/camera_info`;
- the new Gazebo drive bridge should log that it is listening for rover drive commands.

## M) What this Gazebo launch actually gives you
- A Classic Gazebo server.
- The root-level `drives.urdf` robot spawned into the world.
- A Gazebo drive bridge that reacts to rover drive topics.
- A GUI path through the devcontainer's Xpra session at `http://localhost:14500`.

It does **not** give you:
- IMU, GPS, or point cloud topics expected by `autonomous_navigation`;
- a full rover sim;
- an arm Gazebo sim.

## N) Run autonomy sim pipeline with the Gazebo drive bridge
```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autonomous_navigation sim_autonomy.launch.py
```

Notes:
- `sim_autonomy.launch.py` now also starts `gazebo_drive_bridge`;
- if Gazebo is already running, autonomy drive commands should now reach the simulated rover;
- behavior still depends on the expected sensor topics being available.

## O) Run the Sony controller through the existing ROS controller stack
This is the cleaned-up controller path after debugging.

Data flow:
- Sony controller on macOS
- `/Users/agupta/Desktop/terra2/scripts/sony_controller_udp.py`
- existing `cmr_controller_remote/connect.py` on UDP port `5010`
- `/drives_controller/cmd_vel`
- `gazebo_drive_bridge`
- Gazebo wheel efforts

### 1. Start Gazebo and spawn the rover
Terminal 1:
```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gazebo_ros gazebo.launch.py gui:=false
```

Terminal 2:
```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gazebo_ros spawn_entity.py \
  -entity drives \
  -file /cmr/terra2/drives.urdf \
  -x 0 -y 0 -z 0.2
```

### 2. Start the existing remote-control ingress plus the Gazebo bridge
Terminal 3:
```bash
cd /cmr/terra2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cmr_controls gazebo_drives.launch.py with_remote:=true
```

This launch starts:
- `cmr_controller_remote/connect_node`
- `cmr_controls/gazebo_drive_bridge`

### 3. On the Mac host, install `pygame` once
```bash
python3 -m pip install pygame
```

### 4. On the Mac host, run the Sony controller sender if you are testing gazebo or drive inputs
```bash
cd /Users/agupta/Desktop/terra2
python3 scripts/sony_controller_udp.py --host 127.0.0.1 --port 5010
```

Controls:
- left stick Y: forward/back
- right stick X: turn
- L1: slow mode
- R1: boost
- Cross / A: stop

If your controller maps the right stick to a different SDL axis, try:
```bash
python3 scripts/sony_controller_udp.py --host 127.0.0.1 --port 5010 --right-x-axis 3
```

Important:
- after changing the devcontainer networking, use `Dev Containers: Rebuild and Reopen in Container`;
- the published UDP ports only exist after that rebuild.

### 5. What success looks like
In the `gazebo_drive_bridge` terminal you should see repeated logs like:
- `Drive command from /drives_controller/cmd_vel: linear_x=... angular_z=...`

In the `connect_node` terminal you should now also see repeated logs like:
- `Drive packet from ... -> lx=... ly=... rx=... ry=...`

In Gazebo:
- the rover should move or rotate as you move the controller;
- the `pose` values in the left panel should begin updating.

### 6. If the rover does not move
Check these in order:
1. Gazebo is not paused.
2. `connect_node` is running.
3. `gazebo_drive_bridge` is running.
4. The host script is printing changing `lx/ly/rx/ry` values.
5. The bridge terminal is printing `Drive command from /drives_controller/cmd_vel`.

Useful checks:
```bash
source /opt/ros/humble/setup.bash
source /cmr/terra2/install/setup.bash
ros2 topic echo /drives_controller/cmd_vel
```

```bash
source /opt/ros/humble/setup.bash
ros2 service list | grep -E 'apply_joint_effort|clear_joint_efforts'
```

## P) Quick manual drive test
With Gazebo running and `gazebo_drive_bridge` launched:

Forward:
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -r 5 /cmd_vel_drives geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

Rotate:
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -r 5 /cmd_vel_drives geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Stop:
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -1 /cmd_vel_drives geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Q) Make the Gazebo camera follow the rover
Fast manual option in the Gazebo GUI:
1. Expand `Models`.
2. Right-click `drives`.
3. Choose `Follow` if that menu option is available in your Classic Gazebo view.

If the GUI does not expose a follow action cleanly, use the view controls to keep the camera centered on `drives`. The current repo does not yet ship a custom world file that pins the camera to the rover automatically.

## R) Optional: run arm simulation (MoveIt + fake ros2_control)
```bash
ros2 launch moveit_servo servo_example.launch.py
```

Notes:
- This is RViz/FakeSystem simulation for the arm.
- It is not Gazebo physics simulation.

## S) If you insist on testing autonomy loop end-to-end without Gazebo
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
