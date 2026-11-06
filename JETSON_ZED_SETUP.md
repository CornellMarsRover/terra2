# Jetson ZED astronaut-autonomy setup

Use this guide on the NVIDIA Jetson/Linux computer, from the root of the
`terra2` repository. Do not run the ZED depth node on macOS.

## 1. Install the matching ZED SDK and Python API

Check the Jetson Linux/L4T version:

```bash
cat /etc/nv_tegra_release
```

Install the Stereolabs ZED SDK release that exactly matches that L4T/JetPack
version. The official Jetson instructions are:

<https://docs.stereolabs.com/docs/development/zed-sdk/linux/work-with-nvidia-jetson>

After installing the SDK, install its matching Python API with the same system
Python that ROS 2 uses:

```bash
test -f /usr/local/zed/get_python_api.py
python3 /usr/local/zed/get_python_api.py
python3 -c "import pyzed.sl as sl; print('pyzed import OK')"
```

Do not use a generic `pip install pyzed`. The binding must match the installed
ZED SDK, JetPack/CUDA version, CPU architecture, and Python version.

If `get_python_api.py` is missing, stop and install the matching ZED SDK first.

## 2. Install the repository dependencies

```bash
source /opt/ros/humble/setup.bash
./run setup
./run gesture-setup
./run voice-setup
```

On a Jetson, install the NVIDIA-supported PyTorch build before running
`gesture-setup`. Do not replace it with an incompatible desktop PyTorch wheel.

Download the offline voice model if it is not already present:

```bash
mkdir -p models
if [ ! -d models/vosk-model-small-en-us-0.15 ]; then
  curl -fL -o /tmp/vosk-model-small-en-us-0.15.zip \
    https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  unzip /tmp/vosk-model-small-en-us-0.15.zip -d models
fi
```

## 3. Build only the required ROS packages

`--packages-up-to` also builds required dependencies such as `cmr_msgs`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to cmr_zed astronaut_autonomy
source install/setup.bash
```

## 4. Run the system

Terminal 1 — publish synchronized ZED image and depth data:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cmr_zed threaded
```

Terminal 2 — run human gesture recognition:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./run gesture
```

Terminal 3 — run command fusion, mission control, and offline voice fallback:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./run astronaut-mission \
  enable_voice:=true \
  voice_model_path:="$(pwd)/models/vosk-model-small-en-us-0.15"
```

The mission launch produces drive proposals only. Physical drive output remains
disabled unless the separately reviewed drive adapter is explicitly enabled.

## 5. Verify ROS data

```bash
ros2 topic hz /zed/image
ros2 topic type /camera/points
ros2 topic echo /autonomy/astronaut/perception
ros2 topic echo /autonomy/astronaut/mission_state
ros2 topic echo /autonomy/astronaut/drive_command
```

Expected camera topic types:

- `/zed/image`: `sensor_msgs/msg/Image`
- `/camera/points`: `sensor_msgs/msg/PointCloud2`

## `ModuleNotFoundError: No module named 'pyzed'`

First run:

```bash
python3 /usr/local/zed/get_python_api.py
python3 -c "import pyzed.sl as sl; print('pyzed import OK')"
```

If the direct import succeeds but `ros2 run cmr_zed threaded` still fails,
compare the Python interpreters:

```bash
which python3
python3 -m pip show pyzed
head -1 install/cmr_zed/lib/cmr_zed/threaded
```

The ROS executable and the Python installation containing `pyzed` must use the
same interpreter. Rebuild `cmr_zed` and `astronaut_autonomy` from the same
shell after correcting the environment.
