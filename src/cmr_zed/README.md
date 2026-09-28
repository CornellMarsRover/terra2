# CMR ZED Human Gesture Detection

This package contains the ZED camera publisher and an experimental human
gesture detector for astronaut autonomy. The gesture detector uses an
Ultralytics YOLO pose model to locate a person's body keypoints and classify
simple static arm gestures.

## System overview

The currently working perception path is:

```text
ZED camera node (`threaded`)
        |
        | /zed/image
        v
Human gesture detection node
        |
        +-- /autonomy/human_gesture
        +-- /autonomy/human_gesture/confidence
        +-- /autonomy/human_gesture/debug_image
```

The intended autonomy path is:

```text
Gesture label -> autonomy state machine -> drive command -> rover controller
```

The autonomy state machine is **not connected to the gesture topics yet**. The
gesture node currently performs perception and publishes results only; it does
not move the rover. When integration is added, the state machine should
subscribe to the gesture label and confidence topics rather than process the
camera image itself.

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
- A Jetson-compatible PyTorch installation when running on a Jetson
- Internet access during the first model setup
- Enough GPU memory and compute capacity for real-time inference

The camera node must publish `sensor_msgs/Image` messages on `/zed/image`. Note
that `/zed/image` is singular; `/zed/images` is not used by this node.

## First-time setup

Run these commands from the root of the `terra2` repository:

```bash
./run setup
./run gesture-setup
./run build
```

The commands perform these tasks:

- `./run setup` installs ROS package dependencies declared in `package.xml`.
- `./run gesture-setup` installs `ultralytics` from `requirements.txt` and
  downloads `yolo26n-pose.pt` into `src/cmr_zed/config`.
- `./run build` builds the ROS workspace and copies the configuration and pose
  weights into the installed `cmr_zed` package.

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
starts the `human_gesture_detection` ROS node.

## Verifying the system

Confirm that the camera is publishing at a steady rate:

```bash
ros2 topic hz /zed/image
```

Read the recognized gesture and its confidence:

```bash
ros2 topic echo /autonomy/human_gesture
ros2 topic echo /autonomy/human_gesture/confidence
```

View the annotated image if `rqt_image_view` is installed:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/autonomy/human_gesture/debug_image` in the image viewer.

The initial labels are:

- `hands_up`
- `left_arm_out`
- `right_arm_out`
- `both_arms_out`
- `none`

## Current limitations

- Gesture results do not yet control the autonomy state machine.
- Purple-clothing detection has not been implemented. The node may process any
  visible person rather than only the astronaut.
- The current gestures use programmed body-keypoint rules; a custom trained
  gesture classifier has not been added yet.
- The body-pose model locates wrists but not individual fingers, so detailed
  signs such as a fist or thumbs-up require a hand-landmark model.
- Running gesture and bottle/mallet detection together may reduce frame rate or
  raise Jetson temperatures.

More implementation details are available in
[`GESTURE_DETECTION.md`](GESTURE_DETECTION.md).

## Common problems

**No gesture messages appear**

Check that `/zed/image` exists and is receiving messages:

```bash
ros2 topic list | grep zed
ros2 topic hz /zed/image
```

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
