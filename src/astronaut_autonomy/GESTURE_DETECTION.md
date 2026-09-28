# Human gesture detection

The gesture node is separate from the existing bottle/mallet detector. It uses
the official Ultralytics `yolo26n-pose.pt` model to locate the 17 COCO human
keypoints, then classifies and stabilizes these initial gestures:

- `hands_up`
- `left_arm_out`
- `right_arm_out`
- `both_arms_out`
- `none`

The labels describe the person's anatomical left and right. They are perception
results only; they do not directly command the rover.

## Prepare the rover

Run this once while the rover has internet access, then build the workspace:

```bash
./run gesture-setup
./run build
```

`gesture-setup` installs the Python dependency from `requirements.txt` and asks
Ultralytics to download `yolo26n-pose.pt` into
`src/astronaut_autonomy/config`. The weight file is ignored by Git but is
copied into the installed ROS package by the next build, allowing later field
runs without internet access.

PyTorch installation is platform-specific. On a Jetson, install the NVIDIA-
compatible PyTorch build before running `gesture-setup` if the device does not
already have it.

## Run it

Start the ZED publisher, then run the gesture detector in a separate terminal:

```bash
ros2 run cmr_zed threaded
./run gesture
```

The regular `./run auto` command does not start gesture detection. The existing
bottle/mallet detector and the new gesture detector remain separate, and both
can subscribe to `/zed/image` at the same time. For example, run the existing
detector in one terminal and the gesture detector in another:

```bash
ros2 run cmr_zed test_detection
./run gesture
```

Running both neural networks simultaneously increases GPU usage, so verify the
frame rate and thermals on the Jetson.

The node publishes:

- `/autonomy/human_gesture` (`std_msgs/String`)
- `/autonomy/human_gesture/confidence` (`std_msgs/Float32`)
- `/autonomy/human_gesture/debug_image` (`sensor_msgs/Image`)

The autonomy state machine does not act on these topics yet. That mapping must
be reviewed, implemented, and tested separately.

## Custom gestures

For simple static poses, extend `gesture_core.py` with additional keypoint
geometry and tests. For complex or time-dependent gestures such as waving,
collect labeled keypoint sequences and train a separate temporal gesture
classifier. The pretrained pose model can remain responsible for extracting
body keypoints.
