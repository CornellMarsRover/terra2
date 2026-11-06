# Human gesture detection

The gesture node is separate from the existing bottle/mallet detector. It uses
the official Ultralytics `yolo26n-pose.pt` model to locate the 17 COCO human
keypoints, then classifies and stabilizes these gestures:

- `FOLLOW`: both hands on hips, elbows out
- `STAY`: horizontal T-pose
- `FETCH`: left hand at the neck and right arm pointing
- `COME`: hands together above the head
- `GIVE`: crossed forearms over the chest
- `NONE`

The labels describe the person's anatomical left and right. Display mirroring
does not swap these keypoint identities. The node also maps the detected torso
into the synchronized organized ZED point cloud and obtains a robust median 3-D
position. These are perception results only; they do not directly drive the
rover.

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

- `/autonomy/astronaut/gesture_perception` (`cmr_msgs/AstronautCommand`)
- `/autonomy/astronaut/distance` (`std_msgs/Float32`; NaN if unavailable)
- `/autonomy/astronaut/target` (`geometry_msgs/PointStamped`)
- `/autonomy/human_gesture` (`std_msgs/String`)
- `/autonomy/human_gesture/confidence` (`std_msgs/Float32`)
- `/autonomy/human_gesture/debug_image` (`sensor_msgs/Image`)

`AstronautCommand.gesture` is the stabilized observed pose. Its `command` is
the safe value for a downstream state machine: FOLLOW becomes NONE unless
astronaut depth is currently `VALID`. FETCH remains recognizable without
astronaut depth because its nearby hammer localization belongs to the future
arm coordinator. COME is checked for valid target localization by the mission
controller before movement. A recently lost measurement may be exposed as
`STALE` for diagnostics but never authorizes motion. Coordinates are meters in
the ZED frame: x forward, y left, z up.

The command-fusion node combines this topic with offline voice observations and
publishes `/autonomy/astronaut/perception` for the existing mission state
machine. The gesture node and fusion node never publish directly to the drive
mux.

## Custom gestures

For simple static poses, extend `gesture_core.py` with additional keypoint
geometry and tests. For complex or time-dependent gestures such as waving,
collect labeled keypoint sequences and train a separate temporal gesture
classifier. The pretrained pose model can remain responsible for extracting
body keypoints.
