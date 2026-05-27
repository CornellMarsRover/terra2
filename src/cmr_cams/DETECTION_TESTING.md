# URC Object Detection

Detects the three URC autonomy objects — **orange rubber mallet** (`mallet`),
**rock pick hammer** (`ice_pick`), **water bottle** (`bottle`) — on the base
station, draws one box on the telemetry panel, and stops the rover.

---

# Part 1 — Quick test guide

Goal: confirm the base-station YOLO detector sees a mission object, draws ONE box
on the telemetry panel, and STOPS the rover. ~10 min, no driving needed.

## Prereqs
- Base laptop + Jetson on the **same ROS 2 LAN** (radio up). Check: on the base,
  `ros2 topic list` shows `/zed/image_left`.
- Base laptop has `ultralytics`, `torch`, `cv2`. Model is at
  `cmr_cams/config/urc_objects_v9.pt`.
- Build once on both machines: `colcon build && source install/setup.bash`
  (regenerates `cmr_msgs/Detection`).
- Objects on hand: orange mallet, rock pick hammer, water bottle.

## 1. Bench test (no nav) — detector + overlay
1. **Jetson:** `ros2 run cmr_zed zed_autonomy` then `ros2 run autonomous_navigation live_telemetry_tool`
2. **Base:** `ros2 launch cmr_cams base_detection.launch.py`
3. In the GUI **Camera Feed** panel, tick **"Show detection overlay"**.
4. Hold an object in view. ✅ Expect: exactly **one** green box on it with label
   (`mallet`/`bottle`/`ice_pick`) + confidence. Multiple objects → only the
   highest-confidence one is boxed.

## 2. Arm + stop test (no driving)
Simulate the rover being "in vicinity" so the detector arms and triggers a stop.
1. With step 1 running, **arm a target** (base laptop):
   ```
   ros2 topic pub -r 2 /autonomy/detection/active_target std_msgs/String "{data: 'mallet'}"
   ```
2. Show the mallet. ✅ Expect:
   - Big red **"OBJECT DETECTED: MALLET — ROVER STOPPED"** banner in the GUI.
   - `ros2 topic echo /autonomy/motion/stop` → `data: true`.
3. Stop the publish (Ctrl-C) and re-publish `{data: 'none'}`. ✅ `/autonomy/motion/stop` → `false`.

> The real `state_machine` publishes `active_target` automatically once within a
> target's vicinity radius; step 1's manual publish just stands in for it.

## 3. Verify the rover halts
With the controller running (`full_auton` or nav launch), repeat step 2.
✅ While `/autonomy/motion/stop` is `true`, `ros2 topic echo /autonomy/move/ackerman`
shows `vel: 0.0`, and `/autonomy/move/move_type` shows `stopped`.

## 4. Full run (one click)
Launch Hub (`python3 scripts/jetson_autonomy_launcher.py`) → **"FULL AUTON RUN
(nav + detect)"**. (Start RTK corrections + GPS rover first.) Set the mission
object waypoints in the `self.targets` map in `state_machine.py`.

## Knobs (if needed)
- Misses objects → lower `conf_threshold` (default 0.50).
- Laptop too slow → raise `process_every_n_frames` (default 5 = ~2 Hz).
- Radio chokes on video → `use_compressed:=true` + point `image_topic` at a
  compressed topic.
- Box misaligned → GUI and detector must use the **same** `image_topic`.

## If it fails
- No box: is `/zed/image_left` visible on the base? Is the overlay checkbox on?
- No banner/stop: confirm `active_target` matches the object class and confidence
  ≥ `conf_threshold` (`ros2 topic echo /autonomy/detection/result`).
- No `Detection` type: rebuild `cmr_msgs` and re-source on **both** machines.

<br><br><br><br><br><br><br><br>

---
---

<br><br><br><br><br><br><br><br>

# Part 2 — How it works (documentation of the changes)

## Architecture (where things run)

- The **Jetson** runs the nav stack, the ZED camera node, and the telemetry GUI.
- The **base laptop** runs `base_detection_node`. Inference is on the base, not
  the Jetson. It taps the ZED feed over the shared ROS 2 graph (same radio LAN,
  default DDS domain), runs YOLO at a reduced sample rate, and sends the
  decision back as ROS topics.
- The **"object detected → stop" decision is made on the base** and round-trips
  to the rover as a latched `Bool`. No human is in the loop; the GUI checkbox
  only toggles the on-screen overlay.

```
base laptop:  /zed/image_left ─▶ base_detection_node (YOLO) ─┬▶ /autonomy/detection/result ─▶ GUI draws ONE box + banner
                                                             └▶ /autonomy/motion/stop (Bool) ─▶ controller halts (10 Hz, deterministic)
rover:        state_machine ─▶ /autonomy/detection/active_target ─▶ arms the detector near each object's GNSS coordinate
              state_machine ─▶ /autonomy/detection/arrived (Bool) ─▶ GUI banner + future LED (authoritative onboard state)
```

## Code map — every file in this subsystem

The subsystem spans three packages. Jump to the named function in each file
(grep `URC object-detection` to find the blocks).

| File | What it does | Look at |
|---|---|---|
| `cmr_cams/cmr_cams/base_detection.py` | Base node: YOLO inference, picks ONE box, makes the stop decision | `process_frame`, `choose_one` |
| `cmr_cams/launch/base_detection.launch.py` | Base-side launch + all node params | — |
| `cmr_cams/config/urc_objects_v9.pt` | The YOLO model (`mallet`/`bottle`/`ice_pick`) | — |
| `cmr_msgs/msg/Detection.msg` | The result message (1 box + `detected`) | — |
| `autonomous_navigation/.../controller.py` | The actual rover halt (10 Hz, deterministic) | `update_stop`, top of `follow_waypoint` |
| `autonomous_navigation/.../state_machine.py` | Arms detector near each object, dwell, advance | `update_detection`, `control_loop`, `self.targets` |
| `autonomous_navigation/tooling/Live_telemetry_tool.py` | Draws the ONE box + banner; overlay checkbox | `draw_detection_overlay`, `detection_cb` |
| `autonomous_navigation/launch/full_auton.launch.py` | Jetson: full nav stack + telemetry GUI | — |
| `scripts/jetson_autonomy_launcher.py` | Launch Hub one-click button | `launch_full_auton`, `FULL_AUTON_*` |

## How to launch (reference)

One click in the **Launch Hub** (`scripts/jetson_autonomy_launcher.py`):
**"FULL AUTON RUN (nav + detect)"** — boots the Jetson nav stack + telemetry GUI
over SSH *and* the base detection node locally. Start RTK corrections (base) and
GPS rover (Jetson) first, as in the normal workflow.

Manually / individually:
```bash
# Base laptop (detection only):
ros2 launch cmr_cams base_detection.launch.py

# Jetson (full nav + telemetry GUI):
ros2 launch autonomous_navigation full_auton.launch.py
```

## Topics

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/zed/image_left` | `sensor_msgs/Image` | in | ZED feed (bgra8, HD1080, ~10 Hz). Crosses the radio link. |
| `/autonomy/detection/active_target` | `std_msgs/String` | in | Armed YOLO class (`mallet`/`bottle`/`ice_pick`) or `none`. From state machine. |
| `/autonomy/detection/result` | `cmr_msgs/Detection` | out | The single best box (normalized coords) + `detected` flag. To GUI + state machine. |
| `/autonomy/motion/stop` | `std_msgs/Bool` | out | Latched STOP → controller (`controller.py` zeroes velocity). |
| `/autonomy/detection/arrived` | `std_msgs/Bool` | out (state machine) | Onboard arrived/detected state, held through the dwell. Drives the GUI banner / future LED. |

## Parameters (`base_detection_node`)

| Param | Default | Meaning |
|---|---|---|
| `conf_threshold` | `0.50` | Confidence to latch STOP on the armed object. |
| `display_conf_threshold` | `0.25` | Confidence to draw a box on the panel. |
| `process_every_n_frames` | `5` | Sample 1 in N frames (~10 Hz feed → ~2 Hz). |
| `model_file` | `urc_objects_v9.pt` | YOLO model in `config/`. |
| `imgsz` | `640` | Inference image side (model trained at 640). |
| `image_topic` | `/zed/image_left` | Feed to run inference on. |
| `use_compressed` | `false` | Set true if `image_topic` is a `CompressedImage` (use if raw HD1080 saturates the radio). |

## State machine params (`autonomous_navigation` `state_machine`)

| Param | Default | Meaning |
|---|---|---|
| `detection_vicinity_radius` | `5.0` | Distance to a YOLO object's GNSS coordinate at which sampling starts (fallback when the per-target `search_radius` is unset). Tune to GNSS error: ~3 m objects 1-2, ~10 m object 3. |
| `detection_hold_seconds` | `4.0` | How long the rover holds stopped after a detection before marking the target done and advancing. |

To make a waypoint a detect-and-stop object, give it a YOLO class name in the
`self.targets` map in `state_machine.py` (see the commented examples there); its
`search_radius` is the vicinity at which detection sampling begins.

## GUI overlay

The telemetry GUI (`autonomous_navigation/tooling/Live_telemetry_tool.py`) draws
**exactly one** bounding box (the single detection reported) plus a large
**"OBJECT DETECTED: … — ROVER STOPPED"** banner. Toggle with the **"Show
detection overlay"** checkbox on the Camera Feed panel.
