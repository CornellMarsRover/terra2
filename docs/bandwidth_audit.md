# Bandwidth Audit

Phase 1 estimate of per-topic bandwidth for every topic the rover publishes,
so we can decide which ones must be compressed, throttled, remapped, or kept
off the RF link entirely. Numbers are **computed from source-declared rates
and message shapes**, not measurements. Once the rover is running, these need
to be replaced with `ros2 topic bw` values.

## Assumptions

- Rates: taken from `create_timer(period, …)` calls in source, or documented
  defaults where the timer is data-driven.
- `sensor_msgs/Image` at resolution `W×H`, encoding `bgr8`: `W*H*3` bytes
  payload, plus ~80 B ROS header → rounded up to `W*H*3 + 96 B` for safety.
- `sensor_msgs/PointCloud2` from ZED2i at VGA resolution: 672×376 points ×
  16 B/point (XYZRGBA, 4 float32 fields packed to 16 B with 1-byte-aligned
  RGBA) ≈ 4.0 MB/frame. The ZED SDK's actual output can be 32 B/point with
  normals; 16 B is the conservative lower bound. At 10 Hz this alone is
  ~320 Mbps.
- `foxglove_msgs/CompressedVideo` H.264: bitrate from the GStreamer pipeline
  in `publisher.py` (`x264enc … bitrate=2000`, i.e. **2 Mbps** target per
  camera).
- `cmr_msgs/IMUSensorData` wire size ≈ 120 B after CDR encoding (13
  float64 + 3 int32).
- `sensor_msgs/NavSatFix` ≈ 88 B on the wire.
- `trajectory_msgs/JointTrajectory` carrying 6 joints × 1 point: ≈ 400 B.
- `geometry_msgs/TwistStamped`: 80 B. `sensor_msgs/Joy`: 40 B per entry × (16
  buttons + 8 axes) ≈ 200 B.
- URC 2026 RF bandwidth cap is not confirmed from this codebase (open
  question Q6). Historical URC caps have been **5 Mbps** ingress sustained.
  The audit flags anything ≥ 1 Mbps as a concern under that assumption.

## Per-topic table

Format:
- **Msg size** = rough CDR-encoded bytes per message.
- **BW** = `rate × size × 8` bits/s.

| Topic | Type | Rate | Msg size | BW | Verdict |
|---|---|---|---|---|---|
| `/zed/image` | `sensor_msgs/Image` (bgr8 672×376) | 10 Hz | ~760 KB | **~61 Mbps** | **BLOCKER** — raw. Never send over RF. |
| `/zed/image_left` | `sensor_msgs/Image` (bgr8 672×376) | 10 Hz | ~760 KB | **~61 Mbps** | **BLOCKER** — raw. |
| `/zed/image_right` | `sensor_msgs/Image` (bgr8 672×376) | 10 Hz | ~760 KB | **~61 Mbps** | **BLOCKER** — raw. |
| `/camera/points` | `sensor_msgs/PointCloud2` | 10 Hz | ~4 MB | **~320 Mbps** | **BLOCKER** — do not forward over RF. |
| `/camera/ground_plane` | `cmr_msgs/GroundPlaneStamped` | 10 Hz | depends on sampling (varies, tens of KB typical) | ~1–10 Mbps | **WARN** — throttle or downsample. |
| `/camera_<id>/h264` (×6) | `foxglove_msgs/CompressedVideo` | camera native | 2 Mbps encoded | **~2 Mbps each → 12 Mbps for all 6** | **WARN** — drop unused cams; only forward the 1–3 cams the operator needs at once. |
| `/camera_<id>/image_raw` (×4, 320×240) | `sensor_msgs/Image` | 10 Hz | ~230 KB | ~18 Mbps each | **BLOCKER** — raw; do not expose on bridge. Replace with H.264 variant. |
| `/camera_<id>/image_rect` (×4, 640×480) | `sensor_msgs/Image` | 10 Hz | ~922 KB | ~74 Mbps each | **BLOCKER** — raw. |
| `/camera/stitched_image` | `sensor_msgs/Image` (640×480) | 10 Hz | ~922 KB | ~74 Mbps | **BLOCKER** — raw. |
| `/birdseye/image_raw` | `sensor_msgs/Image` (300×300) | 10 Hz | ~270 KB | ~22 Mbps | **BLOCKER** — raw. |
| `/imu` | `cmr_msgs/IMUSensorData` | 10 Hz | ~120 B | ~10 kbps | OK |
| `/rtk/navsatfix_data` | `sensor_msgs/NavSatFix` | 10 Hz | ~88 B | ~7 kbps | OK |
| `/zed/pose` | `geometry_msgs/TwistStamped` | 10 Hz | ~80 B | ~6 kbps | OK |
| `/autonomy/pose/robot/global` | `geometry_msgs/TwistStamped` | ≤10 Hz | ~80 B | ~6 kbps | OK |
| `/main_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | ~30 Hz | ~400 B | ~100 kbps | OK |
| `/drives_controller/cmd_vel` | `geometry_msgs/TwistStamped` | ≤50 Hz | ~80 B | ~32 kbps | OK |
| `/drives_controller/cmd_buttons` | `cmr_msgs/ControllerReading` | event | ~80 B | <10 kbps typical | OK |
| `/arm_controller/cmd_vel` | `sensor_msgs/Joy` | ≤50 Hz | ~200 B | ~80 kbps | OK |
| `/arm_controller/cmd_buttons` | `cmr_msgs/ControllerReading` | event | ~80 B | <10 kbps typical | OK |
| `/mini_arm_controller/cmd_pos` | `cmr_msgs/MiniArmDegree` | event | ~56 B | <5 kbps | OK |
| `/ccb/read` | `cmr_msgs/MotorReadData` (14 × MotorData) | read-loop | ~600 B | <50 kbps even at 10 Hz | OK |
| `/autonomy/costmap` | `std_msgs/Float32MultiArray` | ≤10 Hz | depends on grid size (N×N floats). 200×200 = 160 KB → ~13 Mbps. 100×100 = 40 KB → ~3 Mbps | **WARN** — grid-size-dependent; verify at runtime. |
| `/autonomy/target/*`, `/autonomy/path/*`, `/autonomy/move/*`, `/autonomy/led`, `/autonomy/target_object/*` | various small msgs | <10 Hz | <200 B | <20 kbps | OK |
| Fabric lifecycle services | `cmr_msgs/*` | on-demand | few hundred B | negligible | OK |

**Hot total (what we'd send if nothing changed):**
`/camera/points` alone is already ~60× the URC bandwidth cap under our
assumption. No RF link will survive the current default topic set.

## Categorized recommendations

### 1. Must-not-forward over RF (blockers)

Configure the GCS bridge `topic_whitelist` (or an `exclude_topics` block at
the Foxglove layer) to **exclude** the following until we have a downsampled
version:

- `/zed/image`, `/zed/image_left`, `/zed/image_right`
- `/camera/points`
- `/camera_<id>/image_raw`, `/camera_<id>/image_rect`
- `/camera/stitched_image`
- `/birdseye/image_raw`

Phase 1's `launch/gcs_bridge.launch.py` does not yet set `topic_whitelist`
because we want the initial bring-up to show everything on a wired test link.
**Before competition**, convert `BEST_EFFORT_TOPIC_REGEXES` into a hard
whitelist that only permits allowed topics.

### 2. Compression fixes (must happen in Phase 2 / driver land)

- Every `sensor_msgs/Image` publisher in this repo should gain a companion
  `image_transport` republisher emitting `sensor_msgs/CompressedImage` (JPEG
  quality 70) on `<topic>/compressed`, **or** be replaced entirely by the
  existing `foxglove_msgs/CompressedVideo` H.264 pipeline in
  `usb_camera_publisher/publisher.py`.
  - The H.264 pipeline is the preferred path on the USB cams because the
    driver is already in place, publishes at 2 Mbps, and is what Foxglove's
    built-in "H.264 Video" panel expects.
  - For the ZED streams, add an `image_transport_plugins` re-publisher
    configured to emit `/zed/image/compressed` at JPEG quality 60–70 and
    stop subscribers that do not need the raw stream.
- `/camera/points` must not leave the rover. If the GCS needs a visual
  indication of point-cloud content, publish a decimated 2D occupancy grid
  (`nav_msgs/OccupancyGrid` at 1–5 Hz) instead — that's already roughly what
  `autonomy/costmap` is.

### 3. Rate throttling

- Camera H.264 pipeline does not pin a frame rate (`v4l2src` is unconstrained
  in `publisher.py`). Add a `v4l2src … ! "image/jpeg,framerate=10/1,width=640,height=480"`
  cap to pin to 10 fps and cut bandwidth proportionally.
- The operator almost never needs all 6 USB cams at once. Add a Foxglove
  extension (see extension plan) that lets the operator pick 1–3 active cams,
  and only subscribe to those topics; `foxglove_bridge` already tears down
  subscriptions on the ROS side when no client is subscribed on the Foxglove
  side, so this is a live savings.
- `/autonomy/costmap` should be throttled to 1–2 Hz on the bridge even if
  the node publishes faster locally — Foxglove doesn't need a 10 Hz grid to
  show costmap state.

### 4. Per-topic QoS (what Phase 1 sets)

See `launch/gcs_bridge.launch.py` — camera/image/pointcloud/IMU/pose topics
are `best_effort_qos_topic_whitelist`'d, so the bridge subscription is:

- `RELIABILITY = BEST_EFFORT`
- `HISTORY = KEEP_LAST`, `DEPTH = 1` (via `min_qos_depth = 1`, `max_qos_depth = 10` with the topic-default depth capped by each publisher's own advertised depth)

This ensures we never back-pressure the rover side when the RF link drops
packets.

Command and service topics (not in the best-effort whitelist) stay at
reliable by default.

### 5. Budget under an assumed 5 Mbps cap

With the recommendations above applied (all raw Image off RF, 3 of 6 H.264
cams selected, costmap at 2 Hz, point cloud off):

| Category | BW |
|---|---|
| 3× H.264 camera streams | 6.0 Mbps |
| /autonomy/costmap 100×100 @ 2 Hz | 0.6 Mbps |
| /imu + /rtk/navsatfix_data + poses + joint telemetry | <0.3 Mbps |
| Control command backflow + service calls | <0.2 Mbps |
| **Total** | **~7.1 Mbps** |

**Still over.** To fit 5 Mbps:

- Drop to 2 cams (saves 2 Mbps) → total ≈ 5.1 Mbps. Just over.
- **Or** drop the per-camera H.264 bitrate from 2 Mbps to 1.2 Mbps in
  `publisher.py` (`bitrate=1200`) — quality loss is visible but acceptable
  at 640×480 for most rover tasks. Three cams at 1.2 Mbps = 3.6 Mbps. Total
  ≈ 4.7 Mbps. Fits.

This bitrate decision should be a parameter on the camera publisher; today
it is hardcoded. Listing as a Phase 2 action item.

## Open items for bandwidth

- Q6 in `open_questions.md`: confirm URC 2026 RF cap.
- Measure actual `ros2 topic bw` for every topic once the rover is bring-up.
- Check whether `usb_camera_publisher/publisher.py` should gain a rate param
  so the GCS can reduce fps under poor link conditions.
- Decide whether point cloud is sent as a 2D decimation (`OccupancyGrid`) or
  fully omitted from the GCS; recommendation above is omit.
