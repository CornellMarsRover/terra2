# Phase 1 GCS Audit — Verification Report

Independent verification of the Phase 1 audit documents on the
`astrotech-gui` branch. Static analysis only; the rover and a live ROS2
graph were not consulted.

Working-tree note: `git status` on entry was already dirty from Cursor's
in-progress work (`README.md` modified, `src/cmr_cams/package.xml`
modified, `docs/` and `launch/` untracked). No source file was changed,
no stash, no commit.

Inputs read:
- `docs/ros_interface_inventory.md`
- `docs/feature_to_topic_map.md`
- `docs/bandwidth_audit.md`
- `docs/foxglove_extension_plan.md`
- `docs/open_questions.md`
- `launch/gcs_bridge.launch.py`
- `src/cmr_cams/package.xml`
- `src/cmr_cams/launch/gui_server.launch.py`
- Primary sources under `src/` cross-referenced per task.

---

## 1. Inventory completeness

**VERDICT: PARTIALLY_CONFIRMED.**

The inventory covers every ROS2 package, all `cmr_msgs` types, and the
major nodes. It misses three real nodes in `cmr_controls` that are
registered entry points, and it is slightly wrong about the `ccb_read`
publish cadence.

### Packages — confirmed

Directory listing matches `docs/ros_interface_inventory.md §2` exactly:
`autonomous_navigation`, `autonomous_typing_package`, `cmr_arm_sim`,
`cmr_arm_simulator`, `cmr_aruco`, `cmr_cams`, `cmr_controller_remote`,
`cmr_controls`, `cmr_fabric`, `cmr_fabric_wrappers`, `cmr_imu`,
`cmr_msgs`, `cmr_param_gui`, `cmr_rovernet`, `cmr_rtkgps`, `cmr_utils`,
`cmr_zed`, `moveit_servo`, `ompl`, `temp_python_files`,
`usb_camera_publisher`, `zed-ros2-wrapper`. No package missed.

### Nodes the inventory missed

All three are registered in `src/cmr_controls/setup.py` entry_points but
are absent from §3 of the inventory:

- **`arm_controller_node`** — `src/cmr_controls/cmr_controls/arm_controller_node.py:12`.
  - Subscribes `/joint_angles/desired` (`std_msgs/Float32MultiArray`) —
    line 43.
  - Subscribes `/joint_angles/offsets` (`std_msgs/Float32MultiArray`) —
    line 49.
- **`ik_node`** — `src/cmr_controls/cmr_controls/ik_node.py:12`.
  - Publishes `/arm/end_effector/pose` (`Float32MultiArray`) — line 17.
  - Publishes `/joint_angles/desired` (`Float32MultiArray`) — line 62.
  - Subscribes `/cmd_vel` (`geometry_msgs/Twist`) — line 48.
  - Subscribes `/arm/joint_increment` (`Float32MultiArray`) — line 55.
- **`keyboard_controller_node`** — `src/cmr_controls/cmr_controls/keyboard_controller_node.py:10`.
  - Publishes `/cmd_vel` (`geometry_msgs/Twist`) — line 13.
  - Publishes `/arm/joint_increment` (`Float32MultiArray`) — line 16.
  - Timer at 10 Hz (`create_timer(0.1, …)`) — line 54.

These are a parallel arm-control path (controller + IK + setpoint node)
that conflicts with the `moveit_servo → armnet` path the inventory
already documents. Only one stack is meant to run at a time.

Also missed: **`kalman_localization`** is a registered entry point
(`autonomous_navigation/setup.py`) that subscribes `/rtk/navsatfix_data`
and `/zed/pose` and publishes `/autonomy/pose/robot/global`
(`src/autonomous_navigation/autonomous_navigation/kalman_localization.py:116-123`).
The inventory mentions it only in passing under "global_planner /
local_planner / costmap / controller / object_detection / led_node /
new_kalman / localization_sim" (§3.8) and gives its topics under the new
aggregated block, so this is defensible but easy to miss — the two
Kalman variants (`kalman_localization` and `new_kalman`) both publish
the same `/autonomy/pose/robot/global` topic; running both concurrently
would be a topic conflict. Flagging for completeness.

Also not separately listed (but all are legitimately skipped because
they are not entry points): `cmr_zed/cmr_zed/pose.py`,
`cmr_zed/cmr_zed/zed_gnss_fusion.py`, `cmr_zed/rover.py` (which is a
misplaced duplicate of `cmr_rtkgps/cmr_rtkgps/rover.py` sitting at
`src/cmr_zed/rover.py`, i.e. outside the Python package dir — it would
not be installed by `setup.py`).

### Rate / behavior mismatch — `ccb_read`

Inventory §3.4 says `/ccb/read` has "rate not set by timer in source
(read loop drives publish)". Source in `src/cmr_rovernet/cmr_rovernet/ccb_read.py:19`
actually sets `self.create_timer(0.001, self.publish_msg)` — a 1000 Hz
timer. The effective publish rate is capped by blocking serial reads
(14 sequential `serial.read(20)` calls per timer tick at 115200 baud,
timeout=1 s), so the inventory's functional claim (read-loop-bound) is
correct, but the source-level claim ("no timer") is wrong. Does not
change any Phase 2a decision but should be corrected.

### Other source-level things the inventory is correct about

- `connect_node` (cmr_controller_remote) publishers and topics match
  `src/cmr_controller_remote/cmr_controller_remote/connect.py:46-52`.
- `swerve_controller_node` (cmr_controls) four subscriptions match
  `swerve_controller_node.py:49-72`.
- `armnet_node` (cmr_rovernet) three subscriptions match
  `armnet.py:25-39`.
- `imu_node` 10 Hz timer matches `cmr_imu/cmr_imu/imu.py:22-23`.
- `gps_rover` 10 Hz timer matches `cmr_rtkgps/cmr_rtkgps/rover.py:66`.
- ZED variants (`zed_autonomy`, `pose`, `threaded`,
  `zed_camera_publisher`, `zed_gnss_fusion`) topics/rates match.
- USB cam publishers (`publisher.py`, `publish_images.py`,
  `stitched.py`, `bev.py`) topics/rates/resolutions match.
- State machine, costmap, controller, global/local planner, object
  detection, LED node, new_kalman all match source.
- `displacement_logger` typo (`/autonomy/pose/global/robot`) is real —
  `src/cmr_rtkgps/cmr_rtkgps/displacement_logger.py:16`.
- `cmr_msgs` message list, service list, and action list match the
  filesystem and the `CMakeLists.txt` exactly.
- `cmr_fabric` C++ services (Acquire/Release/NotifyDeactivate,
  ActivateNode/DeactivateNode/RecoverFault) all present per
  `src/cmr_fabric/src/*.cpp`.

### Impact on Phase 2a

Low. The three missing `cmr_controls` nodes are only relevant if the
mock rover has to impersonate the keyboard-arm-control stack; the
task-brief targets Astrotech, which is unrelated. Flag and move on.

---

## 2. ASTROTECH subteam claim

**VERDICT: CONFIRMED.**

### Evidence

- `ASTROTECH = 0x03` is defined at
  `src/cmr_rovernet/cmr_rovernet/rovernet_utils.py:14`, as a subteam ID
  in the serial protocol (alongside `DRIVES = 0x01`, `ARM = 0x02`,
  `RECEIVE_DATA = 0x04`, `BRAKE = 0x08`).
- Other references to the constant in the current tree:
  - `src/cmr_rovernet/rovernet_utils.py:270` — docstring only
    (`SUBTEAM (1): either 'drives', 'arm', 'astrotech', 'business'`).
  - `src/cmr_rovernet/rovernet_summary.txt:832,1048` — a summary log,
    not executed.
- Case-insensitive tree-wide grep for `astrotech` turns up zero
  additional hits in `src/` outside `rovernet_utils.py` and the summary
  text file. All other hits are in `docs/` (the audit itself).
- Tree-wide grep for `auger`, `raman`, `humidity`, `\bco2\b`,
  `mixing_servo`, `spectrometer` returns zero hits in `src/`. Only
  hits are in `docs/`.

### Cross-branch check

Grepped the same keyword set in every remote branch
(`Ishaan-Combine-IK-Keyboard`, `ab2458/IK_2025`, `anant-test-driving`,
`autonomy`, `autonomy_tools`, `autonomy_usama_testing`,
`backup/devon-ik-before-main-merge`, `camera_config`, `devon/IK_2026`,
`devon/swerve_spin`, `djl364/controls_stack`, `djl364/drives`,
`drive_controll_debug_usama`, `drive_debug`, `gazebo_sim`,
`saesha_test`, `testing-aruco`, `main`). Every branch reproduces exactly
the same two classes of hits: the `ASTROTECH = 0x03` constant and the
docstring/summary-text mentions. **No branch anywhere has a node,
driver, topic, service, or msg type touching auger, raman, humidity,
CO2, mixing servo, or site analysis.** The audit's claim is reproduced
uniformly across the repo's history.

The `ASTROTECH` constant is a reserved slot in the serial byte
protocol; nothing emits or decodes a frame with that subteam byte in
this workspace.

### Impact on Phase 2a

None — the claim the audit makes is the bedrock on which Phase 2a
scoping rests, and it is correct. Whatever the mock rover publishes for
Astrotech features in Phase 2a will be inventing the interface, not
matching an existing one.

---

## 3. SiteAnalyze service signature

**VERDICT: CONFIRMED.**

### Evidence

`src/cmr_msgs/srv/SiteAnalyze.srv`:

```
int8 site_num
---
bool success
```

That is the entire file (3 lines). No additional fields. Request is
`int8 site_num`, response is `bool success`. The audit reproduces this
exactly.

### Use sites

Grep across the tree: **`SiteAnalyze` is registered in
`src/cmr_msgs/CMakeLists.txt:38` and the `.srv` file itself. Nothing
else.** No `create_service<cmr_msgs::srv::SiteAnalyze>` in C++, no
`self.create_service(SiteAnalyze, ...)` in Python, no
`create_client` on it anywhere. It is never advertised and never
called.

### Git blame

`git log -p -- src/cmr_msgs/srv/SiteAnalyze.srv` shows the file was
added in commit `9d27f72e` on 2023-03-13 and has not been touched
since. No recent edits — the audit's characterization is stable.

### Impact on Phase 2a

Low — the field layout is confirmed, so the GCS panel spec in the
extension plan is correct (call with `site_num = 1` or `2`). Open
Question Q4 (semantics of `site_num` — sequence vs. site) remains
unresolved and is listed separately below.

---

## 4. Camera encoding path

**VERDICT: CONFIRMED (with portability caveats).**

### Evidence — message type and encoder

`src/usb_camera_publisher/usb_camera_publisher/publisher.py`:

- Import: `from foxglove_msgs.msg import CompressedVideo` (line 8).
- Publisher created at line 41 with type `CompressedVideo` on topic
  `f"camera_{cam_id}/h264"`.
- CompressedVideo message populated at lines 137-143: `vid.format = "h264"`,
  `vid.data = <byte-stream>`.
- Encoder is **GStreamer** (`gi` / `Gst`, lines 11-13), launched via
  `Gst.parse_launch(pipeline_str)` at line 111.
- Pipeline (line 98-108):
  ```
  v4l2src device=/dev/video<cam_id>
    ! image/jpeg, width=640, height=480
    ! jpegdec ! videoconvert
    ! [optional videocrop]
    ! videoconvert
    ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000
    ! h264parse config-interval=1
    ! capsfilter caps="video/x-h264,stream-format=(string)byte-stream"
    ! appsink …
  ```
- Encoder is `x264enc` (software H.264 from libx264). 2 Mbps target
  bitrate.

Source matches audit claim exactly.

### Runtime dependencies

The encoder is NOT a runtime Python dep; it is a GStreamer plugin. A
machine that runs this node must have:

1. **System GStreamer** (`libgstreamer1.0-0`).
2. GStreamer plugin packs: `gstreamer1.0-plugins-base`,
   `gstreamer1.0-plugins-good` (v4l2src, jpegdec, videoconvert,
   videocrop, capsfilter, appsink), `gstreamer1.0-plugins-bad`
   (`h264parse`), and `gstreamer1.0-plugins-ugly` (`x264enc` — ships
   here on Ubuntu because libx264 is GPL).
3. **Python GObject introspection bindings**: `python3-gi`,
   `gir1.2-gstreamer-1.0`, `gir1.2-gst-plugins-base-1.0` — NOT in
   `src/usb_camera_publisher/package.xml`.
4. The ROS2 package `foxglove_msgs` (declared in `package.xml` as
   `<exec_depend>foxglove_msgs</exec_depend>`, line 18).
5. The `package.xml` also declares `<exec_depend>foxglove_compressed_video_transport</exec_depend>`
   (line 13) which is `ros-humble-foxglove-compressed-video-transport`
   on Humble — used for image_transport integration, not strictly
   required for just publishing the message, but installed anyway
   because `rosdep` will pull it.
6. A real USB camera at `/dev/video{0,2,4,6,8,10}` — without it,
   `v4l2src` fails at pipeline start; the node logs
   `[cam<id>] parse_launch failed` and continues (no crash, no topic).

The calibration YAML path `/home/cmr/cmr/terra2/calib_dir` is
hardcoded at line 195. The node errors per-camera if a file is missing
but does not abort. **Portability bug already flagged by the audit.**

### Phase 2a mock-rover feasibility

Question: can a mock rover realistically publish
`foxglove_msgs/CompressedVideo` H.264?

**Yes, but with non-trivial plumbing.** Options in order of simplicity:

1. **Pre-encoded file replay (cheapest).** Ship a short H.264 .mp4 or
   raw `.h264` bytestream in the mock package. Parse it into access
   units (each IDR / non-IDR NALU) and publish one per frame as a
   `CompressedVideo` with `format="h264"`. Dependencies: `foxglove_msgs`
   and one of `ffmpeg-python` / `pyav` / a hand-rolled NAL splitter.
   No GStreamer needed. Downside: latency/jitter characteristics are
   artificial.

2. **Mirror the real pipeline.** Replicate the GStreamer chain but feed
   it from `videotestsrc pattern=ball` or a file via `filesrc`
   instead of `v4l2src`. Dependencies: same as the rover
   (system-level GStreamer + x264 + gi). Works cross-platform with some
   packaging pain on macOS (homebrew gstreamer) and Windows.

3. **Use OpenCV's FFmpeg-backed VideoWriter to an in-memory pipe** and
   wrap its H.264 bytestream into CompressedVideo messages. Python-only
   dep chain (`opencv-python` brings a bundled FFmpeg on PyPI).
   Adequate for a mock; not identical bitrate/keyframe interval as
   x264enc.

Minimum dependency set for option 1 (recommended for Phase 2a mock):

- `rclpy`
- `foxglove_msgs` (ROS2 message package)
- Python: `numpy` (only if decoding/resizing — typically not needed
  for replay)
- A static `.h264` asset (< 10 MB)

**Call**: the message type is achievable in the mock. The single
biggest scoping risk is that everyone on the Phase 2a team assumes they
need to mirror x264enc exactly. They do not. Phase 2a should
explicitly specify option 1 (file replay) as the mock's publisher and
save option 2 for when the mock gets integrated with real hardware.

### Impact on Phase 2a

Medium. Pick option 1 in the mock spec and Phase 2a has no encoder
problem. Pick option 2 by default and every mock dev spends a day
fighting GStreamer on their laptop.

---

## 5. Bandwidth audit sanity check

**VERDICT: CONFIRMED** for the three topics spot-checked.

### Spot-check 1: `/imu`

- Rate claim: 10 Hz. Source: `src/cmr_imu/cmr_imu/imu.py:22`
  (`self.timer_period = 0.1`). ✓
- Size claim: ~120 B. Schema `IMUSensorData.msg` is `10 × float64 + 3 ×
  int32 = 80 + 12 = 92 B` payload; CDR framing + ROS2 submessage
  overhead lifts it to roughly 100-120 B. Audit value is in the right
  envelope (within 1.3× of a narrower estimate).
- BW claim: ~10 kbps. `10 × 120 × 8 = 9.6 kbps`. ✓

### Spot-check 2: `/camera_<id>/h264`

- Rate claim: camera-native. Source: no explicit framerate cap on
  `v4l2src` in `publisher.py:99`. Framerate is driven by the camera
  hardware; 30 fps at 640×480 is typical for the Logitech-class USB
  cameras implied by MJPEG. ✓
- Bitrate claim: 2 Mbps per cam. Source: `x264enc … bitrate=2000`,
  `publisher.py:104`. The `x264enc bitrate` property is **kbit/s** per
  the GStreamer `x264enc` docs, so 2000 kbit/s = 2 Mbps. ✓
- Aggregate for 6 cams: 12 Mbps. Arithmetic correct. ✓

### Spot-check 3: `/camera/stitched_image`

- Rate claim: 10 Hz. Source: `stitched.py:200` (`publish_rate = 10`),
  timer at `1.0 / publish_rate`. ✓
- Size claim: ~922 KB. Source: stitched is a 2×2 mosaic of 320×240
  subimages, so the final image is `640 × 480 × 3 = 921,600 B` for
  bgr8 (`stitched.py:119`, `np.zeros((height * 2, width * 2, 3), …)`
  with `height, width = 240, 320`). ✓
- BW claim: ~74 Mbps. `10 × 921,600 × 8 ≈ 73.7 Mbps`. ✓

### Other observations (not full re-derivation)

- `/birdseye/image_raw` is 300×300 bgr8 — `bev.py:306` confirms
  `mosaic_w, mosaic_h = 300, 300`. Audit ~270 KB, ~22 Mbps is exact.
- `/ccb/read` bandwidth is overestimated by the audit's implicit
  assumption of 10 Hz. The 1000 Hz timer in `ccb_read.py:19` is
  serial-bound; real rate is whatever the 14 blocking `serial.read(20)`
  calls resolve to. Audit conclusion ("<50 kbps") is conservative
  either way.

No estimate is off by > 2×.

### Impact on Phase 2a

None. Bandwidth claims are defensible; the mock rover can use them as
given for link-budget planning.

---

## 6. Open-questions triage

Classifications: **ANSWERABLE_FROM_CODE** (the codebase has the
answer), **BLOCKS_PHASE_2A** (mock-rover build stalls without it),
**BLOCKS_PHASE_2B** (panel implementation needs it), **DEFERRABLE**
(waits for hardware integration).

| # | Question | Classification | Notes |
|---|---|---|---|
| 1 | Auger controller moteus vs Maxon | **BLOCKS_PHASE_2A** | Controller family dictates the interface shape (velocity topic vs. trigger service). The mock must pick one and publish/serve it so the panel and the mock agree. If deferred, either the mock or the panel will need rework. |
| 2 | Mixing servo controller family | **BLOCKS_PHASE_2A** | Same reasoning as Q1. Dictates whether the panel calls a service or publishes an enum topic. |
| 3 | BDC controller (MCU / Jetson / moteus) | **BLOCKS_PHASE_2A** | Determines whether `SiteAnalyze` stays a service or is upgraded to an action (which would change `cmr_msgs/CMakeLists.txt`). Mock must pick one. |
| 4 | Meaning of `site_num` in SiteAnalyze | **BLOCKS_PHASE_2B** | Mock can accept any `int8` and return success. Panel labels ("Seq 1" vs. "Site 1") depend on this — matters for UX correctness but not for mock data flow. |
| 5 | Raman driver exists elsewhere? | **BLOCKS_PHASE_2A** | If the team has a driver repo, the mock should match its message type instead of inventing `RamanSpectrum.msg`. If not, Phase 2a defines the type in `cmr_msgs`. Either way a decision must precede the mock. |
| 6 | CO2/humidity driver exists elsewhere? | **BLOCKS_PHASE_2A** | Same as Q5 for `EnvSample.msg`. |
| 7 | Snapshot naming / storage / retention | **BLOCKS_PHASE_2B** | Panel-only concern. Mock doesn't snapshot anything. |
| 8 | URC 2026 RF bandwidth cap | **DEFERRABLE** | Mock rover runs over loopback/localhost. Cap only matters when the `topic_whitelist` is frozen for the competition — a pre-event gate, not a development gate. |
| 9 | USB camera logical-name mapping | **BLOCKS_PHASE_2B** | Panels need labels like "auger_cam" to map to a topic. Mock can publish `camera_0/h264` etc. and the panel will resolve via a config file in 2b. |
| 10 | H.264 vs. CompressedImage (MJPEG) | **ANSWERABLE_FROM_CODE** + **BLOCKS_PHASE_2A**. Code already uses `foxglove_msgs/CompressedVideo` (H.264), and the audit recommends keeping it. That answers the "what ships today" half. The "should we change it?" half is a team call and must be decided before the mock picks a publish type. Recommendation: confirm "keep H.264" in Phase 2a planning and move on. |
| 11 | Fabric lifecycle for `foxglove_bridge` | **ANSWERABLE_FROM_CODE**. Phase 1 ships the bridge as a plain `launch_ros.Node` (`launch/gcs_bridge.launch.py:105`). **DEFERRABLE** for production hardening. Mock rover does not need Fabric. |
| 12 | External cam packages `cmr_cv`, `cmr_arm`, `cmr_control`, `cmr_demo` | **DEFERRABLE** | Referenced by `cmr_cams/config/*.toml` for on-rover bring-up. Mock rover doesn't load those TOMLs. Resolve at hardware-integration time. |
| 13 | Per-cam rate/bitrate exposed as GCS params | **DEFERRABLE** | Today `bitrate=2000` is hardcoded (`publisher.py:104`). Nice-to-have, not in Phase 2a critical path. Can ride Phase 2+. |

### Summary

- **Must answer before Phase 2a can build a coherent mock:** Q1, Q2,
  Q3, Q5, Q6, Q10.
- **Can slip to Phase 2b (panel implementation):** Q4, Q7, Q9.
- **Safe to defer to hardware-integration:** Q8, Q11, Q12, Q13.

---

## 7. Foxglove_bridge launch file review

**VERDICT: PARTIALLY_CONFIRMED.** Launch file is syntactically correct
and the QoS whitelist regexes correctly match the documented topic
names. It *does* conflict with `src/cmr_cams/launch/gui_server.launch.py`
if both are launched on default settings — a real risk worth flagging.

### QoS regex coverage (confirmed)

All `BEST_EFFORT_TOPIC_REGEXES` entries match a real topic in `src/`:

| Regex (line) | Matches publisher |
|---|---|
| `^/?camera_\d+/h264$` (50) | `usb_camera_publisher/publisher.py:41` (`f"camera_{cam_id}/h264"`) |
| `^/?camera_\d+/image_raw$` (51) | `publish_images.py:37` (`f"/camera_{cam_id}/image_raw"`) |
| `^/?camera_\d+/image_rect$` (52) | `bev.py:155` (`f"/camera_{cam_id}/image_rect"`) |
| `^/?camera/stitched_image$` (53) | `stitched.py:84` |
| `^/?birdseye/image_raw$` (54) | `bev.py:172` |
| `^/?zed/image(_left|_right)?$` (56) | `threaded.py:54` (`/zed/image`); `zed_camera_publisher.py:16,17` (`/zed/image_left`, `/zed/image_right`) |
| `^/?camera/points$` (58) | `zed_autonomy.py:23`, `pose.py:23`, `threaded.py:51`, `zed_gnss_fusion.py:30` |
| `^/?camera/ground_plane$` (59) | `zed_autonomy.py:24`, `pose.py:24`, `threaded.py:53` |
| `^/?imu$` (61) | `cmr_imu/imu.py:21` |
| `^/?zed/pose$` (63) | `zed_autonomy.py:26`, `pose.py:26`, `threaded.py:52` |
| `^/?autonomy/pose/robot/global$` (65) | `new_kalman.py:52`, `kalman_localization.py:121`, `localization_sim.py:51`, `zed_gnss_fusion.py:32` |

The optional-slash `^/?…$` form handles the publisher.py case where the
topic name is relative (`camera_<id>/h264` with no leading `/` — it
resolves to `/camera_<id>/h264` at runtime because the node has no
namespace). All regexes are ECMAScript-compatible as required by
`foxglove_bridge`.

### Launch argument / README cross-check (confirmed)

`README.md:49-51` says:

> **Launch:** `ros2 launch launch/gcs_bridge.launch.py`
>   - Optional args: `port:=8765 address:=0.0.0.0 tls:=false`
> **Connection URL from Foxglove Studio:** `ws://<rover_ip>:8765`

Launch file defaults match exactly: `port=8765` (line 74),
`address=0.0.0.0` (line 80), `tls=false` (line 84). Connection URL
`ws://<rover_ip>:8765` is consistent.

### Syntax / imports / undefined vars

- `ast.parse` of the file succeeds — no SyntaxError.
- Imports used: `LaunchDescription`, `DeclareLaunchArgument`,
  `LaunchConfiguration`, `Node`. All four are used.
- No undefined variables; `BEST_EFFORT_TOPIC_REGEXES` is defined at
  module scope and referenced on line 131.

One minor concern: `port`, `address`, `tls`, `use_sim_time`,
`num_threads`, and `send_buffer_limit` are passed as `LaunchConfiguration`
objects into the `parameters=[{...}]` block. ROS2 launch resolves these
into strings at launch time, and `foxglove_bridge` declares its
parameters with specific types (e.g. `port` is int, `tls` is bool).
In practice this works because `rclcpp` parameter declaration coerces
string → typed value, but it is the kind of thing that can break with
a future `foxglove_bridge` release. Not a defect in this PR; just
worth noting for Phase 2.

### Conflict with `cmr_cams/launch/gui_server.launch.py` (real)

`src/cmr_cams/launch/gui_server.launch.py` is a three-statement launch
file that:

1. Declares `use_compression` (unused in the body).
2. `IncludeLaunchDescription(FrontendLaunchDescriptionSource(...foxglove_bridge_launch.xml))`
   from the `foxglove_bridge` share — which itself starts a
   `foxglove_bridge` node on the default port **8765**.

So:

- Running **only** `gui_server.launch.py` → one bridge on 8765, no QoS
  overrides (which is the pre-Phase-1 behavior the new launch file is
  supposed to replace).
- Running **only** `gcs_bridge.launch.py` → one bridge on 8765 with QoS
  overrides. Fine.
- Running **both** concurrently (e.g. someone launches the rover's
  default stack which still calls `gui_server.launch.py` and then also
  launches the new GCS bridge) → both try to start a node called
  `foxglove_bridge` and both try to `bind()` TCP port 8765. The
  second-started bridge will fail with "Address already in use" and
  exit. This is not a subtle race; it's a deterministic failure.

Phase 1 did not update `gui_server.launch.py` to stop including the
foxglove launch, nor did it remove `gui_server.launch.py`. Net:
`README.md` now points people at `launch/gcs_bridge.launch.py`, but
`src/cmr_cams/launch/gui_server.launch.py` is still wired into older
bring-up paths. A harness or rover-startup script that still runs
`gui_server.launch.py` will silently double-bind.

**Recommendation (not implemented — Phase 1 scope is audit-only):**
Either delete the foxglove include from `gui_server.launch.py` now,
or add an explicit note to that file's header that it is superseded by
`launch/gcs_bridge.launch.py`. This is a one-line fix and belongs in
the Phase 1 PR, not in Phase 2. Flagging as a Phase 1 gap.

### Impact on Phase 2a

Medium-low. Mock rover does not need `gui_server.launch.py`. Still,
anyone running the real rover's camera stack and the new GCS bridge
side-by-side is going to hit the port collision on first try. Fix
before Phase 2a turns on the mock over a live bridge.

---

## Go/No-Go for Phase 2a

**RECOMMENDATION: CONDITIONAL GO.**

The audit is substantively correct on every load-bearing claim:
- ASTROTECH subteam is a reserved constant with no implementation
  anywhere in the repo or its branches. **Confirmed.**
- `SiteAnalyze.srv` is `int8 site_num → bool success`, unused.
  **Confirmed.**
- `usb_camera_publisher` produces H.264 inside
  `foxglove_msgs/CompressedVideo` via a GStreamer x264enc pipeline.
  **Confirmed.** Mock rover can replay pre-encoded H.264 without
  GStreamer.
- Bandwidth estimates are within tolerance.
- Launch file is well-formed and QoS regexes match the real topics.

### Items that MUST be resolved before Phase 2a starts

1. **Auger controller family (Q1).** Pick moteus or Maxon. Determines
   the mock's auger interface contract. Without this, either the mock
   or the panel will be rewritten.
2. **Mixing servo controller family (Q2).** Same as above for the
   mixing servo.
3. **BDC / analysis-sequence location (Q3).** Decides service vs.
   action. If action, `cmr_msgs/CMakeLists.txt` gets a new entry and
   `TestTargetPosition.action` is no longer the only action.
4. **External driver search for Raman (Q5) and CO2/humidity (Q6).** If
   drivers live in a parallel repo, the mock should match their msg
   types instead of `cmr_msgs` adding new types. If they don't,
   Phase 2a authors them.
5. **Confirm "keep H.264" for cameras (Q10).** Audit recommends it;
   team should bless it in writing so the mock commits to
   `CompressedVideo`.

### Items that SHOULD be resolved before Phase 2a merges

6. Add the three missing `cmr_controls` nodes (`arm_controller_node`,
   `ik_node`, `keyboard_controller_node`) and the `kalman_localization`
   node to `docs/ros_interface_inventory.md` §3 for completeness.
7. Fix the `ccb_read` rate claim in §3.4 / §5 (it's a 1000 Hz timer
   bounded by serial I/O, not "no timer").
8. Resolve the `gui_server.launch.py` ↔ `gcs_bridge.launch.py` port-8765
   conflict with either a file deletion/rewrite or at minimum a header
   comment.

### Items that can wait

- Q4 (`site_num` semantics), Q7 (snapshot), Q9 (cam name mapping) —
  Phase 2b.
- Q8 (RF cap), Q11 (Fabric lifecycle), Q12 (external cam packages),
  Q13 (param-exposed bitrate) — hardware integration.

No hard blockers were discovered. The ASTROTECH gap the audit built
its plan around is real, the bridge launch ships a valid QoS policy,
and the camera pipeline is mock-able. Proceed to Phase 2a once items
1-5 above are answered by the team.
