# GCS Feature → ROS2 Interface Map

Phase 1 mapping of each operator-facing GCS feature to the ROS2 interface that
would implement it, based on the current state of `src/`. Nothing below is
invented; if the interface is not present in the repo it is flagged as
**MISSING** and proposed under a `// proposed` marker for Phase 2 planning
only — Phase 1 must not commit any of the proposed interfaces.

## Summary

| Feature | Hardware (per task brief) | Existing interface | Status |
|---|---|---|---|
| Auger up / down | moteus (team confirmed) | — | MISSING |
| Auger spin fwd / back | moteus (team confirmed) | — | MISSING |
| Analysis Seq 1 start | BDC controller | `cmr_msgs/srv/SiteAnalyze` *(unused)* | PARTIAL (msg defined, no server) |
| Analysis Seq 2 start | BDC controller | `cmr_msgs/srv/SiteAnalyze` *(unused)* | PARTIAL |
| Mixing servo: S1, S2, CO2_1, CO2_2, Retract | servo | — | MISSING |
| Auger camera | USB camera | `camera_<id>/h264` + `camera_<id>/camera_info` | PRESENT, needs logical-name mapping |
| Site camera | USB camera | same | PRESENT, needs logical-name mapping |
| Analysis camera | USB camera | same | PRESENT, needs logical-name mapping |
| Raman spectrum | spectrometer | — | MISSING |
| CO2 + humidity | env sensor | — | MISSING |

## Feature-by-feature detail

### Auger up / down

- **Expected:** rate-limited up/down command (service for discrete or topic for
  continuous).
- **Codebase:** no topic, no service, no driver. `cmr_rovernet/rovernet_utils.py`
  declares `ASTROTECH = 0x03` in the serial byte protocol but nothing in this
  repo emits frames for that subteam.
- **Status:** **MISSING**.
- **Proposed (Phase 2, not committed):** moteus-backed velocity topic.
  - `std_msgs/Float32` on `/astrotech/auger/linear/cmd_velocity`
    (positive = up, negative = down, 0 = stop).
  - `/astrotech/auger/linear/state` — `cmr_msgs/MotorData` for position/vel/torque feedback.
  - Driver node instantiates a `moteus.Controller(id=<auger_id>)` and reuses
    `send_moteus_command_sync` from `cmr_rovernet/rovernet_utils.py`.
  - Reliable QoS, keep_last depth 5.

### Auger spin forward / back

- **Expected:** rotate commanded velocity or on/off with direction.
- **Codebase:** MISSING (same as above).
- **Proposed:** moteus-backed. `std_msgs/Float32` on
  `/astrotech/auger/spin/cmd_velocity` (positive = forward, negative =
  reverse, 0 = stop). State feedback on `/astrotech/auger/spin/state`
  (`cmr_msgs/MotorData`). Reliable QoS.

### Analysis Sequence 1 / Sequence 2 start

- **Expected:** trigger a BDC-driven fixed sequence; GCS gets progress/done.
- **Codebase:**
  - `cmr_msgs/srv/SiteAnalyze` — `int8 site_num → bool success`. Defined in
    `src/cmr_msgs/{srv/SiteAnalyze.srv, CMakeLists.txt}`.
  - No node advertises this service. No node calls it. There is no progress
    feedback channel.
- **Status:** **PARTIAL**. Message exists, backing driver does not.
- **Proposed:**
  - Keep `cmr_msgs/SiteAnalyze` as the service type; GCS panel will call it
    with `site_num = 1` or `site_num = 2`.
  - Need a server in a BDC driver node (not present). If progress feedback is
    required, upgrade to a ROS2 action, e.g. `cmr_msgs/action/SiteAnalyze`
    with `float32 progress` feedback; that is a breaking change to
    `cmr_msgs/CMakeLists.txt` and must be done in a separate PR, not this one.
  - **Caveat:** Request has `int8 site_num` but no enum for which *sequence*
    is being run vs which *site*. Asked in Slack to astrotech (see
    `open_questions.md`); resolving on rover bring-up otherwise.

### Mixing servo presets (S1, S2, CO2_1, CO2_2, Retract)

- **Expected:** service call with enum for preset.
- **Codebase:** **MISSING**. No service, no enum, no driver node.
- **Proposed (Phase 2):**
  - New service type `cmr_msgs/srv/MixingServoPreset`:
    ```
    uint8 S1      = 1
    uint8 S2      = 2
    uint8 CO2_1   = 3
    uint8 CO2_2   = 4
    uint8 RETRACT = 5
    uint8 preset
    ---
    bool success
    string message
    ```
  - Endpoint: `/astrotech/mixing_servo/set_preset`, reliable QoS.
  - **Do not add the service type in Phase 1** — the hardware confirmation for
    which controller drives the servo is still open.

### Auger / Site / Analysis cameras

- **Expected:** `sensor_msgs/CompressedImage` for each logical camera.
- **Codebase:**
  - `usb_camera_publisher/publisher.py` publishes
    `foxglove_msgs/CompressedVideo` (H.264) on
    `camera_<id>/h264` — this is **better than** `CompressedImage` for
    bandwidth on RF and is natively rendered by Foxglove's "H.264 Video" panel,
    but the message type is not the one the task brief requested.
  - There is no mapping from (auger/site/analysis) logical names to
    `/dev/videoN` indices anywhere in the repo.
- **Status:** **PRESENT, requires mapping and a topic-remap decision.**
- **Proposed:**
  1. In Phase 1 we document the gap and do not rename anything.
  2. In Phase 2, add a YAML like `src/cmr_cams/config/astrotech_camera_map.yaml`
     and a `ComposableNode` or launch-time remap to `/astrotech/<logical>/h264`
     (stays `CompressedVideo`, lower RF cost) or `/astrotech/<logical>/image/compressed`
     (`sensor_msgs/CompressedImage`, MJPEG, wider tooling compatibility).
  3. Pick one. Recommended: keep `CompressedVideo` because the existing
     pipeline is already H.264 and the bandwidth delta vs MJPEG is ~3–5× in
     our favor at the bitrates we already use.

### Raman spectrometer spectrum

- **Expected:** `std_msgs/Float32MultiArray` or custom msg with wavelengths +
  intensities.
- **Codebase:** **MISSING.** No driver, no message type, no topic.
- **Proposed (Phase 2):**
  - Add `cmr_msgs/msg/RamanSpectrum.msg`:
    ```
    std_msgs/Header header
    float32[] wavelengths_nm
    float32[] intensities
    float32 integration_time_s
    uint32 acquisition_count
    ```
  - Endpoint: `/astrotech/raman/spectrum`, reliable QoS, keep_last depth 1 (a
    spectrum is a snapshot, not a stream — unless continuous scanning is
    required, in which case best_effort keep_last 5).
  - Phase 1 flags this as a **custom type Foxglove needs schema for.**

### CO2 + humidity

- **Expected:** `sensor_msgs` or custom.
- **Codebase:** **MISSING.** No driver, no topic, no message.
- **Proposed (Phase 2):**
  - Add `cmr_msgs/msg/EnvSample.msg`:
    ```
    std_msgs/Header header
    float32 co2_ppm
    float32 humidity_pct
    float32 temperature_c
    float32 pressure_pa
    ```
  - Endpoint: `/astrotech/env/sample`, reliable QoS, 1–5 Hz (driver-dependent).
  - Temperature and pressure are added even though not in the task brief
    because every env sensor we have looked at exposes them and throwing the
    data away would waste a topic slot; can be defaulted to NaN if not
    available.
  - Phase 1 flags this as a **custom type Foxglove needs schema for.**

## Custom types Foxglove needs schemas for

`foxglove_bridge` serves these automatically from the running ROS2 graph
(by reading `rosidl_runtime` type support). Panel authors must import the
schema name from Foxglove Studio's type picker, not hardcode JSON. Listed
here so that the GCS repo (separate extension-panel repo, not in this PR)
knows which types to reference.

From what exists today in `cmr_msgs`:

- `cmr_msgs/msg/IMUSensorData` — subscribed by any IMU panel.
- `cmr_msgs/msg/MotorReadData` — subscribed by a motor telemetry panel.
- `cmr_msgs/msg/ControllerReading` — for debug view of controller state.
- `cmr_msgs/msg/GroundPlaneStamped`, `cmr_msgs/msg/AutonomyDrive`,
  `cmr_msgs/msg/MiniArmDegree` — autonomy/arm monitoring.
- `cmr_msgs/srv/SiteAnalyze` — service call from the Analysis Seq panel.

From `foxglove_msgs` (already a build dep of `usb_camera_publisher`):

- `foxglove_msgs/msg/CompressedVideo` — the H.264 camera panel is
  built-in Foxglove and does not need a custom extension.

Types that **will be added in Phase 2** and will also need schemas:

- `cmr_msgs/msg/RamanSpectrum` (proposed).
- `cmr_msgs/msg/EnvSample` (proposed).
- `cmr_msgs/srv/MixingServoPreset` (proposed).

## Explicit non-findings

- There is no `/auger/*`, `/astrotech/*`, `/raman/*`, `/co2/*`, `/humidity/*`,
  `/mixing/*`, `/analysis/*`, `/bdc/*`, or `/science/*` topic namespace
  anywhere in `src/`. Confirmed via case-insensitive search of the tree.
- There is no moteus controller instance outside of `armnet.py` (IDs 9–14) and
  the drives code path. No auger moteus ID is assigned anywhere.
- There is no Maxon/EPOS integration of any kind.
- There is no `spectrometer`, `seabreeze`, `ocean_optics`, `wasatch`, or
  equivalent Python dependency in any `package.xml`.

## Consequence for Phase 1

- The bridge launch file (Step 3) will come up healthy even on the current
  codebase: every topic it exposes is present.
- The Foxglove extension-panel plan (Step 5) is therefore bounded by what
  drivers exist today, not by the full GCS wishlist. The missing drivers
  become part of the Phase 2 driver-work backlog.
