# Foxglove Extension Plan

Which Astrotech GCS features need a **custom Foxglove Studio extension
panel**, and which can ride on built-in Foxglove panels. Phase 1 deliverable —
no panels are implemented in this PR, only specified.

## Decision rubric

A feature can use a built-in Foxglove panel when **all** are true:

1. Its ROS interface is a single subscribe or a single service call with a
   trivial payload.
2. The UI it needs is one of: time-series plot, numeric indicator,
   log/console, map, 3D scene, image viewer, teleop joystick, parameter
   editor, publish panel, call-service panel, state transition.
3. No cross-topic composition is required (e.g. "show the spectrum and a
   button together as one widget").

Otherwise it gets a custom extension panel.

## Candidate panels

### 1. Astrotech Control Panel (custom — REQUIRED)

Single operator-facing panel that consolidates every Astrotech actuator so
the operator isn't context-switching between five different service-call
panels during a timed science task.

- **Tabs / sections:**
  - Auger linear: up / stop / down buttons, with optional velocity slider.
  - Auger spin: forward / stop / reverse buttons, with velocity slider.
  - Mixing servo: five preset buttons (S1, S2, CO2_1, CO2_2, Retract).
  - Analysis sequence: "Start Seq 1" / "Start Seq 2" buttons with an
    in-progress indicator, ideally fed by an action feedback stream.
  - Live status row: last command timestamp, last service result (OK / error
    text), last progress %.
- **Subscriptions** (all **MISSING** in repo today — depend on Phase 2
  drivers being written first):
  - *(proposed)* `/astrotech/auger/linear/state` — `cmr_msgs/MotorData`.
  - *(proposed)* `/astrotech/auger/spin/state` — `cmr_msgs/MotorData`.
  - *(proposed)* `/astrotech/mixing_servo/state` — `std_msgs/String` or
    enum-typed.
  - *(proposed)* `/astrotech/analysis/progress` — `std_msgs/Float32` or an
    action feedback channel if we upgrade `SiteAnalyze` to an action.
- **Publications** (from panel → rover):
  - *(proposed)* `/astrotech/auger/linear/cmd_velocity` — `std_msgs/Float32`.
  - *(proposed)* `/astrotech/auger/spin/cmd_velocity` — `std_msgs/Float32`.
- **Service calls:**
  - **Today**: `cmr_msgs/srv/SiteAnalyze` on `/…/site_analyze` (server does
    not exist yet; panel should display "service not available" gracefully
    until Phase 2 driver lands).
  - *(proposed)* `cmr_msgs/srv/MixingServoPreset` on
    `/astrotech/mixing_servo/set_preset`.
- **Why not built-in:**
  - The built-in "Call Service" panel only supports one service per panel
    instance. Putting five of them up is cumbersome and they can't share a
    status row.
  - We want per-preset labels (S1/S2/CO2_1/CO2_2/Retract) as buttons, not a
    JSON editor on an enum field.
  - Operator needs a single ACK/ERROR region that aggregates across
    auger/mixing/sequence — not possible across separate built-ins.
  - Needs to enforce mutual exclusion ("don't start Seq 2 while Seq 1 is in
    flight") — cross-panel state the built-ins don't share.

### 2. Raman Spectrum Viewer (custom — REQUIRED)

Plot wavelength (nm) on X, intensity on Y. **Not a time-series.** Foxglove's
built-in "Plot" panel is time-series-on-X only, so a custom panel is
mandatory.

- **Subscriptions** (MISSING today):
  - *(proposed)* `/astrotech/raman/spectrum` — `cmr_msgs/msg/RamanSpectrum`
    (`wavelengths_nm[]`, `intensities[]`, `integration_time_s`, `acquisition_count`).
- **Service calls** (optional, Phase 2):
  - *(proposed)* `std_srvs/Trigger` on `/astrotech/raman/acquire` for
    on-demand acquisition.
  - *(proposed)* `/astrotech/raman/snapshot` — save the displayed spectrum
    to a file with a mission-tagged filename.
- **UI features:**
  - Live spectrum trace with zoom/pan/reset.
  - Overlay of last N spectra (fade colors) for comparison.
  - Peak annotations: compute local maxima in the client, print wavelength
    at top.
  - Crosshair readout.
  - Export CSV button.
- **Why not built-in:** X axis is wavelength, not time. None of Foxglove's
  stock panels do spectrum plots.

### 3. Environmental Sample Panel (custom — RECOMMENDED)

CO2 + humidity + temperature displayed as three large numeric dials **plus**
a rolling 60-second sparkline per channel.

- **Subscriptions** (MISSING today):
  - *(proposed)* `/astrotech/env/sample` — `cmr_msgs/msg/EnvSample`
    (`co2_ppm`, `humidity_pct`, `temperature_c`, `pressure_pa`).
- **Why not built-in:**
  - Foxglove's built-in "Indicator" panel shows one value per panel. Three
    indicator panels + three plot panels is six panels to lay out every
    mission for the same logical unit. A single 3-channel widget is much
    cleaner.
  - The task brief explicitly lists "CO2 + humidity" together as one feature.
  - It is also acceptable (and cheaper to ship for Phase 2a) to use **three
    Indicator panels + three Plot panels**. Keep this one OPTIONAL — build
    only if Phase 2 has time after panels 1 and 2.

### 4. Camera Select + Multi-Feed Panel (built-in is fine)

Three logical feeds: auger cam, site cam, analysis cam.

- **Subscriptions:** `camera_<id>/h264` (`foxglove_msgs/CompressedVideo`) via
  Foxglove's built-in **"H.264 Video"** panel.
  - Requires a camera-name → `/dev/videoN` mapping file (`astrotech_camera_map.yaml`
    in Phase 2) and a launch-time remap to friendly names like
    `/astrotech/auger_cam/h264`.
  - If we keep raw `camera_<id>/h264` names, operators pick the right cam id
    per feed slot.
- **Built-in suffices:** Foxglove's H.264 Video panel reads
  `foxglove_msgs/CompressedVideo` natively, including per-panel rate/quality
  controls.
- **Optional custom addition:** a small "Camera Select" dropdown panel that
  publishes a camera index to a settings topic which the extension-owned
  H.264 panel subscribes to, so the operator can flip all three feeds in one
  click. Not required for URC and can ship later.

### 5. Motor Telemetry Grid (built-in is fine)

`/ccb/read` is `cmr_msgs/MotorReadData` (14 joints: 4 drive + 4 swerve + 6
arm). Foxglove's built-in "Raw Messages" or "State Transitions" panels can
show the fields. A built-in "Plot" panel can chart
`/ccb/read.back_left.velocity` etc.

No custom panel needed. Just a saved layout.

### 6. Autonomy / Localization (built-in is fine)

- `/autonomy/pose/robot/global` and `/rtk/navsatfix_data` ride on the
  built-in **Map** panel.
- `/autonomy/costmap` rides on the built-in **3D** panel with a
  `std_msgs/Float32MultiArray` display. **Caveat:** since the ROS message is
  not `nav_msgs/OccupancyGrid`, we may need a tiny custom panel that
  reshapes the `Float32MultiArray` into an OccupancyGrid for display, or a
  republisher node on the rover. Phase 2 decision.
- `/camera/points` — not forwarded over RF (see bandwidth audit).

### 7. Arm teleop status (built-in is fine)

- Built-in service call panels for `/servo_node/start_servo`, `/stop_servo`,
  `/pause_servo`, `/unpause_servo`.
- Built-in Plot of `/main_arm_controller/joint_trajectory` joint-by-joint
  positions and velocities.

## Panel-by-panel subscription / publication contract

For the custom panels only — complete reference for when they get
implemented. Panels implemented in TypeScript under
`@foxglove/studio` extension SDK conventions.

### Astrotech Control Panel

```ts
// Subscriptions (all proposed for Phase 2)
ctx.subscribe([
  { topic: "/astrotech/auger/linear/state" },      // cmr_msgs/MotorData
  { topic: "/astrotech/auger/spin/state" },        // cmr_msgs/MotorData
  { topic: "/astrotech/mixing_servo/state" },      // std_msgs/String
  { topic: "/astrotech/analysis/progress" },       // std_msgs/Float32
]);

// Advertise (client publish)
ctx.advertise?.("/astrotech/auger/linear/cmd_velocity", "std_msgs/Float32");
ctx.advertise?.("/astrotech/auger/spin/cmd_velocity",   "std_msgs/Float32");

// Service calls
ctx.callService?.("<bdc_ns>/site_analyze", { site_num: 1 });   // cmr_msgs/srv/SiteAnalyze
ctx.callService?.("<bdc_ns>/site_analyze", { site_num: 2 });
ctx.callService?.("/astrotech/mixing_servo/set_preset", { preset: 1 });  // cmr_msgs/srv/MixingServoPreset
```

### Raman Spectrum Viewer

```ts
ctx.subscribe([{ topic: "/astrotech/raman/spectrum" }]);  // cmr_msgs/msg/RamanSpectrum
ctx.callService?.("/astrotech/raman/acquire", {});        // std_srvs/Trigger
```

### Environmental Sample Panel

```ts
ctx.subscribe([{ topic: "/astrotech/env/sample" }]);  // cmr_msgs/msg/EnvSample
```

## What Phase 1 does NOT do

- **No panels are implemented in this PR.** (Explicit non-goal of Phase 1.)
- **No Foxglove layout is created.** (Also explicit non-goal.)
- Proposed message types (`RamanSpectrum`, `EnvSample`, `MixingServoPreset`)
  are listed for planning purposes only and are **not added to `cmr_msgs`
  in this branch**. They must be added in the Phase 2 PR that wires up the
  corresponding driver nodes, so schema + server ship together.

## Dependency order for Phase 2 (not in this PR)

1. Hardware decision: moteus vs Maxon for auger; which controller for mixing
   servo and BDC. *(Open questions Q1–Q3.)*
2. Add proposed msg / srv types to `cmr_msgs`.
3. Write Raman driver, CO2/humidity driver, auger driver, mixing servo
   driver, BDC analysis-sequence server.
4. Re-run bandwidth audit against measured `ros2 topic bw` values.
5. Build the three custom Foxglove panels and a saved default layout.
