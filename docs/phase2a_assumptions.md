# Phase 2a Assumptions

Mock-rover values for every Astrotech interface that Phase 1 flagged as
**MISSING**. Every number, name, and message type here is an *assumption* —
none of this has been confirmed with hardware or with the Astrotech team. The
mock is built to match this document exactly. When the team answers a
question, the code is updated by finding `TODO(astrotech-q-N)` tags and
changing those files.

## Q-number map (stable across the project)

These numbers are fixed and referenced from code tags
`TODO(astrotech-q-N)`. They are the original numbering from the Phase 1
verification audit; keep them stable even when questions get answered.

| Q | Question | Current answer | Status |
|---|---|---|---|
| Q1 | Auger controller: moteus or Maxon? | **moteus** (team confirmed) | ANSWERED, but interface type still an assumption (see below) |
| Q2 | Mixing servo controller family? | unknown | OPEN |
| Q3 | BDC (analysis sequence) controller? | unknown | OPEN |
| Q4 | What does `site_num` / `sequence_id` mean? | placeholder: 1 / 2 are the two sequences | OPEN — Slack to astrotech |
| Q5 | Does a Raman driver already exist elsewhere? | unknown | OPEN |
| Q6 | Does a CO2/humidity driver already exist elsewhere? | unknown | OPEN |
| Q7 | Snapshot naming / storage / retention? | out of scope for Phase 2a | DEFERRED |
| Q8 | URC 2026 RF bandwidth cap? | 5 Mbps working assumption | DEFERRED |
| Q9 | USB `/dev/videoN` → logical camera mapping? | (camera work moved to a separate branch; not represented in Phase 2a) | DEFERRED |
| Q10 | H.264 vs CompressedImage for cameras? | **H.264** (team confirmed by using existing pipeline) | ANSWERED |
| Q11 | Fabric-manage the foxglove bridge? | no — plain `launch_ros.Node` | ANSWERED |
| Q12 | External cam packages (`cmr_cv`, etc.) | not needed for mock rover | DEFERRED |
| Q13 | Expose per-cam bitrate/fps as params? | out of scope for Phase 2a | DEFERRED |

## Namespace convention

All Astrotech topics/services/actions sit under **`/astrotech/…`**.
Camera feeds (separate branch) reuse the existing `camera_<id>/h264`
naming from `usb_camera_publisher`.

## Auger — moteus stack (Session 5)

The auger is on **moteus**, not Caitlin's CAN-FD library — two
controllers (id=15 lead screw, id=16 auger). Reference pattern at
`docs/reference/auger_keys_test_harness.py`.

- Command topic: **`/astrotech/auger/cmd_vel`** — `cmr_msgs/AugerCommand`
  (`lead_screw_velocity_rev_s` / `lead_screw_max_torque_Nm` /
  `auger_velocity_rev_s` / `auger_max_torque_Nm`). Hold-to-act:
  GCS publishes at 10 Hz while a button is held; driver applies a
  200 ms watchdog.
- State topic: **`/astrotech/auger/state`** — `cmr_msgs/AugerState` —
  20 Hz mock, 50 Hz real (moteus watchdog cadence). Closed-loop fields
  for both motors: position / velocity / torque / temperature / mode /
  fault.

## Mixing servo — `TODO(astrotech-q-2)`

- Presets (preset_name string values, exact casing used as the enum):
  `S1`, `S2`, `CO2_1`, `CO2_2`, `RETRACT`.
- Set-preset service: **`/astrotech/mixing_servo/set_preset`** —
  `cmr_msgs/SetMixingServoPreset`.
- State topic: **`/astrotech/mixing_servo/state`** — `std_msgs/String`
  (last-set preset name) — 5 Hz.
- **Assumption pinned at `TODO(astrotech-q-2)`**: controller type is unknown.
  Mock implements the service with pure state; once the real controller is
  picked (PWM from MCU, moteus, or dedicated servo driver), the driver
  class is swapped but the service contract stays.

## Analysis sequences — `TODO(astrotech-q-3)`, `TODO(astrotech-q-4)`

- Two action servers, one per sequence id:
  - **`/astrotech/analysis/run_sequence_1`** — `cmr_msgs/RunAnalysisSequence`.
  - **`/astrotech/analysis/run_sequence_2`** — `cmr_msgs/RunAnalysisSequence`.
- Goal: `int8 sequence_id` — mock accepts 1 or 2, rejects anything else.
- Feedback: `float32 progress_pct`, `string current_step`.
- Result: `bool success`, `string result_message`.
- Mock duration: 10 seconds linear ramp from 0 → 100% progress.
  - **Assumption pinned at `TODO(astrotech-q-3)`**: duration is arbitrary.
    Real BDC sequences may be seconds or minutes. Step names are purely
    placeholder (`"starting"`, `"running"`, `"finalizing"`).
- **Assumption pinned at `TODO(astrotech-q-4)`**: `sequence_id` and
  `site_num` semantics are unresolved. `sequence_id` is kept as the goal
  field name; if the team says it's actually a site id, rename the field
  and Foxglove panel labels in one place.

## Raman spectrum — `TODO(astrotech-q-5)`

- Topic: **`/astrotech/raman/spectrum`** — `cmr_msgs/RamanSpectrum` — 1 Hz.
- Wavenumber range: 200 cm⁻¹ → 2000 cm⁻¹, 1024 points, linspace.
- Mock intensities: sum of three Gaussian peaks at 1085, 1332, 1580 cm⁻¹
  (calcite, diamond, graphite reference lines) with slowly drifting
  amplitudes plus small Gaussian noise.
- **Assumption pinned at `TODO(astrotech-q-5)`**: if a real Raman driver
  already exists in a parallel repo with its own message type, the mock
  will match that type instead of shipping `cmr_msgs/RamanSpectrum`. That's
  a type rename across the driver file, YAML, TS interface mirror, and the
  `.msg` definition itself.

## Environmental sensor — `TODO(astrotech-q-6)`

- Topic: **`/astrotech/env/sample`** — `cmr_msgs/EnvSample` — 1 Hz.
- Mock signals:
  - `co2_ppm`: baseline 420 ppm + slow sinusoid (amplitude 20 ppm, period
    120 s) + small Gaussian noise. Every 30 s, inject a transient: 3 s ramp
    to 1200 ppm, 5 s hold, 10 s decay back to baseline.
  - `humidity_pct`: slow drift between 40 and 60 %.
  - `temperature_c`: slow drift between 20 and 25 °C.
- **Assumption pinned at `TODO(astrotech-q-6)`**: same as Q5 — if a real
  driver exists it likely has its own message type, rename across the same
  four locations.

## Cameras

Camera feeds are not part of Phase 2a. That work lives on a separate
branch and will merge into `astrotech-gui` later.

## Full interface table (copy into `astrotech_interfaces.yaml`)

Pseudo-YAML form; the real YAML file is at
`src/urc_mock_rover/config/astrotech_interfaces.yaml`.

```yaml
auger:
  # moteus stack (lead screw id=15, auger id=16); not Caitlin's library.
  cmd_topic: /astrotech/auger/cmd_vel
  cmd_type: cmr_msgs/AugerCommand
  cmd_watchdog_ms: 200
  state_topic: /astrotech/auger/state
  state_type: cmr_msgs/AugerState
  state_rate_hz: 20

mixing_servo:
  set_preset_service: /astrotech/mixing_servo/set_preset
  set_preset_type: cmr_msgs/SetMixingServoPreset
  state_topic: /astrotech/mixing_servo/state
  state_type: std_msgs/String
  state_rate_hz: 5
  presets: [S1, S2, CO2_1, CO2_2, RETRACT]

analysis:
  sequences:
    - id: 1
      action: /astrotech/analysis/run_sequence_1
    - id: 2
      action: /astrotech/analysis/run_sequence_2
  action_type: cmr_msgs/RunAnalysisSequence
  num_steps: 5                            # TODO(astrotech-q-3)
  step_duration_sec: 2.0

raman:
  topic: /astrotech/raman/spectrum
  type: cmr_msgs/RamanSpectrum
  rate_hz: 1.0
  wavenumber_min: 200.0
  wavenumber_max: 2000.0
  n_points: 1024
  mock_peaks_cm_inv: [1085.0, 1332.0, 1580.0]

env:
  topic: /astrotech/env/sample
  type: cmr_msgs/EnvSample
  rate_hz: 1.0

# (Camera feeds intentionally absent — separate branch.)
```

## Out of scope for Phase 2a (recorded to prevent scope creep)

- Snapshot service (Q7).
- Pause/resume for sequences — see `docs/post_urc_backlog.md`.
- Camera feeds (separate branch).
- Real panel widgets (Phase 2b).
- Fabric-managed bridge (Q11).
- CI wiring for the smoke test.
