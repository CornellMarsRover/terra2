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
| Q1 | Auger controller: moteus or Maxon? | **moteus** (team confirmed). Interface shape now follows the moteus closed-loop telemetry surface. | ANSWERED |
| Q2 | Mixing servo controller family? | **CMR servo board** on the same fdcanusb as the auger moteus stack (can_id=26, servo_id=15, 1 Mbit/s). Confirmed at bench bring-up 2026-05-08. | ANSWERED |
| Q3 | Real analysis-sequence step set + per-step durations? | placeholder: 5 generic steps × 2 s | OPEN — Caitlin |
| Q4 | What does `site_num` / `sequence_id` mean? | placeholder: 1 / 2 are the two sequences | OPEN — Slack to astrotech |
| Q5 | Does a Raman driver already exist elsewhere? | unknown | OPEN |
| Q6 | Does a CO2/humidity driver already exist elsewhere? | unknown | OPEN |
| Q7 | Snapshot naming / storage / retention? | out of scope for Phase 2a | DEFERRED |
| Q8 | URC 2026 RF bandwidth cap? | 5 Mbps working assumption | DEFERRED |
| Q9 | USB `/dev/videoN` → logical camera mapping? | Plug-order dependent on the rover. Layouts assume cam11=auger, cam12=analysis; operator verifies with `v4l2-ctl --list-devices` per the four-terminal bring-up in [`operator_guide.md`](operator_guide.md), and edits `cameraTopic` from the Image panel if a slot is wrong. | OPERATOR-SIDE |
| Q10 | H.264 vs raw `Image` for cameras? | **Raw `sensor_msgs/Image`** on `/camN/image_raw` (from `cmr_cv.camera_node` via [`src/cmr_cams/`](../src/cmr_cams/)) is what the canonical bring-up exposes. The H.264 path in [`src/usb_camera_publisher/`](../src/usb_camera_publisher/) exists but is **not** what `cmr_cams default.launch.py` starts. | ANSWERED |
| Q11 | Fabric-manage the foxglove bridge? | no — plain `launch_ros.Node` | ANSWERED |
| Q12 | External cam packages (`cmr_cv`, etc.) | `cmr_cv.camera_node` is rover-installed, not in this repo. The orchestration ([`src/cmr_cams/`](../src/cmr_cams/) configs + launch) **is** here; the bring-up procedure runs on the rover Jetson. | EXTERNAL |
| Q13 | Expose per-cam bitrate/fps as params? | out of scope for Phase 2a | DEFERRED |

## Namespace convention

All Astrotech topics/services/actions sit under **`/astrotech/…`**.
Camera feeds use the `/camN/image_raw` names emitted by
`cmr_cv.camera_node` (started via [`src/cmr_cams/launch/default.launch.py`](../src/cmr_cams/launch/default.launch.py))
and the `/zed/zed_node/...` names from the stereolabs `zed_wrapper`
package. The `camera_<id>/h264` topic family from `usb_camera_publisher`
is a separate stack that is not part of the canonical Astrotech bring-up.

## Auger — moteus stack (Session 5)

The auger is on **moteus**, not Caitlin's CAN-FD library — two
controllers (id=15 lead screw, id=16 auger). Reference pattern at
`docs/reference/auger_keys_test_harness.py`.

- Command topic: **`/astrotech/auger/cmd_vel`** — `cmr_msgs/AugerCommand`
  (`lead_screw_velocity_rev_s` / `lead_screw_max_torque_nm` /
  `auger_velocity_rev_s` / `auger_max_torque_nm`). Hold-to-act:
  GCS publishes at 10 Hz while a button is held; driver applies a
  200 ms watchdog.
- State topic: **`/astrotech/auger/state`** — `cmr_msgs/AugerState` —
  20 Hz mock, 50 Hz real (moteus watchdog cadence). Closed-loop fields
  for both motors: `*_position_rev` / `*_velocity_rev_s` / `*_torque_nm`
  / `*_temperature_c` / `*_mode` / `*_fault`. (Field names use lowercase
  unit suffixes — ROS 2 IDL rejects uppercase letters.)

## Mixing servo — Q2 ANSWERED (bench 2026-05-08)

CMR servo board on the fdcanusb at `(can_id=26, servo_id=15)`, bus at
1 Mbit/s. After the bench bring-up the interface model shifted from the
original 5-preset enum to an angle-based model with operator-set home:

- Set-angle service: **`/astrotech/mixing_servo/set_angle`** —
  `cmr_msgs/SetMixingServoAngle` (12-bit `angle_deg` 0..4095, offset
  from the last `set_home`).
- Set-home service: **`/astrotech/mixing_servo/set_home`** —
  `std_srvs/Trigger` (zeros the board at the current physical position).
- State topic: **`/astrotech/mixing_servo/state`** — `std_msgs/Int32`
  (last commanded angle in deg) — 5 Hz.
- The Foxglove panel keeps three editable site offsets locally (Big Box
  / Site 1 / Site 2, defaults 110 / 145 / 185 from the bench rig) and
  calls `set_angle` with whichever value is in the editor for the
  clicked button. Manual goto is also supported.
- Driver bench-discovered gotchas (1 Mbit/s, `clear_faults=1` per
  frame) are documented in
  [`astrotech_canfd_library_notes.md`](astrotech_canfd_library_notes.md).

## Analysis sequences — `TODO(astrotech-q-3)`, `TODO(astrotech-q-4)`

- Two action servers, one per sequence id:
  - **`/astrotech/analysis/run_sequence_1`** — `cmr_msgs/RunAnalysisSequence`.
  - **`/astrotech/analysis/run_sequence_2`** — `cmr_msgs/RunAnalysisSequence`.
- Goal: `int8 sequence_id` — mock accepts 1 or 2, rejects anything else.
- Feedback: `int32 current_step`, `int32 total_steps`,
  `string current_step_description`, `float32 elapsed_seconds`.
- Result: `bool success`, `string message`,
  `int32 last_completed_step`.
- Mock duration: **5 generic steps × 2 s each** by default
  (`num_steps` / `step_duration_sec` in `astrotech_interfaces.yaml`).
  - **Assumption pinned at `TODO(astrotech-q-3)`**: step count and per-step
    duration are placeholder values. Real BDC sequences are 5–10 minutes
    end to end (per Session 3) and use real step descriptions.
  - Cancel returns `success=false` with a message like
    `"cancelled at step N"` and leaves the rover wherever it stopped
    (run-to-completion-with-hard-abort; pause/resume → post-URC backlog).
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
  # CMR servo board, can_id=26, servo_id=15, 1 Mbit/s.
  set_angle_service: /astrotech/mixing_servo/set_angle
  set_angle_type:    cmr_msgs/SetMixingServoAngle
  set_home_service:  /astrotech/mixing_servo/set_home
  set_home_type:     std_srvs/Trigger
  state_topic:       /astrotech/mixing_servo/state
  state_type:        std_msgs/Int32
  state_rate_hz:     5
  # Three site offsets live in panel state, not the YAML; defaults
  # 110 / 145 / 185 from the bench rig (2026-05-08).

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

# Camera feeds aren't part of the mock YAML — the cmr_cv camera nodes
# are rover-installed and started via src/cmr_cams/launch/default.launch.py
# (see docs/operator_guide.md for the four-terminal bring-up).
```

## Out of scope for Phase 2a (recorded to prevent scope creep)

- Snapshot service (Q7).
- Pause/resume for sequences — see `docs/post_urc_backlog.md`.
- Camera publishers are out of the mock — they're launched rover-side via [`src/cmr_cams/`](../src/cmr_cams/); the Foxglove layouts consume the resulting topics directly.
- Real panel widgets (Phase 2b).
- Fabric-managed bridge (Q11).
- CI wiring for the smoke test.
