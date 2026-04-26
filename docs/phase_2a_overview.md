# Phase 2a — Overview and Honest Audit

Snapshot of what Phase 2a built (commit `eef937c`, "phase 2a: initial mock
rover (pre-Caitlin info)"), what it actually verifies, and which of its
assumptions the new Astrotech information has invalidated.

This document is read-only analysis. No Phase 2a source was modified to
produce it.

---

## 1. What was built

Files added or modified in commit `eef937c`, grouped by area.

### Custom interfaces (`src/cmr_msgs/`)

- `msg/RamanSpectrum.msg` — `Header + float32[] wavenumbers_cm_inv + float32[] intensities`.
- `msg/EnvSample.msg` — `Header + co2_ppm + humidity_pct + temperature_c`.
- `msg/AugerState.msg` — `Header + position_rev + velocity_rev_s + torque_nm + is_moving`.
- `srv/SetMixingServoPreset.srv` — `string preset_name -> bool success + string message`.
- `action/RunAnalysisSequence.action` — `int8 sequence_id` ⇒ `bool success + string result_message`, feedback `float32 progress_pct + string current_step`.
- `CMakeLists.txt` updated to register all five new entries.

### Mock rover package (`src/urc_mock_rover/`, ament_python)

- `package.xml`, `setup.py`, `setup.cfg`, `resource/urc_mock_rover` — standard ament_python boilerplate. Console script `mock_rover_node`.
- `config/astrotech_interfaces.yaml` — single source of truth for every topic / service / action / preset / camera mapping the mock advertises.
- `urc_mock_rover/mock_rover_node.py` — coordinator. Loads the YAML, instantiates one driver per feature area, spins under a `MultiThreadedExecutor` so the analysis action server's blocking execute callback does not stall other timers.
- `urc_mock_rover/drivers/auger.py` — subscribes to `geometry_msgs/Twist` cmd, integrates a fake position state, publishes `cmr_msgs/AugerState` at 20 Hz. Watchdog drops to zero velocity after 0.5 s of silence.
- `urc_mock_rover/drivers/mixing_servo.py` — `cmr_msgs/SetMixingServoPreset` service server; republishes the last preset on `std_msgs/String` at 5 Hz.
- `urc_mock_rover/drivers/analysis_sequencer.py` — two ROS 2 action servers, one per `sequence_id`. Linear 0→100 % progress over `mock_duration_sec` (default 10 s). Cancelable; rejects new goals while busy.
- `urc_mock_rover/drivers/raman.py` — synthesizes a 1024-point spectrum (linspace 200–2000 cm⁻¹) with three drifting Gaussians at 1085 / 1332 / 1580 cm⁻¹ + noise. Publishes at 1 Hz.
- `urc_mock_rover/drivers/env.py` — CO₂ baseline 420 ppm + sinusoid + 30 s transient spike to 1200 ppm. Humidity / temperature slow drifts. Publishes at 1 Hz.
- `urc_mock_rover/drivers/camera_replayer.py` — pure-Python H.264 Annex-B NAL splitter. Reads `assets/sample_video.h264`, packs SPS/PPS into the next IDR, publishes `foxglove_msgs/CompressedVideo` at 15 fps in a loop.
- `urc_mock_rover/assets/README.md` — explains the gitignored asset and points at `scripts/fetch_sample_video.sh`.
- `launch/mock.launch.py` — single-command bring-up: starts `mock_rover_node` plus `launch/gcs_bridge.launch.py` (Phase 1).
- `.gitignore` for the H.264 asset.

### Scripts

- `scripts/fetch_sample_video.sh` — `ffmpeg testsrc → H.264 Annex-B`, 10 s @ 30 fps @ 640×480. Writes the gitignored `assets/sample_video.h264`.
- `scripts/smoke_test.sh` — colcon build, launch the mock, assert all expected topics appear, assert `/camera_0/h264` rate ∈ (10, 30) Hz, assert `/astrotech/raman/spectrum` rate > 0.5 Hz, `ros2 service call` with preset `S1` returns `success=True`.

### Foxglove side (`gcs/`)

- `gcs/layouts/urc_astrotech_dashboard.json` — 3×3 mosaic: env Plot + 3 H.264 Image panels + 4 custom panels + a /rosout Log.
- `gcs/extensions/urc-astrotech-panels/` — `@foxglove/extension`-style scaffold with `package.json`, `tsconfig.json`, four panel stubs (`AugerControl`, `AnalysisSequence`, `MixingServo`, `RamanSpectrum`), shared `PanelStub` component, and `interfaces.ts` mirroring the YAML topic names. **Stubs only** — clicks log to `console.log`, no real publishes / service calls / action goals.

### Documentation

- `docs/phase2a_assumptions.md` — every value baked into the YAML, paired with the Q-number it depends on.
- `docs/astrotech_interview_notes.md` — running interview log (Session 1 was already populated).
- `README.md` — added "Developing the Astrotech GCS without the rover" section + `astrotech-q-N` → files-to-change table.

---

## 2. Architecture (data flow as built)

```
                     ┌──────────────────────────────────────────────┐
                     │  src/urc_mock_rover/config/                  │
                     │  astrotech_interfaces.yaml                   │
                     │  (single source of truth for topic names)    │
                     └───────────────┬─────────────────┬────────────┘
                                     │ load            │ hand-mirrored
                                     ▼                 ▼
        ┌────────────────────────────────────┐   gcs/extensions/.../src/
        │  mock_rover_node (coordinator)     │   interfaces.ts
        │  - MultiThreadedExecutor            │
        │  - instantiates 6 drivers           │
        └─────┬──────────────────────────┬────┘
              │ each driver gets its     │
              │ slice of the YAML        │
              ▼                          ▼
   ┌──────────────────┐   ┌──────────────────────────┐
   │ MockAugerDriver  │   │ MockMixingServoDriver    │
   │ /astrotech/auger/cmd  │ /astrotech/mixing_servo/ │
   │ /astrotech/auger/state│   set_preset (srv)       │
   └──────────────────┘   │ /astrotech/mixing_servo/ │
   ┌──────────────────┐   │   state                  │
   │ MockAnalysis-    │   └──────────────────────────┘
   │   Sequencer      │   ┌──────────────────────────┐
   │ run_sequence_1   │   │ MockRamanPublisher       │
   │ run_sequence_2   │   │ /astrotech/raman/spectrum│
   │ (actions)        │   └──────────────────────────┘
   └──────────────────┘   ┌──────────────────────────┐
   ┌──────────────────┐   │ MockEnvPublisher         │
   │ MockCamera-      │   │ /astrotech/env/sample    │
   │   Replayer × 3   │   └──────────────────────────┘
   │ /camera_0/h264   │
   │ /camera_2/h264   │
   │ /camera_4/h264   │
   └──────────────────┘
              │
              │ all topics live on the same ROS 2 graph
              ▼
   ┌──────────────────────────────────────────────────────────────┐
   │  foxglove_bridge (started by launch/gcs_bridge.launch.py)    │
   │   - QoS overrides for video / IMU / pointcloud topics        │
   │   - listens on ws://<host>:8765                              │
   └─────────────────────────────┬────────────────────────────────┘
                                 │ Foxglove WebSocket protocol
                                 ▼
   ┌──────────────────────────────────────────────────────────────┐
   │  Foxglove Studio                                             │
   │  - urc_astrotech_dashboard.json (mosaic layout)              │
   │  - urc-astrotech-panels extension (4 stubs):                 │
   │      urc.auger_control, urc.analysis_sequence,               │
   │      urc.mixing_servo, urc.raman_spectrum                    │
   └──────────────────────────────────────────────────────────────┘
```

---

## 3. What works (statically verified)

These checks were run during Phase 2a and re-confirmed after the
`eef937c` commit:

- `ast.parse` on every `.py` under `src/urc_mock_rover/` (11 files,
  no `SyntaxError`).
- `yaml.safe_load(astrotech_interfaces.yaml)` and shape verification
  (six top-level sections, two analysis sequences, three camera feeds,
  five preset names, three Raman peak entries).
- `json.load(urc_astrotech_dashboard.json)` and bidirectional check
  between every panel id in the mosaic tree and every key in
  `configById` (no orphans on either side).
- Cross-check: every custom panel type used in the layout
  (`urc.auger_control`, `urc.analysis_sequence`, `urc.mixing_servo`,
  `urc.raman_spectrum`) is registered in `gcs/extensions/.../src/index.ts`.
- Cross-check: the set of ROS topic / service / action names in the
  YAML matches the set in `interfaces.ts` byte-for-byte (11 names on
  each side; no drift).
- Hand-built unit test for the H.264 NAL splitter:
    - SPS+PPS+IDR bundling → correct.
    - 3-byte start codes → handled.
    - Empty stream → empty output, no crash.
    - Trailing parameter sets without a following slice → dropped (intended).
- Both shell scripts pass `bash -n`.
- Linter (Cursor) reported no errors on any Phase 2a file.

---

## 4. What is NOT verified

These were explicitly out of reach on the dev machine that produced
Phase 2a (macOS without ROS / npm / ffmpeg installed) and are
**unverified** until someone runs them on a Humble box:

- `colcon build --packages-select cmr_msgs urc_mock_rover` — the only
  proof that the new `.msg` / `.srv` / `.action` files generate without
  errors.
- `ros2 launch urc_mock_rover mock.launch.py` actually starts the
  coordinator and all six drivers.
- `scripts/smoke_test.sh` end-to-end pass — topic enumeration, rate
  checks, service call.
- `npm install && npm run build` in `gcs/extensions/urc-astrotech-panels/`.
  TypeScript will surface real type errors against the
  `@foxglove/extension` API; both the API version (`^2.0.0`) and the
  `Image` panel `imageMode.imageTopic` config shape were chosen by
  documentation review, not by running the build.
- Foxglove Studio actually rendering the layout JSON. Foxglove rewrites
  unknown panel configs silently; it is plausible for the cameras to
  show up unset and require a manual topic pick.
- `scripts/fetch_sample_video.sh` actually producing a parseable bitstream.
  The NAL splitter unit test used hand-built byte arrays, not real
  ffmpeg output.
- The replayer publishing at exactly 15 fps over the bridge under load.

---

## 5. Outstanding `TODO(astrotech-q-N)` tags

`grep`-extracted across the working tree (excluding docs). Cross-checked
against `docs/astrotech_interview_notes.md`:

| Q | Where it appears | What it represents | Status (per interview notes) |
|---|---|---|---|
| Q1 | `src/cmr_msgs/msg/AugerState.msg`, `src/urc_mock_rover/config/astrotech_interfaces.yaml`, `src/urc_mock_rover/urc_mock_rover/drivers/auger.py`, `gcs/extensions/.../interfaces.ts`, `gcs/extensions/.../panels/AugerControl.tsx` | Auger controller family (moteus vs Maxon). Mock currently assumes a single `geometry_msgs/Twist` carrying both linear-Z and angular-Z velocities. | Confirmed moteus by the team in Phase 1; **but** the new Astrotech info reveals the auger is most likely driven by a BDC board on the CAN-FD bus (along with the pumps) — see §6. |
| Q2 | `astrotech_interfaces.yaml`, `drivers/mixing_servo.py`, `interfaces.ts`, `panels/MixingServo.tsx` | Mixing servo controller family. Mock assumes a single device behind a single service. | **Now answered** — the mixing servo is one of (up to 16) servos on a Servo board on the same CAN-FD bus as the BDC pumps. Multiple boards possible (CAN IDs 1, 3, …). |
| Q3 | `astrotech_interfaces.yaml`, `drivers/analysis_sequencer.py` | Where sequence logic lives + how long sequences take. Mock used 10 s linear ramp, 4 dummy step names. | **Now answered (partially)** — sequences are 5–10 minutes, must live on the main computer (not firmware), step set is unknown until a fluidic-protocol walkthrough happens. |
| Q4 | `src/cmr_msgs/action/RunAnalysisSequence.action`, `drivers/analysis_sequencer.py` | `sequence_id` vs. `site_num` semantics. | Still open in code; the new info implies sequences are *sequences* (fluidic protocols) and there are at most a handful of them, not arbitrary site indices. The `int8 sequence_id` shape is probably fine; the *steps inside* a sequence are the new question. |
| Q5 | `astrotech_interfaces.yaml`, `drivers/raman.py`, `interfaces.ts` | Raman driver location / message type. | Open. New info doesn't address it. |
| Q6 | `astrotech_interfaces.yaml`, `drivers/env.py`, `interfaces.ts` | CO₂ / humidity driver location / message type. | Open. New info adds: heater (Ninhydrin) is part of the payload — that's an extra actuator the mock doesn't represent. |
| Q9 | `astrotech_interfaces.yaml`, `drivers/camera_replayer.py`, `interfaces.ts` | USB camera id ↔ logical-role mapping. | Open. Defer until cameras are physically wired. |

There is no `Q7` or `Q8` tag in source — those were marked deferred /
not-blocking in `docs/open_questions.md` after the user's Phase 1 prune
and were never tagged for Phase 2a code edits.

---

## 6. Assumptions Phase 2a baked in that are now wrong

The new Astrotech information from Caitlin (5–10 min sequences, must be
resumable, BDC + Servo boards over CAN-FD with an existing async Python
library, sequence orchestration must live on the main computer)
invalidates several Phase 2a choices.

### 6.1 The action interface does **not** support pause / resume

`src/cmr_msgs/action/RunAnalysisSequence.action`:

```action
# Goal
int8 sequence_id
---
# Result
bool success
string result_message
---
# Feedback
float32 progress_pct
string current_step
```

This is a one-shot action. There is no:

- pause goal field or service,
- resume goal field or service,
- way to pass a "start from step N" / "start from progress P %" hint,
- way for the *server* to volunteer a resume token to the client at
  pause / abort time.

Cancel exists (ROS 2 actions provide it for free) and the mock honors
it, but cancel today is a destructive abort, not a pauseable
suspension.

`progress_pct` (float, 0–100) is fine for a UI progress bar but is a
poor primary key for resume because:

- It's a *commanded-time-elapsed* percentage, not "step 5 of 12"
  (there are no steps in the action interface — the mock assigns text
  labels at three thresholds purely for the GCS to display).
- "Resume from approximately the same point" is naturally a step-level
  notion (resume the last fluidic step that hadn't reported success);
  resuming at 47.3 % isn't physically meaningful.

**Required change in Phase 2b** (specification, not implementation
in this prompt):

- Add a `bool resume` (or, better, `string resume_token`) to the goal.
- Add a separate pause / resume service pair, or extend the action
  with goal-handle messages that carry pause / resume directives. The
  ROS 2 action API does not offer a built-in "pause", so a separate
  service against the running goal is the conventional pattern.
- Add a `int32 step_index` and `int32 total_steps` to the feedback so
  the GCS can show "Step 5 of 12: Pump A run 30 s" instead of a raw
  percentage.
- Decide what the result returns when a sequence is paused vs.
  canceled — likely an enum or a string status code, not just `success`.

### 6.2 The analysis sequencer node is **stateless** across pause / abort

`drivers/analysis_sequencer.py`:

- Carries no persistent state. Each goal handler runs in its own
  coroutine; on cancel or completion all locals are discarded.
- The only cross-call state is `_busy_lock` / `_busy`, used solely to
  reject a second goal while one is in flight.

For 5–10 minute sequences with required pause / resume, the sequencer
must:

- Persist the sequence script (the ordered list of fluidic steps).
- Persist the index of the last successfully-completed step.
- Persist any in-progress step's elapsed time / sub-state if the
  resume granularity is finer than "next step."
- Persist enough state across a node *crash and restart* if the team
  wants restart-resume on top of pause-resume (open question, but a
  realistic operator concern over a 10-minute experiment).
- Treat in-flight CAN-FD commands carefully: BDC motors are commanded
  with a duration ("run forward 30 s"). If the operator pauses 10 s
  in, the sequencer must `stop_motor` *and* remember to re-issue the
  remaining 20 s on resume — the board itself has no notion of pause.

The current mock's "linear ramp 0→100 % over 10 s" is meaningless under
these requirements.

### 6.3 The mixing servo driver assumes a single isolated device

`drivers/mixing_servo.py`:

- One service server, one preset-name string, no concept of "which
  servo on which board."
- Its companion `astrotech_interfaces.yaml` describes a single global
  servo with five named presets.

Reality (per `third_party/astrotech_canfd/`):

- Servos live on a Servo board addressed by a CAN ID (default `1`,
  next board would be `3`).
- Each board hosts up to 16 servos (`servo_id` 0–15).
- The same CAN-FD bus also carries BDC pump traffic, so the driver
  layer must serialize commands across all boards through one
  `FdCanInterface` instance — there is no "one ROS node per servo"
  isolation.
- Presets like "S1, S2, CO2_1, CO2_2, RETRACT" are almost certainly
  *fluidic states of the mixing chamber* (chamber positions for
  inlet/outlet), not arbitrary servo angles, and they map to a
  particular `(servo_id, degrees)` tuple that today is unknown.

**Required change in Phase 2b**:

- The interface contract should still be a single service the GCS
  panel calls (the abstraction is fine — the operator picks a preset
  by name, not by raw angle).
- The *driver* needs to be: (a) a single CAN-FD owner, (b) a registry
  of `preset_name → list[(board_can_id, servo_id, degrees)]` so a
  single preset can move multiple servos in coordinated fashion if the
  fluidic chamber requires it.
- The auger driver must coexist on the same `FdCanInterface` — see
  §6.4.

### 6.4 The auger driver assumes its own dedicated transport

`drivers/auger.py` subscribes to its own topic and integrates a
fake position model. That works fine for the GCS-side stub. But:

- The auger is most likely a BDC motor on a BDC board (`can_id=2`,
  `motor_id=N`) sharing the bus with all the pumps, the heater, and
  whatever else hangs off the BDC controller(s).
- The library doesn't expose encoder / current / completion feedback
  (all commands are "run for D seconds and we trust it"). So
  `AugerState.position_rev` and `velocity_rev_s` are fictional
  precision — there is no source of truth on the rover for either
  field.

**Required changes in Phase 2b** (already partly documented in
`TODO(astrotech-q-1)`):

- Drop `position_rev` / `velocity_rev_s` from `AugerState` or rename
  them to commanded values (e.g. `last_command_kind` enum and
  `last_command_duration_s`). The mock can simulate "position" but a
  real driver cannot.
- Same for `is_moving` — without feedback the driver only knows the
  commanded duration and elapsed time since the last command.

### 6.5 Heater / Ninhydrin actuator is missing entirely

The interview notes (Session 3, just received) mention a heater for
Ninhydrin. Phase 2a has no topic / service / driver for a heater. This
is a clean addition — not a refactor — but it does invalidate the
`feature_to_topic_map.md` claim that the GCS feature list was complete.

### 6.6 The mock under-represents the bus topology

The YAML / mock pretends each feature is a separate node on its own
transport. The actual rover has **one CAN-FD bus, one
`FdCanInterface`** that owns the dongle, and the BDC + Servo + auger +
heater drivers all serialize commands through it. Phase 2b's driver
package will be a single ROS 2 node that:

- Owns the `FdCanInterface`.
- Multiplexes commands from multiple ROS topics / services / actions.
- Cannot run the auger and a sequence step concurrently if they target
  the same controller.

The Phase 2a mock's "swap one driver class" architecture is still a
useful organizing principle for the *panel* side, but the *driver*
side will be more centralized than the YAML implies.

---

## 7. Recommended revisions for Phase 2b (synthesis only — do not implement)

Suggested in priority order. None are committed in this prompt.

1. **Revise `RunAnalysisSequence.action`** to include `string resume_token`
   in the goal, `int32 step_index` / `int32 total_steps` in the
   feedback, and a structured status (e.g. `uint8 status` with enum
   constants for `OK | PAUSED | CANCELED | FAILED`) in the result. Add
   a sibling service or services for pause / resume / abort against
   the running goal handle.
2. **Replace `MockAnalysisSequencer` with a stateful sequencer.** It
   should carry a script (probably a YAML list of steps), persist the
   last-completed step, and translate `pause` into "stop the in-flight
   motor / servo command and remember where we were." Disk persistence
   (so a node restart can offer resume) is desirable but optional.
3. **Add a `cmr_canfd_driver` ROS 2 package** that wraps
   `third_party/astrotech_canfd/servo_bdc_control_ms` (or pip-installs
   it once Astrotech publishes it as a real package). One node owns
   the bus; it exposes:
   - per-BDC-motor topic for run-forward-N / run-reverse-N / stop.
   - per-servo service or topic for `go_to_position(degrees)`.
   - heater topic / service.
   - optional Foxglove-friendly status topic per actuator that echoes
     "last command at T, expected to complete by T+D".
4. **Mixing-servo abstraction**: keep the GCS `set_preset` service
   contract; add a config table `preset_name → list[(board_can_id,
   servo_id, degrees)]` driven by a YAML so Astrotech can edit it
   without touching code.
5. **Auger refactor**: drive it through `cmr_canfd_driver`. Update
   `AugerState.msg` to drop fictional encoder fields.
6. **Heater / Ninhydrin**: new `cmr_msgs/srv/SetHeater.srv` (or a
   topic with target temperature), wrapped by `cmr_canfd_driver`.
7. **GCS panels**: `AnalysisSequence` panel grows real action-client
   code (subscribe to feedback, render step index / total, expose
   pause / resume buttons). Other panels stay stubs until Phase 2c.
8. **Update interview notes + assumptions docs** to lock the answers
   in writing — easy to lose context if the interview is left in
   Slack only.

End of overview.
