# Phase 2a — Overview

What got built in Phase 2a + revision pass, where it lives, and which
assumptions are still placeholders. For the entry-point summary read
[`system_overview.md`](system_overview.md) first; this doc is the
reference-detail companion.

## What was built

- **`src/cmr_msgs/`** — five new interfaces, all listed in
  `CMakeLists.txt`:
  - `msg/RamanSpectrum.msg`, `msg/EnvSample.msg`, `msg/AugerState.msg`
    (commanded-only after the post-Caitlin revision).
  - `srv/SetMixingServoPreset.srv`.
  - `action/RunAnalysisSequence.action` (post-revision: int8 goal,
    `last_completed_step` in result, `current_step / total_steps /
    current_step_description / elapsed_seconds` in feedback. No
    pause/resume — see `post_urc_backlog.md`).
- **`src/urc_mock_rover/`** — ament_python package. Single coordinator
  node (`mock_rover_node`) loads `config/astrotech_interfaces.yaml` and
  spins one driver per feature area:
  - `drivers/auger.py` — commanded-only `AugerState` echo.
  - `drivers/mixing_servo.py` — preset service + state echo.
  - `drivers/analysis_sequencer.py` — stateless action server, N fake
    steps × per-step duration (default 5 × 2 s).
  - `drivers/raman.py` — synthesized 1024-point spectrum at 1 Hz.
  - `drivers/env.py` — CO₂/humidity/temperature with periodic spike.
- **`launch/mock.launch.py`** — single command brings up the mock plus
  the Phase 1 `launch/gcs_bridge.launch.py` (Foxglove on
  `ws://localhost:8765`).
- **`scripts/smoke_test.sh`** — colcon-build → launch → assert topics
  → assert Raman rate > 0.5 Hz → call `set_preset(S1)` returns success.
- **`gcs/layouts/urc_astrotech_dashboard.json`** — 2×3 mosaic of the
  Plot, Log, and four custom panels.
- **`gcs/extensions/urc-astrotech-panels/`** — `@foxglove/extension`
  scaffold with four stub panels (`urc.auger_control`,
  `urc.analysis_sequence`, `urc.mixing_servo`, `urc.raman_spectrum`)
  and a hand-written TypeScript mirror of `astrotech_interfaces.yaml`
  in `src/interfaces.ts`.
- **Documentation**: `phase2a_assumptions.md`,
  `astrotech_interview_notes.md` (this branch's running record),
  `phase_2a_overview.md` (this file), `astrotech_canfd_library_notes.md`,
  `system_overview.md`, `post_urc_backlog.md`.

Camera feeds are intentionally *not* part of Phase 2a — separate
branch, will merge in later.

## Architecture

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
        │  - instantiates 5 drivers           │
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
                          │ MockEnvPublisher         │
                          │ /astrotech/env/sample    │
                          └──────────────────────────┘
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
   │  - urc_astrotech_dashboard.json (2×3 mosaic)                 │
   │  - urc-astrotech-panels extension (4 stubs):                 │
   │      urc.auger_control, urc.analysis_sequence,               │
   │      urc.mixing_servo, urc.raman_spectrum                    │
   └──────────────────────────────────────────────────────────────┘
```

## What is not verified

The whole pipeline is plausible-by-static-check; nothing has actually
run. Open items, in order of how cheaply someone with a Humble box
could close them:

- `colcon build --packages-select cmr_msgs urc_mock_rover` — proves
  the new `.msg` / `.srv` / `.action` files generate.
- `ros2 launch urc_mock_rover mock.launch.py` — proves the coordinator
  starts every driver.
- `scripts/smoke_test.sh` — end-to-end: topics enumerate, Raman rate
  passes, mixing-servo service returns success.
- `npm install && npm run build` in
  `gcs/extensions/urc-astrotech-panels/` — surfaces real TypeScript
  errors against `@foxglove/extension`.
- Foxglove Studio actually rendering the layout JSON.

## Outstanding `TODO(astrotech-q-N)` tags

Cross-reference: `docs/astrotech_interview_notes.md` carries the
question text and the latest answer status.

| Q | In source | Topic |
|---|---|---|
| Q1 | `auger.py`, `AugerState.msg`, `astrotech_interfaces.yaml`, `interfaces.ts`, `panels/AugerControl.tsx` | Auger controller cmd shape (currently `geometry_msgs/Twist`). Moteus confirmed; final wire shape still unconfirmed. |
| Q2 | `astrotech_interfaces.yaml`, `mixing_servo.py`, `interfaces.ts`, `panels/MixingServo.tsx` | Mixing servo controller family. |
| Q3 | `astrotech_interfaces.yaml`, `analysis_sequencer.py` | Real sequence step set + durations. Mock uses 5 generic steps. |
| Q4 | `RunAnalysisSequence.action`, `analysis_sequencer.py` | `sequence_id` vs `site_num` semantics. |
| Q5 | `astrotech_interfaces.yaml`, `raman.py`, `interfaces.ts` | Whether a real Raman driver exists with its own type. |
| Q6 | `astrotech_interfaces.yaml`, `env.py`, `interfaces.ts` | Same question for CO₂/humidity. |

Q9 (camera id ↔ logical role) is dropped from the source tree — that
work moved to a separate branch.

## Section 6 — assumptions Phase 2a *originally* baked in that the
post-Caitlin info invalidated, and what the revision did about each

### 6.1 The original action interface didn't model pause/resume

Phase 2a (commit `eef937c`) shipped a one-shot
`RunAnalysisSequence.action` with `progress_pct` feedback. Pause /
resume can't ride on that.

**Revision pass decision (this commit):** sequences are
**run-to-completion with hard abort**. The action drops `progress_pct`,
adds `current_step`, `total_steps`, `current_step_description`,
`elapsed_seconds` to feedback, and `last_completed_step` to the
result. Cancel returns `success=false` with `"cancelled at step N"`
and leaves the rover in whatever state it was in. Pause/resume is
**deferred to post-URC** — see `docs/post_urc_backlog.md`.

### 6.2 The mock sequencer was stateless and that's correct

Phase 2a sequencer was already stateless across goals; the previous
overview flagged this as wrong because pause/resume was on the
roadmap. With pause/resume removed, statelessness is the intended
model. Sequencer was rewritten anyway to use the new feedback shape
and to step through N discrete steps × per-step duration, but the
"no persistent state" property is preserved on purpose.

### 6.3 The mixing-servo mock under-represents the bus topology

Reality (per `third_party/astrotech_canfd/`): the mixing servo is one
of up to 16 servos on a Servo board sharing the CAN-FD bus with the
BDC pump motors. Multiple servo boards can coexist on different CAN
IDs. The mock pretends it's a single isolated device.

**Revision-pass decision:** out of scope. The *GCS contract* (a
service taking a preset name) is correct; the driver-side topology is
a Phase 2b problem, owned by whichever ROS 2 node wraps the
vendored library.

### 6.4 `AugerState` exposed encoder fields that don't exist

The CAN-FD library has zero feedback (no encoder, current, completion
ack). Phase 2a's `AugerState` had `position_rev`, `velocity_rev_s`,
`is_moving` — all fabricated.

**Revision-pass decision (this commit):** rewrote to commanded-only
fields: `last_command_kind` (enum), `commanded_duration`,
`commanded_at`. `MockAugerDriver` was rewritten to publish these
honestly; no integration of fake position over time.

### 6.5 Heater (Ninhydrin) is not represented

The interview notes mention a heater that is part of the science
payload. Phase 2a did not model it; revision pass does not add it.

**Revision-pass decision:** still out of scope until the team confirms
what board the heater is on and the command shape (BDC channel, GPIO,
or otherwise — see open question B4 in `astrotech_interview_notes.md`).
Treat as Phase 2b deliverable.

### 6.6 The rover bus is one CAN-FD bus with one library owner

The mock's "swap one driver class" architecture is fine for the
*panel* side. The *driver* side will be more centralized — Phase 2b
introduces a single ROS 2 node that owns the `FdCanInterface` and
multiplexes commands from the per-feature topics/services/actions.
Revision pass does not touch that yet; it is Phase 2b's first task.
