# Post-URC Backlog

Features explicitly deferred — not abandoned. We're building for URC
2026 and won't ship these in time, but they have a specific reason
they were dropped and a recorded plan for picking them up later. If
the answer to "should we add X for URC?" is "no, do it after the
competition," it goes here.

## 1. Pause / resume for analysis sequences

**Why deferred.** Run-to-completion with hard abort is correct under
current constraints: the CAN-FD library exposes no motor feedback
(see `docs/astrotech_canfd_library_notes.md`), BDC commands are
fire-and-forget for a fixed duration, and a 5–10 minute sequence is
cheap enough to redo from the top.

**Three options when we revisit.**

1. **Step-level resume.** Resume restarts the next un-completed
   *step* from its beginning. Implementation: persist
   `last_completed_step` after every step ends; on resume, replay
   from `last_completed_step + 1`. Simple, no dependence on motor
   feedback, robust to power loss between steps.
2. **Time-level resume within a step.** Resume continues the current
   step from the elapsed point. Implementation: track
   `(commanded_duration, command_start_monotonic)` per running motor;
   on pause send `stop_motor` for each; on resume send a fresh
   command with `remaining = commanded - elapsed`. More accurate but
   sensitive to drift, network latency, and any board reset between
   pause and resume.
3. **Manual resume.** Operator picks the step to resume from out of a
   list. Implementation: GCS panel shows step list; pause records
   step index; resume goal carries a `resume_from_step` field. Most
   honest, requires UX work.

Recommendation when the time comes: start with (1) plus a manual
override that maps to (3). Skip (2) until board feedback exists.

**What to change.**

- `cmr_msgs/action/RunAnalysisSequence.action`: add a goal field
  (`int32 resume_from_step` or `string resume_token`) and a
  pause/resume service pair, or extend the result enum
  (`SUCCESS | PAUSED | CANCELED | FAILED`).
- `drivers/analysis_sequencer.py`: persist sequence state across
  pause; consider disk persistence so a node restart can offer
  resume.
- `gcs/extensions/.../panels/AnalysisSequence.tsx`: add Pause / Resume
  buttons, wire to the new service / goal field.
- `docs/phase2a_assumptions.md`: re-add the pause/resume row.

## 2. Real motor feedback

**Why deferred.** The current Astrotech library doesn't read board
replies. Adding a feedback path is a hardware-firmware-host ladder
that needs Caitlin (and possibly hardware revisions to the BDC /
Servo board firmware).

**What we'd want.**

- Encoder counts or completion-ack frames per BDC motor.
- Current draw per motor for stall detection.
- Flow sensors on each pump line so the sequencer knows a fluidic
  step actually delivered fluid.
- Heater temperature readback.

**What to change when feedback exists.**

- `cmr_msgs/msg/AugerState.msg` regains real fields (replace or
  extend the commanded-only set).
- The Phase 2b driver wrapper consumes `rcv` lines and decodes board
  telemetry into ROS topics.
- Pause/resume option (2) becomes feasible.

## 3. CI integration of `scripts/smoke_test.sh`

Today the smoke test runs locally only. Wiring it into the
`build-test.yaml` workflow needs a runner that can do `colcon build`
of `cmr_msgs` + `urc_mock_rover` and run a foreground ROS process. The
existing `cornellmarsrover/dev:latest` image already has Humble — this
is a small change that's just lower priority than the URC-blocking work.

## 4. Snapshot service (Q7)

Operator-triggered "save the current spectrum / env reading / camera
frame with metadata" service. Out of scope until the science writeup
process is defined. Open question: storage target (rover SSD vs.
streamed to GCS), filename convention, retention.

## 5. Fabric-managed `foxglove_bridge` (Q11)

Today the bridge runs as a plain `launch_ros.Node`. Wrapping it in
the `cmr_fabric` lifecycle so it gets fault recovery is a hardening
step that's only worth it once we have the rest of the rover under
Fabric for a competition run.
