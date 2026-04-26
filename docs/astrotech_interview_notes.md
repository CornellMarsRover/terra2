# Astrotech Subteam Interview Notes

Running log of questions asked to the Astrotech subteam and their
answers. Source of truth for what the GCS and mock rover are being
built against. If it isn't written here, it wasn't agreed.

Last updated: 2026-04-26

---

## Current state (read me first)

**Decided** (will not change without going back to the team):

- Sequences are **5–10 minutes long, run-to-completion with hard
  abort**. Cancel returns the action with `success=false` and leaves
  the rover wherever it stopped. Pause/resume is **deferred to
  post-URC** (`docs/post_urc_backlog.md`).
- Sequence orchestration **lives on the main computer** (the Jetson,
  in our ROS 2 graph), not on the BDC boards. The boards are
  fire-and-forget "run motor X for D seconds" devices.
- Telemetry messages are **commanded-only** when no real feedback is
  available. The CAN-FD library exposes none, so `AugerState.msg`
  carries `last_command_kind` / `commanded_duration` / `commanded_at`
  rather than fictional encoder counts.
- **Cameras are off this branch.** Camera feed work happens on a
  separate branch that will merge in later.
- **Hardware**: custom **BDC boards** (BDC motors / pumps) and
  **Servo boards** (hobby servos / mixing chamber) on a CAN-FD bus
  via an mjbots `usbcanfd` USB-CAN dongle. Library at
  `third_party/astrotech_canfd/` — see
  `docs/astrotech_canfd_library_notes.md`.

**Pending — ask Caitlin** (does **not** block Phase 2b starting; does
block the real driver wrapper inside Phase 2b). Ordered by impact —
#1 determines whether `AugerState` and future feedback messages can
have honest fields beyond commanded-only:

1. can the boards talk back to the jetson? does setting `query_data=1`
   on a command frame return real telemetry (encoder counts, completion
   ack, current), or is it a leftover bit?
2. diagram of the final electrical layout would help. specifically i
   need:
   - `(can_id, motor_id)` for the auger motor, each pump, and the
     heater;
   - `(can_id, motor_id)` for the mixing servo, plus the
     angle/position value for each preset (s1, s2, co2_1, co2_2,
     retract).
3. the `duration` field is 7 bits, max 127. for pumps that run 5+
   minutes, do i chain commands or is there a "run continuously until
   stopped" mode?
4. when bdc, servo, and any future boards share the same can-fd bus,
   who owns bus initialization and who handles arbitration if two
   coroutines want to send frames at once? should one ros 2 node own
   the bus, or does the library handle multi-writer scenarios?
5. do the boards send unsolicited fault frames (motor stalled,
   over-current, etc.)? if yes, what's the wire format — and does the
   existing library raise/log them, or just drop them?
6. (minor) on the jetson, what device path does the usb-can dongle
   show up as? udev symlink, or hardcode `/dev/ttyACM0`? probably
   answerable with `lsusb`/`dmesg` once hardware is in front of me;
   not a blocker.

**Pending — confirm against URC 2026 rulebook** (only blocks competition
freeze, not development):

- URC 2026 RF bandwidth cap. `docs/bandwidth_audit.md` uses a 5 Mbps
  working assumption from prior years. Confirm before we lock the
  Foxglove topic whitelist.

**Phase status**:

- Phase 2a snapshot at SHA `eef937c`.
- Phase 2a revision (`6bd8fcf`) drops pause/resume from the action,
  swaps `AugerState` to commanded-only, removes camera scaffolding.
- Phase 2b will introduce a real CAN-FD driver wrapper plus build the
  panel widgets. Pending the open questions above.

## Slack draft to Caitlin

Ready to copy-paste. Send all six together, don't drip-feed:

> hey caitlin — wrapping the can-fd library as a ros node and have a
> few questions, ordered by impact:
>
> 1. can the boards talk back? does `query_data=1` on a command frame
>    actually return telemetry (encoder counts, completion ack,
>    current), or is it a leftover bit? this one decides whether the
>    gcs can show real motor state or just "we sent a command N
>    seconds ago."
> 2. a diagram of the final electrical layout would help. specifically
>    i need `(can_id, motor_id)` for the auger motor, each pump, and
>    the heater; plus `(can_id, motor_id)` for the mixing servo and
>    the angle/position value for each preset (s1, s2, co2_1, co2_2,
>    retract).
> 3. the duration field on bdc commands is 7 bits, max 127. for pumps
>    that run 5+ minutes, do i chain commands or is there a
>    continuous-run mode?
> 4. when the bdc and servo boards (and any others) share the can-fd
>    bus, who owns initialization and arbitration? should one ros 2
>    node own the bus, or does the library handle multi-writer
>    scenarios?
> 5. do the boards send unsolicited fault frames (motor stalled,
>    over-current)? if yes, what's the format — and does the existing
>    library raise/log them, or just drop them?
> 6. minor: on the jetson, what device path does the usb-can dongle
>    show up as? udev symlink, or hardcoded `/dev/ttyACM0`? happy to
>    figure this out with `lsusb` once i have hardware.

---

## Session 1 — 2026-04-23

### Q1. What hardware drives the "analysis sequences"?

**Asked:** What drives Seq 1 and Seq 2 on the rover?

**Answer (paraphrased):** Different sequences of BDC boards. Each
sequence commands a set of brushed DC motor controllers to run a
pre-programmed routine.

**Status:** Partially answered. Follow-ups pending in Session 2.

---

## Session 2 — Asked

### Q2. Sequence behavior — step by step

**Asked:** Walk me through what Seq 1 and Seq 2 actually do, step by
step. Rough description is fine ("servo moves to S1, auger drops, BDC
motor A runs for N seconds, then...").

**Answer:** _(still pending)_ — partly subsumed by Session 3
(sequences are fluidic protocols orchestrating pumps + servo + heater)
but no specific step list yet.

---

### Q3. Sequence duration

**Resolved in Session 3:** 5–10 minutes end to end.

---

### Q4. Cancellable mid-run?

**Resolved in Session 4:** run-to-completion with hard abort. Cancel
returns the action with `success=false`; rover stays in whatever state
it was in. Pause/resume deferred to `docs/post_urc_backlog.md`.

---

### Q5. Sequence logic location

**Resolved in Session 3:** main computer (Jetson, in our ROS 2 graph).
Boards are fire-and-forget; the host has to be the sequencer.

---

### Q6. BDC board hardware and comms

**Resolved in Session 3:** custom CMR PCBs, CAN-FD over an mjbots
`usbcanfd` USB-CAN dongle. Library at `third_party/astrotech_canfd/`.

---

## Future sessions — still open after Sessions 1–4

Held questions that haven't been answered or migrated to the Pending
list at the top. Resolved/superseded items have been pruned; what
remains:

### Cluster B — Sensors
- B1. Raman unit make/model. Does a vendor SDK publish to ROS 2, or
      do we write the driver? What does one capture return (array
      length, x-axis units, y-axis scale)?
- B2. CO2 + humidity sensor part number. Response time, sampling rate.
      Combined sensor or two separate?

### Cluster C — Mixing chamber servo presets
- C2. Who calibrates and owns updating preset values when the
      mechanism changes?
- C3. Do presets live in a rover-side config (yaml) or hardcoded in
      firmware?

### Cluster D — Snapshots
Deferred to `docs/post_urc_backlog.md` §4. Re-open after URC if the
science-writeup workflow needs them.

### Cluster E — Mission scope
- E1. Is this GCS layout specifically for the Science Mission, or a
      universal operator view across all four URC missions?
- E2. During the mission window, who operates the GCS? Same student
      every time, or rotating?
- E3. Complete sample-analysis walkthrough: who commands the mixing
      servo at each step, operator or sequence?

### Cluster F — Team / repo hygiene
- F1. Was there previous work on Astrotech ROS 2 nodes that got
      deleted or parked on a branch?

(Resolved or absorbed elsewhere: A1/A2 — moteus + servo board on
CAN-FD per Session 3; C1 — folded into the diagram-of-electrical-
layout question at the top; F2 — RF cap is now in the URC rulebook
pending bucket.)

---

## Session 3 — 2026-04-26

Confirmed answers received from Astrotech (paraphrased; lock these in
the assumptions doc on next pass):

### Sequence duration

- Each analysis sequence runs **5–10 minutes** end to end.
- Implication for Phase 2a mock: the 10-second linear-ramp placeholder
  in `MockAnalysisSequencer` is meaningless at this scale. Replace
  before any operator workflow review.

### Pause / resume requirement

- Sequences must be **resumable from approximately the same point**
  if aborted mid-run.
- "Approximately the same point" almost certainly means **step-level**
  (resume the next fluidic step that hadn't reported success), but
  this is **not yet confirmed** — open question for next Caitlin
  exchange.
- Implication: the `RunAnalysisSequence.action` shape is wrong.
  Pause/resume need to be modelled — see `docs/phase_2a_overview.md §6.1`.

### Sequence content

- Sequences are **physical fluidic protocols**, not "run motor X for
  N seconds in sequence." They orchestrate pumps, valves, mixing
  chamber positions, heater on/off, and (probably) sensor reads.
- The Ninhydrin chemistry path requires a **heater** that is currently
  not represented anywhere in the GCS / mock rover. Add as a Phase 2b
  deliverable.
- Implication: feedback should expose `step_index / total_steps` and
  a step-name (e.g. "Pump A 30 s", "Mixing chamber → CO2_1", "Heater
  on, hold 60 s") rather than a raw progress percentage.

### Hardware layer

- **BDC boards** drive the brushed-DC motors used as fluidic pumps.
  Each board addresses up to 6 motors over CAN-FD. Default board CAN
  ID `2`; a second board would be `4`.
- **Servo boards** drive hobby servos including the mixing chamber.
  Each board addresses up to 16 servos. Default board CAN ID `1`; a
  second board would be `3`.
- Bus is **CAN-FD over a USB-CAN converter** (mjbots `usbcanfd`
  dongle, hosted as a serial port).
- An **asyncio Python library** already exists; it is now vendored
  under `third_party/astrotech_canfd/`. See
  `docs/astrotech_canfd_library_notes.md` for the API surface.

### Sequence orchestration location

- Confirmed: orchestration **must live on the main computer** (the
  Jetson, in our ROS 2 graph), not in board firmware. Pause/resume
  cannot be implemented on the boards because the boards' command
  shape is "run motor X for N seconds, fire and forget" with no
  intra-command pause.
- Implication: the analysis sequencer node owns the sequence script,
  the elapsed time per in-flight motor command, and is the only
  authority that knows where in the protocol the rover currently is.

---

## Session 4 — 2026-04-26 (decisions only — no new questions answered)

After internal scoping discussion, the team made the following calls
about how to handle the open Session 3 ambiguity given URC timing:

### Decision 1: run-to-completion with hard abort

- Sequences run start-to-finish; cancel returns the action with
  `success=false` and leaves the rover wherever it stopped. Recovery
  is the operator's problem.
- Reason: the CAN-FD library exposes no motor feedback, so any
  pause/resume design today would be guessing about elapsed-vs-
  commanded time. A 5–10 minute redo is acceptable.
- Code change: `RunAnalysisSequence.action` revised to drop
  `progress_pct`; new feedback is `current_step / total_steps /
  current_step_description / elapsed_seconds`; new result field
  `last_completed_step`.

### Decision 2: telemetry messages are commanded-only when there's no real feedback

- `AugerState.msg` rewritten to `last_command_kind` /
  `commanded_duration` / `commanded_at`. No fictional encoder counts.
- Other custom messages audited: `RamanSpectrum`, `EnvSample`,
  `SetMixingServoPreset` survive unchanged (they describe sensor
  measurements or service-accept-success, not motor feedback).

### Decision 3: pause/resume deferred to post-URC

- Logged in `docs/post_urc_backlog.md` with three implementation
  options ranked. Re-open after URC.

### Decision 4: camera feed work merging from a different branch

- All Phase 2a camera scaffolding (mock replayer, NAL splitter,
  fetch script, sample H.264 asset, camera entries in YAML / TS
  mirror / Foxglove layout, Q9 tags) removed from `astrotech-gui`.
- Reason: avoid merge conflicts with the camera branch. The Foxglove
  layout drops to a 2×3 grid; `Image` panels can be re-added when
  the branches merge.

(Open questions for Caitlin live at the top of this file under
"Current state → Pending — ask Caitlin", with a copy-paste Slack
draft right after.)

