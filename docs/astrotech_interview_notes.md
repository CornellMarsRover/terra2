# Astrotech Subteam Interview Notes

Running log of questions asked to the Astrotech subteam and their
answers. Source of truth for what the GCS and mock rover are being
built against. If it isn't written here, it wasn't agreed.

Last updated: 2026-04-23

---

## Session 1 — 2026-04-23

### Q1. What hardware drives the "analysis sequences"?

**Asked:** What drives Seq 1 and Seq 2 on the rover?

**Answer (paraphrased):** Different sequences of BDC boards. Each
sequence commands a set of brushed DC motor controllers to run a
pre-programmed routine.

**Status:** Partially answered. Follow-ups pending in Session 2.

---

## Session 2 — Pending

### Q2. Sequence behavior — step by step

**Asked:** Walk me through what Seq 1 and Seq 2 actually do, step by
step. Rough description is fine ("servo moves to S1, auger drops, BDC
motor A runs for N seconds, then...").

**Answer:** _(pending)_

**Implications for mock / panel:** The mock's sequence handler needs
to simulate realistic timing and side effects. The extension panel's
"Seq 1" / "Seq 2" buttons need labels that match the actual operation.

---

### Q3. Sequence duration

**Asked:** Roughly how long does each sequence take end to end?

**Answer:** _(pending)_

**Implications for mock / panel:** Determines whether a progress bar
is warranted and what the mock's `time.sleep()` value should be.

---

### Q4. Cancellable mid-run?

**Asked:** Once a sequence starts, does the operator need to be able
to cancel or pause it mid-run, or is it fire-and-forget?

**Answer:** _(pending)_

**Implications for mock / panel:** Decides the ROS2 interface shape.
Fire-and-forget → `std_srvs/Trigger` service. Cancellable with
progress → ROS2 action. Affects `cmr_msgs` and both the mock and the
extension panel.

---

### Q5. Sequence logic location

**Asked:** Where does the sequence logic live? Is it firmware on the
BDC board (we send a single "start seq 1" and the board handles the
whole routine), or does the main computer run the steps and issue
individual motor commands?

**Answer:** _(pending)_

**Implications for mock / panel:** Determines whether the ROS2 node
is a thin command-forwarder or a full sequencer with state machine,
retry logic, and fault handling.

---

### Q6. BDC board hardware and comms

**Asked:** What are the BDC boards, specifically? Custom PCBs, an
off-the-shelf driver (part number), or Arduino-based? And how do
they talk to the main computer: USB serial, CAN, I2C, something else?

**Answer:** _(pending)_

**Implications for mock / panel:** Affects the eventual real driver
node. Not a mock blocker, but needed before hardware integration.

---

## Future sessions (holding)

Not yet asked. Send in priority order once Session 2 is resolved.

### Cluster A — Motor controllers
- A1. Auger up/down and spin: moteus or Maxon? Exact part number?
- A2. Mixing chamber servo: Dynamixel, standard PWM hobby servo,
      something else?

### Cluster B — Sensors
- B1. Raman unit make/model. Does a vendor SDK publish to ROS2, or
      do we write the driver? What does one capture return (array
      length, x-axis units, y-axis scale)?
- B2. CO2 + humidity sensor part number. Response time, sampling
      rate. Combined sensor or two separate?

### Cluster C — Mixing chamber servo presets
- C1. What do S1, S2, CO2_1, CO2_2, Retract correspond to in servo
      units (angle / pulse width / encoder count)?
- C2. Who calibrates and owns updating them when the mechanism
      changes?
- C3. Do presets live in a rover-side config (yaml) or hardcoded
      in firmware?

### Cluster D — Snapshots and science writeup
- D1. What metadata needs to accompany each snapshot (site ID,
      sample ID, GPS, timestamp, operator)?
- D2. Save to rover filesystem for later rsync, or stream to the
      GCS laptop immediately?
- D3. When we "save the Raman graph," do you need raw spectrum data
      (CSV), plot image (PNG), or both?

### Cluster E — Mission scope
- E1. Is this GCS layout specifically for the Science Mission, or a
      universal operator view across all four URC missions?
- E2. During the mission window, who operates the GCS? Same student
      every time or rotating?
- E3. Complete sample-analysis walkthrough: who commands the mixing
      servo at each step, operator or sequence?

### Cluster F — Team / repo hygiene
- F1. Was there previous work on Astrotech ROS2 nodes that got
      deleted or parked on a branch?
- F2. What's our total bandwidth budget across all subteams and is
      there a throttling priority scheme?

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

## Open with Caitlin (followups for Session 4)

### Library API specifics

- A1. Pause semantics: when the operator pauses, do we (a) `stop_motor`
      every running BDC + `stop` every running servo and reissue
      remaining duration on resume; (b) abandon the current step,
      restart it from the beginning on resume; or (c) commit the
      current step as failed and resume from the next?
- A2. Maximum motor duration is 7 bits = 127 (seconds or ms). For 5–10
      minute sequences, do we chain N × 127 s commands with bridging
      `asyncio.sleep`, or is there a "run continuously until stopped"
      mode we haven't documented?
- A3. Does the command frame's `Query_data=1` bit actually generate a
      board-side reply we can consume? If yes, what's the payload?
- A4. Are unsolicited error / fault frames a thing the boards send?
      If yes, what's the wire format?
- A5. `control_mode` for the servo board reserves 2 bits but only 0
      (position), 2 (stop), 3 (set-home) appear in code. Is `1`
      reserved or undefined?

### Hardware layout

- B1. How many BDC boards and how many Servo boards on the rover at
      competition, and what are their CAN IDs?
- B2. Which BDC motor (board CAN ID + motor_id) drives the auger?
      Which drives each fluidic pump? Need a table.
- B3. Which servo (board CAN ID + servo_id + degrees) corresponds to
      each named mixing-chamber preset (S1, S2, CO2_1, CO2_2, RETRACT)?
- B4. Heater: BDC channel? Separate board? GPIO / I²C? What's the
      command shape?

### Bus / Linux integration

- C1. On the Jetson, what's the persistent device path for the
      `usbcanfd` dongle (a udev rule symlinking it to
      `/dev/usbcanfd`, or do we hardcode `/dev/ttyACM0`)?
- C2. Are simultaneous motor commands across different boards (e.g.
      auger BDC + mixing servo at the same instant) a real use case
      in any existing fluidic protocol, or do sequences always run
      one actuator at a time?

---

## Status markers

- **Phase 2a is committed** at SHA `eef937c` ("phase 2a: initial mock
  rover (pre-Caitlin info)"). It does not reflect the Session 3
  answers; the documents above call out which assumptions are now
  invalid.
- **Phase 2b** will revise the action interface
  (`cmr_msgs/action/RunAnalysisSequence.action`), make
  `MockAnalysisSequencer` stateful with pause / resume, and add a
  real driver wrapper around the vendored CAN-FD library.

