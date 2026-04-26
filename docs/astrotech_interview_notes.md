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
