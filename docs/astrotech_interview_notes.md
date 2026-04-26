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
- Telemetry messages are **commanded-only when no real feedback is
  available, closed-loop where it is.** The BDC / servo / heater
  stack on Caitlin's library exposes no feedback (commanded-only);
  the auger is on moteus and has full closed-loop telemetry — see
  Session 5.
- **Cameras are off this branch.** Camera feed work happens on a
  separate branch that will merge in later.
- **Hardware**: custom **BDC boards** (BDC motors / pumps) and
  **Servo boards** (hobby servos / mixing chamber) on a CAN-FD bus
  via an mjbots `usbcanfd` USB-CAN dongle. Library at
  `third_party/astrotech_canfd/` — see
  `docs/astrotech_canfd_library_notes.md`.

**Pending — ask Caitlin** (BDC + servo + heater stack only; the auger
moved to moteus per Session 5 and has its own questions below):

1. can the boards talk back to the jetson? does setting `query_data=1`
   on a command frame return real telemetry (encoder counts, completion
   ack, current), or is it a leftover bit?
2. diagram of the final electrical layout for the can-fd stack —
   `(can_id, motor_id)` for each pump and the heater, plus
   `(can_id, motor_id)` for the mixing servo and the angle/position
   value for each preset (s1, s2, co2_1, co2_2, retract).
3. the `duration` field is 7 bits, max 127. for pumps that run 5+
   minutes, do i chain commands or is there a "run continuously until
   stopped" mode?
4. when bdc and servo boards share the same can-fd bus, who owns bus
   initialization and arbitration? should one ros 2 node own the bus,
   or does the library handle multi-writer scenarios?
5. do the boards send unsolicited fault frames (motor stalled,
   over-current, etc.)? if yes, what's the wire format — and does the
   existing library raise/log them, or just drop them?
6. (minor) on the jetson, what device path does the usb-can dongle
   show up as? udev symlink, or hardcode `/dev/ttyACM0`? probably
   answerable with `lsusb`/`dmesg` once hardware is in front of me;
   not a blocker.

**Pending — ask project lead / hardware lead** (auger / moteus side):

- are the moteus controllers (lead screw id=15, auger id=16) on a
  separate `fdcanusb` / pi3hat from the bdc-and-servo can-fd bus, or
  do they share one transport?
- gear ratios for lead screw and auger (motor revolutions →
  physical output: mm of vertical travel for the lead screw, real
  rev/s of the auger flutes after gear reduction)?
- safe operating ranges for torque, velocity, and lead-screw
  position — what hard limits should the panel and the driver enforce
  before we trust an operator to hold a button?

**Pending — confirm against URC 2026 rulebook** (only blocks competition
freeze, not development):

- URC 2026 RF bandwidth cap. `docs/bandwidth_audit.md` uses a 5 Mbps
  working assumption from prior years. Confirm before we lock the
  Foxglove topic whitelist.

**Phase status**: Phase 2a snapshot at `eef937c`; revision pass at
`6bd8fcf` (dropped pause/resume, removed cameras); auger pass (this
commit) restored closed-loop `AugerState` and added `AugerCommand`
after Session 5. Phase 2b will wrap **two** driver stacks: moteus
(auger) and Caitlin's CAN-FD library (pumps + servo + heater).

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

| # | Question | Status |
|---|---|---|
| Q2 | Sequence behavior — step by step ("servo moves to S1, auger drops, BDC motor A runs N s, …") | **Still pending.** Partly subsumed by Session 3 (sequences are fluidic protocols) but no specific step list yet. |
| Q3 | Sequence duration | **Resolved (Session 3):** 5–10 minutes end to end. |
| Q4 | Cancellable mid-run? | **Resolved (Session 4):** run-to-completion with hard abort. Pause/resume → `post_urc_backlog.md`. |
| Q5 | Sequence logic location | **Resolved (Session 3):** main computer (Jetson). Boards are fire-and-forget. |
| Q6 | BDC board hardware and comms | **Resolved (Session 3):** custom CMR PCBs, CAN-FD over an mjbots `usbcanfd` dongle. Library at `third_party/astrotech_canfd/`. |

---

## Future sessions — still open after Sessions 1–4

Held questions not migrated to the top-of-file Pending list. Resolved
/ superseded items pruned (A1/A2, C1, D, F2 — see end of section).

- **B1.** Raman unit make/model. Vendor SDK with ROS 2 support, or do
  we write the driver? Capture-return shape (array length, x-axis
  units, y-axis scale)?
- **B2.** CO2 + humidity sensor part number. Response time / sampling
  rate. Combined sensor or two separate?
- **C2.** Who calibrates the mixing-chamber preset values when the
  mechanism changes?
- **C3.** Do presets live in rover-side config (YAML) or in firmware?
- **E1.** Is the GCS layout for the Science Mission specifically, or
  a universal operator view across all four URC missions?
- **E2.** Who operates the GCS during the mission window — fixed or
  rotating student?
- **E3.** Sample-analysis walkthrough: who commands the mixing servo
  at each step, operator or sequence?
- **F1.** Was there previous Astrotech ROS 2 work that got deleted or
  parked on a branch?

(Resolved/absorbed: A1/A2 — moteus + servo board on CAN-FD per
Session 3; C1 — folded into the electrical-layout question at top;
D1–D3 → `post_urc_backlog.md` §4; F2 → URC rulebook pending bucket.)

---

## Session 3 — 2026-04-26

Confirmed by Astrotech (the canonical statements; the *implications*
have all been folded into Current state at the top of this file):

- **Sequence duration:** 5–10 minutes end to end.
- **Pause/resume:** must be resumable from approximately the same
  point if aborted. (Later overruled by Session 4 → run-to-completion;
  the resume requirement is in `docs/post_urc_backlog.md`.)
- **Sequence content:** physical fluidic protocols — pumps, valves,
  mixing-chamber positions, heater on/off, sensor reads. The
  Ninhydrin path needs a heater (not yet represented in GCS / mock).
- **Hardware (BDC + servo stack):** custom **BDC boards** (≤6 motors,
  default CAN id 2) and **Servo boards** (≤16 servos, default CAN id
  1) over **CAN-FD via mjbots `usbcanfd` dongle**. Asyncio Python
  library at `third_party/astrotech_canfd/` (see
  `docs/astrotech_canfd_library_notes.md`).
- **Orchestration location:** main computer (Jetson, in our ROS 2
  graph), not board firmware. Boards are fire-and-forget
  `move_motor_for_N_seconds` devices.

---

## Session 4 — 2026-04-26 (decisions only — no new questions answered)

Team scoping calls in response to Session 3, given URC timing. All
four are now reflected in Current state at the top; recapped briefly
for the historical record.

1. **Run-to-completion with hard abort.** Cancel returns
   `success=false`; rover stays put. Pause/resume → `post_urc_backlog`.
2. **Commanded-only telemetry where no feedback exists.**
   `RamanSpectrum`, `EnvSample`, `SetMixingServoPreset` already
   honest; `AugerState` was rewritten to commanded-only. _(Session 5
   later overrode this for the auger only — moteus does provide real
   telemetry. The BDC + servo + heater stack stays open-loop.)_
3. **Pause/resume deferred to post-URC.** Three implementation
   options ranked in `docs/post_urc_backlog.md`.
4. **Cameras off this branch.** All Phase 2a camera scaffolding
   removed from `astrotech-gui` to avoid merge conflicts with the
   parallel camera branch.

---

## Session 5 — 2026-04-26 (auger is on moteus, not Caitlin's library)

Astrotech sent over `auger_keys.py`, a hand-rolled keyboard test
script (now vendored at `docs/reference/auger_keys_test_harness.py`).
The script uses Josh Pieper's `moteus` Python library against two
controllers — id=15 (lead screw, vertical), id=16 (auger, rotation) —
on `moteus.get_singleton_transport()`. Velocity-mode commands are
issued as `make_position(position=NaN, velocity=X, maximum_torque=Y)`
on a 50 Hz watchdog loop while a key is held. moteus exposes
position, velocity, torque, temperature, mode, and fault per cycle,
so closed-loop telemetry is real for the auger.

Resulting changes (this commit):

- `cmr_msgs/msg/AugerState.msg` rewritten to closed-loop fields for
  both lead screw and auger (overrides Session 4 Decision 2 for the
  auger only — the BDC + servo + heater stack stays open-loop).
- `cmr_msgs/msg/AugerCommand.msg` added: hold-to-act velocity
  command with per-motor max-torque limits, 200 ms watchdog at the
  driver.
- Mock auger driver rewritten to publish 20 Hz simulated telemetry
  with first-order velocity tracking, random-walk torque, and slow
  thermal drift.
- Foxglove auger panel rewritten as hold-to-act with live telemetry
  readout. Defaults match the test script: lead screw ±100 rev/s
  @ 2.0 N·m, auger +10 rev/s forward / −50 rev/s reverse @ 1.0 N·m.

The Caitlin pending list at the top is now scoped to the BDC + servo
stack only. Question 1 (`query_data=1` telemetry) and question 4
(bus arbitration) no longer apply to the auger.
