# Astrotech GCS — Current State

The "where we are right now" doc for the `astrotech-gui` branch. Read
this first. Everything else in `docs/` is historical, reference, or
deferred-work material that this doc points at.

**Last updated:** 2026-05-08 (post-mixing-servo bench bring-up; auger
panel v0.3.2 settle tuning).

## TL;DR

The Astrotech GCS is end-to-end working against the mock rover and
partially-working against real hardware (auger and mixing servo both
move from the Foxglove panels through ROS drivers to their CAN-FD
endpoints). The four Foxglove panels:

| Panel | State |
|---|---|
| **Auger control** | Fully wired. Hold-to-act velocity + position-memory homes + Return-to-home with tuned settle. Tested on hardware 2026-05-07. |
| **Mixing servo** | Fully wired. `set_home` + 3 editable site presets + manual goto + last-commanded readout. Tested on hardware 2026-05-08. |
| **Raman spectrum** | Partial — status line ("last spectrum: N points received at T"). No plot yet. |
| **Analysis sequence** | Stub — clicks log the would-be action server to the browser console. |

Two custom layouts at `gcs/layouts/`:
- `urc_astrotech_auger.json` — auger + mixing servo + auger cam + ZED.
- `urc_astrotech_analysis.json` — analysis seq + Raman + analysis cam + CO₂ plot.

Foxglove extension version: **0.3.2**.

## Hardware reality (what's been driven from a Foxglove panel)

| Subsystem | Stack | Status |
|---|---|---|
| Auger lead screw (moteus id=15) | moteus on fdcanusb | **Driven from panel.** Hardware bring-up 2026-05-07. Safety caps: lead ±50 rev/s, torque 3 Nm. |
| Auger spin (moteus id=16) | moteus on fdcanusb | **Driven from panel.** Caps: ±6 rev/s, 3 Nm. |
| Mixing chamber servo | CMR servo board on fdcanusb, can_id=26, servo_id=15, 1 Mbit/s | **Driven from panel.** Bench bring-up 2026-05-08. |
| BDC pumps | Caitlin's CAN-FD library | Not wired. Phase 2b. |
| Raman, CO₂/humidity, heater | n/a in repo | Not wired. Phase 2b. |
| Cameras (auger / analysis / ZED) | [`src/cmr_cams/`](../src/cmr_cams/) (in-repo launch + lifecycle configs) + `cmr_cv.camera_node` and stereolabs `zed_wrapper` (rover-installed) | Wired. GUI subscribes to `/cam11/image_raw`, `/cam12/image_raw`, `/zed/zed_node/left/image_rect_color`. Bring-up: SSH into the rover Jetson and follow the four-terminal procedure in [`operator_guide.md`](operator_guide.md). |

## ROS interfaces (single source of truth: [`astrotech_interfaces.yaml`](../src/urc_mock_rover/config/astrotech_interfaces.yaml))

```
/astrotech/auger/cmd_vel          cmr_msgs/AugerCommand        (10 Hz hold-to-act, 200 ms watchdog)
/astrotech/auger/state            cmr_msgs/AugerState          (20 Hz mock, 50 Hz real)
/astrotech/mixing_servo/set_angle cmr_msgs/SetMixingServoAngle (deg, 12-bit wire)
/astrotech/mixing_servo/set_home  std_srvs/Trigger
/astrotech/mixing_servo/state     std_msgs/Int32               (last commanded deg, 5 Hz)
/astrotech/analysis/run_sequence_1  cmr_msgs/RunAnalysisSequence (action)
/astrotech/analysis/run_sequence_2  cmr_msgs/RunAnalysisSequence (action)
/astrotech/raman/spectrum         cmr_msgs/RamanSpectrum       (1 Hz)
/astrotech/env/sample             cmr_msgs/EnvSample           (1 Hz)
```

The TypeScript mirror lives at
[`gcs/extensions/urc-astrotech-panels/src/interfaces.ts`](../gcs/extensions/urc-astrotech-panels/src/interfaces.ts). Keep
the two in sync manually.

## Drivers and the mock/real switch

`mock_rover_node` instantiates one driver per feature. Env-var gates
swap mock for real:

| Driver | Mock | Real (env var) |
|---|---|---|
| Auger | `MockAugerDriver` (closed-loop sim, 20 Hz) | `RealAugerDriver` — `URC_AUGER_REAL=1`. moteus singleton transport, 50 Hz cycle. |
| Mixing servo | `MockMixingServoDriver` (echo state only) | `RealMixingServoDriver` — `URC_MIXING_SERVO_REAL=1`. Caitlin's lib `FdCanInterface` direct on `/dev/ttyACM*` at 1 Mbit/s, `clear_faults=1` on every goto. |
| Analysis | `MockAnalysisSequencer` (N steps × duration) | Not implemented. |
| Raman | `MockRamanPublisher` (3-Gaussian synth at 1 Hz) | Not implemented. |
| Env | `MockEnvPublisher` (CO₂ baseline + transient spikes) | Not implemented. |

**Bus contention** — the two real drivers cannot coexist on the same
fdcanusb today: `RealAugerDriver` holds the moteus singleton transport
while `RealMixingServoDriver` opens `/dev/ttyACM*` directly via
pyserial-asyncio. Run only one at a time, or wait for the unified
driver-node (post-URC). The user's plan is to physically split onto
two fdcanusbs, which makes this moot. The constraint is documented in
the driver docstrings.

## Auger panel button set (v0.3.0+, refined through 0.3.2)

Replaced the original "Setup position / Up no spin / Down + spin CCW / Spin CW / Spin CCW" set after the 2026-05-07 hardware session.

**Velocity buttons (hold-to-act, sum on overlap):**
- *Up (no spin)* — retract lead screw, no rotation.
- *Down + spin CCW* — drilling.
- *Spin CW* / *Spin CCW* — rotation only.

**Position-memory buttons (one-tap):**
- *Set Up Home* / *Set Bottom Home* — snapshot current
  `lead_screw_position_rev` into in-memory React state.
- *Return to Up Home* / *Return to Bottom Home* — autonomous move
  toward saved target. Two-phase profile: fast 50 rev/s outside the
  ±2 rev slow zone, then proportional creep (Kp=0.45, capped at
  0.5 rev/s) inside it. Velocity-gated debounce (5 ticks of
  |lead_velocity| < 0.45 rev/s) clears the move. Above 0.95 rev
  the streak resets.
- *Stop* — clears all holds + active Return, publishes a zero command.

Returns and velocity holds are mutually exclusive (manual control wins).
Saved homes are in-memory only — the moteus position counter is not
physically anchored (seen +0.265 → −1432.578 across a power cycle), so
persisting "home" across rover lifetimes would be unsafe until the rig
gets limit switches and a homing routine.

CW/CCW sign mapping is still arbitrary — `CW_DIR = +1`, `CCW_DIR = -1`
in [`AugerControl.tsx`](../gcs/extensions/urc-astrotech-panels/src/panels/AugerControl.tsx). Flip on hardware bring-up if labels are inverted.

## Mixing servo panel (v0.3.0+, set 2026-05-08)

Three editable site offsets persisted via Foxglove's `saveState`:

- *Set Home (current = 0°)* — calls `set_home` (`std_srvs/Trigger`),
  zeroing the board at the chamber's current physical position.
- *Big Box → N°*, *Site 1 → N°*, *Site 2 → N°* — call `set_angle`
  with the saved offset. Defaults 110 / 145 / 185 from the bench rig.
- *Manual goto* — number input + button, sends `set_angle(N)`.
- *Edit presets* — three number fields update the saved offsets.

Driver holds the chamber for `move_settle_s` (default 5 s) before
returning the service response, so each button is naturally disabled
during the previous call. Telemetry section shows the last commanded
angle from the state topic.

## Bench-discovered facts (real hardware, do not lose)

These are load-bearing and not derivable from the vendored library:

1. **CAN-FD bus runs at 1 Mbit/s.** Caitlin's library hardcodes
   500 kbit/s in `configure_bus()`; we override with the six bring-up
   commands manually. Both `mixing_servo_min.py` and
   `RealMixingServoDriver` do this.
2. **`clear_faults=1` must be set on every goto frame** (or on a
   session-start clear-faults frame) until a long-lived bus session
   exists. Without it, the board ACKs frames on the bus but ignores
   them mechanically. The library's `ServoController.go_to_position()`
   does **not** set this bit — build the frame via
   `_make_control_frame(control_mode=0, control_data=N, clear_faults=1)`
   and write it directly.
3. **Bench rig addressing.** Mixing chamber is at `(can_id=26,
   servo_id=15)`; auger moteus controllers at id=15 / id=16. CMR
   `servo_id` is a payload slot, not a CAN address — no collision
   with moteus id=15.
4. **fdcanusb path on the dev laptop:**
   `/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00`. The Jetson's
   stable path is still TBD.

The vendored library notes
([`astrotech_canfd_library_notes.md`](astrotech_canfd_library_notes.md))
mirror this in its "Bench-discovered gotchas" section.

## Bench scripts (one-shot probes, kept out of the ROS graph)

- `scripts/auger_slight_move.py` — moteus ping + small move, the cold-start
  probe before launching the ROS driver.
- `scripts/auger_drv8323_status.py` — read the moteus DRV8323 fault register.
- `scripts/mixing_servo_min.py` — single absolute goto, `clear_faults=1`
  default-on. `--also-set-home N` chains a `set_home` first.
- `scripts/mixing_servo_jog.py` — REPL with `--rel`/`--goto`/`--nudge`/
  `--probe` modes; state at `/tmp/mixing_servo_jog.last_deg`.

## Open questions / placeholders

The canonical Q-number map is in
[`phase2a_assumptions.md`](phase2a_assumptions.md). After the
2026-05-08 bench:

| Q | Status |
|---|---|
| Q1 — auger controller | **ANSWERED** (moteus). |
| Q2 — mixing servo controller | **ANSWERED** (CMR servo board on fdcanusb @ can_id=26, servo_id=15, 1 Mbit/s). |
| Q3 — real analysis sequence step set + durations | OPEN — Caitlin. |
| Q4 — `sequence_id` vs. `site_num` semantics | OPEN. |
| Q5 — does a Raman driver already exist? | OPEN. |
| Q6 — same for CO₂ / humidity | OPEN. |
| Q7-Q13 | Deferred / answered — see assumptions doc. |

Questions still pending for project / hardware lead:
- Gear ratios (motor rev → mm of lead-screw travel, drum rev/s after reduction).
- Hard limits on torque / velocity / lead-screw position to enforce in panel + driver.
- Whether the auger moteus controllers will move to a dedicated transport, or share with the mixing servo (currently both on the same fdcanusb — see bus-contention note above).

## Phase 2b targets (loose ends to close)

The remaining work in priority-ish order:

1. **Analysis sequence panel** — wire a real action client for
   `run_sequence_1` / `run_sequence_2`. Render
   `current_step / total_steps / current_step_description / elapsed_seconds`
   feedback. Disable buttons while a sequence is running.
2. **Raman panel** — replace the status line with an actual
   wavenumber-vs-intensity plot. Foxglove's built-in Plot is
   time-series-only; the custom panel exists for this reason.
3. **BDC + heater driver wrapping** — one ROS 2 node owning Caitlin's
   `FdCanInterface`, multiplexing per-feature topics / services / actions.
   Resolves the bus-contention problem with the mixing servo as a side
   effect (or becomes moot once the buses physically split). Outstanding
   questions for Caitlin are in
   [`astrotech_interview_notes.md`](astrotech_interview_notes.md).
4. **Real Raman / Env drivers** — gated on Q5 / Q6.

Sharp edges (lower priority) from the static audit:

- Auger watchdog (200 ms) vs. publish period (100 ms) leaves only
  ~100 ms of slack. Bump `Auger.publishHz` from 10 to 20 in
  `interfaces.ts` if browser jitter ever trips it.
- `MockAnalysisSequencer._goal_callback` accepts then aborts on a
  bad `sequence_id`; cleaner to reject up front.
- Auger panel has no UI feedback when `clientPublish` is missing from
  the bridge capabilities (e.g. recording playback) — buttons render
  and silently do nothing.
- `MultiThreadedExecutor` thread count: on a 1-core CI runner the
  blocking `time.sleep` in the sequencer can starve other drivers.

## How to bring this up (mock side)

```bash
cd ~/terra2-astrotech-gui
conda deactivate                       # see gotchas below
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble
colcon build --symlink-install --packages-select cmr_msgs urc_mock_rover
source install/setup.bash

# Terminal A — mock + bridge in one launch
ros2 launch urc_mock_rover mock.launch.py
# Or, on cmr@jam where Cursor holds 8765 (see gotcha #3):
#   ros2 run urc_mock_rover mock_rover_node
#   ros2 launch launch/gcs_bridge.launch.py port:=8766
```

Foxglove Studio (2.39 tested): connect to `ws://localhost:8765` (or
`:8766` on cmr@jam), import both layouts from `gcs/layouts/`. Image
panels show "no messages on topic" against the mock — that's expected.

## How to bring this up (real hardware, single-actuator)

Auger only:

```bash
export URC_AUGER_REAL=1
ros2 launch urc_mock_rover mock.launch.py
```

Mixing servo only:

```bash
export URC_MIXING_SERVO_REAL=1
ros2 launch urc_mock_rover mock.launch.py
```

Both at the same time on a single fdcanusb is currently unsupported;
see "Bus contention" above. `--symlink-install` is required for the
mixing-servo real driver to locate the vendored `CMR_CANFD` library.

## Bringing up cameras (rover-side)

Camera **orchestration** lives in this repo at
[`src/cmr_cams/`](../src/cmr_cams/): the launch file
([`default.launch.py`](../src/cmr_cams/launch/default.launch.py)), the
per-camera lifecycle TOMLs ([`cam0.toml` … `cam14.toml`](../src/cmr_cams/config/)),
and the [`activate.sh`](../src/cmr_cams/config/activate.sh) helper.
The actual `cmr_cv.camera_node` driver that those tomls reference, and
the stereolabs `zed_wrapper` package the ZED leg uses, are **not** in
this repo — they're preinstalled on the rover Jetson at
`192.168.1.69`. So bring-up is rover-side, even though our orchestration
runs from this repo.

The canonical four-terminal procedure is documented in
[`operator_guide.md`](operator_guide.md). The Astrotech layouts
subscribe to `/cam11/image_raw` (auger), `/cam12/image_raw` (analysis),
and `/zed/zed_node/left/image_rect_color` (ZED) — same topic names the
procedure exposes. The cam11/cam12 ↔ physical-camera mapping is
plug-order-dependent (`/dev/videoN` numbering), so verify with
`v4l2-ctl --list-devices` at bring-up and edit `cameraTopic` from the
Image panel sidebar if a slot shows the wrong camera.

## cmr@jam dev-laptop gotchas

1. **Conda is auto-activated.** `python3` on `PATH` is conda's 3.12,
   incompatible with Humble's `rclpy` (3.10). `conda deactivate` before
   sourcing ROS.
2. **`ROS_DISTRO` isn't set by `/opt/ros/humble/setup.bash`.** Export
   `ROS_DISTRO=humble` after sourcing.
3. **Foxglove bridge port: 8766, not 8765.** Cursor binds
   `127.0.0.1:8765`. Override with
   `ros2 launch launch/gcs_bridge.launch.py port:=8766`.
   `mock.launch.py` doesn't pass through a port arg, so on this laptop
   start the mock node and the bridge separately.
4. **Node is via nvm.** Prepend `~/.nvm/versions/node/v20.20.2/bin` for
   the npm scripts in `gcs/extensions/urc-astrotech-panels/`.
5. **Push from this laptop fails** — no HTTPS credential helper and the
   loaded SSH key isn't authorized. Push from a host with auth or
   install `gh` + `gh auth login`.

## Where the rest of the docs live

| Want to know… | Read |
|---|---|
| How to launch the GUI for an operator (cheat sheet) | [`operator_guide.md`](operator_guide.md) |
| Every Q-number and the current answer | [`phase2a_assumptions.md`](phase2a_assumptions.md) |
| Astrotech subteam Q&A history + pending Slack draft to Caitlin | [`astrotech_interview_notes.md`](astrotech_interview_notes.md) |
| Vendored CAN-FD library API + bench-discovered gotchas | [`astrotech_canfd_library_notes.md`](astrotech_canfd_library_notes.md) |
| Features deferred to after URC | [`post_urc_backlog.md`](post_urc_backlog.md) |
| Auger keyboard test harness (Phase 2b reference) | [`reference/auger_keys_test_harness.py`](reference/auger_keys_test_harness.py) |
