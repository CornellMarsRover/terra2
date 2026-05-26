# Changelog

## 0.4.0

Raman plot + plot snapshots.

- **Raman panel** now draws a real **wavenumber-vs-intensity plot** on a
  canvas (replacing the status-line stub). Auto-scales, labels the axes, and
  falls back to a pixel-index x-axis when the spectrum isn't calibrated yet.
- **New `urc.snapshots` panel** — *Save Raman* / *Save CO₂* buttons that call
  `/astrotech/snapshot/{raman,co2}` (`std_srvs/Trigger`). A Foxglove panel
  can't write files, so the rover-side snapshot driver renders the PNG and
  saves it to `~/Desktop/astrotech_snapshots/` (fallback `<repo>/snapshots/`);
  the panel shows the saved path. Added to the analysis layout alongside the
  built-in CO₂ Plot.
- No change to the auger or mixing-servo panels.

## 0.3.2

Further fixes **Return-to-home hunting** when driving to a saved "up home"
near the mechanical top:

- **Proportional slow approach** inside the 2 rev slow zone (replaces fixed
  0.5 rev/s creep) so commanded speed drops continuously as error shrinks.
- **Zero commanded creep** while |error| is inside the snap tolerance — the
  motor coasts while a **debounce streak** runs, instead of alternating
  ±creep every time position jitters across the threshold.
- **Velocity-gated debounce**: streak increments only when reported
  `lead_screw_velocity_rev_s` is below ~0.45 rev/s, so a screw bouncing on
  a hard stop (or still moving after creep) does not count as "arrived."
- **Wide reset**: if |error| exceeds ~0.95 rev, the debounce streak resets
  (operator must re-approach).

## 0.3.1

Return-to-home used full lead-screw speed (50 rev/s) until within 0.2 rev of
the saved target, which caused **bang-bang oscillation**: overshoot, reverse
at 50 rev/s, overshoot the other way, repeat ("weird back and forth"). The
Return move is now **two-phase**: outside `±2 rev` of the target it still uses
50 rev/s; inside that "slow zone" it creeps at **0.5 rev/s** so the screw can
settle into the tolerance band without hunting.

## 0.3.0

Auger panel rearrangement after the 2026-05-07 hardware bring-up
session: the previous "Setup position" hold-to-retract button was
labeled like a destination but acted like a direction key, which the
operator found confusing. Replaced with explicit position-memory
buttons.

- **New buttons** — *Set Up Home*, *Set Bottom Home*,
  *Return to Up Home*, *Return to Bottom Home*, and an explicit
  *Stop*. Set buttons one-tap-snapshot the current
  `lead_screw_position_rev` from the latest `AugerState` into in-memory
  React state. Return buttons one-tap-start an autonomous move toward
  the saved target at 50 rev/s; the panel detects "within 0.2 rev of
  target" and clears the move with an explicit zero command. Stop
  clears every held / active state and publishes ZERO_CMD.
- **Removed** — the *Setup position* button. *Up (no spin)* now spans
  full width in its group.
- **In-memory only** — saved home positions are reset on panel reload
  or rover restart. The moteus position counter is not physically
  anchored (tonight's session saw it drift +0.265 → -1432.578 rev
  across a power cycle), so persisting "home" across rover lifetimes
  would be unsafe until the rig has limit switches and a homing
  routine.
- **Mutual exclusion** — Returns and velocity holds are exclusive:
  pressing any velocity button cancels an active Return. Returns are
  also exclusive with each other (only one Return can be in progress).
- **Disabled state** — the *Return to ... Home* button is greyed out
  and unclickable until the corresponding *Set ... Home* has been
  pressed at least once, and also when no `AugerState` telemetry has
  been received yet.
- **No driver / message changes** — `AugerCommand`, `AugerState`,
  `RealAugerDriver`, and the `interfaces.ts` topic / schema names are
  all unchanged. The new behavior is implemented entirely on the panel
  side as velocity commands; the autonomous-move semantics live in the
  10 Hz publish loop and are gated by the same 200 ms watchdog as the
  hold-to-act buttons.

## 0.2.1

Fixes **Unknown panel type** when importing layouts after `local-install`:
Foxglove keys locally-installed extensions by normalized
`publisher` + `name` from `package.json` (e.g.
`cornellmarsrover.urc-astrotech-panels`), **not** by `displayName`.
`urc_astrotech_auger.json` and `urc_astrotech_analysis.json` are updated
accordingly. See `src/index.ts` for the derivation rule.

## 0.2.0

Operator workflow rearrangement requested by the Astrotech subteam on
2026-04-30. Splits the single combined dashboard into two task-specific
layouts and reshuffles the auger panel buttons.

- **Auger panel buttons** — rebuilt around five Astrotech-named actions
  grouped by intent: **Setup position** / **Up (no spin)** (Position),
  **Down + spin CCW** (Drilling), **Spin CW** / **Spin CCW** (Rotation
  only). Hold-to-act loop, watchdog cadence, telemetry table, and
  `cmr_msgs/AugerCommand` shape are unchanged. Multiple holds compose by
  summing per-axis velocities (e.g. holding `Down + spin CCW` and tapping
  `Spin CW` cancels rotation but keeps drive-down going). `Setup position`
  is a hold-to-retract today; Phase 2b will turn it into a one-tap
  closed-loop go-to-home once the auger driver supports a position
  setpoint and the team has live-tested the home pose.
- **Layouts** — replaced `urc_astrotech_dashboard.json` with
  `urc_astrotech_auger.json` (auger control + mixing servo + auger cam
  Image + ZED left-rect Image) and `urc_astrotech_analysis.json`
  (analysis sequence + Raman + analysis cam Image + CO₂ Plot). Camera
  topics are placeholders (`/cam11`, `/cam12`,
  `/zed/zed_node/left/image_rect_color`) — edit per actual ID mapping at
  bring-up.

## 0.1.0

Initial release of the four URC 2026 Astrotech custom panels:
`urc.auger_control`, `urc.analysis_sequence`, `urc.mixing_servo`,
`urc.raman_spectrum`. State of each panel as shipped:

- **Auger control** — fully wired. Subscribes to
  `/astrotech/auger/state`, advertises and publishes
  `/astrotech/auger/cmd_vel` (`cmr_msgs/AugerCommand`) as a 10 Hz
  hold-to-act loop with release-edge zero command, plus a live
  telemetry table. Defaults match the Astrotech keyboard test harness
  (lead screw ±100 rev/s @ 2.0 N·m, auger +10 fwd / −50 rev @ 1.0 N·m).
- **Raman spectrum** — partial. Subscribes to the spectrum topic and
  renders a "last spectrum: N points received at T" status line.
  Wavenumber-vs-intensity plot lands in Phase 2b.
- **Analysis sequence**, **Mixing servo** — Phase 2a stubs. Buttons log
  to the browser console which service / action would be called; real
  clients land in Phase 2b.
