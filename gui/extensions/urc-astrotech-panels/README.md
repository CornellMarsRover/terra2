# URC Astrotech Panels — Foxglove Extension

Custom Foxglove Studio panels for the URC 2026 Astrotech GCS. Five panel
types are registered:

- `urc.auger_control` — **fully wired** (hardware-tested 2026-05-07).
  Subscribes to `/astrotech/auger/state`, advertises and publishes
  `/astrotech/auger/cmd_vel` (`cmr_msgs/AugerCommand`) at 10 Hz while a
  button is held, sends a single zero command on release. Hold-to-act
  velocity buttons (**Up (no spin)** / **Down + spin CCW** / **Spin CW** /
  **Spin CCW**, summed on overlap) plus one-tap position-memory buttons
  (**Set Up/Bottom Home**, **Return to Up/Bottom Home** with a tuned
  two-phase settle profile, **Stop**). Live telemetry table for both
  lead-screw and auger motors (position / velocity / torque / temperature
  / mode / fault).
- `urc.mixing_servo` — **fully wired** (hardware-tested 2026-05-08).
  Calls `/astrotech/mixing_servo/set_home` (`std_srvs/Trigger`) and
  `/astrotech/mixing_servo/set_angle` (`cmr_msgs/SetMixingServoAngle`).
  **Set Home** + three editable site presets (Big Box / Site 1 / Site 2,
  persisted via `saveState`) + manual goto + last-commanded-angle readout
  from `/astrotech/mixing_servo/state`. Buttons disable during the
  driver's move-settle window.
- `urc.raman_spectrum` — **wired**. Subscribes to
  `/astrotech/raman/spectrum` and draws a live wavenumber-vs-intensity plot
  on a canvas (Foxglove's built-in Plot is time-series-only, so the spectrum
  needs a custom panel). Falls back to a pixel-index x-axis until the sensor
  calibration is provided.
- `urc.snapshots` — **wired**. Save Raman / Save CO₂ buttons that call
  `/astrotech/snapshot/{raman,co2}` (`std_srvs/Trigger`). The rover-side
  snapshot driver renders the PNG and saves it (panels can't write files);
  the panel shows the saved path. Pairs with the built-in CO₂ Plot panel.
- `urc.analysis_sequence` — stub. Buttons log the action server
  name that *would* be called. Phase 2b wires action clients with feedback
  display.

The auger and mixing-servo panels are the reference implementations for
the wiring pattern (subscribe / advertise / hold-to-act publish or
service call / live telemetry).

## Foxglove panel-type IDs

For locally-installed extensions (`npm run local-install`), Foxglove derives
the extension id from **`package.json` `publisher` and `name`**: lowercased,
publisher stripped of non-alphanumeric characters, then joined with a dot (same
as the `foxglove-extension` toolchain’s `getPackageId()`).

With `publisher: "cornell-mars-rover"` and `name: "urc-astrotech-panels"` the
prefix is `cornellmarsrover.urc-astrotech-panels` and the four panel types are:

- `cornellmarsrover.urc-astrotech-panels.urc.auger_control`
- `cornellmarsrover.urc-astrotech-panels.urc.analysis_sequence`
- `cornellmarsrover.urc-astrotech-panels.urc.mixing_servo`
- `cornellmarsrover.urc-astrotech-panels.urc.raman_spectrum`

`displayName` is for the Extensions UI only — **do not** use it in layout
JSON. The rule above is duplicated as a comment at the top of `src/index.ts`.

If you change `publisher` or `name` in `package.json`, re-run `local-install`
and update every layout that references the old prefix.

## Topic / service names

All topic, service, and action names are centralized in
[`src/interfaces.ts`](src/interfaces.ts). That file is a hand-written
mirror of
[`src/astrotech_rover/config/astrotech_interfaces.yaml`](../../../src/astrotech_rover/config/astrotech_interfaces.yaml),
which is the real source of truth. Keep them in sync manually.

## Develop

```bash
cd gui/extensions/urc-astrotech-panels
npm install
npm run build
npm run local-install
```

`local-install` copies the built extension into Foxglove Studio's user
extensions directory (`~/.foxglove-studio/extensions/` on Linux /
`~/Library/Application Support/Foxglove Studio/extensions/` on macOS).

## Reload after changes

Foxglove Studio **does not hot-reload extensions.** After `npm run build &&
npm run local-install`:

1. Fully quit Foxglove Studio (Cmd/Ctrl+Q — not just close the window).
2. Relaunch.
3. New panel types appear in *Add Panel → Custom*.

If your panels don't show up, check:

- `npm run build` produced `dist/extension.js` without TypeScript errors.
- Foxglove's log (Help → About → "Show app data") for load errors.

## Lint / typecheck

```bash
npm run lint
```

Runs `tsc --noEmit`.

## Packaging for distribution

```bash
npm run package
```

Produces a `.foxe` file you can hand out or attach to a GitHub release.
Other developers install with Foxglove's *Settings → Extensions → Install
from file*.

## Phase 2b work remaining

- **Analysis sequence panel** — real action client for `run_sequence_N`
  with feedback display (`current_step / total_steps /
  current_step_description / elapsed_seconds`) and run-state button
  disabling. Still the only stub panel.
- **Raman plot** — actual wavenumber-vs-intensity plot (Foxglove Plot is
  time-series-only, hence the custom panel). The real driver
  (`raman_real.py`) now feeds the topic; the panel just needs the plot.
- Auger panel hardening: kill-switch UI; visual fault / over-temp
  indication; guard against the wrong data source (no UI feedback today
  if `clientPublish` was stripped from the bridge `capabilities`).
- Auger panel CW/CCW sign mapping is currently arbitrary
  (CW = +`augerSpinForwardRevS`). Confirm against hardware on bring-up and
  flip the `CW_DIR` / `CCW_DIR` constants in
  [`src/panels/AugerControl.tsx`](src/panels/AugerControl.tsx) if needed.
- Camera feeds are wired in via Foxglove's built-in **Image** panel
  (referenced from the layout JSONs at `gui/layouts/`); the camera
  publishers themselves live in `src/cmr_cams/` (the rover-side ROS
  pipeline).
