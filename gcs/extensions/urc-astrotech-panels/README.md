# URC Astrotech Panels — Foxglove Extension

Custom Foxglove Studio panels for the URC 2026 Astrotech GCS. Four panel
types are registered:

- `urc.auger_control` — **fully wired** hold-to-act controller. Subscribes
  to `/astrotech/auger/state`, advertises and publishes
  `/astrotech/auger/cmd_vel` (`cmr_msgs/AugerCommand`) at 10 Hz while a
  button is held, sends a single zero command on release. Five
  Astrotech-named action buttons grouped by intent: **Setup position** /
  **Up (no spin)** (Position group), **Down + spin CCW** (Drilling),
  **Spin CW** / **Spin CCW** (Rotation only). Multiple holds compose by
  summing velocities. Live telemetry table for both lead-screw and auger
  motors (position / velocity / torque / temperature / mode / fault).
- `urc.raman_spectrum` — partially wired. Subscribes to
  `/astrotech/raman/spectrum` and renders a "last spectrum: N points
  received at T" status line. Phase 2b adds a real wavenumber-vs-intensity
  plot (Foxglove's built-in Plot is time-series-only).
- `urc.analysis_sequence` — Phase 2a stub. Buttons log the action server
  name that *would* be called. Phase 2b wires action clients with feedback
  display.
- `urc.mixing_servo` — Phase 2a stub. One button per preset; clicks log
  the service name + preset. Phase 2b wires the real service call with
  busy-state UI.

The auger panel is the reference implementation for the Phase 2b pattern
(subscribe / advertise / hold-to-act publish / live telemetry).

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
[`src/urc_mock_rover/config/astrotech_interfaces.yaml`](../../../src/urc_mock_rover/config/astrotech_interfaces.yaml),
which is the real source of truth. Keep them in sync manually.

## Develop

```bash
cd gcs/extensions/urc-astrotech-panels
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

- Auger panel: closed-loop **Setup position** (one-tap go-to-home) once the
  auger driver exposes a position setpoint; live testing required to nail
  down the home position. Kill-switch UI; visual indication of fault /
  over-temp; guard against staring at the wrong data source (no UI feedback
  today if `clientPublish` was stripped from the bridge `capabilities`).
- Auger panel CW/CCW sign mapping is currently arbitrary
  (CW = +`augerSpinForwardRevS`). Confirm against hardware on bring-up and
  flip the `CW_DIR` / `CCW_DIR` constants in
  [`src/panels/AugerControl.tsx`](src/panels/AugerControl.tsx) if needed.
- Real action client for `run_sequence_N` with feedback display
  (`current_step / total_steps / current_step_description /
  elapsed_seconds`).
- Real service client for `mixing_servo/set_preset` with busy state.
- Actual wavenumber-vs-intensity plot for Raman (Foxglove Plot is
  time-series-only, hence the custom panel).
- Camera feeds are wired in via Foxglove's built-in **Image** panel
  (referenced from the layout JSONs at `gcs/layouts/`); the camera
  publishers themselves live in `src/cmr_cams/` (the rover-side ROS
  pipeline).
