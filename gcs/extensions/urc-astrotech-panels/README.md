# URC Astrotech Panels — Foxglove Extension

Phase 2a stubs for the URC 2026 Astrotech GCS. Four custom Foxglove Studio
panels are registered:

- `urc.auger_control` — buttons for auger up / down / spin fwd / back / stop.
- `urc.analysis_sequence` — buttons to start Sequence 1 / Sequence 2.
- `urc.mixing_servo` — one button per preset from
  `astrotech_interfaces.yaml`.
- `urc.raman_spectrum` — subscribes to the Raman topic, shows frame count.

**Phase 2a is stubs only.** Clicks log the service/topic that *would* be
called to the browser console. Phase 2b wires up real publishes, service
calls, and action clients.

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

- Real publishes on `auger/cmd` (with throttle + kill-switch logic).
- Real action client for `run_sequence_N` with progress display.
- Real service client for `mixing_servo/set_preset` with busy state.
- Actual Plot (wavenumbers vs. intensities) for Raman.
- (Camera feeds: handled on a separate branch; not owned by this extension.)
