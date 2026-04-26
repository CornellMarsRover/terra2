# System Overview

This is the entry-point doc. Read it first; the others are reference
material it points you to.

## What this is

The University Rover Challenge 2026 ground control station for the
Cornell Mars Rover **Astrotech** science payload (auger, mixing
chamber, BDC pumps, Raman spectrometer, CO₂ + humidity + temperature
sensors, heater). The GCS is **Foxglove Studio** plus four custom
panels; the rover side is a ROS 2 Humble graph the GCS connects to
via `foxglove_bridge` over WebSocket.

## What's built

- A **mock rover** (`src/urc_mock_rover/`) that publishes every
  Astrotech topic / advertises every service / runs every action the
  GCS expects to find. Lets all GCS development happen with no
  hardware.
- A single **Foxglove bridge** launch
  (`launch/gcs_bridge.launch.py`) with QoS overrides tuned for an RF
  link.
- A **Foxglove layout** (`gcs/layouts/urc_astrotech_dashboard.json`)
  showing CO₂/humidity, the auger, the analysis sequence, the mixing
  servo, and the Raman spectrum panel.
- A **Foxglove extension** (`gcs/extensions/urc-astrotech-panels/`)
  registering four custom panel types as **stubs** — clicks log to
  the browser console; Phase 2b lights them up.
- New custom interfaces in `cmr_msgs`: `RamanSpectrum`, `EnvSample`,
  `AugerState` (commanded-only), `SetMixingServoPreset`,
  `RunAnalysisSequence`.

## What's not built

- **Camera feeds** — being built on a separate branch, will merge in
  later.
- **Pause/resume on analysis sequences** — deferred to post-URC. Today
  is run-to-completion with hard abort. See `post_urc_backlog.md`.
- **Real driver wrapper for Caitlin's CAN-FD library** — Phase 2b.
  Pending answers to a few questions in `astrotech_interview_notes.md`
  (chunking long durations, board feedback, board addressing).

## Data flow

```
operator clicks panel button
       │
       ▼
Foxglove Studio  ──── ws://<host>:8765 ────►  foxglove_bridge
                                                    │
                                                    ▼
                                          ROS 2 graph (Humble)
                                                    │
                                                    ▼
                                       mock_rover_node coordinator
                                                    │
                              ┌───────────┬──────────┼──────────┬───────────┐
                              ▼           ▼          ▼          ▼           ▼
                            auger      mixing     analysis    raman        env
                            driver     servo      sequencer   pub          pub
                                       driver
```

Phase 2b replaces the mock drivers with a real `cmr_canfd_driver`
node that wraps `third_party/astrotech_canfd/`. The flow above and
all topic / service / action names stay the same.

## Where to look

| If you want to know… | Read |
|---|---|
| What was built and why | this file, then `phase_2a_overview.md` |
| The exact list of topics / services / actions | `src/urc_mock_rover/config/astrotech_interfaces.yaml` |
| Which assumptions are still placeholders | `phase2a_assumptions.md` |
| What the Astrotech team has confirmed | `astrotech_interview_notes.md` (start with "Current state" at top) |
| The vendored CAN-FD library API | `astrotech_canfd_library_notes.md` |
| Features deferred to after URC | `post_urc_backlog.md` |
| How to bring up the mock rover locally | `../README.md` (top-level) |
| Phase 1 GCS wiring (bridge, bandwidth, etc.) | `ros_interface_inventory.md`, `bandwidth_audit.md`, `feature_to_topic_map.md`, `foxglove_extension_plan.md` |
| Open questions still tagged in code | grep `TODO(astrotech-q-` in `src/`, `gcs/`, `docs/` |
