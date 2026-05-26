# NOTICE

The code under this directory is **not authored by the GCS / autonomy
team.** It belongs to the Cornell Mars Rover Astrotech subteam and was
written by team member **Caitlin Lee-Ying Rapalski** (and possibly other
Astrotech contributors).

It was sourced from the team's Confluence space and vendored into this
repository on **2026-04-26** for two purposes:

1. **Reference** — so the GCS / mock-rover authors can read the actual
   command primitives the Astrotech boards understand without round-
   tripping through Confluence each time.
2. **Wrapping** — Phase 2b will write a thin ROS 2 driver node that
   imports `CMR_CANFD` (this library) and exposes its primitives over
   ROS topics / services / actions.

## Rules for this directory

- **Do not edit files in place.** If a fix is needed, raise it with
  Caitlin / the Astrotech subteam so the upstream Confluence copy is
  updated and re-vendored.
- **Do not import these files directly from `src/`.** Phase 2b will copy
  the relevant subset into a proper ROS 2 package (or pip-install the
  library if Astrotech publishes it). Treat this directory as
  documentation that happens to be executable.
- **Do not commit modifications to vendored files** without first
  changing the source on Confluence and re-extracting; otherwise the two
  copies drift silently.

## Provenance

Two zip archives were extracted into this folder:

- `servoboard_and_bdc_test_python_code/` — from
  `servoboard_and_bdc_test_python_code(1).zip`. Earlier snapshot.
  Second-resolution motor durations only (`move_motor_forward(seconds)`).
- `servo_bdc_control_ms/` — from `servo_bdc_control_ms.zip`. Newer
  snapshot. Adds **millisecond**-resolution motor commands
  (`move_motor_forward_ms(milliseconds)`, etc.) on top of the original
  second-resolution API.

`servo_bdc_control_ms/` is the version we intend to wrap in Phase 2b.

## What was stripped before commit

Each zip originally contained a Windows-style Python virtual environment
(`venv/Lib/site-packages/...`) and `__pycache__/` directories totaling
~14 MB of pip-reinstallable dependencies plus compiled bytecode. Those
were removed before commit. Only the actual source files (`.py`) under
`test/` remain.
