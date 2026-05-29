# Analysis reference — fluidic protocol (from the Astrotech lead)

Reference scripts for the science-analysis sequence, handed over by the
Astrotech lead. **Bench WIP** — the final competition version isn't out yet;
this is the most recent code as of 2026-05-27.

These are **not** run by `astrotech_node` and are **not** ROS code. They're the
source-of-truth *protocol* a future analysis driver should implement. Kept here
as reference (like `scripts/auger_keys_test_harness.py` is for the auger).

## Files
- `testreal.py`, `testreal-1.py` — near-identical standalone `asyncio` scripts
  that drive the pumps / servos / heaters directly over the fdcanusb using the
  vendored `CMR_CANFD` library (the same stack as the mixing servo). Hardcoded
  dev ports (`/dev/cu.usbmodem…`); most sequence steps are commented out (bench
  iteration), with one water-system snippet currently active.
- `temperature_sensor_mars_rover.ino` — Arduino thermistor sketch
  (Steinhart-Hart) printing temperature over serial; the heater/temperature
  monitor, separate from the fluidics.

## Hardware CAN map (read off the scripts)
| Board (`can_id`) | Devices |
|---|---|
| Servo board `1` | `servo_id` 0–15 — cuvette / sample-positioning servos (homes e.g. 40/60/80/100°). |
| BDC board `18` ("DMSO, double green") | `dmsoS1`, `dmsoS2` (syringes), `ninview`, `dmsoWaste`, `heattry1`, `heattry2`. |
| BDC board `20` ("H2O, one blue") | `h2oS1`, `h2oS2`, `h2obayers`, `bayview`, `mbview`, `h2owaste`. |
| BDC board `21` ("optics, double blue") | `raman`, `led`, `heat1`, `heat2`. |

BDC = brushed-DC pump motors driven with `move_motor_forward/reverse(n)`;
servos use `go_to_position(deg)` / `set_home(deg)`.

## The two assays
Each is a timed sequence of pump moves with `asyncio.sleep` waits (including
~20 s color-development holds), run for site 1 then site 2:

1. **Ninhydrin (NIN) assay** — DMSO system (board 18): prime, draw DMSO into the
   cuvette, add ninhydrin (`ninview`), develop ~20 s, flush; repeat for site 2.
2. **Water-system assay** — H2O system (board 20) + optics board 21: prime, draw
   water + reagents past `bayview` / `mbview` and the Raman pump, develop, flush;
   repeat for site 2.

(Reagent/chemistry names are inferred from the variable names — confirm the
exact protocol, volumes, and wait times with the lead.)

## To wire this into the GUI (bigger than the current mock)
1. A **pumps + heater + analysis-servo ROS driver** wrapping `CMR_CANFD` — these
   devices aren't in `astrotech_node` yet (see the main README "Still to do").
   Mind the fdcanusb contention: this shares the bus with the auger / mixing
   servo (EBUSY if both open it).
2. A **sequencer** that runs these protocols as discrete, **cancellable** steps
   (operators need an abort during a 20 s hold).
3. The **Foxglove-native interface** ("Option B" in the main README: a start
   service + a status topic + a cancel service) so the panel can drive it —
   Foxglove panels have no ROS action client.
