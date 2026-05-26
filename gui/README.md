# Astrotech GUI

This is the ground control station for the Cornell Mars Rover **Astrotech**
science payload (URC 2026): the auger drill, the mixing chamber, the Raman
spectrometer, the CO₂ / humidity / temperature sensor, and the camera feeds.

The GUI is **Foxglove Studio plus a custom 5-panel extension**. The rover
side is a ROS 2 (Humble) program that Foxglove talks to over
`foxglove_bridge`.

If you just want to run it during a mission, read
[`operator_guide.md`](operator_guide.md) instead — that's the short
step-by-step. This file is the fuller picture: what everything is, how to
run it, and (at the bottom) the technical reference for anyone building on
or fixing the GUI.

---

## Running it

**Easiest — the Astrotech Hub.** A one-window launcher with a button for every
step (build, launch the node + bridge, cameras/ZED, diagnostics, Raman
calibration, shut down):

```bash
python3 src/astrotech_rover/scripts/astrotech_hub.py
```

In a mission the payload node runs on the **Jetson** (where the hardware is);
the hub's **Jetson** tab brings it up over SSH, and you operate from Foxglove on
the base-station laptop. The [operator guide](operator_guide.md) has the full
walkthrough plus the one-time SSH setup.

**Manual — what the hub runs under the hood.** One-time build (on the machine
that runs the node):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cmr_msgs astrotech_rover
source install/setup.bash
```

Start the node + Foxglove bridge:

```bash
ros2 launch astrotech_rover astrotech.launch.py
```

That serves Foxglove at `ws://localhost:8765` (or `ws://<jetson-ip>:8765` when
it runs on the rover). To run with no hardware, prefix the launch with the mock
flags: `URC_AUGER_MOCK=1 URC_MIXING_SERVO_MOCK=1 URC_RAMAN_MOCK=1 URC_ENV_MOCK=1`.

## Connecting Foxglove

1. Open Foxglove Studio.
2. **Open Connection** → `ws://<jetson-ip>:8765` (e.g.
   `ws://192.168.1.69:8765`), or `ws://localhost:8765` if the node runs on this
   machine.
3. **Layouts → Import from file** → pick `urc_astrotech_auger.json`
   (drilling) or `urc_astrotech_analysis.json` (lab work).

First time on a machine, install the panels (then fully quit + relaunch
Foxglove) — or use the hub's "Build + install Foxglove panels" button:

```bash
cd gui/extensions/urc-astrotech-panels
npm install && npm run build && npm run local-install
```

The full operator walkthrough — what every button does, plus the
rover-side camera bring-up — is in [`operator_guide.md`](operator_guide.md).

## What's in here

| Piece | Where | What it is |
|---|---|---|
| Foxglove panels | [`gui/extensions/urc-astrotech-panels/`](extensions/urc-astrotech-panels/) | The custom panels: auger, mixing servo, Raman, snapshots, analysis. |
| Layouts | [`gui/layouts/`](layouts/) | `urc_astrotech_auger.json`, `urc_astrotech_analysis.json`. |
| Rover node | [`src/astrotech_rover/`](../src/astrotech_rover/) | `astrotech_node` — one driver per piece of hardware. |
| Messages | [`src/cmr_msgs/`](../src/cmr_msgs/) | `AugerCommand`/`AugerState`, `RamanSpectrum`, `EnvSample`, `SetMixingServoAngle`, `RunAnalysisSequence`. |
| Bridge | [`launch/gcs_bridge.launch.py`](../launch/gcs_bridge.launch.py) | The Foxglove WebSocket bridge. |
| Cameras | [`src/cmr_cams/`](../src/cmr_cams/) | Launch + config for the rover-side camera drivers. |

## The five panels

| Panel | State |
|---|---|
| **Auger control** | Working, tested on hardware 2026-05-07. Hold-to-drive buttons + saved home positions + return-to-home. |
| **Mixing servo** | Working, tested on hardware 2026-05-08. Set Home + 3 editable site presets + manual go-to. |
| **Raman spectrum** | Working — live wavenumber-vs-intensity plot of the latest spectrum. |
| **Analysis sequence** | Not done yet — the buttons are placeholders. |
| **Snapshots** | Working — Save Raman / Save CO₂ buttons that write a PNG of the current plot to the laptop Desktop (fallback: `<repo>/snapshots/`). |

The extension is version **0.4.0**.

## Switching which camera a panel shows

Each camera tile is a built-in Foxglove **Image** panel, so you can point
it at any camera that's currently publishing:

- **In Foxglove:** click the Image panel, open its **settings** (the gear
  / settings sidebar), and change the **topic** to a different feed. The
  USB cameras publish as `/cam0/image_raw` … `/cam14/image_raw` (whichever
  ones are activated). The ZED's left image is on `/zed/image_left`
  (published by the `cmr_zed` node, e.g. `ros2 run cmr_zed
  zed_publisher_node`); if you run the stereolabs `zed_wrapper` instead, its
  left image is `/zed/zed_node/left/image_rect_color`.
- **In the layout file:** the topic each panel starts on is the
  `cameraTopic` field in the layout JSON ([`gui/layouts/`](layouts/)). The
  auger layout defaults to `/cam11/image_raw` (auger cam) and the ZED left
  image (`/zed/image_left`); the analysis layout defaults to
  `/cam12/image_raw`. Edit those if you want a different default.

Heads up: which physical camera is `/cam11` vs `/cam12` can change between
boots (it depends on USB plug order), so if a tile shows the wrong camera,
just switch its topic in the panel settings.

---

# Reference

Everything below is for whoever is building on or fixing the GUI. You don't
need any of it just to drive the rover — that's all in the operator guide.

## ROS topics & services

The one place that defines all the names is
[`src/astrotech_rover/config/astrotech_interfaces.yaml`](../src/astrotech_rover/config/astrotech_interfaces.yaml).
There's a matching copy on the TypeScript side in
[`extensions/urc-astrotech-panels/src/interfaces.ts`](extensions/urc-astrotech-panels/src/interfaces.ts)
— if you change one, change the other to match.

```
/astrotech/auger/cmd_vel            cmr_msgs/AugerCommand        (10 Hz while a button is held, 200 ms watchdog)
/astrotech/auger/state              cmr_msgs/AugerState          (50 Hz telemetry from the real driver)
/astrotech/mixing_servo/set_angle   cmr_msgs/SetMixingServoAngle (degrees)
/astrotech/mixing_servo/set_home    std_srvs/Trigger
/astrotech/mixing_servo/state       std_msgs/Int32               (last commanded angle, 5 Hz)
/astrotech/analysis/run_sequence_1  cmr_msgs/RunAnalysisSequence (action)
/astrotech/analysis/run_sequence_2  cmr_msgs/RunAnalysisSequence (action)
/astrotech/raman/spectrum           cmr_msgs/RamanSpectrum       (1 Hz)
/astrotech/env/sample               cmr_msgs/EnvSample           (about every 2 s)
/astrotech/snapshot/raman           std_srvs/Trigger             (save the latest Raman plot to a PNG)
/astrotech/snapshot/co2             std_srvs/Trigger             (save the recent CO₂ plot to a PNG)
```

The two `/astrotech/analysis/run_sequence_*` actions exist on the rover, but
the Foxglove panel API has **no ROS action client**, so the analysis panel
doesn't drive them yet — see "Still to do".

## What each driver talks to

`astrotech_node` runs one driver per piece of hardware:

| Driver | Talks to |
|---|---|
| `auger_real.py` | Two moteus controllers (id=15 lead screw, id=16 auger) over the USB-CAN dongle, at 50 Hz. Has built-in speed/torque caps so a button press can't over-drive the rig (lead ±50 rev/s, auger ±6 rev/s, 3 Nm). |
| `mixing_servo_real.py` | The CMR servo board over the same dongle, at 1 Mbit/s. |
| `raman_real.py` | The TCD1340 spectrometer over a USB serial port. |
| `env_real.py` | The Adafruit SCD-30 sensor over I²C (through an FT232H USB→I²C bridge by default). |

If a driver's hardware or its Python libraries aren't there, it just logs
an error and sits quietly — it won't crash the rest of the node, so the
other panels keep working.

(There's no real analysis-sequence driver yet, so that panel's backend is
still a placeholder.)

(`snapshot.py` is an always-on, non-hardware driver: it buffers the latest
Raman spectrum and recent CO₂ samples and, on a `/astrotech/snapshot/{raman,co2}`
service call, renders a PNG with matplotlib to the operator laptop's Desktop
— fallback `<repo>/snapshots/`. A missing matplotlib disables only snapshots,
not the rest of the node.)

## Sharing the CAN dongle (auger vs mixing servo)

The auger and the mixing servo are both on the **same** USB-CAN dongle, and
only one program can hold that dongle at a time — if both try, the second
one fails to open it. So today you run one of them at a time. The plan is
to give the mixing servo its own second dongle, after which both can run
together; that's set by the `real_port` value in the config file. (The
operator guide shows the exact command for running just one.)

## How the auger panel buttons work

- **Hold-to-drive buttons** (Up, Down + spin CCW, Spin CW, Spin CCW): while
  you hold one, the panel sends a velocity command 10 times a second; when
  you let go it sends a stop. Holding more than one adds their motions
  together.
- **Set Up Home / Set Bottom Home**: saves the auger's current height so you
  can come back to it. These are remembered only while the panel is open —
  they reset if the rover restarts. (The motor's position counter isn't tied
  to a physical zero yet — it drifted across a power cycle in testing — so
  saving a "home" across restarts wouldn't be safe until the rig has limit
  switches and a homing routine.)
- **Return to Up/Bottom Home**: drives back to a saved height on its own —
  fast until it's close, then slow so it settles without bouncing. Disabled
  until you've saved that home.
- **Stop**: cancels everything.
- Returns and the hold buttons cancel each other — whichever you press last
  wins.
- The CW vs CCW labels were picked arbitrarily (`CW_DIR` in
  `AugerControl.tsx`); flip them if the hardware turns the other way.

## How the mixing servo panel works

- **Set Home**: press once after you've put the chamber at its reference
  spot — that position becomes 0°. Do this first.
- **Big Box / Site 1 / Site 2**: drive to a saved angle (defaults
  110 / 145 / 185°). The button greys out for about 5 seconds while the
  chamber moves, then re-enables.
- **Manual go-to**: type any angle and drive there.
- **Edit presets**: change the saved Big Box / Site angles; the panel
  remembers them.

## Hardware facts worth remembering

These were learned on the bench and aren't obvious from the code — keep them
around:

1. **The CAN bus runs at 1 Mbit/s.** The vendored CAN library defaults to
   500 kbit/s, which is wrong for our rig — at that speed nothing moves. We
   set 1 Mbit/s ourselves.
2. **Every servo move needs the "clear faults" bit set.** The library's
   `go_to_position()` doesn't set it, and without it the board says OK but
   doesn't actually move once anything else has used the bus. Our driver
   builds the command frame with that bit on.
3. **Addresses:** mixing chamber is `can_id=26, servo_id=15`. Auger moteus
   controllers are id=15 (lead screw) and id=16 (auger). The servo's "15"
   and the moteus "15" don't clash — they mean different things.
4. **The dongle** shows up on the dev laptop as
   `/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00`.

The full notes on the vendored CAN library are in
[`src/astrotech_rover/third_party/astrotech_canfd/API_NOTES.md`](../src/astrotech_rover/third_party/astrotech_canfd/API_NOTES.md).

## Still to do

- **Analysis sequence panel** — still a placeholder: the Start buttons only
  log to the browser console. The rover already runs the
  `RunAnalysisSequence` action servers, but the Foxglove panel API has **no
  ROS action client**, so the panel can't drive them directly — wiring this
  up will need a Foxglove-native interface on the driver (e.g. a start
  service + a status topic + a cancel service). The real sequence design /
  step list from the project lead is still open, so the panel is left
  intentionally isolated until then.
- **Raman plot** — the plot is built; what's left is the real sensor: confirm
  how the spectrometer streams its data, set `raman.n_points` to 3648, and add
  the pixel→wavenumber calibration once we have it. (Details are in
  `raman_real.py`.)
- **Env sensor** — confirm how the SCD-30 is wired and set
  `env.real_connection` to match. (Options are documented in `env_real.py`
  and the config file.)
- **Pumps + heater** — these still need a ROS driver wrapping the vendored
  CAN library. See the API notes linked above.
- **Bench-test the Raman and env drivers** — they're written but haven't
  been run against the real sensors yet.
- **Camera bandwidth** — the raw camera feeds are big; we'll want to limit
  or compress them on the bridge before relying on the RF link at a comp.

## Bench / debug scripts

In [`src/astrotech_rover/scripts/`](../src/astrotech_rover/scripts/):

- `auger_slight_move.py` — small moteus move, the first thing to try on a
  cold start.
- `auger_drv8323_status.py` — reads the moteus fault register.
- `mixing_servo_min.py` — drive the chamber to one angle.
- `mixing_servo_jog.py` — jog the chamber from a prompt.
- `auger_keys_test_harness.py` — Astrotech's original keyboard test script
  (the pattern the auger driver is based on).
- `smoke_test.sh` — builds, launches, and checks the topics show up.
- `raman_calibrate.py` — fit the Raman pixel→wavelength calibration from known
  peaks and print the YAML block to paste (stdlib only; see operator guide §6).
- `astrotech_hub.py` — tkinter operator launcher: buttons to build, launch the
  node + bridge (real/mock), bring up cameras/ZED on the Jetson over SSH, run
  diagnostics, calibrate Raman, and shut down. `python3 src/astrotech_rover/scripts/astrotech_hub.py`.

## Where every file lives

If you're looking for a specific thing, here's the whole layout with a
one-line note on what each file does.

```
gui/                                         ← the GUI (this folder)
├── README.md                                this file
├── operator_guide.md                        short mission-day how-to (the Confluence page)
├── layouts/
│   ├── urc_astrotech_auger.json             drilling layout: auger + servo + 2 cameras
│   └── urc_astrotech_analysis.json          lab layout: analysis + Raman + CO₂ + camera
└── extensions/urc-astrotech-panels/         the Foxglove panel extension
    ├── package.json                          name/version (bump version when you change panels)
    ├── src/index.ts                          registers the 5 panels with Foxglove
    ├── src/interfaces.ts                     topic/service names (must match the YAML below)
    └── src/panels/
        ├── AugerControl.tsx                  auger panel (working)
        ├── MixingServo.tsx                   mixing-servo panel (working)
        ├── RamanSpectrum.tsx                 Raman panel (wavenumber-vs-intensity plot)
        ├── AnalysisSequence.tsx              analysis panel (placeholder)
        └── Snapshots.tsx                     Save Raman / Save CO₂ buttons

src/astrotech_rover/                         ← the rover-side ROS package
├── package.xml  setup.py  setup.cfg          ROS package build files
├── config/astrotech_interfaces.yaml          all topic/service names + driver settings
├── launch/astrotech.launch.py                starts the node + the Foxglove bridge
├── astrotech_rover/
│   ├── astrotech_node.py                     starts one driver per piece of hardware
│   └── drivers/
│       ├── auger_real.py                     auger → moteus controllers
│       ├── mixing_servo_real.py              mixing servo → CMR servo board
│       ├── raman_real.py                     Raman → TCD1340 over USB serial
│       ├── env_real.py                       CO₂/temp/humidity → Adafruit SCD-30
│       ├── snapshot.py                       save-plot-to-PNG services (Raman/CO₂)
│       └── mock/                             stand-in drivers for running with no hardware
├── scripts/                                  bench / debug tools (see section above)
└── third_party/astrotech_canfd/              vendored CAN library + API_NOTES.md (don't edit)

src/cmr_msgs/                                 ← shared message package (rover-wide)
├── msg/AugerCommand.msg  AugerState.msg       auger command + telemetry
├── msg/RamanSpectrum.msg  EnvSample.msg        Raman + environment readings
├── srv/SetMixingServoAngle.srv                mixing-servo "go to angle" service
└── action/RunAnalysisSequence.action          analysis-sequence action

launch/gcs_bridge.launch.py                   ← the Foxglove bridge (rover ↔ Foxglove)
src/cmr_cams/                                 ← camera launch + per-camera configs
```

