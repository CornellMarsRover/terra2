# Astrotech GUI — Operator Guide

How to drive the Astrotech panels from a fresh terminal. For the
implementation details and "what's wired" status, see
[`current_state.md`](current_state.md).

## ASTROTECH GUI ROS COMMANDS TO SEND TO FOXGLOVE

Run from the repo root (`~/terra2-astrotech-gui`) after one-time setup:

```bash
conda deactivate                    # cmr@jam only; conda hides python3.10
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble
colcon build --symlink-install --packages-select cmr_msgs urc_mock_rover
source install/setup.bash
```

Then pick one:

**MOCK (no hardware needed — everything is faked):**
```
ros2 launch urc_mock_rover mock.launch.py
```

**AUGER (real moteus, mixing servo still mock):**
```
URC_AUGER_REAL=1 ros2 launch urc_mock_rover mock.launch.py
```

**MIXING SERVO (real CMR servo board, auger still mock):**
```
URC_MIXING_SERVO_REAL=1 ros2 launch urc_mock_rover mock.launch.py
```

**BOTH AT ONCE — not supported on a single fdcanusb today.** Setting
`URC_AUGER_REAL=1 URC_MIXING_SERVO_REAL=1` together will fail: both
drivers try to open the same `/dev/ttyACM*` (the fdcanusb) and the
second one errors with `EBUSY`. To run them simultaneously you need
either:

1. **A second fdcanusb** so each driver has its own `/dev/ttyACM*`.
   Plug it in, then update `mixing_servo.real_port` in
   [`src/urc_mock_rover/config/astrotech_interfaces.yaml`](../src/urc_mock_rover/config/astrotech_interfaces.yaml)
   to point at the new dongle's `by-id` symlink. (Auger uses the
   moteus singleton transport which auto-resolves whichever fdcanusb is
   on `/dev/ttyACM0`, so make the *mixing servo* take the new dongle.)
2. **The unified driver node** from
   [`post_urc_backlog.md`](post_urc_backlog.md) — one ROS node owning
   the fdcanusb and multiplexing moteus + CMR-servo frames over one
   transport. Phase 2b.

The user's plan is option 1.

## FOXGLOVE SIDE

After `ros2 launch` is running:

1. Open **Foxglove Studio**.
2. **Open Connection** → `ws://localhost:8765`
   (on `cmr@jam` use `:8766` — Cursor holds 8765).
3. **Layouts → Import from file** → pick one:
   - [`gcs/layouts/urc_astrotech_auger.json`](../gcs/layouts/urc_astrotech_auger.json)
     — auger + mixing servo + auger cam + ZED.
   - [`gcs/layouts/urc_astrotech_analysis.json`](../gcs/layouts/urc_astrotech_analysis.json)
     — analysis sequence + Raman + analysis cam + CO₂ plot.

**If panels show "Unknown panel type"**, the custom extension isn't
installed. One-time:

```bash
cd gcs/extensions/urc-astrotech-panels
npm install
npm run build
npm run local-install
```

Then fully quit (`Cmd/Ctrl+Q`) and relaunch Foxglove Studio.

## CAMERAS (ROVER-SIDE)

Camera orchestration is in this repo at
[`src/cmr_cams/`](../src/cmr_cams/) (launch file + per-cam TOMLs +
activate.sh), but the underlying `cmr_cv.camera_node` driver and the
stereolabs `zed_wrapper` package live on the rover Jetson at
`192.168.1.69`. Bring-up is therefore rover-side, four SSH sessions:

**TERMINAL 1 — SSH + BUILD + LAUNCH CAMERAS**
```
ssh cmr@192.168.1.69
cd ~/cmr/terra2
colcon build
source install/setup.bash
ros2 launch cmr_cams default.launch.py
```

**TERMINAL 2 — ACTIVATE CAMERAS**
```
ssh cmr@192.168.1.69
cd ~/cmr/terra2
source install/setup.bash
v4l2-ctl --list-devices
```
Run for **EACH** camera that maps to a video (the *first* video output
in the list per physical camera, but **NOT** the ZED):
```
ros2 lifecycle set /camX configure
ros2 lifecycle set /camX activate
```
Or use the bundled helper (loops over all `/cam*` nodes):
```
./src/cmr_cams/config/activate.sh
# (chmod +x src/cmr_cams/config/activate.sh first if needed)
```
If some don't activate, try **deactivating and reactivating** them.
Verify a feed:
```
ros2 topic hz /cam11/image_raw
```

**TERMINAL 3 — LAUNCH ZED**
```
ssh cmr@192.168.1.69
cd ~/cmr/terra2
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

**TERMINAL 4 — FOXGLOVE BRIDGE (on the rover)**
```
ssh cmr@192.168.1.69
cd ~/cmr/terra2
source install/setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

Then in Foxglove Studio, **Open Connection → `ws://192.168.1.69:8765`**
and import the auger or analysis layout. The Image panels are
preconfigured for:
- `/cam11/image_raw` — auger cam (auger layout)
- `/cam12/image_raw` — analysis cam (analysis layout)
- `/zed/zed_node/left/image_rect_color` — ZED (auger layout)

`/dev/videoN` numbering depends on plug order, so cam11 may not be the
auger camera on every boot. If a panel shows the wrong feed, click the
Image panel's settings sidebar and edit `cameraTopic`.

### Camera screenshots

Each Foxglove Image panel has **Settings → Download image** that saves
the current frame as a PNG. A dedicated on-rover snapshot service
(spectrum / env / camera with metadata) is listed in
[`post_urc_backlog.md`](post_urc_backlog.md).

## TEAR DOWN

```
pkill -9 -f mock_rover_node
pkill -9 -f foxglove_bridge
```

## TROUBLESHOOTING

| Symptom | Likely cause |
|---|---|
| `Could not find package 'urc_mock_rover'` | Didn't `source install/setup.bash` (or didn't build). |
| Real driver: `EBUSY` opening `/dev/ttyACM0` | Both `URC_*_REAL` set, or another script (e.g. `mixing_servo_min.py`, `auger_slight_move.py`) is still holding the fdcanusb. |
| Real mixing servo: ACKs but chamber doesn't move | `clear_faults=1` was missed somewhere — the real driver sets it on every goto, so this usually means a script-not-driver is on the bus. |
| `Unknown panel type` in Foxglove | Extension not installed for *this* Foxglove instance, or extension was rebuilt without quit+relaunch. |
| Foxglove panels mount but show no service | Connected to a recording or non-bridge data source. Use `ws://...` to a live `foxglove_bridge`. |
