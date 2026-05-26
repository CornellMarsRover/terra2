# Astrotech GUI — Operator Manual

How to run and operate the Astrotech science-payload GUI during a mission.
For system internals and hardware setup, see `README.md` in this folder.

---

## 1. Start the rover software

One-time, on the machine running the payload:

```
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cmr_msgs astrotech_rover
source install/setup.bash
```

Every session — start the node + Foxglove bridge:

```
ros2 launch astrotech_rover astrotech.launch.py
```

This runs the real hardware and serves Foxglove at `ws://localhost:8765`.
(To run without hardware for a demo, see
`src/astrotech_rover/astrotech_rover/drivers/mock/README.md`.)

## 2. Connect Foxglove Studio

1. Open **Foxglove Studio**.
2. **Open Connection** → `ws://localhost:8765`
   (or `ws://<rover-ip>:8765` from another laptop).
3. **Layouts → Import from file**, pick one:
   - **`urc_astrotech_auger.json`** — drilling: auger + mixing servo + cameras.
   - **`urc_astrotech_analysis.json`** — lab: analysis + Raman + CO₂ + camera.

**First time only — install the panels:**

```
cd gui/extensions/urc-astrotech-panels
npm install && npm run build && npm run local-install
```

Then fully quit (Cmd/Ctrl+Q) and relaunch Foxglove. If panels show
**"Unknown panel type,"** the extension isn't installed for this instance.

## 3. Using the panels

### Auger (drilling)
- **Hold** a drive button; release to stop:
  - **Up (no spin)** — raise the drill.
  - **Down + spin CCW** — drill down.
  - **Spin CW / Spin CCW** — rotate in place (clear / unjam).
- **Set Up Home / Set Bottom Home** — save the current height as a marker.
- **Return to Up Home / Return to Bottom Home** — auto-drive back to a saved
  marker (greyed out until you've set it).
- **Stop** — halt all motion immediately.
- The telemetry table shows live position / speed / torque / temperature
  for both motors.
- ⚠️ Saved homes reset when the rover restarts — re-set them each session.

### Mixing servo (sample chamber)
- **Set Home** — press once after positioning the chamber at its reference;
  that becomes 0°. Do this first.
- **Big Box / Site 1 / Site 2** — drive to the saved drop angle. The button
  greys out for ~5 s while the chamber moves.
- **Manual goto** — type an angle and drive there.
- **Edit presets** — change the saved Big Box / Site angles.

### Raman / CO₂ / cameras (read-only)
- **Raman** — live wavenumber-vs-intensity plot of the latest spectrum.
- **CO₂ plot** — CO₂ level over time.
- **Image panels** — live camera feeds (bring cameras up in §5).

### Snapshots
- **Save Raman / Save CO₂** — saves a PNG of the current plot. It lands in
  `~/Desktop/astrotech_snapshots/` on the machine running `astrotech_node`
  (the laptop), or the project's `snapshots/` folder if the Desktop can't be
  written. The button shows the saved file path when it's done.

### Analysis sequence
Not active yet — the buttons are placeholders. Skip for now.

## 4. Running one actuator at a time

The auger and the mixing servo share one USB-CAN dongle and **can't both
run on it at once**. If you only need one, mock the other so it doesn't
grab the bus:

```
URC_MIXING_SERVO_MOCK=1 ros2 launch astrotech_rover astrotech.launch.py   # auger only
URC_AUGER_MOCK=1        ros2 launch astrotech_rover astrotech.launch.py    # mixing servo only
```

## 5. Bring up the cameras (on the rover)

Cameras run on the rover Jetson (`192.168.1.69`), in four SSH terminals:

**Terminal 1 — launch cameras**
```
ssh cmr@192.168.1.69
cd ~/cmr/terra2 && colcon build && source install/setup.bash
ros2 launch cmr_cams default.launch.py
```

**Terminal 2 — activate them**
```
ssh cmr@192.168.1.69 && cd ~/cmr/terra2 && source install/setup.bash
v4l2-ctl --list-devices          # see which camera is which /dev/videoN
./src/cmr_cams/config/activate.sh   # activates all cameras
ros2 topic hz /cam11/image_raw   # confirm a feed is publishing
```
If one doesn't come up, deactivate and reactivate it.

**Terminal 3 — ZED camera**
```
ssh cmr@192.168.1.69 && cd ~/cmr/terra2 && source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

**Terminal 4 — Foxglove bridge**
```
ssh cmr@192.168.1.69 && cd ~/cmr/terra2 && source install/setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

Then in Foxglove connect to `ws://192.168.1.69:8765` and open a layout.
**If an Image panel shows the wrong camera,** open its settings sidebar
and change `cameraTopic` (camera ↔ `/dev/videoN` numbering can shift
between boots).

**Screenshots:** each Image panel has **Settings → Download image** to save
the current frame as a PNG.

## 6. Shut down

```
pkill -9 -f astrotech_node
pkill -9 -f foxglove_bridge
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| Panels say **"Unknown panel type"** | Extension not installed (or rebuilt without relaunch). Run the install in §2 and fully quit + relaunch Foxglove. |
| Buttons do nothing / are greyed out | Not connected to a live rover. Connect to a `ws://…` bridge, not a recording. |
| Camera panel: **"no messages on topic"** | Camera not activated, or wrong topic — see §5 and check `cameraTopic`. |
| Actuator won't move / **`EBUSY`** error | Auger and mixing servo are both on one dongle — mock one (§4). Also make sure no bench script is still holding the bus. |
| `Could not find package 'astrotech_rover'` | You didn't `source install/setup.bash` (or didn't build) — see §1. |
| Auger / servo silent on real hardware | Check the rover terminal where you launched — each driver logs a clear error line if its hardware isn't found. |
