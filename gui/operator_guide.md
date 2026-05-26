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

### Or use the Astrotech Hub (one-window launcher)

Instead of typing commands by hand, run the launcher:

```
python3 src/astrotech_rover/scripts/astrotech_hub.py
```

It's a small GUI with a button for every step — build, launch the node +
bridge (real or all-mock), bring the cameras / ZED up on the Jetson, run
diagnostics, calibrate Raman, shut down. Each button opens a terminal and runs
the command there. The **Jetson** tab (the default) is the real rover bring-up;
the **Base station** tab is just this laptop's panel build + tools.

**Jetson tab (SSH into `cmr@192.168.1.69`)** — pick whichever auth you like:

- **Simplest — type it in the panel:** there's a **Jetson SSH password** field
  in the hub's header. Type the team password (`*****`) there and it's used for
  that session only — held in memory, never written to disk. Leave it blank to
  use one of the options below. (Needs `sshpass`: `sudo apt install sshpass`.)
- **No password at all — SSH key:**
  ```
  ssh-copy-id cmr@192.168.1.69
  ```
  Enter the Jetson password that one time; afterward the hub uses your key.
- **Remember it across sessions — `.env` file:** create `.env` in the repo root
  — it's **gitignored, never commit it** — and lock it down:
  ```
  cp .env.example .env
  chmod 600 .env
  ```
  then edit `.env` so it reads (use the real team password, not the stars):
  ```
  JETSON_SSH_PASSWORD=*****
  ```

## 2. Connect Foxglove Studio

1. Open **Foxglove Studio**.
2. **Open Connection** → `ws://192.168.1.69:8765` (the Jetson), or
   `ws://localhost:8765` if you're running the node on this machine.
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
- **Raman** — live spectrum plot (pixel-index x-axis until calibrated — see §6).
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
ros2 run cmr_zed zed_publisher_node      # publishes /zed/image_left (+ _right)
```
This is what the auger layout's ZED tile points at (`/zed/image_left`). If you
run the stereolabs `zed_wrapper` instead, its left image is
`/zed/zed_node/left/image_rect_color` — switch the tile's `cameraTopic` to that.

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

## 6. Calibrate the Raman spectrometer (one-time)

Out of the box the Raman x-axis is the **pixel index** (0…N), not real Raman
shift. To switch it to cm⁻¹, give the driver two values in
`src/astrotech_rover/config/astrotech_interfaces.yaml` under `raman:`:

```yaml
  n_points: 3648            # TCD1340 pixel count (raise from the 1024 default)
  real_calibration:
    laser_nm: 785.0                          # YOUR excitation laser wavelength
    pixel_to_wavelength_poly: [c0, c1, c2]   # nm = c0 + c1·p + c2·p²  (ascending!)
```

You supply two things:

1. **`laser_nm`** — your excitation laser's wavelength in nm. You already know
   this (it's a property of the laser, e.g. 532 / 633 / 785 nm).
2. **`pixel_to_wavelength_poly`** — from a quick fit. Take a spectrum of
   something with known peaks (a calibration lamp, or a Raman standard such as
   **silicon at 520.7 cm⁻¹**), read off the **pixel** each known peak lands on,
   and run the helper:

   ```
   python3 src/astrotech_rover/scripts/raman_calibrate.py --laser-nm 785 \
       --point 410:520.7:cm-1 \
       --point 1980:1332:cm-1 \
       --point 3050:2900:cm-1
   ```

   Each `--point` is `PIXEL:VALUE:UNIT`, where UNIT is `cm-1` (a Raman shift)
   or `nm` (an absolute wavelength). Use ≥3 points for the default quadratic
   fit. It prints the `real_calibration:` block ready to paste — already in the
   ascending coefficient order the driver wants — plus a per-point residual so
   you can sanity-check the fit (aim for an RMS of a few cm⁻¹ or better). It's
   plain Python: no ROS or numpy needed, runs on any laptop.

Paste the block in, rebuild + relaunch (§1), and the Raman panel switches to a
cm⁻¹ x-axis. Leave `real_calibration` commented out to keep the pixel-index
axis (the plot still works — peaks just aren't in cm⁻¹ yet).

## 7. Shut down

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
