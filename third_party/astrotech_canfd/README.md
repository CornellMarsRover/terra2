# Astrotech CAN-FD library — vendored copy

Reference Python library used by the Astrotech subteam to drive the
custom **BDC boards** (brushed-DC motor drivers, used for fluidic pumps)
and **Servo boards** (16-channel hobby-servo drivers, used for the
mixing chamber and other actuators) over CAN-FD via an
mjbots `usbcanfd` USB-CAN dongle.

See `NOTICE.md` for ownership / vendoring rules.

## Contents

| Subfolder | Source zip | What's different |
|---|---|---|
| `servoboard_and_bdc_test_python_code/` | `servoboard_and_bdc_test_python_code(1).zip` | Original. BDC commands take **seconds** only. |
| `servo_bdc_control_ms/` | `servo_bdc_control_ms.zip` | Adds `move_motor_forward_ms` / `move_motor_reverse_ms` for **millisecond** resolution. Otherwise identical. |

Both subfolders share the same `CMR_CANFD/` package layout under
`test/`:

```
test/
  CMR_CANFD/
    __init__.py          # re-exports FdCanInterface, BDCController, ServoController
    FdCanInterface.py    # asyncio + serial-asyncio wrapper for the usbcanfd dongle
    BDCController.py     # 1 board = up to 6 BDC motors; pump / linear-actuator commands
    ServoController.py   # 1 board = up to 16 servos; mixing-chamber / positional commands
  test.py                # ad-hoc bring-up script with commented-out command examples
  bus_monitor.py         # raw pyserial loop for sniffing CAN frames off the dongle
```

Diff between the two zips:

- `servo_bdc_control_ms/test/CMR_CANFD/BDCController.py` adds two
  methods (`move_motor_forward_ms`, `move_motor_reverse_ms`).
- `servo_bdc_control_ms/test/test.py` updates a port name (`COM7` vs.
  `COM6`), uncomments a `bdc6` example, and adds a `# READ ME` comment
  block summarizing every public command.

## Which version Phase 2b will wrap

**`servo_bdc_control_ms/`.** It's a strict superset of the older zip
(both second- and ms-resolution commands), and the ms commands are
needed for short pulses in the fluidic protocol that the Astrotech team
described.

## Hardware addressing summary (extracted from the code)

- Each **Servo board** answers on a configurable CAN ID (default `1`).
  It hosts up to 16 servos addressed by `servo_id` (0..15) inside the
  4-byte command frame. Multiple servo boards can be daisy-chained on
  the bus by using different CAN IDs (e.g. `1` and `3`).
- Each **BDC board** answers on a configurable CAN ID (default `2`).
  It hosts up to 6 BDC motors addressed by `motor_id` (0..5). Multiple
  BDC boards can coexist on the bus on different CAN IDs (e.g. `2` and
  `4`).
- The bus is one mjbots `usbcanfd` dongle reachable from the Jetson as
  a serial port (`/dev/ttyACM*` on Linux; `COMx` in the team's Windows
  development setup).
- Bitrate: 500 kbit/s arbitration with bit-rate switching enabled. FD
  frame mode is *off* in the configured pipeline (BDC / Servo frames
  fit in a classical 8-byte CAN frame plus padding to 32 bytes that the
  library zero-fills).

For the full API surface, see `docs/astrotech_canfd_library_notes.md`.
