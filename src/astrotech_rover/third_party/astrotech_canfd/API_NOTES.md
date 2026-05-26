# Astrotech CAN-FD Library — API Notes

Read-only analysis of the vendored library at
[`src/astrotech_rover/third_party/astrotech_canfd/`](../third_party/astrotech_canfd/).
Author: Caitlin Lee-Ying Rapalski. We will wrap this in a ROS 2 node
in Phase 2b — **never edit it in place**; see
`src/astrotech_rover/third_party/astrotech_canfd/NOTICE.md`.

The version we plan to wrap is `servo_bdc_control_ms/` (strict
superset of the older zip; adds ms-resolution motor commands).

## API surface

`CMR_CANFD/__init__.py` re-exports `FdCanInterface`, `BDCController`,
`ServoController`. Everything else is private bytecode caches or test
scripts.

| Class | Method | Notes |
|---|---|---|
| `FdCanInterface` | `__init__(port, baud=115200, timeout=1.0)` | Owns the serial port. `port` defaults to `"COM16"`; on Linux it is `/dev/ttyACM*`. `timeout` is stored and never used. |
| | `async open()` | Opens port, sleeps 1 s (boot drain), clears buffer. |
| | `async close()` | Closes the writer. |
| | `async configure_bus()` | Six bring-up `conf set` commands. **`fdcan_frame off`** — frames go out as classical CAN with bit-rate switching, not FD frame format. Library hardcodes **500 kbit/s**, but the actual Astrotech rig (verified 2026-05-08 bench bring-up) runs at **1 Mbit/s** — pass `--bitrate 1000000` to `src/astrotech_rover/scripts/mixing_servo_jog.py` / `--bitrate 1000000` to `src/astrotech_rover/scripts/mixing_servo_min.py`, and any future Phase 2b ROS driver must override this. |
| | `async write_frame(std_id, data_hex, flags="FB")` | Fire-and-forget transmit. Drain timeout (1 s) is logged via `print` but not raised. No verification the dongle accepted the frame. |
| | `async read_loop()` | Infinite — prints any `rcv` lines. Debug only. |
| | `async request_response(std_id, payload, flags, timeout)` | Sends a 7-byte payload prefixed with an 8-bit sequence; waits for a frame on the same id whose first byte matches. **Currently unused** by `BDCController` / `ServoController`. |
| `BDCController` | `__init__(can, motor_id=0, can_id=2)` | One instance per BDC motor. `motor_id` 0..5 (board hosts 6). `can_id` is the *board* id; default 2; second board would be 4. |
| | `async clear_faults()` | |
| | `async move_motor_forward(Duration)` | `Duration_Units=1` → seconds. |
| | `async move_motor_reverse(Duration)` | |
| | `async move_motor_forward_ms(Duration)` | ms variant. **Only in `_ms` zip.** |
| | `async move_motor_reverse_ms(Duration)` | |
| | `async stop_motor()` | |
| `ServoController` | `__init__(can, servo_id=0, home=0, can_id=1)` | `servo_id` 0..15. `home` is local-only (used by `go_home`, never sent). `can_id` is the board id; default 1; second board would be 3. |
| | `async go_to_position(degrees)` | `control_mode=0`. |
| | `async stop()` | `control_mode=2`. |
| | `async set_home(degrees)` | `control_mode=3`, `reset_home=1`. |
| | `async go_home()` | Local: `go_to_position(self.home)`. Does **not** read whatever `set_home` last sent to the board. |

### Frame format gotchas

- **BDC `Duration` is 7 bits.** Max value is 127. So
  `move_motor_forward(seconds=127)` is the longest single command.
  For 5–10 minute sequences the host has to chain commands or the
  team has to expose a "run continuously" mode we haven't documented.
- **Servo `control_data` is 12 bits.** Max 4095 — plenty for
  0–360° angles.
- All command frames are 32 bytes total but only the first 4 are
  populated; the remaining 28 are zero. The dongle is configured with
  `fdcan_frame off`, so a classical 8-byte CAN frame is what the
  board sees plus padding the library zero-fills.

## Async model

- Library is `asyncio` end to end through `serial_asyncio`.
- `move_motor_forward(D)` returns *as soon as the frame is on the
  wire*, not D seconds later. Callers do
  `await bdc.move_motor_forward(D); await asyncio.sleep(D)` if they
  want to wait for completion.
- Cancelling the asyncio task that called `move_motor_forward` does
  **not** stop the motor — the frame is already gone. The board will
  run for the full commanded duration unless we send `stop_motor`.
- `FdCanInterface` is a single shared transport; multiple controllers
  funnel through one `await write_frame(...)`. Asyncio's
  single-threadedness serializes the awaits — there's no internal mutex
  but two coroutines won't interleave bytes mid-frame.

## Feedback / sensing — there is none

This is the load-bearing fact for the design.

- `BDCController` has no `get_state` / `read_position` / `read_current`
  / `is_running` method.
- `ServoController` likewise.
- Every command frame sets `Query_data=1` (BDC) or has a
  `query_data` field (servo); whether the boards reply with telemetry
  is unknown — nothing in the library consumes a reply. (See open
  question A3.)
- `read_loop` will print `rcv` lines if you await it concurrently;
  no parser turns those into structured data.

**Implication.** The host can only know what it *commanded* and how
much wall-clock has elapsed since. There is no way to detect a stalled
motor, a reset board, or a successfully-completed move from the
library alone. Hence:

- Run-to-completion + hard abort is the only honest behaviour today
  for sequences (which is exactly what Phase 2a's revision settled on).
- Pump / mixing-servo / heater telemetry stays commanded-only.
- Pause/resume needs board-side feedback to be more than a guess about
  elapsed-vs-commanded; that's the post-URC backlog item.

(The auger is on moteus, not this library, so `AugerState.msg`
exposes real closed-loop telemetry. See gui/README.md.)

## Error handling — there is none

- No custom exceptions.
- No retries.
- No board health check before sending.
- `_send_command` blocks indefinitely on a missing OK/ERR.
- `write_frame` swallows the drain timeout with a `print`.

The Phase 2b ROS driver wrapper has to add: timeout guards on
`_send_command`, an unsolicited-frame consumer (parse `rcv` lines),
and graceful detection of a disconnected dongle.

## Bench-discovered gotchas (2026-05-08)

These are not in Caitlin's library or this doc as originally written.
Confirmed on the bench against the real Astrotech rig.

1. **Bus bitrate is 1 Mbit/s, not 500 kbit/s.** The library's
   `configure_bus()` hardcodes `conf set can.bitrate 500000`. At that
   rate the rig boards see only off-baud noise; nothing moves. The
   Phase 2b driver must set 1 Mbit/s; both bench scripts default to it.
2. **`clear_faults=1` must be set on every goto frame** (or on a
   session-start clear-faults frame) **until a Phase 2b driver maintains
   a single long-lived bus session**. Empirical observation: after any
   other traffic on the same fdcanusb (moteus auger commands, prior
   `can off`/`can on` cycles, prior session that ended with our
   "stop" frame), the servo board ACKs `go_to_position` on the bus but
   ignores it mechanically. Setting bit 7 of byte 3 (`clear_faults`)
   in the frame unsticks the rig. `ServoController.go_to_position()`
   does **not** set this bit, so wrappers must build the frame via
   `_make_control_frame(control_mode=0, control_data=N, clear_faults=1)`
   instead of calling `go_to_position()` directly. Both bench scripts
   do this; copy the pattern into the Phase 2b driver.
3. **Bench rig is at `(can_id=26, servo_id=15)`**, not the library's
   defaults of `can_id=1, servo_id=0`. The user's vendored
   `servo_min.py` (pasted into chat 2026-05-08) is the source of
   truth; both bench scripts default to these values.

## Open questions for Caitlin

These are the things the code does not make clear and need verbal
answers before Phase 2b commits to a design.

1. **Maximum duration encoding.** Is there a "run continuously until
   stopped" mode we're missing, or do we chain N × 127-second
   commands?
2. **Board feedback.** Does `Query_data=1` produce a real reply with
   telemetry, or is it a leftover bit? Reply payload format?
3. **Unsolicited fault frames.** Do boards send "motor stalled" /
   "over-current" frames? Wire format?
4. **Multi-board layout.** How many BDC and Servo boards on the rover
   at competition, and which CAN IDs?
5. **Per-actuator addressing.** Which `(can_id, motor_id)` is the
   auger? Each fluidic pump? Mixing chamber? Heater (separate board?
   GPIO?).
6. **Mixing-chamber preset definitions.** S1 / S2 / CO2_1 / CO2_2 /
   RETRACT → `(servo_id, degrees)` tuples (or multi-servo coordinated
   moves)?
7. **Persistent device path.** Symlink for the `usbcanfd` dongle on
   the Jetson (`/dev/usbcanfd` or do we hardcode `/dev/ttyACM0`)?
8. **Concurrency.** Are simultaneous commands across boards (auger
   BDC + mixing servo at the same instant) a real use case, or do
   sequences always run one actuator at a time?
