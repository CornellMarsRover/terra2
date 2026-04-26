# Astrotech CAN-FD Library — API Notes

Read-only analysis of the vendored library at
[`third_party/astrotech_canfd/`](../third_party/astrotech_canfd/). Author
is Caitlin Lee-Ying Rapalski (Astrotech subteam). We will wrap this code
in a ROS 2 driver node in Phase 2b — **we will not edit it in place**;
see `third_party/astrotech_canfd/NOTICE.md`.

This document is descriptive: every claim below is grounded in a file
and line number from the vendored source.

---

## 1. Files inventory

Two zips, one strict superset of the other. `servo_bdc_control_ms/` is
the version we plan to wrap.

| Path under `third_party/astrotech_canfd/` | Purpose |
|---|---|
| `servo_bdc_control_ms/test/CMR_CANFD/__init__.py` | Re-exports `FdCanInterface`, `BDCController`, `ServoController`. Declares `__version__ = "0.1.0"`. |
| `servo_bdc_control_ms/test/CMR_CANFD/FdCanInterface.py` | Async wrapper over `serial_asyncio` for an mjbots `usbcanfd` dongle. Owns the only serial connection. |
| `servo_bdc_control_ms/test/CMR_CANFD/BDCController.py` | Per-motor command builder for a BDC board. One instance per motor. |
| `servo_bdc_control_ms/test/CMR_CANFD/ServoController.py` | Per-servo command builder for a Servo board. One instance per servo. |
| `servo_bdc_control_ms/test/test.py` | Bring-up script. Constructs 16 servos and up to 12 BDC motors against a hardcoded COM port; almost every command is commented out, included as documentation. The active code path is `await bdc6.move_motor_forward(5)` followed by `while(1): await asyncio.sleep(1.5)`. |
| `servo_bdc_control_ms/test/bus_monitor.py` | Standalone (synchronous) `pyserial` script that reconfigures the dongle and prints raw `rcv ...` lines. Useful for sniffing; not used by the library proper. |
| `servoboard_and_bdc_test_python_code/test/CMR_CANFD/*` | Earlier snapshot. Differs only in `BDCController.py` (no `_ms` methods) and `test.py` (a different COM port + minor edit churn). |
| `servoboard_and_bdc_test_python_code/test/{test.py,bus_monitor.py}` | Earlier copies of the same scripts. |

Total active code we care about (in `servo_bdc_control_ms/test/CMR_CANFD/`): four files, ~210 lines combined.

---

## 2. Public API surface

### 2.1 `FdCanInterface`

`FdCanInterface.py:4`. The transport layer. One instance per `usbcanfd`
dongle. Owns the `asyncio.StreamReader` / `StreamWriter` pair and a
sequence counter for request-response framing.

#### Constructor

```python
FdCanInterface(port: str = "COM16", baud: int = 115200, timeout: float = 1.0)
```

- `port` — serial device. Defaults are Windows COM names; on Linux this
  is typically `/dev/ttyACM0`. Hardcoded in callers (the test scripts
  use `COM7` / `COM6`).
- `baud` — fixed at 115 200 in all examples. The CAN-FD bitrate (500 k)
  is configured *over* this serial link with `conf set` commands; this
  baud is just the host↔dongle UART rate.
- `timeout` — stored but never used (`request_response` uses its own
  `timeout` parameter; `_send_command` blocks until OK/ERR; `read_loop`
  blocks forever). Setting this has no observable effect.

#### Methods

| Method | Signature | Notes |
|---|---|---|
| `open()` | `async def open(self)` | Opens the serial port via `serial_asyncio.open_serial_connection`, sleeps 1 s, then drains the port. **Blocking 1 s `asyncio.sleep`** at startup. |
| `close()` | `async def close(self)` | Closes writer if present. |
| `_clear_buffer()` | private | Drains the input buffer with 0.1 s timeouts until a read times out. Called from `open()`. |
| `_send_command(cmd: str)` | `async def _send_command(self, cmd: str) -> str` | Writes `<cmd>\n` to the dongle and reads lines until one is `"OK"` or starts with `"ERR"`. **Blocks indefinitely** on a wedged dongle — no timeout. |
| `configure_bus()` | `async def configure_bus(self)` | Issues six bring-up commands: `can off`, `conf set can.bitrate 500000`, `conf set can.fdcan_frame off`, `conf set can.bitrate_switch on`, `conf set can.termination on`, `can on`. **Note** that `fdcan_frame` is **off** — frames go out as classical CAN with bit-rate switching, not FD frame format. The library still pads command payloads to 32 bytes, but only the first 4 bytes are meaningful and the remainder is wire fill. |
| `write_frame(std_id, data_hex, flags="FB")` | `async def write_frame(self, std_id: int, data_hex: str, flags="FB") -> None` | Fire-and-forget transmit. Writes `can std <id> <hex> <flags>\n`. Drain has a 1 s timeout that is logged via `print` if it fires; the method does **not** raise on drain timeout. No verification that the dongle accepted the frame. |
| `read_loop()` | `async def read_loop(self)` | Infinite loop that prints any `rcv ...` lines seen on the serial. Call once, await it, never returns. Useful for debugging only. |
| `request_response(std_id, payload, flags="FB", timeout=1.0)` | `async def request_response(...) -> tuple[bytes, list[str]]` | Sends a 7-byte payload prefixed with an 8-bit sequence number, then waits for a frame on the same `std_id` whose first byte equals that sequence. **Note**: `BDCController` and `ServoController` do *not* use this method — both call `write_frame` directly. So request-response is implemented in `FdCanInterface` but is currently dead code as far as the higher-level controllers go. |

#### State after errors

- A serial-side error (port unplugged, dongle reset) surfaces as the
  underlying `asyncio.StreamReader` returning empty / raising a
  `ConnectionError`. The library does not catch any of these. Caller
  must handle.
- `_send_command` and `read_loop` will block forever if no data ever
  arrives.
- There is no automatic reconnect logic.

### 2.2 `BDCController` (one instance per BDC motor)

`BDCController.py:3`.

#### Constructor

```python
BDCController(can: FdCanInterface, motor_id: int = 0, can_id: int = 2)
```

- `can` — shared `FdCanInterface`.
- `motor_id` — `0..5`. Encoded into bits `[2..4]` of the first command
  byte (`(self.motor_id & 0x07) << 2`). One BDC board hosts up to 6
  motors.
- `can_id` — the CAN ID of the *board*. Default `2`; a second board on
  the same bus would be `4` per the test script.

#### Methods (all async, all fire-and-forget)

| Method | Effect |
|---|---|
| `clear_faults()` | `Msg_Type=0`, `Clear_Faults=1`, `Query_data=1`. Sent on the board's CAN ID. |
| `move_motor_forward(Duration: int)` | `Msg_Type=1`, `Duration_Units=1` (seconds), `Mode_Select=2`, `Query_data=1`. Tells the board "run motor `motor_id` forward for `Duration` seconds." |
| `move_motor_reverse(Duration: int)` | Same shape with `Msg_Type=3`, `Mode_Select=3`. |
| `move_motor_forward_ms(Duration: int)` | `Duration_Units=0` (milliseconds). **Only present in `servo_bdc_control_ms/`.** |
| `move_motor_reverse_ms(Duration: int)` | Same. ms variant. |
| `stop_motor()` | `Msg_Type=0`, `Duration=0`. Effectively a NOP on the bit field for `Mode_Select` (it stays at default `0`); the board interprets this as "stop the addressed motor". |

#### Frame format (BDC)

`_make_control_frame` builds a 32-byte frame, of which only the first 4
bytes are populated; the remaining 28 are `\x00`. Bit fields:

```
byte 0:  bits[0..1]=Msg_Type, bits[2..4]=motor_id, bits[5..7]=Mode_Select
byte 1:  bits[0..1]=Duration_Units (0=ms, 1=s)
byte 2:  bits[0..6]=Duration              (7 bits → 0..127)
byte 3:  bits[0]=Clear_Faults, bits[1]=Query_data
bytes 4..31: zero
```

Two notable things:

- `Duration` is a **7-bit field**. Maximum value is **127**. So
  `move_motor_forward(seconds=127)` is the longest command; longer
  durations require splitting into multiple commands (or using the ms
  variant, which still tops out at 127 ms — too short to be useful for
  long pump runs unless we reissue periodically).
- `Mode_Select` is set in `move_motor_forward` (=2) and
  `move_motor_reverse` (=3) but defaults to `0` in `stop_motor` and
  `clear_faults`. Whether the board cares is unverified from the code.

### 2.3 `ServoController` (one instance per servo)

`ServoController.py:3`.

#### Constructor

```python
ServoController(can: FdCanInterface, servo_id: int = 0, home: int = 0, can_id: int = 1)
```

- `servo_id` — `0..15`. One servo board hosts 16 servos.
- `home` — stored angle to use when `go_home()` is called; not sent to
  the board, just remembered locally.
- `can_id` — board CAN ID (default `1`; second servo board would be
  `3`).

#### Methods (all async, all fire-and-forget)

| Method | Effect |
|---|---|
| `go_to_position(degrees: int)` | `control_mode=0`, `control_data=degrees`. Position command. |
| `stop()` | `control_mode=2`, `control_data=0`. Stop / freeze. |
| `set_home(degrees: int)` | `control_mode=3`, `control_data=degrees`, `reset_home=1`. Tells the board to remember `degrees` as home. |
| `go_home()` | `go_to_position(self.home)`. Local; uses the stored `home` from the constructor, **not** whatever was last `set_home`'d. |

#### Frame format (servo)

```
byte 0:  bit[6]=1 (servo-frame marker), bits[2..5]=servo_id (4 bits),
         bit[1]=home, bit[0]=reset_home
byte 1:  bits[6..7]=control_mode (2 bits), bits[0..5]=high 6 bits of control_data
byte 2:  bits[2..7]=low 6 bits of control_data, bits[0..1]=zero
byte 3:  bit[7]=clear_faults, bit[6]=query_data
bytes 4..31: zero
```

`control_data` is a **12-bit** field (6 high bits in byte 1 and 6 low
bits in byte 2 shifted left by 2). Maximum value is `4095`. For a
0–360° angle, that's plenty of resolution; for sub-millisecond pulse
widths the math is the board's problem.

### 2.4 Multi-board addressing

There is no chaining mechanism in code. Multi-board support is purely
"construct N controllers with different `can_id` values":

```python
ee  = ServoController(can=fd, servo_id=0,  can_id=1)   # board 1
ee16= ServoController(can=fd, servo_id=0,  can_id=3)   # board 2 (commented out)
bdc = BDCController(can=fd, motor_id=0,    can_id=2)   # BDC board 1
bdc6= BDCController(can=fd, motor_id=0,    can_id=4)   # BDC board 2
```

`FdCanInterface` is a single shared transport; all controllers funnel
through `await self.can.write_frame(...)`. There is **no internal mutex**
on `write_frame`, but the asyncio event loop's single-threadedness
serializes the awaits. Two coroutines awaiting `write_frame` will not
interleave bytes mid-frame.

---

## 3. Async model

The library is consistently `asyncio` end to end:

- All public controller methods are `async def`.
- All bus I/O goes through `serial_asyncio`'s `StreamReader` /
  `StreamWriter`, which integrates with the asyncio loop natively.
- `asyncio.sleep(D)` is the supported way to insert a delay between
  commands. The example script comments document this:
  > `asyncio.sleep(duration in seconds) -- wait command`.

### Blocking / non-cancellable spots

- `FdCanInterface.open()` calls `await asyncio.sleep(1.0)` — that
  yields, but a startup of 1 s is unconditional.
- `_send_command` reads lines without a timeout and blocks until the
  dongle returns OK / ERR. If the dongle is unresponsive this hangs.
- `read_loop` is intentionally infinite.
- `request_response` honors a timeout but raises `asyncio.TimeoutError`
  on expiry. The fact that the controllers don't use it means timeouts
  effectively don't exist on motor / servo commands today.

### Cancellation semantics

- `write_frame` writes once and returns. There's nothing to cancel
  mid-flight (the write is small enough to drain in one quantum).
- A `move_motor_forward(N)` command, once accepted by the board, runs
  on the board for N seconds. The host has no way to "cancel that
  command from the board's perspective" except by sending a fresh
  `stop_motor` frame. **This is the key fact for pause / resume
  design**: pause requires us to (a) record commanded-duration vs.
  elapsed-since-command, (b) send `stop_motor`, and (c) on resume,
  reissue `move_motor_*` with the *remaining* duration.
- Cancelling the asyncio task that called `move_motor_forward` does
  *not* stop the motor — the frame has already been sent.

### How `asyncio.sleep` interacts with motor commands

Two important properties:

- `move_motor_forward(D)` returns *as soon as the frame is on the
  wire*, not D seconds later. The board runs autonomously.
- A typical sequence step looks like:
  ```python
  await bdc.move_motor_forward(5)
  await asyncio.sleep(5)         # wait for the motor to actually finish
  ```
  The `asyncio.sleep` is the team's only way to know "the motor should
  be done now." There is no completion event / encoder check.

This pattern shapes the pause/resume design completely: the sequencer
must track `(command_kind, commanded_duration, command_start_monotonic)`
for every in-flight motor at the moment of pause.

---

## 4. Error handling

The library does virtually none. Specifically:

- **No exceptions defined.** No custom exception class anywhere.
- **No retry logic.** `write_frame` writes once. If the OS-level
  serial write succeeds, the function returns success.
- **No board health check.** Nothing pings the boards before sending
  commands; if a board is offline the frame goes out and is silently
  ignored.
- **No CAN-side error reporting consumed.** Boards may reply with
  ACK / NAK / fault frames; nothing in `BDCController` /
  `ServoController` looks at them.
- **`_send_command` swallows nothing but waits forever on a missing
  OK/ERR.** A wedged dongle hangs the entire event loop's interaction
  with that future.
- **`write_frame` swallows the drain timeout** with a `print`. Caller
  has no signal that the write was slow.

Failure modes from the wrapping ROS 2 driver's perspective:

| Failure | Library behavior | Driver mitigation needed |
|---|---|---|
| Dongle unplugged at runtime | Reader returns empty / `ConnectionError` propagates out of any `await reader.readline()` | Catch in driver, mark bus offline, refuse new commands, log loudly. |
| Dongle present but dead | `_send_command` and `read_loop` hang forever | Wrap calls in `asyncio.wait_for` at the driver layer; `_send_command` is only called during `configure_bus`, so this matters at startup. |
| BDC/Servo board offline | Frame goes out, no error. Subsequent sequence step assumes it ran. | Only safeguard is sending a `clear_faults` + waiting for an ACK we can verify. The current library doesn't read ACKs, so the driver must add a readback path (parse `rcv ...` lines that look like board status replies). |
| Power loss to a board mid-run | Same as above. | Same. The board reboots into a known state when power returns; our sequencer needs to detect the gap. |
| CAN bus error frames | Library does not parse them. | Driver adds a parallel `read_loop`-style consumer. |

---

## 5. Feedback / sensing

**There is no feedback exposed by the library.** Specifically:

- `BDCController` has no `get_state` / `read_position` / `read_current`
  / `is_running` method. There is `Query_data=1` set in every command
  frame, which suggests the board *does* respond with telemetry, but
  the library does not consume any reply.
- `ServoController` likewise has no read methods. `query_data=0` by
  default in `_make_control_frame`, but no plumbing to consume a
  response either way.
- `FdCanInterface.read_loop` will print `rcv` lines to stdout if you
  await it concurrently, but no parser turns those into structured
  data.
- `request_response` exists in `FdCanInterface` but is currently
  unused by the controllers.

**Implication for pause / resume**: there is no way to know whether a
motor command actually completed. The host can only know how long it
*commanded* the motor and how much wall-clock has elapsed since that
command. Hence the pause-resume granularity will be:

- step-level (resume the next step from the start), or
- intra-step time-resumable (resume the current step with a shortened
  remaining duration).

Sub-step "where is the motor right now in its travel" is not knowable
without library changes that consume `Query_data` replies.

---

## 6. Open questions to ask Caitlin

These are things the code does not make clear and need a verbal answer
before Phase 2b commits to a design.

1. **Pause semantics.** When the operator hits "pause" on a long
   fluidic protocol, the right host-side behavior is one of:
   (a) `await bdc.stop_motor()` for every BDC currently running plus
       `await servo.stop()` for every servo, then on resume re-issue
       only the *remaining* duration of any motor that was mid-command;
   (b) Just stop everything and on resume restart the *current step*
       from the top;
   (c) Stop everything, on resume start at the *next step* (the
       paused step is considered failed-but-acknowledged).
   Which of these matches the science requirements? Mid-pump pauses
   tend to wreck fluid hand-off, so (b) or (c) is more likely
   correct, but the spec needs to say so explicitly.

2. **Maximum duration encoding.** The `Duration` field is 7 bits, max
   127. For 5–10 minute sequences the host must reissue commands
   periodically — does the board support a "run continuously until
   stopped" mode (Mode_Select value we haven't documented) or do we
   really need to chain N × 127-second commands with bridging
   `asyncio.sleep`?

3. **Multi-board chaining.** The test script comments out a second
   servo board (`can_id=3`) and a second BDC board (`can_id=4`). Are
   those boards actually present on the rover today, planned, or
   purely speculative? Answer affects how many distinct CAN IDs the
   driver must address.

4. **Error reporting.** Does the board ever send unsolicited error
   frames (e.g. "motor stalled", "over-current")? If yes, what's the
   wire format? The library doesn't parse anything; we'll need
   a spec — or the team's blessing to ignore them for now and treat
   every command as fire-and-forget.

5. **Feedback on commands.** Is `Query_data=1` in the command frame
   actually answered by the board with telemetry (encoder counts,
   current draw, completion ack), or is it a leftover bit? If it
   responds, what is the reply payload format? This is the single
   biggest lever for resume granularity.

6. **Servo board "control_mode" range.** The frame format reserves 2
   bits for `control_mode`, but only modes 0 (position), 2 (stop),
   and 3 (set-home) are used in code. Is mode 1 reserved, future, or
   undefined?

7. **Heater control.** The interview notes mention a Ninhydrin heater.
   It's not represented in this library. Is the heater on the BDC
   board (one of the 6 channels) under a different command shape, on
   a separate board entirely, or driven from a non-CAN-FD path
   (GPIO / I²C)?

8. **Mixing-chamber preset definitions.** S1, S2, CO2_1, CO2_2,
   RETRACT need to map to `(servo_id, degrees)` tuples (or possibly
   multi-servo coordinated moves). What's the mapping today?

9. **Dongle port detection.** All examples hardcode a Windows COM
   port. On the actual Jetson, what's the udev rule / persistent
   symlink we should use (`/dev/usbcanfd` symlinked to whichever
   `/dev/ttyACM*` it happens to enumerate as)?

10. **Concurrency / serialization.** Are simultaneous motor commands
    on different boards (e.g. BDC A and Servo B at the same time) a
    real use case, or do sequences always run one motor at a time?
    Affects whether the driver needs to model "the bus is currently
    busy" as a top-level concept.

End of API notes.
