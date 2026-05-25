#!/usr/bin/env python3
"""Interactive jog tool for the Astrotech mixing servo (and any other servo
on a CMR Servo board).

Purpose
-------

This script answers the long-pending calibration question pinned at
``TODO(astrotech-q-2)`` in ``docs/phase2a_assumptions.md`` and as open
question 6 in ``docs/astrotech_canfd_library_notes.md``:

    S1 / S2 / CO2_1 / CO2_2 / RETRACT  ->  (servo_id, degrees) tuples

Walk the rig through each preset position by hand, type the commanded
angle into the REPL, and write down the value once the chamber visually
reaches the target box. Once all five presets are recorded, the next
commit (not this script) will lift them into ``astrotech_interfaces.yaml``
as a ``preset_angles`` map and teach ``drivers/mixing_servo.py`` to use it.

Hardware assumptions
--------------------

* Dongle:  mjbots fdcanusb / usbcanfd (single ASCII CLI either way),
           presented as ``/dev/ttyACM*`` or a stable
           ``/dev/serial/by-id/...`` symlink.
* Bus:     500 kbit/s, classical 8-byte CAN frames (``fdcan_frame off``),
           bit-rate switch on, termination on. Same six bring-up
           commands ``CMR_CANFD/FdCanInterface.configure_bus`` issues.
* Servo board: default ``can_id=1`` (override with ``--can-id`` or the
           in-REPL ``board N`` command). Up to 16 servos per board,
           ids 0..15 (override with ``--servo-id`` or ``id N``).

Wire-format source of truth: ``ServoController._make_control_frame`` in
``third_party/astrotech_canfd/servo_bdc_control_ms/test/CMR_CANFD/``.
We duplicate the 4 populated bytes here rather than importing the
vendored library, per ``third_party/astrotech_canfd/NOTICE.md``
("do not import these files directly from src/" -- the same spirit
applies to throwaway bring-up tools).

What it does NOT do
-------------------

* No feedback. The library provides none, the boards may or may not
  reply to ``query_data=1``, and no one has parsed those replies yet.
  Confirmation that the chamber actually reached the commanded angle
  is *visual*. (This script tails ``rcv`` lines in the background so
  if the board does talk back you'll see something, but nothing is
  decoded.)
* No multi-servo coordination. If the mixing chamber turns out to be
  driven by N>1 servos, command them one at a time with ``id N`` and
  record both angles per preset.

Bus contention
--------------

The fdcanusb is a single serial device. **Do not** run any moteus code
(the auger Foxglove panel, ``scripts/auger_slight_move.py``, etc.) at
the same time as this script -- they'll fight for ``/dev/ttyACM0``.
Auger and servo bus operations have to be serialized at the host level
until Phase 2b's CAN-FD driver node owns the transport.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import sys
import threading
import time
from typing import Optional

try:
    import serial  # type: ignore[import-not-found]
except ModuleNotFoundError:
    sys.stderr.write(
        "missing dependency: pyserial. install with `pip install pyserial`.\n"
    )
    raise

DEFAULT_PORT = "/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00"
DEFAULT_BAUD = 115200  # ignored by USB-CDC ACM; matches the vendored library.
DEFAULT_CAN_ID = 1
DEFAULT_SERVO_ID = 0
# Caitlin's vendored ``FdCanInterface.configure_bus`` and the team's docs
# (``docs/astrotech_canfd_library_notes.md``) both say 500 kbit/s, but the
# only known-working bench script on this rig (the user's minimal
# ``servo_min.py``) configures 1 Mbit/s and that's what the boards
# actually expect. Default here matches the vendored library so any
# future board that genuinely runs at 500 kbps still works without a flag;
# pass ``--bitrate 1000000`` for the current Astrotech rig.
DEFAULT_BITRATE = 500_000

# Persists last-commanded angle across script invocations so that
# ``--rel +1`` works as a "nudge by 1 from wherever I left it" command.
# /tmp is the right scope: bench-session state, not project state.
STATE_FILE = "/tmp/mixing_servo_jog.last_deg"


def _read_last_deg() -> Optional[int]:
    try:
        with open(STATE_FILE, "r") as f:
            return int(f.read().strip())
    except (FileNotFoundError, ValueError):
        return None


def _write_last_deg(deg: int) -> None:
    try:
        with open(STATE_FILE, "w") as f:
            f.write(str(deg))
    except OSError as exc:  # pragma: no cover - host fs path
        print(f"  warning: could not persist last angle to {STATE_FILE}: {exc}",
              file=sys.stderr)

# Conservative software clamp. The wire field is 12 bits (4095 max) but
# mechanical range is unknown; start narrow, raise with --max-deg if
# you've checked end-stops.
DEFAULT_MIN_DEG = 0
DEFAULT_MAX_DEG = 180

def _configure_bus_cmds(bitrate: int) -> tuple[str, ...]:
    return (
        "can off",
        f"conf set can.bitrate {bitrate}",
        "conf set can.fdcan_frame off",
        "conf set can.bitrate_switch on",
        "conf set can.termination on",
        "can on",
    )

# Servo board control_mode values, from
# ServoController._make_control_frame in the vendored library:
CONTROL_MODE_GO_TO = 0
CONTROL_MODE_STOP = 2
CONTROL_MODE_SET_HOME = 3


def _make_servo_frame(
    servo_id: int,
    control_mode: int,
    control_data: int,
    *,
    reset_home: int = 0,
    clear_faults: int = 0,
    query_data: int = 0,
) -> bytes:
    """Build the 32-byte servo-board command frame.

    Byte layout (mirrors ``ServoController._make_control_frame``):

        byte0 [7]=0, [6]=1 (servo command marker), [5:2]=servo_id,
              [1]=home (unused here), [0]=reset_home
        byte1 [7:6]=control_mode, [5:0]=control_data top 6 bits
        byte2 [7:2]=control_data bottom 6 bits, [1:0]=0
        byte3 [7]=clear_faults, [6]=query_data, [5:0]=0
        bytes 4..31 = 0 (FD-frame padding; the dongle has fdcan_frame
                         off, so only bytes 0..7 hit the wire)
    """
    if not 0 <= servo_id <= 15:
        raise ValueError(f"servo_id out of range 0..15: {servo_id}")
    if not 0 <= control_data < (1 << 12):
        raise ValueError(f"control_data out of 12-bit range: {control_data}")

    frame = bytearray(32)
    frame[0] = (1 << 6) | ((servo_id & 0x0F) << 2) | (reset_home & 0x01)
    frame[1] = ((control_mode & 0x03) << 6) | ((control_data >> 6) & 0x3F)
    frame[2] = (control_data & 0x3F) << 2
    frame[3] = ((clear_faults & 0x01) << 7) | ((query_data & 0x01) << 6)
    return bytes(frame)


@dataclasses.dataclass
class JogState:
    can_id: int
    servo_id: int
    last_commanded_deg: Optional[int] = None
    min_deg: int = DEFAULT_MIN_DEG
    max_deg: int = DEFAULT_MAX_DEG


class FdCanCli:
    """Thin wrapper around pyserial that speaks the fdcanusb ASCII CLI.

    Only the subset we need:
      - ``configure_bus()``  -- the six bring-up commands the vendored
        library sends.
      - ``send_can_std(std_id, data)`` -- one ``can std`` line.
      - ``rcv`` line tailer in a background thread, for visibility.
    """

    def __init__(
        self, port: str, baud: int = DEFAULT_BAUD,
        bitrate: int = DEFAULT_BITRATE,
        debug: bool = False,
    ) -> None:
        self._ser = serial.Serial(port, baudrate=baud, timeout=0.2)
        self._bitrate = bitrate
        self._debug = debug
        self._stop_reader = threading.Event()
        self._rcv_lock = threading.Lock()
        self._rcv_ids: set[int] = set()
        self._rcv_count = 0
        self._reader = threading.Thread(
            target=self._read_loop, name="fdcanusb-reader", daemon=True
        )
        self._reader.start()
        time.sleep(1.0)  # boot drain, mirrors FdCanInterface.open()
        self._drain()

    def close(self) -> None:
        self._stop_reader.set()
        # The reader thread is daemon=True; don't block on join, the
        # serial read with timeout will return on its own quickly.
        try:
            self._ser.close()
        except Exception:  # pragma: no cover - hardware path
            pass

    def _drain(self) -> None:
        # Ask the kernel to throw out anything buffered.
        try:
            self._ser.reset_input_buffer()
        except Exception:  # pragma: no cover - hardware path
            pass

    def _read_loop(self) -> None:
        # Daemon thread: pulls lines off the serial port and prints any
        # `rcv` (received CAN frame) lines. Useful for spotting whether
        # the board talks back at all -- nothing in this script decodes
        # them.
        buf = bytearray()
        while not self._stop_reader.is_set():
            try:
                chunk = self._ser.read(64)
            except Exception:
                return
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                line_b, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                line = line_b.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("rcv "):
                    print(f"  [bus] {line}")
                    parts = line.split()
                    if len(parts) >= 2:
                        try:
                            rcv_id = int(parts[1], 16)
                        except ValueError:
                            rcv_id = -1
                        with self._rcv_lock:
                            self._rcv_count += 1
                            if rcv_id >= 0:
                                self._rcv_ids.add(rcv_id)
                elif self._debug:
                    print(f"  [dbg] {line}")

    def reset_rcv_log(self) -> None:
        with self._rcv_lock:
            self._rcv_ids.clear()
            self._rcv_count = 0

    def rcv_summary(self) -> tuple[int, frozenset[int]]:
        with self._rcv_lock:
            return self._rcv_count, frozenset(self._rcv_ids)

    def _write_line(self, cmd: str) -> None:
        payload = (cmd + "\n").encode("ascii")
        self._ser.write(payload)
        self._ser.flush()

    def configure_bus(self) -> None:
        for cmd in _configure_bus_cmds(self._bitrate):
            self._write_line(cmd)
            # Caitlin's library waits for OK/ERR here; we just sleep
            # briefly and let the reader thread surface any ERR lines
            # if uncommented above. The dongle is fast enough that
            # 50 ms between bring-up commands is plenty.
            time.sleep(0.05)

    def send_can_std(self, std_id: int, data: bytes, flags: str = "FB") -> None:
        self._write_line(f"can std {std_id:X} {data.hex()} {flags}")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--port", default=DEFAULT_PORT,
                   help=f"serial device or by-id symlink (default: {DEFAULT_PORT})")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help="serial baud rate (USB-CDC: ignored)")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                   help=f"CAN bus bitrate (default: {DEFAULT_BITRATE}). "
                        "Use 1000000 for the current Astrotech rig.")
    p.add_argument("--can-id", type=int, default=DEFAULT_CAN_ID,
                   help=f"servo board CAN id (default: {DEFAULT_CAN_ID})")
    p.add_argument("--servo-id", type=int, default=DEFAULT_SERVO_ID,
                   help=f"servo id on that board, 0..15 (default: {DEFAULT_SERVO_ID})")
    p.add_argument("--min-deg", type=int, default=DEFAULT_MIN_DEG,
                   help=f"minimum commandable angle (default: {DEFAULT_MIN_DEG})")
    p.add_argument("--max-deg", type=int, default=DEFAULT_MAX_DEG,
                   help=f"maximum commandable angle (default: {DEFAULT_MAX_DEG})")
    p.add_argument("--no-configure", action="store_true",
                   help="skip the six configure_bus() commands "
                        "(use if another tool already brought the bus up)")
    p.add_argument("--probe", action="store_true",
                   help="non-interactive wire-level probe: send one "
                        "stop+query frame to (--can-id, --servo-id), "
                        "tail rcv lines for --probe-listen-s, print a "
                        "summary, and exit. Servo will not move.")
    p.add_argument("--probe-can-ids", type=str, default=None,
                   help="comma-separated list of CAN ids to probe "
                        "(overrides --can-id in --probe mode). "
                        "Example: 1,3,5,7")
    p.add_argument("--probe-listen-s", type=float, default=0.7,
                   help="seconds to listen for rcv replies after each "
                        "probe frame (default: 0.7)")
    p.add_argument("--nudge", type=str, default=None,
                   metavar="FROM,TO",
                   help="non-interactive visual-nudge sequence: "
                        "settle at FROM, step to TO, return to FROM, "
                        "with pauses you can watch. Example: 3,4")
    p.add_argument("--nudge-settle-s", type=float, default=1.5,
                   help="seconds to hold each nudge step (default: 1.5)")
    p.add_argument("--nudge-can-ids", type=str, default=None,
                   help="comma-separated list of CAN ids to sweep with "
                        "the --nudge sequence (overrides --can-id). "
                        "Hard-skips 15 and 16 (moteus auger controllers). "
                        "Example: 10,11,12,13,14,17,18,19,20")
    p.add_argument("--goto", type=int, default=None, metavar="N",
                   help="one-shot absolute move: send go_to_position(N) "
                        "to (--can-id, --servo-id), record N as the "
                        "new last-known angle, and exit.")
    p.add_argument("--rel", type=int, default=None, metavar="DELTA",
                   help="one-shot relative move: read last angle from "
                        f"{STATE_FILE}, send go_to_position(last+DELTA), "
                        "record the new angle, exit. Use --goto first "
                        "to set a baseline, then --rel +1 / --rel -1 "
                        "for calibration walks.")
    p.add_argument("--no-auto-stop", action="store_true",
                   help="skip the stop frame in the finally block. "
                        "Use during calibration if 'stop' on this servo "
                        "firmware turns out to release torque / let the "
                        "chamber droop. (Library docs don't say what it "
                        "does.) NOTE: --goto and --rel modes already "
                        "skip auto-stop by default; this flag is for "
                        "REPL / --nudge / --probe modes.")
    p.add_argument("--auto-stop", action="store_true",
                   help="force the stop frame in the finally block "
                        "even for --goto / --rel one-shot modes "
                        "(opposite of the new default).")
    p.add_argument("--debug", action="store_true",
                   help="surface every serial line the dongle sends "
                        "(OK / ERR / banners), not just rcv frames. "
                        "Use to verify configure_bus actually succeeded.")
    args = p.parse_args()
    if not 0 <= args.servo_id <= 15:
        p.error(f"--servo-id out of range 0..15: {args.servo_id}")
    if not 0 <= args.can_id <= 0x7FF:
        p.error(f"--can-id out of 11-bit range: {args.can_id}")
    if args.min_deg >= args.max_deg:
        p.error("--min-deg must be < --max-deg")
    return args


HELP_TEXT = """
commands:
  N             go to N degrees (clamped to --min-deg / --max-deg)
  +N            jog +N from last commanded angle
  -N            jog -N from last commanded angle
  id N          switch active servo_id to N (0..15)
  board N       switch active CAN id (servo board) to N
  stop          stop active servo (control_mode=2)
  home N        set the active servo's home to N degrees (persists board-side)
  ?, help       show this help
  q, quit, ^D   stop and exit
"""


def _cmd_goto(cli: FdCanCli, st: JogState, deg: int) -> None:
    if deg < st.min_deg or deg > st.max_deg:
        print(
            f"  refused: {deg} outside [{st.min_deg}, {st.max_deg}]. "
            "raise --max-deg / --min-deg if you really want to."
        )
        return
    # NOTE: clear_faults=1 is load-bearing, not optional. During the
    # 2026-05-08 bench bring-up we hit a state where the servo board
    # ACKed every goto on the bus but ignored them mechanically; setting
    # this bit unstuck the rig immediately and is the only path that has
    # been demonstrated to work after the bus has seen any other traffic
    # (moteus on the same fdcanusb, prior can off/can on cycles, etc.).
    # Cost is one bit on the wire; benefit is the rig never silently
    # eats commands again. Do not remove without on-hardware verification.
    frame = _make_servo_frame(
        st.servo_id, CONTROL_MODE_GO_TO, deg, clear_faults=1
    )
    cli.send_can_std(st.can_id, frame)
    st.last_commanded_deg = deg
    _write_last_deg(deg)
    ts = dt.datetime.now().strftime("%H:%M:%S")
    print(
        f"  {ts}  servo (can_id={st.can_id}, servo_id={st.servo_id}) "
        f"-> {deg} deg"
    )


def _cmd_stop(cli: FdCanCli, st: JogState) -> None:
    frame = _make_servo_frame(st.servo_id, CONTROL_MODE_STOP, 0)
    cli.send_can_std(st.can_id, frame)
    print(f"  stop sent to (can_id={st.can_id}, servo_id={st.servo_id})")


def _cmd_set_home(cli: FdCanCli, st: JogState, deg: int) -> None:
    if deg < 0 or deg >= (1 << 12):
        print(f"  refused: home {deg} out of 12-bit range")
        return
    frame = _make_servo_frame(
        st.servo_id, CONTROL_MODE_SET_HOME, deg, reset_home=1
    )
    cli.send_can_std(st.can_id, frame)
    print(
        f"  set_home {deg} deg -> servo "
        f"(can_id={st.can_id}, servo_id={st.servo_id})"
    )


def _probe_one(cli: FdCanCli, can_id: int, servo_id: int, listen_s: float) -> bool:
    """Send a stop+query frame to (can_id, servo_id) and listen briefly.

    Returns True if any ``rcv`` line landed on the bus during the listen
    window. Note: a True doesn't *prove* it was the servo we addressed
    (could be unsolicited traffic from another node); a False just means
    nothing replied, which is consistent with both "nothing there" and
    "the board ignores query_data=1". Read alongside a visual nudge.
    """
    cli.reset_rcv_log()
    frame = _make_servo_frame(
        servo_id, CONTROL_MODE_STOP, 0, query_data=1
    )
    print(
        f"  probe: stop+query -> can_id={can_id} servo_id={servo_id}, "
        f"listening {listen_s:.2f}s..."
    )
    cli.send_can_std(can_id, frame)
    time.sleep(listen_s)
    count, ids = cli.rcv_summary()
    if count == 0:
        print(f"  probe: silent on can_id={can_id}")
        return False
    print(
        f"  probe: saw {count} rcv frame(s) on ids "
        f"{sorted(hex(i) for i in ids)} after addressing can_id={can_id}"
    )
    return True


def _do_probe(cli: FdCanCli, args: argparse.Namespace) -> int:
    if args.probe_can_ids:
        try:
            candidates = [int(x.strip(), 0) for x in args.probe_can_ids.split(",")]
        except ValueError:
            print(
                f"  --probe-can-ids: could not parse {args.probe_can_ids!r}",
                file=sys.stderr,
            )
            return 2
    else:
        candidates = [args.can_id]

    print(
        f"probe mode: candidates={candidates}, servo_id={args.servo_id}, "
        f"listen={args.probe_listen_s}s. servo should NOT move."
    )
    print(
        "(reminder: per docs/astrotech_canfd_library_notes.md the boards "
        "may not reply to query_data=1; silence is inconclusive.)"
    )
    any_hit = False
    for can_id in candidates:
        if _probe_one(cli, can_id, args.servo_id, args.probe_listen_s):
            any_hit = True
    print()
    if any_hit:
        print("probe: at least one candidate produced rcv traffic. "
              "Pair with a visual nudge to confirm the right board.")
        return 0
    print("probe: silent across all candidates. Either:")
    print("  - the board doesn't reply to query_data=1 (most likely);")
    print("  - the board is on a can_id we didn't try; or")
    print("  - power/wiring issue.")
    print("Recommend a visual nudge: re-run without --probe and try "
          "`5` then `0` in the REPL on the candidate (can_id, servo_id).")
    return 1


MOTEUS_RESERVED_CAN_IDS = (15, 16)  # auger lead_screw + auger.


def _nudge_one(
    cli: FdCanCli, st: JogState, from_deg: int, to_deg: int, settle_s: float
) -> tuple[int, frozenset[int]]:
    """One settle/step/return cycle on whatever (can_id, servo_id) `st`
    currently points at. Returns the rcv summary captured during the
    cycle (count, ids)."""
    cli.reset_rcv_log()

    print(f"  step 1/3: settle at {from_deg}")
    _cmd_goto(cli, st, from_deg)
    time.sleep(settle_s)

    print(f"  step 2/3: step to {to_deg}  <-- watch for twitch")
    _cmd_goto(cli, st, to_deg)
    time.sleep(settle_s)

    print(f"  step 3/3: return to {from_deg}  <-- watch for return")
    _cmd_goto(cli, st, from_deg)
    time.sleep(settle_s)

    return cli.rcv_summary()


def _do_nudge(cli: FdCanCli, args: argparse.Namespace, st: JogState) -> int:
    try:
        from_s, to_s = args.nudge.split(",")
        from_deg = int(from_s)
        to_deg = int(to_s)
    except (ValueError, AttributeError):
        print(f"  --nudge: expected FROM,TO ints (got {args.nudge!r})",
              file=sys.stderr)
        return 2

    if args.nudge_can_ids:
        try:
            candidates = [int(x.strip(), 0) for x in args.nudge_can_ids.split(",")]
        except ValueError:
            print(
                f"  --nudge-can-ids: could not parse {args.nudge_can_ids!r}",
                file=sys.stderr,
            )
            return 2
    else:
        candidates = [st.can_id]

    skipped = [c for c in candidates if c in MOTEUS_RESERVED_CAN_IDS]
    safe = [c for c in candidates if c not in MOTEUS_RESERVED_CAN_IDS]
    if skipped:
        print(
            f"  hard-skipping moteus-reserved can_ids: {skipped} "
            f"(lead_screw=15, auger=16)"
        )
    if not safe:
        print("  no candidate can_ids left after safety filter; nothing to do.")
        return 2

    print(
        f"nudge sweep: candidates={safe}, servo_id={st.servo_id}, "
        f"FROM={from_deg} TO={to_deg}, settle={args.nudge_settle_s}s/step. "
        f"watch the rig and call out which can_id was active when it "
        f"twitched."
    )

    rcv_by_id: dict[int, tuple[int, frozenset[int]]] = {}
    for can_id in safe:
        st.can_id = can_id
        print()
        print(f"=== TESTING can_id={can_id} (servo_id={st.servo_id}) ===")
        count, ids = _nudge_one(
            cli, st, from_deg, to_deg, args.nudge_settle_s
        )
        rcv_by_id[can_id] = (count, ids)

    print()
    print("=== sweep summary (rcv traffic during each can_id's nudge) ===")
    for can_id in safe:
        count, ids = rcv_by_id[can_id]
        if count:
            print(f"  can_id={can_id:>2}: {count} rcv frame(s) on "
                  f"{sorted(hex(i) for i in ids)}")
        else:
            print(f"  can_id={can_id:>2}: silent")
    print()
    print("Visual is the source of truth; bus rcv is mostly informational "
          "(library docs note the boards may not reply to query_data=1).")
    return 0


def _repl(cli: FdCanCli, st: JogState) -> None:
    print(HELP_TEXT)
    print(
        f"active: can_id={st.can_id}  servo_id={st.servo_id}  "
        f"range=[{st.min_deg}, {st.max_deg}]"
    )
    while True:
        try:
            raw = input("jog> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not raw:
            continue
        head, _, tail = raw.partition(" ")
        head = head.lower()
        tail = tail.strip()

        if head in ("q", "quit", "exit"):
            return
        if head in ("?", "h", "help"):
            print(HELP_TEXT)
            continue
        if head == "stop" or head == "s":
            _cmd_stop(cli, st)
            continue
        if head == "id":
            try:
                new_id = int(tail)
            except ValueError:
                print("  usage: id N   (N in 0..15)")
                continue
            if not 0 <= new_id <= 15:
                print("  servo_id must be in 0..15")
                continue
            st.servo_id = new_id
            print(f"  active servo_id -> {st.servo_id}")
            continue
        if head == "board":
            try:
                new_can = int(tail)
            except ValueError:
                print("  usage: board N   (N is a CAN std id)")
                continue
            if not 0 <= new_can <= 0x7FF:
                print("  CAN id must be 0..0x7FF")
                continue
            st.can_id = new_can
            print(f"  active can_id -> {st.can_id}")
            continue
        if head == "home":
            try:
                deg = int(tail)
            except ValueError:
                print("  usage: home N   (N in degrees)")
                continue
            _cmd_set_home(cli, st, deg)
            continue

        # numeric: absolute or relative
        try:
            if raw.startswith(("+", "-")):
                if st.last_commanded_deg is None:
                    print("  no previous angle; type an absolute number first")
                    continue
                delta = int(raw)
                target = st.last_commanded_deg + delta
            else:
                target = int(raw)
        except ValueError:
            print(f"  unknown: {raw!r}. type ? for help.")
            continue
        _cmd_goto(cli, st, target)


def main() -> int:
    args = _parse_args()

    print(f"opening {args.port} @ {args.baud} (bus bitrate {args.bitrate})...")
    try:
        cli = FdCanCli(
            args.port, baud=args.baud, bitrate=args.bitrate, debug=args.debug
        )
    except serial.SerialException as exc:
        print(f"failed to open {args.port}: {exc}", file=sys.stderr)
        return 2

    st = JogState(
        can_id=args.can_id,
        servo_id=args.servo_id,
        min_deg=args.min_deg,
        max_deg=args.max_deg,
    )

    rc = 0
    try:
        if not args.no_configure:
            print("running configure_bus...")
            cli.configure_bus()
            print("bus up.")
        if args.probe:
            rc = _do_probe(cli, args)
        elif args.nudge:
            rc = _do_nudge(cli, args, st)
        elif args.goto is not None:
            _cmd_goto(cli, st, args.goto)
            # Hold open long enough for the servo to physically reach the
            # target. Without this, the auto-stop / port-close races the
            # mechanical move and (depending on firmware) can cancel it.
            # Heuristic: 200ms minimum, plus 50ms per degree of estimated
            # travel from the previous commanded angle.
            delta = abs(args.goto - (st.last_commanded_deg or args.goto))
            time.sleep(max(0.2, 0.2 + 0.05 * delta))
        elif args.rel is not None:
            last = _read_last_deg()
            if last is None:
                print(f"  no recorded last angle in {STATE_FILE}.",
                      file=sys.stderr)
                print("  run --goto N first to set a baseline, then "
                      "--rel +1 / --rel -1.", file=sys.stderr)
                rc = 2
            else:
                target = last + args.rel
                print(f"  rel: last={last}  delta={args.rel:+d}  -> {target}")
                _cmd_goto(cli, st, target)
                time.sleep(max(0.2, 0.2 + 0.05 * abs(args.rel)))
        else:
            _repl(cli, st)
    finally:
        # Default: send stop in REPL / --nudge / --probe modes, but NOT
        # in --goto / --rel one-shot modes (where stop would race the
        # move on slow firmware). --no-auto-stop forces skip everywhere;
        # --auto-stop forces send everywhere.
        one_shot = args.goto is not None or args.rel is not None
        send_stop = (not args.no_auto_stop) and (args.auto_stop or not one_shot)
        if send_stop:
            try:
                _cmd_stop(cli, st)
            except Exception as exc:  # pragma: no cover - hardware path
                print(f"warning: failed to send stop on shutdown: {exc}",
                      file=sys.stderr)
        cli.close()
        print("closed.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
