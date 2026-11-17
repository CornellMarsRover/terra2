#!/usr/bin/env python3
"""Fake fdcanusb on a pty: answers every 'can send' line with OK and logs it.
usage: fake_fdcanusb.py <symlink path> <log file> [ok|mute|stall] [responding ids, e.g. 9,10,11]
  mute:  reads and logs frames but never answers (dead / wrong tty)
  stall: never reads, so the node's tty writes eventually block"""
import os, pty, struct, sys, time, tty, termios, select
from moteus.fdcanusb_device import _append_checksum, _strip_and_validate_checksum

link, logpath = sys.argv[1], sys.argv[2]
mode = sys.argv[3] if len(sys.argv) > 3 else "ok"
responders = {int(x) for x in sys.argv[4].split(",")} if len(sys.argv) > 4 else None
master, slave = pty.openpty()
tty.setraw(slave)
tty.setraw(master)
try:
    os.unlink(link)
except FileNotFoundError:
    pass
os.makedirs(os.path.dirname(link) or ".", exist_ok=True)
os.symlink(os.ttyname(slave), link)
log = open(logpath, "w", buffering=1)
buf = b""
while mode == "stall":
    time.sleep(1.0)
while True:
    r, _, _ = select.select([master], [], [], 1.0)
    if not r:
        continue
    try:
        data = os.read(master, 4096)
    except OSError:
        time.sleep(0.01)
        continue
    buf += data
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        raw, had_ck, _ok = _strip_and_validate_checksum(line.strip())
        line = raw.decode(errors="replace").strip()
        if not line:
            continue
        log.write(f"{time.monotonic():.4f} {line}\n")

        if mode == "mute":
            continue

        def send(text):
            out = text.encode()
            if had_ck:
                out = _append_checksum(out)
            os.write(master, out + b"\r\n")

        send("OK")
        p = line.split()
        if len(p) >= 4 and p[0] == "can" and p[1] == "send":
            arb = int(p[2], 16)
            if arb & 0x8000:  # reply requested: fake servo telemetry
                mid = arb & 0x7F
                if responders is not None and mid not in responders:
                    continue  # no such servo on this bus
                # mode(int8)=0, then F32 regs 0x01..0x06: position, velocity, torque,
                # q_current, d_current, abs_position
                payload = bytes([0x21, 0x00, 0x00, 0x2C, 0x06, 0x01]) + struct.pack(
                    "<6f", 0.5 * mid, 0.0, 0.0, 0.0, 0.0, 0.125 * mid)
                send(f"rcv {mid << 8:04X} {payload.hex().upper()}")
