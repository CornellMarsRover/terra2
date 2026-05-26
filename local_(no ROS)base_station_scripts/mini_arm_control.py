#!/usr/bin/env python3

import time
import serial
import socket
import struct

# ─────── CONFIG ───────
SERIAL_PORT = 'COM7'      # Windows serial port
BAUD_RATE   = 9600
TIMEOUT     = 1           # seconds

UDP_IP      = '192.168.1.69' # or wherever the ROS2 node is listening
UDP_PORT    = 5030        # must match the ROS2 package's UDP port
# ──────────────────────

def main():
    # open serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(2.0)  # let the port stabilize
    except serial.SerialException as e:
        print(f"[ERROR] could not open {SERIAL_PORT}: {e}")
        return

    # open UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    offsets = None
    print(f"Listening on {SERIAL_PORT}@{BAUD_RATE}, sending binary floats to UDP {UDP_IP}:{UDP_PORT}")

    while True:
        raw = ser.readline().decode('utf-8', errors='ignore').strip()
        if not raw:
            continue

        parts = raw.split('|')
        if len(parts) != 6:
            print(f"[WARN] bad packet (expected 6 vals): {raw!r}")
            continue

        try:
            readings = [float(p) for p in parts]
        except ValueError:
            print(f"[WARN] non-float data: {parts!r}")
            continue

        # first valid reading → capture offsets (rounded)
        if offsets is None:
            offsets = [round(v, 3) for v in readings]
            print(f"[INFO] offsets set to: {offsets}")
            continue

        # zero-offset & round
        zeroed = [round(r - off, 3) for r, off in zip(readings, offsets)]

        # pack as six 32-bit floats (native byte order, no padding)
        payload = struct.pack('6f', *zeroed)

        # send the raw 24 bytes
        sock.sendto(payload, (UDP_IP, UDP_PORT))
        print(f"[SEND] raw={zeroed}")

if __name__ == '__main__':
    main()
