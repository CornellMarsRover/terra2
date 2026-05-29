#!/usr/bin/env python3

import argparse
import time
import serial
import socket
import struct

# Run:
# python mini_arm_control.py --serial-port COM9 --baud-rate 115200
# python mini_arm_control.py --serial-port /dev/ttyACM0 --baud-rate 115200

DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT = 1.0
DEFAULT_UDP_IP = "192.168.1.69"
DEFAULT_UDP_PORT = 5030
DEFAULT_SEND_RATE_HZ = 30


def parse_args():
    parser = argparse.ArgumentParser(
        description="Forward mini-arm serial joint data to the Jetson over UDP."
    )
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baud-rate", type=int, default=DEFAULT_BAUD_RATE)
    parser.add_argument("--udp-ip", default=DEFAULT_UDP_IP)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument("--send-rate-hz", type=float, default=DEFAULT_SEND_RATE_HZ)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument(
        "--zero-on-start",
        action="store_true",
        help="Use the first valid serial reading as an offset and send relative angles.",
    )
    return parser.parse_args()


def parse_joint_data(line):
    try:
        parts = line.strip().split("|")
        if len(parts) != 6:
            raise ValueError(f"expected 6 values, got {len(parts)}")
        return [float(part) for part in parts]
    except ValueError as exc:
        print(f"[WARN] Failed to parse line: {line.strip()!r} ({exc})")
        return None


def main():
    args = parse_args()
    period = 1.0 / args.send_rate_hz if args.send_rate_hz > 0 else 0.0

    try:
        ser = serial.Serial(args.serial_port, args.baud_rate, timeout=args.timeout)
        time.sleep(2.0)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {args.serial_port}: {e}")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    offsets = None
    destination = (args.udp_ip, args.udp_port)

    print(
        f"[INFO] Connected to mini arm on {args.serial_port} @ {args.baud_rate}"
    )
    print(f"[INFO] Sending UDP packets to {args.udp_ip}:{args.udp_port}")

    try:
        while True:
            line = ser.readline().decode(errors="ignore")
            if not line:
                continue

            joint_values = parse_joint_data(line)
            if joint_values is None:
                continue

            if args.zero_on_start:
                if offsets is None:
                    offsets = [round(value, 3) for value in joint_values]
                    print(f"[INFO] Offsets set to: {offsets}")
                    continue
                joint_values = [
                    round(value - offset, 3)
                    for value, offset in zip(joint_values, offsets)
                ]

            payload = struct.pack("6f", *joint_values)
            sock.sendto(payload, destination)
            print(f"[SEND] {joint_values}")

            if period:
                time.sleep(period)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        ser.close()
        sock.close()


if __name__ == "__main__":
    main()
