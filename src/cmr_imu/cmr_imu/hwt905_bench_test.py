#!/usr/bin/env python3
"""Bench utilities for identifying HWT905 serial protocol output."""

import argparse
import glob
import os
import time
from typing import Iterable, List, Optional

import serial


DEFAULT_BAUD = 9600
DEFAULT_BAUDS = [4800, 9600, 19200, 38400, 57600, 115200, 230400]
WIT_FRAME_TYPES = {
    0x51: "accel",
    0x52: "gyro",
    0x53: "angle",
    0x54: "mag",
}
MODBUS_PROBE = bytes.fromhex("50 03 00 30 00 29 88 50")


def discover_ports() -> List[str]:
    ports = []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        ports.extend(sorted(glob.glob(pattern)))
    return list(dict.fromkeys(ports))


def print_ports() -> None:
    ports = discover_ports()
    if not ports:
        print("No /dev/serial/by-id, /dev/ttyUSB, or /dev/ttyACM devices found.")
        return

    print("Detected serial devices:")
    for port in ports:
        target = os.path.realpath(port) if os.path.islink(port) else port
        print(f"  {port} -> {target}")
    print("Prefer a /dev/serial/by-id path when available.")


def read_raw(port: str, baud: int, duration: float, chunk_size: int, output_path: Optional[str]) -> bytes:
    collected = bytearray()
    start = time.time()

    with serial.Serial(port, baud, timeout=1) as serial_port:
        print(f"Reading {port} at {baud} baud for {duration:.1f}s.")
        while time.time() - start < duration:
            data = serial_port.read(chunk_size)
            if not data:
                continue
            collected.extend(data)
            print(data.hex(" "))

    if output_path:
        with open(output_path, "w", encoding="utf-8") as output_file:
            for offset in range(0, len(collected), chunk_size):
                output_file.write(collected[offset : offset + chunk_size].hex(" ") + "\n")
        print(f"Saved raw hex capture to {output_path}")

    summarize(collected)
    return bytes(collected)


def scan_bauds(port: str, bauds: Iterable[int], duration: float) -> None:
    for baud in bauds:
        print(f"=== {baud} ===")
        try:
            data = read_once(port, baud, duration)
            print(data.hex(" ") if data else "<no data>")
            summarize(data)
        except serial.SerialException as exc:
            print(f"Serial error: {exc}")


def read_once(port: str, baud: int, duration: float) -> bytes:
    collected = bytearray()
    end = time.time() + duration
    with serial.Serial(port, baud, timeout=1) as serial_port:
        while time.time() < end:
            data = serial_port.read(128)
            if data:
                collected.extend(data)
    return bytes(collected)


def probe_modbus(port: str, baud: int) -> bytes:
    with serial.Serial(port, baud, timeout=1) as serial_port:
        serial_port.reset_input_buffer()
        serial_port.write(MODBUS_PROBE)
        time.sleep(0.2)
        data = serial_port.read(128)

    print(data.hex(" ") if data else "<no data>")
    if len(data) >= 3 and data[0] == 0x50 and data[1] == 0x03:
        print("Likely WIT 485/Modbus response: starts with 50 03.")
    else:
        print("No obvious 50 03 Modbus response detected.")
    return data


def summarize(data: bytes) -> None:
    counts = {frame_type: 0 for frame_type in WIT_FRAME_TYPES}
    checksum_valid = 0
    checksum_invalid = 0

    for index in range(0, max(0, len(data) - 10)):
        if data[index] != 0x55:
            continue
        frame = data[index : index + 11]
        if len(frame) < 11 or frame[1] not in WIT_FRAME_TYPES:
            continue
        counts[frame[1]] += 1
        if (sum(frame[:10]) & 0xFF) == frame[10]:
            checksum_valid += 1
        else:
            checksum_invalid += 1

    found = [f"55 {frame_type:02x} ({name}) x{count}" for frame_type, name in WIT_FRAME_TYPES.items() if (count := counts[frame_type])]
    if found:
        print("WIT standard frame candidates: " + ", ".join(found))
        print(f"Checksum-valid frames: {checksum_valid}; checksum-invalid candidates: {checksum_invalid}")
        if checksum_valid > 0:
            print("Likely HWT905-RS232/WIT standard stream.")
    else:
        print("No 55 51/52/53/54 WIT standard frame candidates found.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Identify whether an HWT905 is streaming WIT RS232 frames or responding as WIT 485/Modbus."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port to read, preferably /dev/serial/by-id/...")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate.")
    parser.add_argument("--duration", type=float, default=10.0, help="Seconds to read for raw captures.")
    parser.add_argument("--chunk-size", type=int, default=64, help="Bytes to read and print per line.")
    parser.add_argument("--output", help="Optional path for saving raw hex output.")
    parser.add_argument("--list-ports", action="store_true", help="List likely serial devices and exit.")
    parser.add_argument("--scan-baud", action="store_true", help="Try common baud rates and summarize any data.")
    parser.add_argument("--modbus-probe", action="store_true", help="Send the current 485 driver's read-register probe.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_ports:
        print_ports()
        return

    if args.scan_baud:
        scan_bauds(args.port, DEFAULT_BAUDS, min(args.duration, 3.0))
        return

    if args.modbus_probe:
        probe_modbus(args.port, args.baud)
        return

    read_raw(args.port, args.baud, args.duration, args.chunk_size, args.output)


if __name__ == "__main__":
    main()
