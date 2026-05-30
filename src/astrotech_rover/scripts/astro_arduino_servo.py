#!/usr/bin/env python3
"""Keyboard wrapper for the Arduino rack-and-pinion servo controller.

Upload ``astro_arduino_servo_controller.ino`` to the Arduino first. This
script only sends serial commands from the laptop.
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import serial


PORT = "/dev/ttyACM0"
BAUD = 115200


def send_command(arduino: serial.Serial, command: str) -> None:
    arduino.write(f"{command}\n".encode("ascii"))


def read_key(timeout_s: float = 0.03) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not ready:
        return None
    return sys.stdin.read(1)


def print_help() -> None:
    print("connected to Arduino.")
    print("commands:")
    print("  1  move to all-the-way-left")
    print("  2  move to startup position")
    print("  3  move to position 3")
    print("  4  move to all-the-way-right")
    print("  s  stop")
    print("  q  quit")
    print()


def main() -> int:
    arduino = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(2)  # Arduino resets when serial opens.

    old_tty_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        print_help()

        while True:
            key = read_key()

            if key in ("1", "2", "3", "4"):
                send_command(arduino, key)
                print(f"sent position {key}")

            elif key in ("s", "S", " "):
                send_command(arduino, "S")
                print("stop")

            elif key in ("q", "Q"):
                send_command(arduino, "S")
                print("quit")
                return 0

            elif key == "\x03":
                raise KeyboardInterrupt

            if arduino.in_waiting:
                response = arduino.read(arduino.in_waiting).decode(
                    "utf-8", errors="replace"
                )
                print(response, end="")

    except KeyboardInterrupt:
        send_command(arduino, "S")
        print("\nclosing serial connection.")
        return 130

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty_settings)
        arduino.close()
        print("closed.")


if __name__ == "__main__":
    raise SystemExit(main())
