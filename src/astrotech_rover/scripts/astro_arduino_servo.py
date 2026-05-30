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

# PORT = "/dev/ttyACM0"
PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536373332351306182-if00"
BAUD = 115200
RELEASE_TIMEOUT_S = 0.22
JOG_KEEPALIVE_S = 0.08


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
    print("  1  move to drill/CO2a")
    print("  2  move to CO2b")
    print("  3  move to startup position")
    print("  4  move to position 4")
    print("  5  move to position 5")
    print("  hold -  manual jog left")
    print("  hold =  manual jog right")
    print("  r  reset software position to 3")
    print("  s  stop")
    print("  q  quit")
    print()


def main() -> int:
    arduino = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(2)  # Arduino resets when serial opens.

    old_tty_settings = termios.tcgetattr(sys.stdin)
    active_jog_command: str | None = None
    last_jog_key_time: float | None = None
    last_jog_send_time: float | None = None

    try:
        tty.setcbreak(sys.stdin.fileno())
        print_help()

        while True:
            now = time.monotonic()
            key = read_key()

            if key in ("1", "2", "3", "4", "5"):
                send_command(arduino, key)
                active_jog_command = None
                last_jog_key_time = None
                last_jog_send_time = None
                print(f"sent position {key}")

            elif key == "-":
                if active_jog_command != "L":
                    print("manual jog left")
                send_command(arduino, "L")
                active_jog_command = "L"
                last_jog_key_time = now
                last_jog_send_time = now

            elif key in ("+", "="):
                if active_jog_command != "+":
                    print("manual jog right")
                send_command(arduino, "+")
                active_jog_command = "+"
                last_jog_key_time = now
                last_jog_send_time = now

            elif key in ("r", "R"):
                send_command(arduino, "R")
                active_jog_command = None
                last_jog_key_time = None
                last_jog_send_time = None
                print("reset software position to 3")

            elif key in ("s", "S", " "):
                send_command(arduino, "S")
                active_jog_command = None
                last_jog_key_time = None
                last_jog_send_time = None
                print("stop")

            elif key in ("q", "Q"):
                send_command(arduino, "S")
                print("quit")
                return 0

            elif key == "\x03":
                raise KeyboardInterrupt

            if (
                active_jog_command is not None
                and last_jog_key_time is not None
                and now - last_jog_key_time > RELEASE_TIMEOUT_S
            ):
                send_command(arduino, "S")
                active_jog_command = None
                last_jog_key_time = None
                last_jog_send_time = None
                print("stop")
            elif (
                active_jog_command is not None
                and last_jog_send_time is not None
                and now - last_jog_send_time > JOG_KEEPALIVE_S
            ):
                send_command(arduino, active_jog_command)
                last_jog_send_time = now

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
