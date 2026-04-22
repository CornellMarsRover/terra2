#!/usr/bin/env python3

"""Read a Sony-style game controller on the host and stream rover packets over UDP.

This script is meant to run on the laptop host (macOS), while ROS 2 / Gazebo run
inside the devcontainer. It reuses the repo's existing
`cmr_controller_remote/connect.py` listener instead of adding a second container-side
teleop receiver.
"""

import argparse
import socket
import sys
import time

try:
    import pygame
except ImportError as exc:  # pragma: no cover - host-side helper
    raise SystemExit(
        "pygame is required for sony_controller_udp.py. Install it with "
        "`python3 -m pip install pygame` on the host."
    ) from exc


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return scaled if value > 0 else -scaled


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def axis_to_packet_byte(value: float) -> int:
    """Convert a joystick axis in [-1, 1] to the controller UDP byte format."""
    normalized = clamp(value, -1.0, 1.0)
    return int(round(((normalized + 1.0) / 2.0) * 255.0))


def main() -> int:
    parser = argparse.ArgumentParser(description="Sony controller UDP teleop for rover/Gazebo")
    parser.add_argument("--host", default="127.0.0.1", help="UDP host running cmr_controller_remote connect_node")
    parser.add_argument("--port", type=int, default=5010, help="UDP port")
    parser.add_argument("--rate", type=float, default=20.0, help="Send rate in Hz")
    parser.add_argument("--deadzone", type=float, default=0.12, help="Joystick deadzone")
    parser.add_argument("--left-x-axis", type=int, default=0, help="Joystick axis index for left-stick X")
    parser.add_argument("--left-y-axis", type=int, default=1, help="Joystick axis index for left-stick Y")
    parser.add_argument("--right-x-axis", type=int, default=2, help="Joystick axis index for right-stick X")
    parser.add_argument("--right-y-axis", type=int, default=3, help="Joystick axis index for right-stick Y")
    parser.add_argument(
        "--dump-axes",
        action="store_true",
        help="Print all raw joystick axis values so the controller mapping can be calibrated.",
    )
    parser.add_argument("--slow-scale", type=float, default=0.35, help="Scale applied while L1 is held")
    parser.add_argument("--boost-scale", type=float, default=1.35, help="Scale applied while R1 is held")
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise SystemExit("No game controller detected.")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.rate

    print(f"Using controller: {joystick.get_name()}")
    print(f"Sending rover-format UDP packets to {args.host}:{args.port}")
    print(
        "Controls: left stick Y = forward/back, right stick X = turn, "
        "L1 slow, R1 boost, Cross/A stop"
    )

    try:
        while True:
            pygame.event.pump()

            left_x = apply_deadzone(joystick.get_axis(args.left_x_axis), args.deadzone)
            left_y = apply_deadzone(joystick.get_axis(args.left_y_axis), args.deadzone)
            right_x = apply_deadzone(joystick.get_axis(args.right_x_axis), args.deadzone)
            right_y = apply_deadzone(joystick.get_axis(args.right_y_axis), args.deadzone)
            raw_axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]

            axis_scale = 1.0

            # Shoulder buttons vary slightly by controller mapping, but these are
            # common for Sony controllers in SDL / pygame.
            slow_button = joystick.get_button(4) if joystick.get_numbuttons() > 4 else 0
            boost_button = joystick.get_button(5) if joystick.get_numbuttons() > 5 else 0
            cross_button = joystick.get_button(0) if joystick.get_numbuttons() > 0 else 0
            circle_button = joystick.get_button(1) if joystick.get_numbuttons() > 1 else 0
            triangle_button = joystick.get_button(2) if joystick.get_numbuttons() > 2 else 0
            square_button = joystick.get_button(3) if joystick.get_numbuttons() > 3 else 0
            l2_button = joystick.get_button(6) if joystick.get_numbuttons() > 6 else 0
            r2_button = joystick.get_button(7) if joystick.get_numbuttons() > 7 else 0

            if slow_button:
                axis_scale *= args.slow_scale
            if boost_button:
                axis_scale *= args.boost_scale

            if cross_button:
                left_x = 0.0
                left_y = 0.0
                right_x = 0.0
                right_y = 0.0
            else:
                left_x *= axis_scale
                left_y *= axis_scale
                right_x *= axis_scale
                right_y *= axis_scale

            # Button bytes are ordered to match cmr_controller_remote/connect.py:
            # [l1, r1, l2, r2, square, cross, circle, triangle]
            packet = bytes(
                [
                    axis_to_packet_byte(left_x),
                    axis_to_packet_byte(left_y),
                    axis_to_packet_byte(right_x),
                    axis_to_packet_byte(right_y),
                    int(bool(slow_button)),
                    int(bool(boost_button)),
                    int(bool(l2_button)),
                    int(bool(r2_button)),
                    int(bool(square_button)),
                    int(bool(cross_button)),
                    int(bool(circle_button)),
                    int(bool(triangle_button)),
                ]
            ) + b"neutral\x00"

            sock.sendto(packet, (args.host, args.port))

            raw_axis_suffix = ""
            if args.dump_axes:
                formatted_axes = " ".join(f"a{i}={value:+.3f}" for i, value in enumerate(raw_axes))
                raw_axis_suffix = f" raw[{formatted_axes}]"

            sys.stdout.write(
                f"\rlx={left_x:+.3f} ly={left_y:+.3f} rx={right_x:+.3f} ry={right_y:+.3f} "
                f"slow={int(bool(slow_button))} boost={int(bool(boost_button))}{raw_axis_suffix}   "
            )
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping controller teleop.")
        stop_packet = bytes([127, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00"
        sock.sendto(stop_packet, (args.host, args.port))
        return 0
    finally:
        joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    raise SystemExit(main())
