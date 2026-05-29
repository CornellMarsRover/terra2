import argparse
import asyncio
import sys
import termios
import tty

from .end_effector_servo_mapper import (
    SERVO_CAN_IDS,
    SERVO_EE15,
    SERVO_IDS,
    SERVO_MAX,
    SERVO_MIN,
)


DEFAULT_CAN_PORT = "/dev/serial/by-id/usb-mjbots_fdcanusb_062C45D7-if00"
DEFAULT_BAUD = 115200


async def _run(args):
    from .CMR_CANFD import FdCanInterface, ServoController

    if not sys.stdin.isatty():
        raise RuntimeError("simple_gripper_keyboard must be run from a terminal")

    close_degrees = args.close_degrees
    open_degrees = args.open_degrees

    fd = FdCanInterface(port=args.port, baud=args.baud)
    servo = ServoController(
        can=fd,
        servo_id=SERVO_IDS[SERVO_EE15],
        can_id=SERVO_CAN_IDS[SERVO_EE15],
    )

    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)

    try:
        print(f"Opening CAN interface on {args.port} at {args.baud} baud")
        await fd.open()
        await fd.configure_bus()

        tty.setcbreak(stdin_fd)
        print(
            "\nSimple gripper keyboard control\n"
            f"  + : open to {open_degrees} deg\n"
            f"  - : close to {close_degrees} deg\n"
            "  q : quit\n"
        )

        while True:
            key = sys.stdin.read(1)
            if key == "+":
                print(f"open -> {open_degrees} deg")
                await servo.go_to_position(open_degrees)
            elif key == "-":
                print(f"close -> {close_degrees} deg")
                await servo.go_to_position(close_degrees)
            elif key.lower() == "q":
                break
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
        await fd.close()


def build_parser():
    parser = argparse.ArgumentParser(
        description="Keyboard control for only the end-effector gripper servo.",
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_CAN_PORT,
        help="Serial port for the mjbots usbcanfd device",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help="Serial baud rate for the mjbots usbcanfd device",
    )
    parser.add_argument(
        "--open-degrees",
        type=int,
        default=SERVO_MAX[SERVO_EE15],
        help="Angle sent when + is pressed",
    )
    parser.add_argument(
        "--close-degrees",
        type=int,
        default=SERVO_MIN[SERVO_EE15],
        help="Angle sent when - is pressed",
    )
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        asyncio.run(_run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
