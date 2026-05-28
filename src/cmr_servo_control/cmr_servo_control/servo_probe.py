import argparse
import asyncio

from .CMR_CANFD import FdCanInterface, ServoController

#test bench to probe/bump servos
#ex:ros2 run cmr_servo_control servo_probe 8 --angle 85
#ex:ros2 run cmr_servo_control servo_probe 13 --angle 180


async def _run_probe(args):
    fd = FdCanInterface(port=args.port, baud=args.baud)
    fd.rx_callback = _handle_rx_frame

    print(f'Opening CAN interface on {args.port} at {args.baud} baud')
    await fd.open()
    await fd.configure_bus()

    read_loop_task = asyncio.create_task(fd.read_loop())

    try:
        servo = ServoController(
            can=fd,
            servo_id=args.servo_id,
            can_id=args.can_id,
        )

        if args.set_home is not None:
            print(f'Setting servo ID {args.servo_id} home to {args.set_home} degrees')
            await servo.set_home(args.set_home)
            await asyncio.sleep(args.pause)

        if args.stop:
            print(f'Sending STOP to servo ID {args.servo_id}')
            await servo.stop()
            await asyncio.sleep(args.pause)
            return

        if args.repeat > 1:
            print(
                f'Pulsing servo ID {args.servo_id} to {args.angle} degrees '
                f'{args.repeat} times with {args.pause:.2f}s pause'
            )
        else:
            print(f'Commanding servo ID {args.servo_id} to {args.angle} degrees')

        for index in range(args.repeat):
            await servo.go_to_position(args.angle)
            if args.repeat > 1 and index < args.repeat - 1:
                await asyncio.sleep(args.pause)

        if args.hold > 0.0:
            print(f'Waiting {args.hold:.2f}s for any RX traffic')
            await asyncio.sleep(args.hold)
    finally:
        read_loop_task.cancel()
        try:
            await read_loop_task
        except asyncio.CancelledError:
            pass
        await fd.close()


def _handle_rx_frame(can_id: int, data_hex: str, flags: list[str]):
    print(f'Probe RX frame ID=0x{can_id:X} data={data_hex} flags={flags}')


def build_parser():
    parser = argparse.ArgumentParser(
        description='Probe a raw servo ID on the CMR servo board.',
    )
    parser.add_argument('servo_id', type=int, help='Logical servo ID to command, e.g. 8, 13, or 14')
    parser.add_argument(
        '--angle',
        type=int,
        default=90,
        help='Target angle in degrees for the probe command',
    )
    parser.add_argument(
        '--can-id',
        type=int,
        default=24,
        help='CAN ID of the servo board',
    )
    parser.add_argument(
        '--port',
        default='/dev/ttyACM0',
        help='Serial port for the usbcanfd device',
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=115200,
        help='Serial baud rate for the usbcanfd device',
    )
    parser.add_argument(
        '--repeat',
        type=int,
        default=1,
        help='How many times to resend the same command',
    )
    parser.add_argument(
        '--pause',
        type=float,
        default=0.5,
        help='Pause in seconds between repeated commands',
    )
    parser.add_argument(
        '--hold',
        type=float,
        default=1.0,
        help='How long to keep listening for RX traffic after sending commands',
    )
    parser.add_argument(
        '--stop',
        action='store_true',
        help='Send a stop command instead of a position command',
    )
    parser.add_argument(
        '--set-home',
        type=int,
        default=None,
        help='Optionally set a new home angle before the main command',
    )
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    asyncio.run(_run_probe(args))


if __name__ == '__main__':
    main()
