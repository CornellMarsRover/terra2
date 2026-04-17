import asyncio
import sys
import tty
import termios
from CMR_CANFD import FdCanInterface, ServoController

# --- Servo position state ---
state = {
    'ee13': 0,
    'ee14': 0,
    'ee15': 90,
}

STEP = 5
EE15_MIN = 40
EE15_MAX = 150

def process_key(k):
    if k == '1':
        state['ee13'] = 180
    elif k == '2':
        state['ee13'] = 0
    elif k == '3':
        state['ee14'] = 180
    elif k == '4':
        state['ee14'] = 0
    elif k == '6':
        state['ee15'] = min(EE15_MAX, state['ee15'] + STEP)
    elif k == '5':
        state['ee15'] = max(EE15_MIN, state['ee15'] - STEP)

async def read_keys():
    loop = asyncio.get_event_loop()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    key_queue = asyncio.Queue()

    def on_key():
        import os
        ch = os.read(fd, 1).decode('utf-8', errors='ignore')
        loop.call_soon_threadsafe(key_queue.put_nowait, ch)

    loop.add_reader(fd, on_key)
    try:
        while True:
            ch = await key_queue.get()
            if ch == '\x03':  # Ctrl+C
                raise KeyboardInterrupt
            process_key(ch)
    finally:
        loop.remove_reader(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

async def servo_loop(ee13, ee14, ee15):
    while True:
        await ee13.go_to_position(state['ee13'])
        await ee14.go_to_position(state['ee14'])
        await ee15.go_to_position(state['ee15'])

        print(
            f"ee13: {state['ee13']:3d}°  "
            f"ee14: {state['ee14']:3d}°  "
            f"ee15: {state['ee15']:3d}°    ",
            end='\r'
        )

        await asyncio.sleep(0.05)  # 20 Hz

async def main():
    fd = FdCanInterface(port="/dev/tty.usbmodemF5914A041", baud=115200)
    await fd.open()
    await fd.configure_bus()

    # Use can_id=24 because that is what your working script uses
    ee13 = ServoController(can=fd, servo_id=13, can_id=24)
    ee14 = ServoController(can=fd, servo_id=14, can_id=24)
    ee15 = ServoController(can=fd, servo_id=15, can_id=24)

    print("=======================================")
    print("  SERVO KEYBOARD CONTROL               ")
    print("=======================================")
    print("  1/2 : ee13 -> 180° / 0°              ")
    print("  3/4 : ee14 -> 180° / 0°              ")
    print("  5/6 : ee15 +5 / -5                   ")
    print(f"  ee15 range: {EE15_MIN} to {EE15_MAX} degrees")
    print(f"  Step size: {STEP} degrees            ")
    print("  Ctrl+C to exit                       ")
    print("=======================================\n")

    try:
        await asyncio.gather(
            read_keys(),
            servo_loop(ee13, ee14, ee15),
        )
    except KeyboardInterrupt:
        pass
    finally:
        await fd.close()
        print("\nStopped and closed the interface.")

if __name__ == "__main__":
    asyncio.run(main())