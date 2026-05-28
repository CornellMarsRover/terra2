import asyncio
import sys
import tty
import termios
from CMR_CANFD import FdCanInterface, ServoController

# --- Servo position state ---
state = {
    'ee0': 101,
    'ee11': 100,
    'ee15': 95,
}

STEP = 5
EE15_MIN = 40
EE15_MAX = 150
EE14_MIN = 60
EE14_MAX = 150

def process_key(k):
    if k == '1':
        state['ee0'] = 0
    elif k == '2':
        state['ee0'] = 180
    elif k == '3':
        state['ee11'] = max(EE14_MIN, state['ee11'] - STEP)
    elif k == '4':
        state['ee11'] = min(EE14_MAX, state['ee11'] + STEP)
    elif k == '0':
        state['ee11'] = 100
    elif k == '9':
        state['ee0'] = 101
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

async def servo_loop(ee0, ee11, ee15):
    last = {'ee0': None, 'ee11': None, 'ee15': None}
    keep_alive = 0
    while True:
        changed = any(state[k] != last[k] for k in last)
        keep_alive -= 1
        if changed or keep_alive <= 0:
            await ee0.go_to_position(state['ee0'])
            await ee11.go_to_position(state['ee11'])
            await ee15.go_to_position(state['ee15'])
            last.update(state)
            keep_alive = 20  # resend every ~2s when idle

        print(
            f"ee0: {state['ee0']:3d}°  "
            f"ee11: {state['ee11']:3d}°  "
            f"ee15: {state['ee15']:3d}°  ",
            end='\r'
        )

        await asyncio.sleep(0.1)  # 10 Hz poll

async def main():
    fd = FdCanInterface(port="/dev/tty.usbmodemD9E9AF951", baud=115200)
    await fd.open()
    await fd.configure_bus()

    # Board 1 (can_id=25): ee0, ee11, ee15
    ee0 = ServoController(can=fd, servo_id=0, can_id=25)
    ee11 = ServoController(can=fd, servo_id=11, can_id=25)
    ee15 = ServoController(can=fd, servo_id=15, can_id=25)

    print("=======================================")
    print("  SERVO KEYBOARD CONTROL               ")
    print("=======================================")
    print("  1/2 : ee0  -> 0° / 180°  (board id25) ")
    print("  9   : ee0  -> 93°        (board id25) ")
    print("  3/4 : ee11 -5 / +5       (board id25) ")
    print("  0   : ee11 -> 100°       (board id25) ")
    print("  5/6 : ee15 -5 / +5       (board id25) ")
    print(f"  ee15 range: {EE15_MIN} to {EE15_MAX} degrees")
    print(f"  Step size: {STEP} degrees            ")
    print("  Ctrl+C to exit                       ")
    print("=======================================\n")

    try:
        await asyncio.gather(
            read_keys(),
            servo_loop(ee0, ee11, ee15),
        )
    except KeyboardInterrupt:
        pass
    finally:
        await fd.close()
        print("\nStopped and closed the interface.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
