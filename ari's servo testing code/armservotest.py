#!/usr/bin/env python3

import asyncio
import moteus
import math
import sys
import os
import tty
import termios

# Add CMR_CANFD package path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "servoboard_and_bdc_test_python_code 2/test"))
from CMR_CANFD import FdCanInterface, ServoController

SERIAL_PORT = "/dev/tty.usbmodemD8957AFF1"
SERVO_BOARD_CAN_ID = 24


# --------------------------------------------------------------------------
# Bus configuration — fdcan_frame ON so moteus FD frames are supported
# --------------------------------------------------------------------------
async def configure_bus(fd: FdCanInterface):
    for cmd in (
        "can off",
        "conf set can.bitrate 1000000",
        "conf set can.fdcan_frame on",
        "conf set can.bitrate_switch on",
        "conf set can.termination on",
        "can on",
    ):
        resp = await fd._send_command(cmd)
        print(f"> {cmd} → {resp}")


# --------------------------------------------------------------------------
# Arm joint control
# moteus.Controller.make_position() generates the correct moteus byte
# encoding without needing a transport. We send those bytes directly via
# FdCanInterface so there is no port conflict with the servo controllers.
# --------------------------------------------------------------------------

# Headless encoders — used only for byte generation, no transport needed
_arm_encoders = {
    9:  moteus.Controller(id=9),
    10: moteus.Controller(id=10),
    11: moteus.Controller(id=11),
    12: moteus.Controller(id=12),
    13: moteus.Controller(id=13),
    14: moteus.Controller(id=14),
}

def _make_arm_command(controller_id: int, position: float,
                      maximum_torque=0.7, velocity_limit=5.0, accel_limit=5.0):
    enc = _arm_encoders[controller_id]
    return enc.make_position(
        position=position,
        velocity=math.nan,
        maximum_torque=maximum_torque,
        velocity_limit=velocity_limit,
        accel_limit=accel_limit,
        feedforward_torque=0.0,
    )

async def _send_arm_command(fd: FdCanInterface, cmd):
    # Use "can std" (same format as servo controllers) with FD+BRS flags.
    # Read back the OK so the buffer stays clean.
    raw_cmd = f"can std {cmd.destination:X} {cmd.data.hex().upper()} FB\n"
    fd.writer.write(raw_cmd.encode("ascii"))
    await fd.writer.drain()
    try:
        line = await asyncio.wait_for(fd.reader.readline(), timeout=0.5)
        resp = line.decode("ascii", errors="ignore").strip()
        if resp.startswith("ERR"):
            print(f"[arm ERR] id={cmd.destination}: {resp}")
    except asyncio.TimeoutError:
        print(f"[arm timeout] id={cmd.destination} — no OK received")

# Shared arm target state — updated by set_arm_positions(), held by arm_loop()
_arm_configs = [
    (9,  0.0, 0.7, 5.0,  5.0),   # base
    (10, 0.0, 0.7, 10.0, 10.0),  # shoulder
    (11, 0.0, 0.7, 10.0, 10.0),  # elbow
    (12, 0.0, 0.7, 5.0,  5.0),   # wrist_rotate_1
    (13, 0.0, 0.7, 5.0,  5.0),   # wrist_tilt
    (14, 0.0, 0.7, 5.0,  5.0),   # wrist_rotate_2
]

async def move_joints(fd: FdCanInterface, positions: list):
    """
    Sets the arm target positions and sends one round of stop+position to
    clear faults and kick off motion. arm_loop() then holds the position.
    positions: [base, shoulder, elbow, wrist_rotate_1, wrist_tilt, wrist_rotate_2]
               in moteus revolutions
    """
    global _arm_configs
    _arm_configs = [
        (9,  positions[0], 0.7, 5.0,  5.0),
        (10, positions[1], 0.7, 10.0, 10.0),
        (11, positions[2], 0.7, 10.0, 10.0),
        (12, positions[3], 0.7, 5.0,  5.0),
        (13, positions[4], 0.7, 5.0,  5.0),
        (14, positions[5], 0.7, 5.0,  5.0),
    ]
    # Send stop (clears faults) then position for each joint
    for (cid, pos, mt, vl, al) in _arm_configs:
        stop_cmd = _arm_encoders[cid].make_stop()
        pos_cmd  = _make_arm_command(cid, pos, mt, vl, al)
        await _send_arm_command(fd, stop_cmd)
        await _send_arm_command(fd, pos_cmd)
    print(f"Arm joints commanded: {positions}")



# --------------------------------------------------------------------------
# End-effector servo keyboard control
# Keys: 1/2 = ee13 180°/0°,  3/4 = ee14 180°/0°,  5/6 = ee8 +5°/-5°
# --------------------------------------------------------------------------
STEP = 5
EE8_MIN = 40
EE8_MAX = 150

ee_state = {
    'ee13': 0,
    'ee14': 0,
    'ee8':  90,
}

def process_key(k):
    if   k == '1': ee_state['ee13'] = 180
    elif k == '2': ee_state['ee13'] = 0
    elif k == '3': ee_state['ee14'] = 180
    elif k == '4': ee_state['ee14'] = 0
    elif k == '6': ee_state['ee8']  = min(EE8_MAX, ee_state['ee8'] + STEP)
    elif k == '5': ee_state['ee8']  = max(EE8_MIN, ee_state['ee8'] - STEP)
    elif k == '0':
        ee_state['ee13'] = 90
        ee_state['ee14'] = 90
        ee_state['ee8']  = 90


async def read_keys():
    loop = asyncio.get_event_loop()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    key_queue = asyncio.Queue()

    def on_key():
        ch = os.read(fd, 1).decode('utf-8', errors='ignore')
        loop.call_soon_threadsafe(key_queue.put_nowait, ch)

    loop.add_reader(fd, on_key)
    try:
        while True:
            ch = await key_queue.get()
            if ch == '\x03':   # Ctrl+C
                raise KeyboardInterrupt
            process_key(ch)
    finally:
        loop.remove_reader(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


async def servo_loop(ee13, ee14, ee8):
    while True:
        await ee13.go_to_position(ee_state['ee13'])
        await ee14.go_to_position(ee_state['ee14'])
        await ee8.go_to_position(ee_state['ee8'])
        print(
            f"ee13: {ee_state['ee13']:3d}°  "
            f"ee14: {ee_state['ee14']:3d}°  "
            f"ee8:  {ee_state['ee8']:3d}°    ",
            end='\r'
        )
        await asyncio.sleep(0.05)   # 20 Hz


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
async def main_async():
    fd = FdCanInterface(port=SERIAL_PORT, baud=115200)
    await fd.open()
    await configure_bus(fd)   # fdcan_frame ON for moteus FD frame compatibility

    # End-effector servo controllers (CMR frames → servo board at CAN ID 24)
    ee13 = ServoController(can=fd, servo_id=13, can_id=SERVO_BOARD_CAN_ID)
    ee14 = ServoController(can=fd, servo_id=14, can_id=SERVO_BOARD_CAN_ID)
    ee8  = ServoController(can=fd, servo_id=8,  can_id=SERVO_BOARD_CAN_ID)

    print("=======================================")
    print("  ARM + END-EFFECTOR CONTROL           ")
    print("=======================================")
    print("  1/2 : ee13 -> 180° / 0°              ")
    print("  3/4 : ee14 -> 180° / 0°              ")
    print("  5/6 : ee8  +5° / -5°                 ")
    print("  0   : all servos to home              ")
    print(f"  ee8  range: {EE8_MIN} to {EE8_MAX} degrees")
    print("  Ctrl+C to exit                       ")
    print("=======================================\n")

    # Send arm to home position
    positions = [0, 0, 0, 0, 0, 0]
    await move_joints(fd, positions)
    await asyncio.sleep(0.25)

    # positions = [25.57765201851387, 4.273956711899999, 20.149706431730408, 12.277593691662496, 12.852379647024328, -24.7166071586458]
    # await move_joints(fd, positions)
    # await asyncio.sleep(6)

    # positions = [26.211102311997408, 10.128891391932349, 24.784013347554218, 12.596952709147516, 12.89430932396008, -21.69263390483982]
    # await move_joints(fd, positions)
    # await asyncio.sleep(6)

    # POSITION 1 ___________________________________________________
    # positions = [28.702207182474936, 9.356190327781057, 26.188667168937446, 13.45866790658038, 13.885922163912326, -20.35033650316049]
    # await move_joints(fd, positions)
    # await asyncio.sleep(6)

    # positions = [28.682191885158336, 9.350822427776144, 26.19987191199069, 13.547583317639308, 14.1270711190446, -8.979009308131637]
    # await move_joints(fd, positions)
    # await asyncio.sleep(4)

    # positions = [28.702207182474936, 9.356190327781057, 26.188667168937446, 13.45866790658038, 13.885922163912326, -20.35033650316049]
    # await move_joints(fd, positions)
    # await asyncio.sleep(6)
    # _______________________________________________________________

    # POSITION 2 ___________________________________________________
    # positions = [32.52485025982394, 11.475550020980592, 22.39198663649406, 14.519018034685352, 15.580760030771607, -21.922227589199792]
    # await move_joints(fd, positions)
    # await asyncio.sleep(6)

    # positions = [32.52485025982394, 11.475550020980592, 22.39198663649406, 14.519018034685352, 15.580760030771607, -8.979009308131637]
    # await move_joints(fd, positions)
    # await asyncio.sleep(6)

    # positions = [32.52485025982394, 11.475550020980592, 22.39198663649406, 14.519018034685352, 15.580760030771607, -21.922227589199792]
    # await move_joints(fd, positions)
    # await asyncio.sleep(6)
    # _______________________________________________________________

    # Wait 5s before starting servo loop — isolates whether arm holds on its own
    await asyncio.sleep(5)
    print("Starting servo loop...")

    # Run end-effector keyboard control continuously
    try:
        await asyncio.gather(
            read_keys(),
            servo_loop(ee13, ee14, ee8),
        )
    except KeyboardInterrupt:
        pass
    finally:
        await fd.close()
        print("\nStopped and closed.")


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()