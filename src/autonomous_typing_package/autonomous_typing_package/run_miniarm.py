#!/usr/bin/env python3

import serial

import time

import asyncio

import moteus

import re

import math# === :hammer_and_wrench: USER CONFIGURABLE VALUES ===

#SERIAL_PORT = 'COM4' # Replace with your actual portz

BAUD_RATE = 115200# Adjust these multipliers to scale mini-arm joint angles to big arm rotations

JOINT_MULTIPLIERS = [0.25, 0.3, 0.25, 0.125, 0.125, 0.125]  # <- Edit these# here also# === MOTOR CONTROLLERS ===



controllers = [

    moteus.Controller(id=9),  # base

    moteus.Controller(id=10), # shoulder

    moteus.Controller(id=11), # elbow

    moteus.Controller(id=12), # wrist rotate 1

    moteus.Controller(id=13), # wrist tilt

    moteus.Controller(id=14)  # wrist rotate 2

]





async def move_joints(positions):

    """Sends position commands to all 6 Moteus motors."""

    tasks = []

    for ctrl, pos in zip(controllers, positions):

        task = ctrl.set_position(

            position=pos,

            velocity=math.nan,

            maximum_torque=0.7,

            velocity_limit=10.0,

            accel_limit=10.0,

            feedforward_torque=0.0

        )

        tasks.append(task)

    await asyncio.gather(*tasks)





def parse_joint_data(line):

    """Parses comma-separated angles from the mini-arm."""

    try:

        values = [float(v.strip()) for v in re.split(r'[|,]', line) if v.strip()]

        if len(values) < 5:

            raise ValueError("Expected 5 or 6 values for 6 joints.")

        scaled = [raw * mul for raw, mul in zip(values, JOINT_MULTIPLIERS)]

        return scaled

    except ValueError as ve:

        print(f"Invalid input: {line} — {ve}")

        return None



async def read_and_update_loop():



    """Main loop to read serial and update arm position."""

    try:

        ser = serial.Serial("/dev/ttyACM0", BAUD_RATE, timeout=1)

        time.sleep(3)

        print("Listening to mini-arm and updating large arm...")

        line = ser.readline().decode('utf-8').strip()

        print(f"Mini-arm raw: {line}")        

        initial = parse_joint_data(line)

        print(initial)

        while True:

            if ser.in_waiting > 0:

                line = ser.readline().decode('utf-8').strip()

                print(f"Mini-arm raw: {line}")

                positions = parse_joint_data(line)

                if positions:

                    if len(positions) == 5:

                        new_positions = [0.0]

                        new_positions.extend(positions)

                        positions = new_positions                    

                    positions[0]-=initial[0]

                    positions[1]-=initial[1]

                    positions[2]-=initial[2]

                    positions[3]-=initial[3]

                    positions[4]-=initial[4]

                    positions[5]-=initial[5]

                    temp=positions[4]

                    positions[4]=positions[3]

                    positions[3]=temp                    

                    print(f"Sending to motors: {positions}")

                    await move_joints(positions)    

    except serial.SerialException as e:

        print(f"Serial error: {e}")

    except KeyboardInterrupt:

        print("\nStopped by user.")

    finally:

        if 'ser' in locals() and ser.is_open:

            ser.close()# here

def main():

    asyncio.run(read_and_update_loop())

if __name__ == "__main__":

    main()