#!/usr/bin/env python3

import asyncio
import moteus
import math
import time

async def move_joints(positions):
    """
    Moves the 6 arm joints to the specified positions.

    Arguments:
        positions (list or tuple of float):
            A list/tuple of 6 positions for the joints:
            [base, shoulder, elbow, wrist_rotate_1, wrist_tilt, wrist_rotate_2]
    """

    # Create Controller instances for each arm joint
    base           = moteus.Controller(id=9)
    shoulder       = moteus.Controller(id=11)
    elbow          = moteus.Controller(id=10)
    wrist_rotate_1 = moteus.Controller(id=12)
    wrist_tilt     = moteus.Controller(id=13)
    wrist_rotate_2 = moteus.Controller(id=14)

    # Prepare the set_position tasks for each joint
    tasks = [
        base.set_position(
            position=positions[0],
            velocity=math.nan,         
            maximum_torque=0.7,
            velocity_limit=5.0,
            accel_limit=5.0,
            feedforward_torque=0.0
        ),
        shoulder.set_position(
            position=positions[1],
            velocity=math.nan,
            maximum_torque=0.7,
            velocity_limit=10.0,
            accel_limit=10.0,
            feedforward_torque=0.0
        ),
        elbow.set_position(
            position=positions[2],
            velocity=math.nan,
            maximum_torque=0.7,
            velocity_limit=10.0,
            accel_limit=10.0,
            feedforward_torque=0.0
        ),
        wrist_rotate_1.set_position(
            position=positions[3],
            velocity=math.nan,
            maximum_torque=0.7,
            velocity_limit=5.0,
            accel_limit=5.0,
            feedforward_torque=0.0
        ),
        wrist_tilt.set_position(
            position=positions[4],
            velocity=math.nan,
            maximum_torque=0.7,
            velocity_limit=5.0,
            accel_limit=5.0,
            feedforward_torque=0.0
        ),
        wrist_rotate_2.set_position(
            position=positions[5],
            velocity=math.nan,
            maximum_torque=0.7,
            velocity_limit=5.0,
            accel_limit=5.0,
            feedforward_torque=0.0
        )
    ]

    # Execute all tasks concurrently
    await asyncio.gather(*tasks)
    print("All arm joints commanded to specified positions!")

async def main_async():
    """
    Runs multiple move_joints calls in a single event loop.
    """


    positions = [0, 0, 0, 0, 1, 0]
    await move_joints(positions)
    await asyncio.sleep(10)

    # positions = [25.57765201851387, 4.273956711899999, 20.149706431730408, 12.277593691662496, 12.852379647024328, -24.7166071586458]
    # await move_joints(positions)
    # await asyncio.sleep(6)

    # positions = [26.211102311997408, 10.128891391932349, 24.784013347554218, 12.596952709147516, 12.89430932396008, -21.69263390483982]
    # await move_joints(positions)
    # await asyncio.sleep(6)

    # POSITION 1 _______________________________________________________________________________________________________________________
    # positions = [28.702207182474936, 9.356190327781057, 26.188667168937446, 13.45866790658038, 13.885922163912326, -20.35033650316049]
    # await move_joints(positions)
    # await asyncio.sleep(6)

    # positions = [28.682191885158336, 9.350822427776144, 26.19987191199069, 13.547583317639308, 14.1270711190446, -8.979009308131637]
    # await move_joints(positions)
    # await asyncio.sleep(4)

    # positions = [28.702207182474936, 9.356190327781057, 26.188667168937446, 13.45866790658038, 13.885922163912326, -20.35033650316049]
    # await move_joints(positions)
    # await asyncio.sleep(6)
    #____________________________________________________________________________________________________________________________________

    # POSITION 2 _______________________________________________________________________________________________________________________
    # positions = [32.52485025982394, 11.475550020980592, 22.39198663649406, 14.519018034685352, 15.580760030771607, -21.922227589199792]
    # await move_joints(positions)
    # await asyncio.sleep(6)

    # positions = [32.52485025982394, 11.475550020980592, 22.39198663649406, 14.519018034685352, 15.580760030771607, -8.979009308131637]
    # await move_joints(positions)
    # await asyncio.sleep(4)

    # positions = [32.52485025982394, 11.475550020980592, 22.39198663649406, 14.519018034685352, 15.580760030771607, -21.922227589199792]
    # await move_joints(positions)
    # await asyncio.sleep(6)
    #____________________________________________________________________________________________________________________________________
    # Spot 2 pre drop [32.52485025982394, 11.475550020980592, 22.39198663649406, 14.519018034685352, 15.580760030771607, -21.922227589199792]
    # spot 2 post drop [32.52485025982394, 11.475550020980592, 22.39198663649406, 14.519018034685352, 15.580760030771607, --8.979009308131637]


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()

