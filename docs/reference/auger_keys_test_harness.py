#!/usr/bin/env python3
"""Reference test harness from the Astrotech subteam.

Hand-rolled keyboard-driven auger script provided by Astrotech that
revealed the auger uses Josh Pieper's `moteus` Python library (id=15
lead screw, id=16 auger) on a separate transport from the BDC + servo
boards. **This file is not part of the build, not imported, and not
on `sys.path`.** It lives under `docs/reference/` purely as
documentation of the moteus control pattern (velocity via
`make_position(position=nan, velocity=X, maximum_torque=Y)` at a
50 Hz watchdog cadence). The `keyboard` library and the bare
``while True`` loop are test-harness details only — the ROS 2 driver
will replicate the *control pattern*, not the I/O.
"""

import asyncio
import moteus
import math
import keyboard  # Imports our new non-blocking keyboard listener

async def main_async():
    transport = moteus.get_singleton_transport()
    lead_screw = moteus.Controller(id=15)
    auger = moteus.Controller(id=16)

    print("Clearing faults...")
    await transport.cycle([
        lead_screw.make_stop(),
        auger.make_stop()
    ])

    print("\n--- Keyboard Control Active ---")
    print("Hold 'W' to spin Auger FORWARD")
    print("Hold 'S' to spin Auger BACKWARD")
    print("Hold 'Q' to QUIT")
    print("-------------------------------\n")

    # The Control Loop
    while True:
        # Check for the quit command first
        if keyboard.is_pressed('q'):
            print("\nExiting keyboard control...")
            break

        # Check which key is currently being held down
        if keyboard.is_pressed('w'):
            # Command continuous POSITIVE velocity
            auger_cmd = auger.make_position(
                position=math.nan, 
                velocity=10.0,       # Spin at 2 revs/sec
                maximum_torque=1.0
            )
            print("\rStatus: Spinning FORWARD...   ", end="", flush=True)

        elif keyboard.is_pressed('s'):
            # Command continuous NEGATIVE velocity
            auger_cmd = auger.make_position(
                position=math.nan, 
                velocity=-50.0,      # Spin at -2 revs/sec
                maximum_torque=1.0
            )
            print("\rStatus: Spinning BACKWARD...  ", end="", flush=True)
        
        elif keyboard.is_pressed('a'):
            lead_cmd = lead_screw.make_position(
                position=math.nan,
                velocity=100.0,
                maximum_torque=2.0
            )
        elif keyboard.is_pressed('d'):
            lead_cmd = lead_screw.make_position(
                position=math.nan,
                velocity=-100.0,
                maximum_torque=2.0
            )
        else:
            # If no keys are held, command the motor to actively stop
            # You can also use auger.make_stop() here if you want it to relax completely
            auger_cmd = auger.make_position(
                position=math.nan,
                velocity=0.0,
                maximum_torque=1.0
            )
            lead_cmd = lead_screw.make_position(
                position=math.nan,
                velocity=0.0,
                maximum_torque=1.0
            )
            print("\rStatus: Holding position...   ", end="", flush=True)

        # Send the command to the auger (and keep the lead screw stopped/happy)
        await transport.cycle([
            lead_cmd,
            auger_cmd
        ])

        # Maintain our ~50Hz loop to keep the watchdog fed
        await asyncio.sleep(0.02)

    # Clean up and relax motors when breaking out of the loop
    await transport.cycle([
        lead_screw.make_stop(),
        auger.make_stop()
    ])

def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()