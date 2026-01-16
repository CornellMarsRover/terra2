import math
import socket
import struct
import asyncio
import moteus
import moteus.aioserial
import time

# ====================== UDP CONFIGURATION ======================
UDP_IP = "0.0.0.0"   # Listen on all interfaces
UDP_PORT = 5030      # Must match PC sender port

# ====================== CONTROL PARAMETERS ======================
CONTROL_RATE_HZ = 200   # Hz for main loop
#MAX_STEP = 0.01         # rad per tick (200Hz → 2 rad/s)
VELOCITY_LIMIT = 2.0    # rad/s
ACCEL_LIMIT = 4.0       # rad/s²

# ====================== SOFTWARE REDUCTION ======================
SOFT_GEAR_RATIO = [
    0.15,  # joint 1
    0.15,  # joint 2
    0.12,  # joint 3
    0.10,  # joint 4
    0.10,  # joint 5
    0.08   # joint 6
]

DEADBAND = 0.002  # rad, ignores hand jitter

# ====================== MOTEUS SETUP ======================
async def make_controllers():
    print("[INFO] Initializing moteus controllers...")
    transport = moteus.Fdcanusb()  # automatically detects /dev/serial/by-id/usb-mjbots_fdcanusb_*
    ids = [9, 10, 11, 12, 13, 14]
    controllers = [moteus.Controller(id=i, transport=transport) for i in ids]
    return controllers

# ====================== INITIALIZATION ======================
async def initialize_positions(controllers):
    """Read initial motor positions (non-blocking with timeout)."""
    print("[INFO] Reading initial motor positions...")
    current_positions = [0.0] * len(controllers)
    for i, c in enumerate(controllers):
        try:
            result = await asyncio.wait_for(c.set_position(query=True), timeout=0.5)
            current_positions[i] = result.values[moteus.Register.POSITION]
        except asyncio.TimeoutError:
            print(f"[WARN] Motor ID {c.id} did not respond, defaulting to 0.0 rad")
            current_positions[i] = 0.0
    print("[INFO] Initial positions:", [round(p, 3) for p in current_positions])
    return current_positions

# ====================== UDP LISTENER TASK ======================
async def udp_listener(target_positions):
    """Continuously listen for new UDP packets and update target positions."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)
    print(f"[INFO] Listening for joint commands on UDP {UDP_IP}:{UDP_PORT}...")

    prev_mini = [0.0] * 6
    initialized = False

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) == 24:
                mini_vals = list(struct.unpack('6f', data))
                print(f"[RECV] From {addr}: {mini_vals}")
                # First packet → initialize
                if not initialized:
                    prev_mini = mini_vals[:]
                    target_positions[:] = [0.0] * 6
                    initialized = True
                    continue

                for i in range(6):
                    delta = mini_vals[i] - prev_mini[i]

                    # Deadband to kill jitter
                    if abs(delta) < DEADBAND:
                        delta = 0.0

                    target_positions[i] += SOFT_GEAR_RATIO[i] * delta

                prev_mini = mini_vals[:]

            else:
                print(f"[WARN] Ignoring invalid packet length: {len(data)}")
        except BlockingIOError:
            await asyncio.sleep(0.001)
        except Exception as e:
            print(f"[ERROR] UDP listener: {e}")
            await asyncio.sleep(1)

# ====================== CONTROL LOOP TASK ======================
async def control_loop(controllers, target_positions):
    """Smoothly move from current to target positions."""
    dt = 1.0 / CONTROL_RATE_HZ
    print("[INFO] Entering control loop...")

    while True:
        try:
            tasks = []
            for ctrl, pos in zip(controllers, target_positions):
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
        except Exception as e:
            print(f"[ERROR] Control loop: {e}")
            await asyncio.sleep(0.1)

        await asyncio.sleep(dt)

# ====================== MAIN ======================
async def main():
    controllers = await make_controllers()
    #current_positions = await initialize_positions(controllers)
    target_positions = [0.0] * 6

    await asyncio.gather(
        udp_listener(target_positions),
        control_loop(controllers, target_positions)
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")


