import socket
import struct
import asyncio
import moteus

# ====================== UDP CONFIGURATION ======================
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5030     # Must match PC sender port

# Set up UDP socket (non-blocking using timeout)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.01)  # 10ms timeout for non-blocking behavior
print(f"[INFO] Listening for joint commands on UDP {UDP_IP}:{UDP_PORT}...")

# ====================== SMOOTHING PARAMETERS ======================
VELOCITY_LIMIT = 1.0   # rad/s
ACCEL_LIMIT = 2.0      # rad/s^2
CONTROL_RATE_HZ = 200  # 200 Hz control loop
MAX_STEP = 0.01        # Max radians per cycle (0.01 rad @ 200 Hz = 2 rad/s max speed)

# ====================== MOTEUS CONTROLLERS ======================
controllers = [
    moteus.Controller(id=9),   # base
    moteus.Controller(id=10),  # shoulder
    moteus.Controller(id=11),  # elbow
    moteus.Controller(id=12),  # wrist rotate 1
    moteus.Controller(id=13),  # wrist tilt
    moteus.Controller(id=14)   # wrist rotate 2
]

# Initial joint positions (we'll read from motors on startup)
current_positions = [0.0] * 6
target_positions = [0.0] * 6

async def initialize_positions():
    """Read initial positions from motors so motion starts smoothly."""
    global current_positions
    print("[INFO] Reading initial motor positions...")
    tasks = [c.set_position(query=True) for c in controllers]
    results = await asyncio.gather(*tasks)
    current_positions = [r.values[moteus.Register.POSITION] for r in results]
    print("[INFO] Initial motor positions:", current_positions)

async def udp_listener():
    """Listen for incoming UDP packets and update target positions."""
    global target_positions
    print("[INFO] Starting UDP listener...")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) == 24:
                target_positions = list(struct.unpack('6f', data))
                print(f"[UDP] Received target positions: {target_positions}")
            else:
                print(f"[WARN] Ignoring packet of size {len(data)} bytes (expected 24)")
        except socket.timeout:
            # No data received this cycle, simply continue
            pass
        except Exception as e:
            print(f"[ERROR] UDP listener exception: {e}")
        await asyncio.sleep(0)  # Allow event loop to switch tasks

async def control_loop():
    """Smoothly move from current to target positions using interpolation."""
    global current_positions, target_positions
    print("[INFO] Starting control loop...")
    dt = 1.0 / CONTROL_RATE_HZ
    while True:
        for i in range(6):
            error = target_positions[i] - current_positions[i]
            step = max(-MAX_STEP, min(MAX_STEP, error))
            current_positions[i] += step

        tasks = [
            controllers[i].set_position(
                position=current_positions[i],
                velocity_limit=VELOCITY_LIMIT,
                accel_limit=ACCEL_LIMIT
            )
            for i in range(6)
        ]

        try:
            await asyncio.gather(*tasks)
        except Exception as e:
            print(f"[ERROR] Control loop exception: {e}")

        await asyncio.sleep(dt)

async def main():
    await initialize_positions()
    await asyncio.gather(udp_listener(), control_loop())

if __name__ == "__main__":
    print("[INFO] Starting Jetson Arm Receiver (Smooth Mode)")
    asyncio.run(main())
