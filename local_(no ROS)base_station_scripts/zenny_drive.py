import json
from pydualsense import pydualsense
import time
import socket

UDP_IP = "192.168.1.69"
UDP_PORT = 5005

DEADZONE = 8

def apply_deadzone(v: int, dz: int = DEADZONE) -> int:
    return 0 if abs(v) < dz else v

def norm_stick(v: int) -> float:
    # pydualsense stick values are centered around 0
    v = apply_deadzone(v)
    return max(-1.0, min(1.0, v / 127.0))

def norm_trigger(v: int, deadband: int = 30) -> float:
    v = max(0, min(255, int(v)))
    if v < deadband:
        return 0.0
    return v / 255.0

def main():
    ds = pydualsense()
    ds.init()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            # Example mapping — adjust to your preferred controls
            vx = -norm_stick(ds.state.LY)          # forward/back
            vy =  norm_stick(ds.state.LX)          # strafe
            omega = norm_stick(ds.state.RX)        # rotate

            pkt = {
                "vx": vx,
                "vy": vy,
                "omega": omega,
            }

            data = json.dumps(pkt).encode("utf-8")
            print(pkt)
            sock.sendto(data, (UDP_IP, UDP_PORT))

            ds.light.setColorI(15, 0, 90)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Stopping controller read...")

    finally:
        try:
            stop_pkt = json.dumps({"vx": 0.0, "vy": 0.0, "omega": 0.0}).encode("utf-8")
            sock.sendto(stop_pkt, (UDP_IP, UDP_PORT))
        except Exception:
            pass

        ds.close()
        sock.close()

if __name__ == "__main__":
    main()