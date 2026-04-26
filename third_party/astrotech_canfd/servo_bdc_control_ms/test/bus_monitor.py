import serial
import time

PORT = "COM5"
BAUD = 115200

def send_can_message(ser, byte_array, description=""):
    hex_data = ''.join(f'{b:02X}' for b in byte_array)
    cmd = f"can std 1 {hex_data} C\n".encode()
    ser.write(cmd)
    print(f"Sent {description} Message")
    line = ser.readline()
    if line:
        print(">>", line.decode(errors="ignore").strip())
    else:
        print("No response")


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(1.0)
    ser.reset_input_buffer()

    # helper to send a command and wait for OK
    def send(cmd):
        ser.write((cmd + "\n").encode())
        resp = ser.readline().decode().strip()
        print("→", cmd, "|", resp)
        if not resp.startswith("OK"):
            raise RuntimeError(f"Command failed: {cmd} → {resp}")

    # 1) Turn bus off so we can reconfigure
    send("can off")
    # 2) Set nominal (arbitration) bitrate
    send("conf set can.bitrate 500000")
    # 3) Set FD data-phase bitrate
    # send("conf set can.fd_bitrate 1000000")
    # 4) Enable FD frame format
    # send("conf set can.fdcan_frame on")
    send("conf set can.fdcan_frame off")
    # 5) Enable bit-rate switching (BRS)
    send("conf set can.bitrate_switch on")
    # 6) Enable on-board termination
    send("conf set can.termination on")
    # 7) Turn the bus back on
    send("can on")

    send("can std 11 1122334455667788")

    # Now we’re in Bus-On, FD+BRS mode.
    print("CAN bus monitor running — press Ctrl+C to exit.\n")

    count = 0  # initialize counter

    # Transmit CAN-FD frame with BRS
    try:
        while True:
            while True:
                line = ser.readline()
                if not line:
                    break
                line_text = line.decode().strip()
                count += 1
                print(f"Message {count}←", line_text)
    except KeyboardInterrupt:
        print("\nStopping CAN Monitor")
if __name__ == "__main__":
    main()
