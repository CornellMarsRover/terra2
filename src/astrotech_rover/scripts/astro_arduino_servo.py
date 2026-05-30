import serial
import time

PORT = "COM10"   # /dev/ttyACM0 or /dev/ttyUSB0 (or similar) for linux machines, COM ports for windows
BAUD = 115200

arduino = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # arduino resets when serial opens

print("connected to Arduino.")
print("type 'ccw' for counterclockwise, 's' for stop, or 'cw' for clockwise.")
print("use KeyboardInterrupt to quit.")

try:
    while True:
        user_input = input("mix direction: ").strip()

        try:
            if (user_input == 'ccw'):
                angle = 0
            elif (user_input == 's'):
                angle = 90
            elif (user_input == 'cw'):
                angle = 180
            else:
                print("enter one of the three valid inputs: ccw for counterclockwise, s for stop, or cw for clockwise.")

        except ValueError:
            print("enter one of the three valid inputs.")
            continue

        arduino.write(f"{angle}\n".encode())

except KeyboardInterrupt:
    print("\nclosing serial connection.")

finally:
    arduino.close()
    print("closed.")