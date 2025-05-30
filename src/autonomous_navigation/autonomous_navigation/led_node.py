#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import atexit

# --- serial commands for the Adafruit USB tower light w/alarm ---
RED_ON        = 0x11
RED_OFF       = 0x21
GREEN_ON      = 0x14
GREEN_OFF     = 0x24
BUZZER_ON     = 0x18
BUZZER_OFF    = 0x28

class LedController(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # open serial port
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Could not open serial port {port}: {e}')
            raise

        # ensure we turn everything off first
        for cmd in (BUZZER_OFF, RED_OFF, GREEN_OFF):
            self._send(cmd)

        # default: turn RED on
        self._send(RED_ON)

        # subscribe to /autonomy/led
        self.sub = self.create_subscription(
            String,
            '/autonomy/led',
            self.led_callback,
            10
        )

        # make sure to clean up on shutdown
        atexit.register(self.cleanup)

    def _send(self, cmd: int):
        """Send a single-byte command over serial."""
        self.ser.write(bytes([cmd]))

    def led_callback(self, msg: String):
        if msg.data.strip().lower() == 'blink':
            self.get_logger().info('Received blink command – blinking green + buzzing for 5s')
            # turn default red off
            self._send(RED_OFF)
            # buzzer on for full duration
            self._send(BUZZER_ON)

            # blink green: on 0.5s, off 0.5s, total ~5s
            cycles = int(5.0 / 1.0)  # one full on+off = 1s
            for _ in range(cycles):
                self._send(GREEN_ON)
                time.sleep(0.5)
                self._send(GREEN_OFF)
                time.sleep(0.5)

            # stop buzzing, restore red
            self._send(BUZZER_OFF)
            self._send(RED_ON)
            self.get_logger().info('Blink complete, returned to red')

    def cleanup(self):
        """Turn everything off on exit."""
        for cmd in (BUZZER_OFF, RED_OFF, GREEN_OFF):
            try:
                self._send(cmd)
            except Exception:
                pass
        try:
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
