import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
import serial

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/drives_controller/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_speed = 0
        self.last_time = time.time()
        self.port = "/dev/ttyTHS0"
        self.baud_rate = 115200
        self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)

    def scale_value(self, value, old_min, old_max, new_min, new_max):
        # Scale the old range to the new range
        return (new_max - new_min) * (value - old_min) / (old_max - old_min) + new_min

    def gradually_increase_speed(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time
        self.last_time = current_time

        # Base rate of speed change
        base_rate = 24 * time_difference

        # Modify the rate based on how close the target speed is to neutral (0)
        # The rate of change is faster when the joystick is near neutral
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35)  # Assuming 35 is the max speed

        # Ensure dynamic_rate does not drop below a certain threshold
        dynamic_rate = max(dynamic_rate, base_rate * 0.5)

        if self.current_speed < target_speed:
            self.current_speed += dynamic_rate
            if self.current_speed > target_speed:
                self.current_speed = target_speed
        elif self.current_speed > target_speed:
            self.current_speed -= dynamic_rate
            if self.current_speed < target_speed:
                self.current_speed = target_speed

    def send_number(self, serial_port, byte):
        """
        Send a number over the specified serial port.
        """
        # Convert the number to a string and encode it to bytes
        
        # Write the data to the serial port
        serial_port.write(byte)


    def listener_callback(self, msg):
        target_speed = self.scale_value(msg.twist.linear.x, -2.5, 2.5, -53, 53)
        self.gradually_increase_speed(target_speed)

        direction = int(self.current_speed < 0)
        direction_byte = direction.to_bytes(1, byteorder='big')

        hex_vel = abs(int(self.current_speed))
        hex_vel_byte = hex_vel.to_bytes(1, byteorder='big')

        #Log direction and velocity
        direction_str = "{:02x}".format(direction)
        hex_vel_str = "{:02x}".format(hex_vel)
        self.get_logger().info(f'Velocity: {hex_vel_str}')
        self.get_logger().info(f'Direction: {direction_str}')

        self.send_number(self.serial_port, bytes([0x01, 0x03, 0xFF, 0xFF]) + hex_vel_byte + direction_byte + bytes([0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]))




def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
