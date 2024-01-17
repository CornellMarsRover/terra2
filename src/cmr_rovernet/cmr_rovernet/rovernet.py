import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
import serial
import struct

valid_subteams = ["arm", "drives", "astrotech", "business", "homeless san fran ben dodson"]
valid_motors = ["front_right", "front_left", "back_right", "back_left"]

#subteam = which subteam is being controller
#motor_id = which motor is being commanded
#position = the position of the arm motor from 0 to 100 (0 = 0 degress or n o turn, 100 = 360 degrees or full turn )

def byte_command_converter(subteam, motor, position, drives_velocity, max_torque, max_vel, max_accel, logger):
    if subteam not in valid_subteams:
        raise ValueError(f"Invalid Subteam in command, needs to be one of {valid_subteams}")
    if motor not in valid_motors:
        raise ValueError("Invalid motor_id")
    if position is not None and position not in range(100):
        raise ValueError("Invalid position")
    if not isinstance(drives_velocity, float) and not isinstance(drives_velocity, int) :
        raise TypeError("drives_velocity must be an float or int")
    
    #Init hex values to be output


    subteam_ids = {"drives": 0x01, "arm": 0x02, "astrotech": 0x03}
    motor_ids = {"front_left": 0x01, "front_right": 0x03, "back_right": 0x02, "back_left": 0x04}
    
    subteam_hex = subteam_ids.get(subteam, 0x00)
    motor_hex = motor_ids.get(motor, 0x00)
    position_hex = struct.pack('f', position) if position is not None else b'\xFF\xFF\xFF\xFF'
    drives_vel_hex = struct.pack('B', abs(int(drives_velocity)))
    direction_hex = struct.pack('B', int(drives_velocity < 0))
    max_torque_hex = struct.pack('f', max_torque) if max_torque is not None else b'\xFF\xFF\xFF\xFF'
    max_vel_hex = struct.pack('f', max_vel) if max_vel is not None else b'\xFF\xFF\xFF\xFF'
    max_accel_hex = struct.pack('f', max_accel) if max_accel is not None else b'\xFF\xFF\xFF\xFF'

    # drives_vel_hex_string = drives_vel_hex.hex()
    # logger.info(f'Vel: {drives_vel_hex_string}')

    # max_torque_hex_string = max_torque_hex.hex()
    # logger.info(f'Vel: {max_torque_hex_string}')

    # max_accel_hex_string = max_accel_hex.hex()
    # logger.info(f'Vel: {max_accel_hex_string}')

    # Concatenate the parts and pad to ensure the total length is 40 bytes
    output = bytes([subteam_hex, motor_hex]) + position_hex + drives_vel_hex + direction_hex + max_torque_hex + max_vel_hex + max_accel_hex

    output_string = output.hex()
    logger.info(f'Output: {output_string}')
    output = output.ljust(40, b'\x00')

    return output


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

        # direction = int(self.current_speed < 0)
        # direction_byte = direction.to_bytes(1, byteorder='big')

        # hex_vel = abs(int(self.current_speed))
        # hex_vel_byte = hex_vel.to_bytes(1, byteorder='big')

        # #Log direction and velocity
        # direction_str = "{:02x}".format(direction)
        # hex_vel_str = "{:02x}".format(hex_vel)
        # self.get_logger().info(f'Velocity: {hex_vel_str}')
        # self.get_logger().info(f'Direction: {direction_str}')

        output = byte_command_converter("drives", "back_right", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
        self.send_number(self.serial_port, output)

        # self.send_number(self.serial_port, bytes([0x01, 0x03, 0xFF, 0xFF]) + hex_vel_byte + direction_byte + bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]))




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
