from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import DrivesControllerReading
import socket
import rclpy 
from rclpy.node import Node

UDP_IP = "0.0.0.0" # Listen on all available IPs
UDP_PORT = 5010

directions = {
    "neutral" : 8, "N" : 0, "NE" : 1, "E" : 2, "SE" : 3,
    "S" : 4, "SW" : 5, "W" : 6, "NW" : 7
}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))


class CmdVelPublisher(Node):
  def __init__(self):
    super().__init__('cmd_vel_publisher')
    self.publisher_ = self.create_publisher(TwistStamped, '/drives_controller/cmd_vel', 10)
    self.button_publisher_ = self.create_publisher(DrivesControllerReading, '/drives_controller/cmd_buttons', 10)
    self.timer = self.create_timer(0.1, self.publish_msg)
  
  def publish_msg(self):
    data, addr = sock.recvfrom(1024)
    linear_velocity, angular_velocity = self.parse_controller_data(data)
    button_array, dpad = self.parse_button_data(data)
    msg = self.create_twist_stamped(linear_velocity, angular_velocity)
    button_msg = self.create_button_message(button_array, dpad)
    self.publisher_.publish(msg)
    self.button_publisher_.publish(button_msg)
    #self.get_logger().info('Publishing: "%s"' % msg)

  def create_twist_stamped(self, linear_velocity, angular_velocity):
    twist_msg = TwistStamped()
    twist_msg.twist.linear.x, twist_msg.twist.angular.z = linear_velocity, angular_velocity
    return twist_msg
  
  def create_button_message(self, button_array, dpad):
    button_msg = DrivesControllerReading()
    button_msg.button_array, button_msg.dpad = button_array, dpad
    return button_msg

  def parse_controller_data(self, raw_data):

    # Extract the first two bytes as linear and angular velocities
    linear_velocity_raw, angular_velocity_raw = raw_data[0], raw_data[1]

    # Scale the raw values to the desired range
    linear_velocity = self.scale_value(linear_velocity_raw, 0, 255, -2.5, 2.5)
    angular_velocity = self.scale_value(angular_velocity_raw, 0, 255, -2.5, 2.5)

    print(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")

    # Extract and decode the hat switch direction string
    # Assuming it starts from the 11th byte to the end
    hat_switch_data = raw_data[10:]
    hat_switch = hat_switch_data.split(b'\x00')[0].decode('utf-8') # Split at null byte and decode
    print("Hat Switch Direction:", hat_switch)
    return float(linear_velocity), float(angular_velocity)

  def parse_button_data(self, raw_data):
    buttons = raw_data[2:10]
    dpad = raw_data[10]
    dpad = directions.get(dpad, -1)
    return buttons, dpad




  def scale_value(self, value, old_min, old_max, new_min, new_max):
    # Scale the old range to the new range
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min


def main(args = None):
  rclpy.init(args=args)
  cmd_vel_publisher = CmdVelPublisher()

  try:
    rclpy.spin(cmd_vel_publisher)
  except KeyboardInterrupt:
    pass

if __name__ == '__main__':
    main()