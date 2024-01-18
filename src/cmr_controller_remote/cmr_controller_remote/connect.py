from geometry_msgs.msg import TwistStamped
import socket
import rclpy 
from rclpy.node import Node

UDP_IP = "0.0.0.0" # Listen on all available IPs
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

class CmdVelPublisher(Node):
  def __init__(self):
    super().__init__('cmd_vel_publisher')
    self.publisher_ = self.create_publisher(TwistStamped, '/drives_controller/cmd_vel', 10)
    self.timer = self.create_timer(0.1, self.publish_msg)
  
  def publish_msg(self):
    data, addr = sock.recvfrom(1024)
    linear_velocity, angular_velocity = self.parse_controller_data(data)
    msg = self.create_twist_stamped(linear_velocity, angular_velocity)
    self.publisher_.publish(msg)
    #self.get_logger().info('Publishing: "%s"' % msg)


  def create_twist_stamped(self, linear_velocity, angular_velocity):
    twist_msg = TwistStamped()
    twist_msg.twist.linear.x, twist_msg.twist.angular.z = linear_velocity, angular_velocity
    return twist_msg

  def parse_controller_data(self, raw_data):

    # Extract the first two bytes as linear and angular velocities
    linear_velocity_raw, angular_velocity_raw = raw_data[0], raw_data[1]

    # Scale the raw values to the desired range
    linear_velocity = self.scale_value(linear_velocity_raw, 0, 255, -2.5, 2.5)
    angular_velocity = self.scale_value(angular_velocity_raw, 0, 255, -2.5, 2.5)

    print(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")

    # Extract button states (assuming each state is one byte)
    button_states = raw_data[2:10] # Adjust the index based on actual data
    button_states_binary = [(format(byte, '08b'))[-1] for byte in button_states]
    print(button_states_binary)

    # Extract and decode the hat switch direction string
    # Assuming it starts from the 11th byte to the end
    hat_switch_data = raw_data[10:]
    hat_switch = hat_switch_data.split(b'\x00')[0].decode('utf-8') # Split at null byte and decode
    print("Hat Switch Direction:", hat_switch)
    return float(linear_velocity), float(angular_velocity)

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