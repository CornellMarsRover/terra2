from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import ControllerReading
import socket
import rclpy 
from rclpy.node import Node
import struct

UDP_IP = "0.0.0.0" # Listen on all available IPs
UDP_PORT_DRIVES = 5010
UDP_PORT_ARM = 5020

directions = {
    "neutral" : 8, "N" : 0, "NE" : 1, "E" : 2, "SE" : 3,
    "S" : 4, "SW" : 5, "W" : 6, "NW" : 7
}


print("Connecting Drives")
drives_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drives_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
drives_sock.setblocking(0)
drives_sock.bind((UDP_IP, UDP_PORT_DRIVES))
print(drives_sock)

print("Connecting Arm")
arm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
arm_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
arm_sock.setblocking(0)
arm_sock.bind((UDP_IP, UDP_PORT_ARM))
print(arm_sock)

class CmdVelPublisher(Node):
  def __init__(self):
    super().__init__('cmd_vel_publisher')
    self.publisher_ = self.create_publisher(TwistStamped, '/drives_controller/cmd_vel', 10)
    self.arm_publisher_ = self.create_publisher(TwistStamped, '/arm_controller/cmd_vel', 10)

    self.button_publisher_ = self.create_publisher(ControllerReading, '/drives_controller/cmd_buttons', 10)
    self.arm_button_publisher_ = self.create_publisher(ControllerReading, '/arm_controller/cmd_buttons', 10)
    self.timer = self.create_timer(0.1, self.publish_msg)

    self.logger = self.get_logger()
  
  def publish_msg(self):

      try:
          data, addr = drives_sock.recvfrom(1024)
          lx, ly, rx, ry = self.parse_controller_data(data)
          button_array, dpad = self.parse_button_data(data)
          msg = self.create_twist_stamped([lx, ly, rx, ry])
          button_msg = self.create_button_message(button_array, dpad)
          self.publisher_.publish(msg)
          self.button_publisher_.publish(button_msg)
          # process data
      except BlockingIOError:
          # Handle the case where no data is received
          pass
      
      try:
        arm_data, addr = arm_sock.recvfrom(1024)
        # process arm_data
        velocities = self.parse_arm_data(arm_data)
        arm_button_array = velocities[1]
        arm_msg = self.create_twist_stamped(velocities[0])
        arm_button_msg = self.create_arm_button_message(arm_button_array)
        self.arm_publisher_.publish(arm_msg)
        self.arm_button_publisher_.publish(arm_button_msg)
      except BlockingIOError:
        # Handle the case where no data is received
        pass

  def create_twist_stamped(self, velocities):
    if len(velocities) == 4:
      twist_msg = TwistStamped()
      twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.angular.x, twist_msg.twist.angular.y = velocities[0], velocities[1], velocities[2], velocities[3]
      return twist_msg
    elif len(velocities) == 3:
      twist_msg = TwistStamped()
      twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z = velocities[0], velocities[1], velocities[2]
      return twist_msg
    else:
      self.get_logger().warn('Received unexpected number of velocity states')
      return None
  
  def create_button_message(self, button_array, dpad):
    button_msg = ControllerReading()
    button_msg.button_array, button_msg.dpad = button_array, dpad
    return button_msg
  
  def create_arm_button_message(self, button_array):
    button_msg = ControllerReading()
    button_msg.button_array = button_array
    return button_msg

  def parse_controller_data(self, raw_data):

    # Extract the first two bytes as linear and angular velocities
    lx_raw, ly_raw, rx_raw, ry_raw = raw_data[0], raw_data[1], raw_data[2], raw_data[3]

    # Scale the raw values to the desired range
    lx = self.scale_value(lx_raw, 0, 255, -2.5, 2.5)
    ly = self.scale_value(ly_raw, 0, 255, -2.5, 2.5)
    rx = self.scale_value(rx_raw, 0, 255, -2.5, 2.5)
    ry = self.scale_value(ry_raw, 0, 255, -2.5, 2.5)

    #print(f"Linear Velocity: {lx}, Angular Velocity: {rx}")

    # Extract and decode the hat switch direction string
    # Assuming it starts from the 11th byte to the end
    hat_switch_data = raw_data[12:]
    hat_switch = hat_switch_data.split(b'\x00')[0].decode('utf-8') # Split at null byte and decode
    print("Hat Switch Direction:", hat_switch)
    return float(lx), float(ly), float(rx), float(ry)

  def parse_button_data(self, raw_data):
    buttons = raw_data[4:12]
    # dpad = raw_data[12]
    # dpad = directions.get(dpad, -1)
    # DPAD UNIMPLEMENTED
    dpad = 0
    return buttons, dpad
  
  def parse_arm_data(self, raw_data):
      
      x_axis = struct.unpack('<d', raw_data[0:8])[0]
      y_axis = struct.unpack('<d', raw_data[8:16])[0]
      z_axis = struct.unpack('<d', raw_data[16:24])[0]
      button_data = struct.unpack('16B', raw_data[24:])
      print(button_data)
      print(f"X: {x_axis}, Y: {y_axis}, Z: {z_axis}")
      return [[float(x_axis), float(y_axis), float(z_axis)], button_data]

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