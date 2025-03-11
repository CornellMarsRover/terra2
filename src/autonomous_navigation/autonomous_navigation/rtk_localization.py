import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from cmr_msgs.msg import IMUSensorData
from pyubx2 import llh2ecef  # assumes this returns (x, y, z) in meters

class RTKLocalization(Node):
    def __init__(self):
        super().__init__('rtk_localization_node')
        # Subscribers for GPS and IMU data
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/rtk/navsatfix_data',
            self.gps_callback,
            10
        )
        self.sub_imu = self.create_subscription(
            IMUSensorData,
            '/imu',
            self.imu_callback,
            10
        )
        # Publisher for the filtered pose estimate
        self.pub = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        
        # Timer for 10Hz updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # For GPS offset initialization (using ECEF conversion)
        self.initial_x = None
        self.initial_y = None

        # Storage for latest sensor measurements
        self.gps_n = 0.0  # north displacement from GPS (meters)
        self.gps_w = 0.0  # west displacement from GPS (meters)
        self.gps_noise = 1.0  # horizontal std. deviation from GPS

        self.yaw = 0.0

    def gps_callback(self, msg: NavSatFix):
        """
        Process GPS data. Note:
          - The altitude field is used as the horizontal standard deviation (in m)
          - llh2ecef converts (lat, lon, height) to ECEF x,y,z (in meters)
        """
        lat = msg.latitude
        lon = msg.longitude
        # Use a known fixed height (e.g. basestation height)
        basestation_height = 245.0  
        x, y, _ = llh2ecef(lat, lon, basestation_height)
        # Initialize offset from first measurement
        if self.initial_x is None:
            self.initial_x = x
            self.initial_y = y
        # Compute displacement relative to initial position
        disp_x = x - self.initial_x 
        disp_y = y - self.initial_y 
        self.gps_n = disp_y
        self.gps_w = -1.0 * disp_x

    def imu_callback(self, msg: IMUSensorData):
        """
        Process IMU data.
        IMUSensorData fields:
          - accx, accy: linear accelerations in m/s^2 (body frame)
          - gyroz: angular velocity in rad/s
          - anglez: yaw angle in degrees (convert to radians)
        """
        self.yaw = math.radians(msg.anglez)

    def timer_callback(self):
        """
        Publish pose data
        """
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = self.gps_n
        twist_msg.twist.linear.y = self.gps_w
        twist_msg.twist.angular.z = self.yaw
        self.pub.publish(twist_msg)
        self.get_logger().info(f"Pose:\nx: {self.gps_n}m\ny: {self.gps_w}m\nyaw: {math.degrees(self.yaw)}deg")

def main(args=None):
    rclpy.init(args=args)
    node = RTKLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
