import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import NavSatFix
from cmr_msgs.msg import IMUSensorData, AutonomyDrive

L = 0.83  # wheelbase of the rover (meters)

class RTKFilter(Node):

    def __init__(self):
        super().__init__('rtk_filter_node')

        self.declare_parameter('real', False) # FALSE IF RUNNING IN SIMULATION
        self.real = self.get_parameter('real').get_parameter_value().bool_value

        # Robot drive command subscribers
        self.sub_ackerman = self.create_subscription(
            AutonomyDrive,
            '/autonomy/move/ackerman',
            self.ackerman_callback,
            10
        )
        self.sub_turn = self.create_subscription(
            Twist,
            '/autonomy/move/point_turn',
            self.point_turn_callback,
            10
        )
        # Publisher for the filtered pose estimate
        self.pub = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        
        # For GPS coordinate initialization
        self.initial_lat = None
        self.initial_lon = None
        self.yaw = 0.0
        # Velocities of center of rover (derived from drive commands)
        self.d_north = 0.0  # linear velocity in north direction (m/s)
        self.d_west = 0.0   # linear velocity in west direction (m/s)
        self.d_yaw = 0.0    # angular velocity about yaw axis (rad/s)
        # GPS antenna offset from center of rover in meters
        # first element is forward in frame of rover and second element is left
        self.r = [0.26, 0.08, 0.0]

    def point_turn_callback(self, msg: Twist):
        """
        Update omega yaw and linear velocities for center of rover from point turn commands.
        """
        self.d_north = 0
        self.d_west = 0
        omega = msg.angular.z
        n, w = self.lin_from_ang(omega)
        self.d_north = n
        self.d_west = w
        self.d_yaw = omega
        self.get_logger().info(f"dx: {self.d_north}\ndy: {self.d_west}\nw: {self.d_yaw}")

    def ackerman_callback(self, msg: AutonomyDrive):
        """
        Update omega yaw and linear velocities for center of rover from Ackerman drive commands.
        """
        R = L / math.tan(math.radians(msg.fl_angle))
        omega = msg.vel / R
        n, w = self.lin_from_ang(omega)
        if w < 0:
            n *= -1.0
        self.d_north = (math.cos(self.yaw) * msg.vel) + n
        self.d_west = (math.sin(self.yaw) * msg.vel) + w
        self.d_yaw = omega
        self.get_logger().info(f"dx: {self.d_north}\ndy: {self.d_west}\nw: {self.d_yaw}")

    def lin_from_ang(self, omega):
        """
        Helper function to calculate the linear velocity components
        resulting from angular rotation at the reference point.
        """
        w_vec = np.array([0.0, 0.0, omega])
        if self.real:
            R_mat = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],
                            [math.sin(self.yaw),  math.cos(self.yaw)]])
        else:
            R_mat = np.array([[math.cos(-1.0*self.yaw), -math.sin(-1.0*self.yaw)],
                            [math.sin(-1.0*self.yaw),  math.cos(-1.0*self.yaw)]])
        rr = R_mat.dot(np.array([self.r[0], self.r[1]]))
        r_vec = np.array([rr[0], rr[1], self.r[2]])
        vr = np.cross(w_vec, r_vec)
        return vr[0], vr[1]

    def get_north_west_meters(self, lat, lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        relative to the initial GPS coordinate.
        """
        R = 6378137.0  # Earth's radius in meters
        lat1_rad = math.radians(lat)
        lat2_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(lon)
        lon2_rad = math.radians(self.initial_lon)
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        # Compute north and west distances
        north = -1.0 * delta_lat * R
        west = delta_lon * R * math.cos(mean_lat)
        return north, west

def main(args=None):
    rclpy.init(args=args)
    node = RTKFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
