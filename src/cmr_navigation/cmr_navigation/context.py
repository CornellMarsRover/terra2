import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix # PUT TWIST BACK
from cmr_msgs.msg import IMUSensorData, MotorReadData, AutonomyDrive 
import sys 
import time 
import math

'''
Context 
    - INIT
        - Publishers for cmd_vel and swerve
        - Subscriptions for gps and compass data
    - World to Cartesian
    - Sensor Fusion

    Function sort of like an API
'''

class Context(Node): 
    # GPS DATA
    LAT = 0.0
    LONG = 0.0 

    # IMU DATA 
    ANGLE_Z = 0.0

    # # CCB DATA
    # FRONT_LEFT = 0.0
    # BACK_RIGHT = 0.0
    # FRONT_RIGHT = 0.0
    # BACK_LEFT = 0.0

    def __init__(self, LAT_TARGET, LONG_TARGET): 
        super().__init__('context')
        self.gps_sub = self.create_subscription(NavSatFix, '/fake_navsatfix_data', self.gps_callback, 10) 
        # Change to NavSatFix IRL
        self.imu_sub = self.create_subscription(IMUSensorData, '/imu', self.imu_callback, 10)
        self.motor_sub = self.create_subscription(MotorReadData, '/ccb_read', self.ccb_read_callback, 10)
        self.automove_pub = self.create_publisher(AutonomyDrive, '/autonomy_move', 10)

        # GNSS GOAL
        self.LAT_TARGET = LAT_TARGET
        self.LONG_TARGET = LONG_TARGET

        # GPS DATA
        self.LAT = 0.0
        self.LONG = 0.0 

        # IMU DATA 
        self.ANGLE_Z = 0.0

        # CCB DATA
        self.FRONT_LEFT = 0.0
        self.BACK_RIGHT = 0.0
        self.FRONT_RIGHT = 0.0
        self.BACK_LEFT = 0.0

    def gps_callback(self, msg): 
        self.get_logger().info('GPS Data: {msg.latitude}, {msg.longitude}')
        self.LAT = msg.latitude
        self.LONG = msg.longitude

    def imu_callback(self, msg):
        self.get_logger().info('IMU Data: {msg.angle_z}')
        self.ANGLE_Z = msg.angle_z

    def ccb_read_callback(self, msg):
        self.get_logger().info('CCB Data: {msg.front_left_swerve}, {msg.back_right_swerve}, {msg.front_right_swerve}, {msg.back_left_swerve}')
        self.FRONT_LEFT_SWERVE= msg.front_left_swerve
        self.BACK_RIGHT_SWERVE = msg.back_right_swerve
        self.FRONT_RIGHT_SWERVE = msg.front_right_swerve
        self.BACK_LEFT_SWERVE = msg.back_left_swerve

    def calc_bearing(self, lat1, long1, lat2, long2):
    # Convert latitude and longitude to radians
        lat1 = math.radians(lat1)
        long1 = math.radians(long1)
        lat2 = math.radians(lat2)
        long2 = math.radians(long2)
        # Calculate the bearing
        bearing = math.atan2(
            math.sin(long2 - long1) * math.cos(lat2),
            math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1)
        )
        # Convert the bearing to degrees
        bearing = math.degrees(bearing)
        # Make sure the bearing is positive
        bearing = (bearing + 360) % 360
        return bearing
    
    def haversine(self, lat1, lon1, lat2, lon2):
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        # Radius of the Earth in kilometers
        R = 6371.0
        # Differences in coordinates
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        # Haversine formula
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        # Distance in kilometers
        distance_km = R * c
        # Convert distance from kilometers to meters
        distance_meters = distance_km * 1000
        return distance_meters

def main(args = None):
    rclpy.init(args=args)
    context = Context()
    try: 
        rclpy.spin(context)
    except KeyboardInterrupt: 
        pass
    finally: 
        context.destroy_node()
        rclpy.shutdown()
