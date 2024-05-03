import rclpy
from cmr_navigation.context import Context
from rclpy.node import Node
import sys 
#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import NavSatFix
from cmr_msgs.msg import IMUSensorData, MotorReadData, AutonomyDrive
import math

class DriveToCoord(): 
    def __init__(self, LAT_GOAL, LONG_GOAL): 
        self.context = Context(LAT_TARGET=LAT_GOAL, LONG_TARGET=LONG_GOAL)
        self.reached_goal = False
        self.threshold = 1.5

    def drive_to_goal(self):
        distance = self.context.haversine(self.context.LAT, 
                                          self.context.LONG, 
                                          self.context.LAT_TARGET,
                                          self.context.LONG_TARGET)

        while distance > self.threshold: 
            bearing = self.context.calc_bearing(self.context.LAT, 
                                          self.context.LONG, 
                                          self.context.LAT_TARGET,
                                          self.context.LONG_TARGET)
            
            print(f'Distance: {distance}   bearing: {bearing}')

            PLACEHOLDER = 0.0 #TODO

            close_enough = lambda curr_pos, turn_pos : abs(curr_pos - turn_pos) < PLACEHOLDER

            if (close_enough(self.context.FRONT_LEFT_SWERVE, PLACEHOLDER) and close_enough(self.context.FRONT_RIGHT_SWERVE, PLACEHOLDER)
            and close_enough(self.context.BACK_LEFT_SWERVE, PLACEHOLDER) and close_enough(self.context.BACK_RIGHT_SWERVE, PLACEHOLDER)):
                autonomy_msg = AutonomyDrive()
                autonomy_msg.fl_angle = -self.context.ANGLE_Z + bearing
                autonomy_msg.fr_angle = -self.context.ANGLE_Z + bearing
                autonomy_msg.bl_angle = 0.0
                autonomy_msg.br_angle = 0.0
                autonomy_msg.vel = 2.0
                self.context.automove_pub.publish(autonomy_msg)

            distance = self.context.haversine(self.context.LAT, 
                                          self.context.LONG, 
                                          self.context.LAT_TARGET,
                                          self.context.LONG_TARGET)
            

            



        self.reached_goal = True

def main(args=None):
    rclpy.init(args=args)
    # Parse command-line arguments
    if args is None:
        args = sys.argv[1:]
    if len(args) != 2:
        print("Usage: ros2 run your_package your_node <goal_latitude> <goal_longitude>")
        return
    GOAL_LAT = float(args[0])
    GOAL_LON = float(args[1])

    drive_to_coord = DriveToCoord(LAT_GOAL=GOAL_LAT, LONG_GOAL=GOAL_LON)


    try:
        drive_to_coord.drive_to_goal()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()