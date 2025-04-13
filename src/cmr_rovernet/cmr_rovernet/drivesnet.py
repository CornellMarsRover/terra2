import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import ControllerReading, AutonomyDrive
import time
import serial
import math
from cmr_rovernet.rovernet_utils import *


class CmdVelSubscriber(Node):
    """
    This node subscribes to the /drives_controller/cmd_vel topic output by the 
    drivescontroller node. It will then convert the output to a pre-defined 40-byte 
    format and send the output to the CCB via UART. 
    """
    
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/drives_controller/cmd_vel',
            self.listener_callback,
            10)
        self.button_subscription = self.create_subscription(
            AutonomyDrive,
            '/autonomy_move',
            self.autonomy_callback,
            10)
        self.autonomy_subscription = self.create_subscription(
            ControllerReading,
            '/drives_controller/cmd_buttons',
            self.listener_button_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_speed_left = 0
        self.current_speed_right = 0
        self.turn_thread_lock = 0
        self.controller_command_ly = 0
        self.controller_command_lx = 0
        self.controller_command_ry = 0
        self.controller_command_rx = 0
        self.last_time_left = time.time()
        self.last_time_right = time.time()
        # self.port = "/dev/ttyTHS0"
        self.baud_rate = 115200
        # self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
        self.feed_forward_torque = 0.0
        self.velocity = 0
        # self.serial_port = None
        self.logger = self.get_logger()
        self.swerve = False
        
        # Init constants given TOML file
        #drives_controller_table = parse_toml("connect")
        drives_net_table = parse_toml("drivesnet")
        drives_net_node = drives_net_table['node']
        self.CONTROLLER_MAX_SPEED = 2.5
        self.MOTOR_MAX_SPEED = drives_net_node['motor_max_speed']
        self.GRADUAL_INCREASE_RATE = drives_net_node['gradual_increase_rate']
        self.BASE_DYNAMIC_RATIO = drives_net_node['base_dynamic_ratio']
        self.MAX_ACCELERATION = drives_net_node['acceleration_limit']
        self.MAX_TORQUE = drives_net_node['torque_limit']
        self.MAX_FEED_FORWARD_TORQUE = drives_net_node['feed_forward_torque_limit']

    # def set_feed_forward_torque(self, trigger_input):
    #     trigger_input = trigger_input/255
    #     ff_torque_output = trigger_input*self.MAX_FEED_FORWARD_TORQUE
    #     self.feed_forward_torque = ff_torque_output
    #     return ff_torque_output
    
    def gradually_increase_speed_left(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time_left
        self.last_time_left = current_time

        # Base rate of speed change
        base_rate = self.GRADUAL_INCREASE_RATE * time_difference

        # Modify the rate based on how close the target speed is to neutral (0)
        # The rate of change is faster when the joystick is near neutral
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35)  # Assuming 35 is the max speed

        # Ensure dynamic_rate does not drop below a certain threshold
        dynamic_rate = max(dynamic_rate, base_rate * self.BASE_DYNAMIC_RATIO)


        if self.current_speed_left < target_speed:
            self.current_speed_left = min(self.current_speed_left+dynamic_rate, target_speed)
        elif self.current_speed_left > target_speed:
            self.current_speed_left = max(self.current_speed_left-dynamic_rate, target_speed)

    def gradually_increase_speed_right(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time_right
        self.last_time_right = current_time

        # Base rate of speed change
        base_rate = self.GRADUAL_INCREASE_RATE * time_difference

        # Modify the rate based on how close the target speed is to neutral (0)
        # The rate of change is faster when the joystick is near neutral
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35)  # Assuming 35 is the max speed
        # Ensure dynamic_rate does not drop below a certain threshold
        dynamic_rate = max(dynamic_rate, base_rate * self.BASE_DYNAMIC_RATIO)

        if self.current_speed_right < target_speed:
            self.current_speed_right = min(self.current_speed_right+dynamic_rate, target_speed)
        elif self.current_speed_right > target_speed:
            self.current_speed_right = max(self.current_speed_right-dynamic_rate, target_speed)
        #self.logger.info(f'{self.current_speed_right}')

    #Script to calculate swerve speed and angles
    def wheelAnglesAndSpeeds(self, Vx, Vy, omega, L, W):
        """calculates the wheel angles and speeds for a swerve module

        Args:
            Vx (double): desired velocity in the X direction, as a porportion of the max velocity
            Vy (double): desired velocity in the Y direction, as a porportion of max velocity
            omega (double): desired angular roll rate
            L (double): wheel base length
            W (double): wheel base width
        """
        R = math.sqrt(L**2 + W**2)
        
        Vx = round(Vx, 2)
        Vy = round(Vy, 2)
        omega = round(omega, 2)

        #define helpful variables A-D
        A = Vy - omega * (L/R)
        B = Vy + omega * (L/R)
        C = Vx - omega * (W/R)
        D = Vx + omega * (W/R)

        #define wheel speeds
        ws1 = math.sqrt(B**2 + C**2)
        ws2 = math.sqrt(B**2 + D**2)
        ws3 = math.sqrt(A**2 + D**2)
        ws4 = math.sqrt(A**2 + C**2)

        #define wheel angles
        wa1 = math.atan2(B,C) * 180 / math.pi
        wa2 = math.atan2(B,D) * 180 / math.pi
        wa3 = math.atan2(A,D) * 180 / math.pi
        wa4 = math.atan2(A,C) * 180 / math.pi

        #normalize wheel speeds
        max = ws1
        if(ws2 > max): max = ws2
        if(ws3 > max): max = ws3
        if(ws4 > max): max = ws4
        if(max > 1): ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max

        ws1 = round(ws1, 3) 
        ws2 = round(ws2, 3) 
        ws3 = round(ws3, 3)
        ws4 = round(ws4, 3) 
        
        wa1 = (round(wa1, 3)) / 360 * 50
        wa2 = (round(wa2, 3)) / 360 * 50
        wa3 = (round(wa3, 3)) / 360 * 50
        wa4 = (round(wa4, 3)) / 360 * 50
        
        return ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4
    
    def listener_callback(self, msg):
        # if msg.twist.angular.z == 2.5: 
        #    output = byte_command_converter(DRIVES, "back_right", None, None, None, None, None, self.get_logger())
        # target_speed_left = scale_value(msg.twist.linear.y, -self.CONTROLLER_MAX_SPEED, self.CONTROLLER_MAX_SPEED, 
        #                         -self.MOTOR_MAX_SPEED, self.MOTOR_MAX_SPEED)
        # target_speed_right = scale_value(msg.twist.angular.y, -self.CONTROLLER_MAX_SPEED, self.CONTROLLER_MAX_SPEED, 
        #                         -self.MOTOR_MAX_SPEED, self.MOTOR_MAX_SPEED)
        # self.gradually_increase_speed_left(target_speed_left)
        # self.gradually_increase_speed_right(target_speed_right)
        # if self.current_speed_right < 0.2 and self.current_speed_right > -0.2:
        #     right_speed = 0
        # else:
        #     right_speed = self.current_speed_right

        # if self.current_speed_left < 0.2 and self.current_speed_left > -0.2:
        #     left_speed = 0
        # else:
        #     left_speed = self.current_speed_left
        # back_right =  byte_command_converter(DRIVES, BACK_RIGHT, None, -left_speed, self.MAX_TORQUE, None,  self.MAX_ACCELERATION, self.feed_forward_torque, self.logger)
        # front_right = byte_command_converter(DRIVES, FRONT_RIGHT, None, -left_speed, self.MAX_TORQUE, None, self.MAX_ACCELERATION, self.feed_forward_torque, self.logger)
        # front_left =  byte_command_converter(DRIVES, FRONT_LEFT, None, left_speed, self.MAX_TORQUE, None,    self.MAX_ACCELERATION, self.feed_forward_torque, self.logger)
        # back_left =   byte_command_converter(DRIVES, BACK_LEFT, None, left_speed, self.MAX_TORQUE, None,     self.MAX_ACCELERATION, self.feed_forward_torque, self.logger)
        # self.logger.info(f'Left Speed: {left_speed}, Right Speed: {right_speed}')
        # send_number(self.serial_port, back_right)
        # send_number(self.serial_port, front_right)
        # send_number(self.serial_port, front_left)
        # send_number(self.serial_port, back_left)
        
        self.controller_command_ly = msg.twist.linear.y
        self.controller_command_lx = msg.twist.linear.x
        self.controller_command_ry = msg.twist.angular.y
        self.controller_command_rx = msg.twist.angular.x
        if abs(msg.twist.linear.y) < 0.1:
            self.controller_command_ly = 0
        if abs(msg.twist.linear.x) < 0.1:
            self.controller_command_lx = 0
        if abs(msg.twist.angular.y) < 0.1:
            self.controller_command_ry = 0
        if abs(msg.twist.angular.x) < 0.1:
            self.controller_command_rx = 0
        

        self.logger.info(f'{self.wheelAnglesAndSpeeds(-self.controller_command_ly, self.controller_command_lx, -self.controller_command_rx, ROVER_LENGTH, ROVER_WIDTH)}')
        ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4 = self.wheelAnglesAndSpeeds(-self.controller_command_ly, self.controller_command_lx, self.controller_command_rx, ROVER_LENGTH, ROVER_WIDTH)
        self.logger.info(f'VEL: {self.velocity}')
        # back_right =  byte_command_converter(DRIVES, BACK_RIGHT,  None, -self.velocity*ws1,  self.MAX_TORQUE, self.MOTOR_MAX_SPEED,  self.MAX_ACCELERATION, 0, self.logger)
        # front_right = byte_command_converter(DRIVES, FRONT_RIGHT, None, -self.velocity*ws4,  self.MAX_TORQUE, self.MOTOR_MAX_SPEED,  self.MAX_ACCELERATION, 0, self.logger)
        # front_left =  byte_command_converter(DRIVES, FRONT_LEFT,  None, self.velocity*ws2, self.MAX_TORQUE, self.MOTOR_MAX_SPEED,  self.MAX_ACCELERATION, 0, self.logger)
        # back_left =   byte_command_converter(DRIVES, BACK_LEFT,   None, self.velocity*ws3, self.MAX_TORQUE, self.MOTOR_MAX_SPEED,  self.MAX_ACCELERATION, 0, self.logger)
        logger = logging.getLogger("DrivesLogger")
        logger.setLevel(logging.INFO)
        
        front_left = moteus.Controller(id=1)
        back_left = moteus.Controller(id=2)
        front_right = moteus.Controller(id=3)
        back_right = moteus.Controller(id=4)

        send_moteus_command_sync(controller=front_left, motor=1, position=math.nan, drives_velocity=(-self.velocity*ws2), maximum_torque=self.MAX_TORQUE, velocity_limit=self.MOTOR_MAX_SPEED,  accel_limit=self.MAX_ACCELERATION, ff_torque=0, logger=logger)
        send_moteus_command_sync(controller=back_left, motor=2, position=math.nan, drives_velocity=(-self.velocity*ws3), maximum_torque=self.MAX_TORQUE, velocity_limit=self.MOTOR_MAX_SPEED,  accel_limit=self.MAX_ACCELERATION, ff_torque=0, logger=logger)
        send_moteus_command_sync(controller=front_right, motor=3, position=math.nan, drives_velocity=(self.velocity*ws4), maximum_torque=self.MAX_TORQUE, velocity_limit=self.MOTOR_MAX_SPEED,  accel_limit=self.MAX_ACCELERATION, ff_torque=0, logger=logger)
        send_moteus_command_sync(controller=back_right, motor=4, position=math.nan, drives_velocity=(self.velocity*ws1), maximum_torque=self.MAX_TORQUE, velocity_limit=self.MOTOR_MAX_SPEED,  accel_limit=self.MAX_ACCELERATION, ff_torque=0, logger=logger)
        # br_swerve = byte_command_converter(ARM, BACK_RIGHT_SWERVE, wa4, None, 5, 120, 120, None, self.logger)
        # fr_swerve = byte_command_converter(ARM, FRONT_RIGHT_SWERVE, wa1, None, 5, 120, 120, None, self.logger)
        # fl_swerve = byte_command_converter(ARM, FRONT_LEFT_SWERVE, wa2, None, 5, 120, 120, None, self.logger)
        # bl_swerve = byte_command_converter(ARM, BACK_LEFT_SWERVE, wa3, None, 5, 120, 120, None, self.logger)
        
        fr_swerve = moteus.Controller(id=7)
        br_swerve = moteus.Controller(id=8)
        bl_swerve = moteus.Controller(id=6)
        fl_swerve = moteus.Controller(id=5)

        send_moteus_command_sync(controller=fr_swerve, motor=7, position=wa1, drives_velocity=None, maximum_torque=10, velocity_limit=60, accel_limit=40, ff_torque=None, logger=logger)
        send_moteus_command_sync(controller=br_swerve, motor=8, position=wa4, drives_velocity=None, maximum_torque=10, velocity_limit=60, accel_limit=40, ff_torque=None, logger=logger)
        send_moteus_command_sync(controller=bl_swerve, motor=6, position=wa3, drives_velocity=None, maximum_torque=10, velocity_limit=60, accel_limit=40, ff_torque=None, logger=logger)
        send_moteus_command_sync(controller=fl_swerve, motor=5, position=wa2, drives_velocity=None, maximum_torque=10, velocity_limit=60, accel_limit=40, ff_torque=None, logger=logger)

        # send_number(self.serial_port, back_right)
        # send_number(self.serial_port, front_right)
        # send_number(self.serial_port, front_left)
        # send_number(self.serial_port, back_left)
        # send_number(self.serial_port, br_swerve)
        # send_number(self.serial_port, fr_swerve)
        # send_number(self.serial_port, fl_swerve)
        # send_number(self.serial_port, bl_swerve)
        
    def autonomy_callback(self, msg):
        vel = msg.vel
        fl_angle = scale_value(msg.fl_angle, -360, 360, -100, 100)
        fr_angle = scale_value(msg.fr_angle, -360, 360, -100, 100)
        bl_angle = scale_value(msg.bl_angle, -360, 360, -100, 100)
        br_angle = scale_value(msg.br_angle, -360, 360, -100, 100)


        back_right =  byte_command_converter(DRIVES, BACK_RIGHT,  None, -vel,  self.MAX_TORQUE, self.MOTOR_MAX_SPEED,  self.MAX_ACCELERATION, 0, self.logger)
        front_right = byte_command_converter(DRIVES, FRONT_RIGHT, None, -vel,  self.MAX_TORQUE, self.MOTOR_MAX_SPEED,  self.MAX_ACCELERATION, 0, self.logger)
        front_left =  byte_command_converter(DRIVES, FRONT_LEFT,  None, vel, self.MAX_TORQUE, self.MOTOR_MAX_SPEED,  self.MAX_ACCELERATION, 0, self.logger)
        back_left =   byte_command_converter(DRIVES, BACK_LEFT,   None, vel, self.MAX_TORQUE, self.MOTOR_MAX_SPEED,  self.MAX_ACCELERATION, 0, self.logger)
        
        br_swerve = byte_command_converter(ARM, BACK_RIGHT_SWERVE, br_angle, None, 5, 120, 120, None, self.logger)
        fr_swerve = byte_command_converter(ARM, FRONT_RIGHT_SWERVE, fr_angle, None, 5, 120, 120, None, self.logger)
        fl_swerve = byte_command_converter(ARM, FRONT_LEFT_SWERVE, fl_angle, None, 5, 120, 120, None, self.logger)
        bl_swerve = byte_command_converter(ARM, BACK_LEFT_SWERVE, bl_angle, None, 5, 120, 120, None, self.logger)
    #     send_number(self.serial_port, back_right)
    #     send_number(self.serial_port, front_right)
    #     send_number(self.serial_port, front_left)
    #     send_number(self.serial_port, back_left)
    #     send_number(self.serial_port, br_swerve)
    #     send_number(self.serial_port, fr_swerve)
    #     send_number(self.serial_port, fl_swerve)
    #     send_number(self.serial_port, bl_swerve)


    def r2TriggerConverter(self, val):
        # Convert the integer to a hexadecimal string
        hex_value = hex(val & 0xFFFFFFFF)  # Mask with 0xFFFFFFFF to handle negative values correctly
        # Take the first two hex digits (after '0x')
        first_two_hex = hex_value[2:4]
        # Convert the two hex digits back to an integer
        result_int = int(first_two_hex, 16)
        return result_int
            
    def listener_button_callback(self, msg):
        trigger_val = msg.button_array[0]
        button_val = msg.button_array[1]
        # self.logger.info(f'button_array: {msg.button_array[0]}')
        R2trigger = self.r2TriggerConverter(trigger_val)
        if R2trigger >= 0 and R2trigger <= 255:
            vel = scale_value(R2trigger, float(0), float(255), 0, self.MOTOR_MAX_SPEED)
            self.velocity = vel 
        if trigger_val >= L2_MIN and trigger_val <= L2:
            vel = scale_value(trigger_val, L2_MIN, L2, 0, self.MOTOR_MAX_SPEED)
            self.velocity = -vel
            # self.logger.info(f'R2: {r2val}')
            
        if trigger_val == L1 and button_val == TRIANGLE: 
            self.current_speed = 0
            self.current_speed_angular = 0

            logger = self.get_logger()

            back_right_stop = byte_command_converter(DRIVES, BACK_RIGHT, None, None, None, None, None, None, self.logger)
            front_right_stop = byte_command_converter(DRIVES, FRONT_RIGHT, None, None, None, None, None, None, self.logger)
            front_left_stop = byte_command_converter(DRIVES, FRONT_LEFT, None, None, None, None, None, None, self.logger)
            back_left_stop = byte_command_converter(DRIVES, BACK_LEFT, None, None, None, None, None, None, self.logger)
            # send_number(self.serial_port, back_right_stop)
            # send_number(self.serial_port, front_right_stop)
            # send_number(self.serial_port, front_left_stop)
            # send_number(self.serial_port, back_left_stop)

            front_left  = moteus.Controller(id=1)
            back_left   = moteus.Controller(id=2)
            front_right = moteus.Controller(id=3)
            back_right  = moteus.Controller(id=4)

            send_moteus_stop_sync(front_left,  motor=1, logger=logger)
            send_moteus_stop_sync(back_left,   motor=2, logger=logger)
            send_moteus_stop_sync(front_right, motor=3, logger=logger)
            send_moteus_stop_sync(back_right,  motor=4, logger=logger)

        if trigger_val == L1 and button_val == CIRCLE: 
            self.turn_thread_lock = not self.turn_thread_lock
            if self.turn_thread_lock == False:
                self.logger.info('Switching to Linear')
            else:
                self.logger.info('Switching to Angular')

def main(args=None):
    rclpy.init(args=args)
    init_moteus_loop()
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
