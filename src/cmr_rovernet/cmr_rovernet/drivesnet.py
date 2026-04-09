import math

import moteus
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import ControllerReading, AutonomyDrive
import math
from cmr_rovernet.rovernet_utils import *


class DrivesnetDiagnostic(Node):
    def __init__(self):
        super().__init__("drivesnet_diagnostic")

        self.create_subscription(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            self.listener_button_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.turn_thread_lock = 0
        self.controller_command_ly = 0
        self.controller_command_lx = 0
        self.controller_command_ry = 0
        self.controller_command_rx = 0
        self.logger = self.get_logger()
        self.velocity_scale = 1.0
        self.deadband = 0.1
        
        # Init constants given TOML file
        drives_net_table = parse_toml("drivesnet")
        drives_net_node = drives_net_table['node']
        self.CONTROLLER_MAX_SPEED = 2.5
        self.MOTOR_MAX_SPEED = float(drives_net_node['motor_max_speed'])
        self.MAX_ACCELERATION = float(drives_net_node['acceleration_limit'])
        self.MAX_TORQUE = float(drives_net_node['torque_limit'])

        qr = moteus.QueryResolution()
        qr.mode = moteus.INT8
        qr.position = moteus.F32
        qr.velocity = moteus.F32
        qr.torque = moteus.F32
        qr.q_current = moteus.F32

        self.drive_controllers = {
            1: moteus.Controller(id=1, query_resolution=qr),
            2: moteus.Controller(id=2, query_resolution=qr),
            3: moteus.Controller(id=3, query_resolution=qr),
            4: moteus.Controller(id=4, query_resolution=qr),
        }
        self.swerve_controllers = {
            5: moteus.Controller(id=5, query_resolution=qr),
            6: moteus.Controller(id=6, query_resolution=qr),
            7: moteus.Controller(id=7, query_resolution=qr),
            8: moteus.Controller(id=8, query_resolution=qr),
        }

    def apply_deadband(self, value):
        if abs(value) < self.deadband:
            return 0.0
        return value

    def compute_drive_scale(self):
        max_input = max(
            abs(self.controller_command_lx),
            abs(self.controller_command_ly),
            abs(self.controller_command_rx),
        )
        normalized = min(1.0, max_input / self.CONTROLLER_MAX_SPEED)
        return normalized * self.velocity_scale * self.MOTOR_MAX_SPEED

    def stop_all_motors(self):
        for motor_id, controller in self.drive_controllers.items():
            send_moteus_stop_sync(controller, motor=motor_id, logger=self.logger)
        for motor_id, controller in self.swerve_controllers.items():
            send_moteus_stop_sync(controller, motor=motor_id, logger=self.logger)

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
        self.controller_command_ly = self.apply_deadband(msg.twist.linear.y)
        self.controller_command_lx = self.apply_deadband(msg.twist.linear.x)
        self.controller_command_ry = self.apply_deadband(msg.twist.angular.y)
        self.controller_command_rx = self.apply_deadband(msg.twist.angular.x)

        drive_speed = self.compute_drive_scale()
        ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4 = self.wheelAnglesAndSpeeds(
            -self.controller_command_ly,
            self.controller_command_lx,
            self.controller_command_rx,
            ROVER_LENGTH,
            ROVER_WIDTH,
        )

        send_moteus_command_sync(
            controller=self.drive_controllers[1],
            motor=1,
            position=math.nan,
            drives_velocity=(-drive_speed * ws2),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.drive_controllers[2],
            motor=2,
            position=math.nan,
            drives_velocity=(-drive_speed * ws3),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.drive_controllers[3],
            motor=3,
            position=math.nan,
            drives_velocity=(drive_speed * ws4),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.drive_controllers[4],
            motor=4,
            position=math.nan,
            drives_velocity=(drive_speed * ws1),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )

        send_moteus_command_sync(
            controller=self.swerve_controllers[6],
            motor=6,
            position=wa3,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.swerve_controllers[7],
            motor=7,
            position=wa1,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.swerve_controllers[8],
            motor=8,
            position=wa4,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.swerve_controllers[5],
            motor=5,
            position=wa2,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )
        
    def autonomy_callback(self, msg):
        self.logger.info("Ignoring legacy /autonomy_move command; teleop uses /drives_controller/cmd_vel")


    def r2TriggerConverter(self, val):
        # Convert the integer to a hexadecimal string
        hex_value = hex(val & 0xFFFFFFFF)  # Mask with 0xFFFFFFFF to handle negative values correctly
        # Take the first two hex digits (after '0x')
        first_two_hex = hex_value[2:4]
        # Convert the two hex digits back to an integer
        result_int = int(first_two_hex, 16)
        return result_int
            
    def listener_button_callback(self, msg):
        trigger_val = int(msg.button_array[0]) if len(msg.button_array) > 0 else 0
        button_val = int(msg.button_array[1]) if len(msg.button_array) > 1 else 0

        if trigger_val == L1 and button_val == TRIANGLE:
            self.velocity_scale = 0.0
            self.stop_all_motors()
            return

        if trigger_val == R1:
            self.velocity_scale = 1.0
        elif trigger_val == L1:
            self.velocity_scale = 0.4

        if trigger_val == L1 and button_val == CIRCLE:
            self.send_drive_test("FR", 3, drive_velocity)
            return

        if trigger_val == L1 and button_val == R1:
            self.send_drive_test("BR", 4, drive_velocity)
            return

        # STEER TESTS
        if trigger_val == R1 and button_val == SQUARE:
            self.send_steer_test("FL", 5, steer_position)
            return

        if trigger_val == R1 and button_val == X:
            self.send_steer_test("BL", 6, steer_position)
            return

        if trigger_val == R1 and button_val == CIRCLE:
            self.send_steer_test("FR", 7, steer_position)
            return

        if trigger_val == R1 and button_val == TRIANGLE:
            self.send_steer_test("BR", 8, steer_position)
            return

        self.logger.info(f"No diagnostic action for trigger={trigger_val}, button={button_val}")


def main(args=None):
    rclpy.init(args=args)
    init_moteus_loop()

    node = DrivesnetDiagnostic()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()