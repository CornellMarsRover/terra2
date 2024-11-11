'''import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import math
import moteus
import yaml
import os
import time

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')

        # Constants
        self.L = 39
        self.W = 20
        self.swerve_motor_ids = None
        self.drive_motor_ids = None
        self.max_torque = 5
        self.max_vel = 120
        self.max_acc = 120
        self.velocity = 0
        # Dictionary with motor control info for each joint 
        # pulled from armnet in the following format:
        # (motor_id, max_velocity, max_acceleration, max_torque)
        self._joint_info = {
            "ARM_BASE": (0x09, ,
            "ARM_SHOULDER": 0x0B,
            "ARM_ELBOW": 0x0A,
            "WRIST_ROTATE_1": 0x0C,
            "WRIST_ROTATE_2": 0x0E,
            "WRIST_TILT": 0x0D
        }


        self.joint_limits = {
            "ARM_BASE": (),
            "ARM_SHOULDER": 0x0B,
            "ARM_ELBOW": 0x0A,
            "WRIST_ROTATE_1": 0x0C,
            "WRIST_ROTATE_2": 0x0E,
            "WRIST_TILT": 0x0D
        }


        # Create a subscription to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.transport = None
        self.servos = None

        self.loop = asyncio.get_event_loop()

        self.loop.run_until_complete(self.__async_initialize_moteus())

        self.get_logger().info('Swerve Controller Node has been started.')

    

    async def __async_initialize_moteus(self):
        self.transport = moteus.Fdcanusb()
        # 1,  2,  3,  4  = drives: front left, back left, front right, back right
        # 16, 17, 18, 19 = swerves: front right, back right, back left, front left
        self.servos = {
            servo_id : moteus.Controller(id=servo_id, transport=self.transport) for servo_id in [1,2,3,4,16,17,18,19]
        }

        # reset servo positions
        await self.transport.cycle([x.make_stop() for x in self.servos.values()])

    def set_drive(self, ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4):
        return self.loop.run_until_complete(self.__async_set_drive(ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4))
    

    async def __async_set_drive(self, ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4):
        self.swerves_max_torque = 5
        self.max_vel = 120
        self.max_acc = 120
        self.max_torque = 8
        self.drives_max_vel = 1
        self.drives_max_acc = 1

        # Initialize Moteus transport and controllers

        now = time.time()
        commands = [
            self.servos[1].make_position(
                position = math.nan,
                query=True,
                velocity = ws2,
                accel_limit=self.drives_max_acc,
                velocity_limit=self.drives_max_vel,
                maximum_torque=self.drives_max_torque 
            ),
            self.servos[2].make_position(
                position = math.nan,
                query=True,
                velocity = ws3,
                accel_limit=self.drives_max_acc,
                velocity_limit=self.drives_max_vel,
                maximum_torque=self.drives_max_torque
            ),
            self.servos[3].make_position(
                position = math.nan,
                query=True,
                velocity = ws4,
                accel_limit=self.drives_max_acc,
                velocity_limit=self.drives_max_vel,
                maximum_torque=self.drives_max_torque
            ),
            self.servos[4].make_position(
                position = math.nan,
                query=True,
                velocity = ws1,
                accel_limit=self.drives_max_acc,
                velocity_limit=self.drives_max_vel,
                maximum_torque=self.drives_max_torque
            ),
            self.servos[16].make_position(
                position = wa1,
                query=True,
                accel_limit=self.swerves_max_acc,
                velocity_limit=self.swerves_max_vel,
                maximum_torque=self.swerves_max_torque 
            ),
            self.servos[17].make_position(
                position = wa4,
                query=True,
                accel_limit=self.swerves_max_acc,
                velocity_limit=self.swerves_max_vel,
                maximum_torque=self.swerves_max_torque
            ),
            self.servos[18].make_position(
                position = wa3,
                query=True,
                accel_limit=self.swerves_max_acc,
                velocity_limit=self.swerves_max_vel,
                maximum_torque=self.swerves_max_torque
            ),
            self.servos[19].make_position(
                position = wa2,
                query=True,
                accel_limit=self.swerves_max_acc,
                velocity_limit=self.swerves_max_vel,
                maximum_torque=self.swerves_max_torque
            )
        ]
        result = await self.transport.cycle(commands)



def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''