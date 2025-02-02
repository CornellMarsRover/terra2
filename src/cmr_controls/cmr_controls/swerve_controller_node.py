import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from cmr_msgs.msg import AutonomyDrive

import asyncio
import math
import moteus
import yaml
import os
import time

class SwerveControllerNode(Node):
    def __init__(self):
        super().__init__('swerve_controller_node')

        # Constants
        self.L = 39
        self.W = 20
        self.swerve_motor_ids = None
        self.drive_motor_ids = None
        self.swerves_max_torque = 5
        self.swerves_max_vel = 120
        self.swerves_max_acc = 120
        self.drives_max_torque = 8
        self.drives_max_vel = 1
        self.drives_max_acc = 1
        self.velocity = 0

        self.angles = {"FRONTLEFT" : 0, "BACKLEFT" : 0, "FRONTRIGHT" : 0, "BACKRIGHT" : 0}
        self.velocities = {"FRONTLEFT" : 0, "BACKLEFT" : 0, "FRONTRIGHT" : 0, "BACKRIGHT" : 0}
        self._id_to_wheel = {19 : "FRONTLEFT",
                             18 : "BACKLEFT",
                          16 : "FRONTRIGHT",
                          17 : "BACKRIGHT",
                          1 : "FRONTLEFT",
                          2 : "BACKLEFT",
                          3 : "FRONTRIGHT",
                          4  : "BACKRIGHT"}

        # Create a subscription to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Autonomy drive command subscriptions
        self.subscription = self.create_subscription(
            Twist,
            '/autonomy/move/point_turn',
            self.point_turn_callback,
            10
        )
        self.subscription = self.create_subscription(
            AutonomyDrive,
            '/autonomy/move/ackerman',
            self.ackerman_callback,
            10
        )

        self.pt_turn_constants = {
            'wa1': (-62.85)/3.6,
            'wa2': (62.85)/3.6,
            'wa3': (-62.85)/3.6,
            'wa4': (62.85)/3.6
        }
        
        self.transport = None
        self.servos = None

        self.loop = asyncio.get_event_loop()

        self.loop.run_until_complete(self.__async_initialize_moteus())

        self.get_logger().info('Swerve Controller Node has been started.')


    def cmd_vel_callback(self, msg):
        Vx = msg.linear.x
        Vy = msg.linear.y
        omega = msg.angular.z

        # Compute wheel speeds and angles
        ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4 = self.wheelAnglesAndSpeeds(Vx, Vy, omega, self.L, self.W)
        self.set_drive(ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4)

    def point_turn_callback(self, msg):
        v = msg.angular.z
        
        self.set_drive(v,v,v,v,
                       self.pt_turn_constants['wa1'],
                       self.pt_turn_constants['wa2'],
                       self.pt_turn_constants['wa3'],
                       self.pt_turn_constants['wa4'])

    def ackerman_callback(self, msg):
        '''
        Ackerman drive command that sets wheel positions directly
        Values are reversed so the rover drives backwards
        '''
        s = -1.0 * msg.vel
        wa3 = -1.0 * (msg.fl_angle / 360) * 100
        wa4 = -1.0 * (msg.fr_angle / 360) * 100
        wa2 = 0.0
        wa1 = 0.0
        self.set_drive(-1 * s, s, s, -1 *  s, wa1, wa2, wa3, wa4)
        
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
        
        wa1 = (round(wa1, 3)) / 360 * 100
        wa2 = (round(wa2, 3)) / 360 * 100
        wa3 = (round(wa3, 3)) / 360 * 100
        wa4 = (round(wa4, 3)) / 360 * 100
        k=3.6
        self.get_logger().info(f"FL: {wa2*k} FR: {wa1*k}\nBL: {wa3*k} BR: {wa4*k}")

        return -1 * ws1, ws2, ws3, -1 * ws4, wa1, wa2, wa3, wa4
    

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
        self.swerves_max_vel = 120
        self.swerves_max_acc = 120
        self.drives_max_torque = 8
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
        '''for servo in result:
            wheel = self._id_to_wheel[servo.id]
            self.velocities[wheel] = servo.velocity
            self.get_logger().info(str(servo))'''
        #self.get_logger().info(str(result))


def main(args=None):
    rclpy.init(args=args)
    node = SwerveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        #node.destroy_transport()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
