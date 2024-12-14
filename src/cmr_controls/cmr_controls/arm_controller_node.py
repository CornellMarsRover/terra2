import rclpy
from rclpy.node import Node
import asyncio
import math
import moteus
import yaml
import os
import time
import numpy as np
from std_msgs.msg import Float32MultiArray

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')

        # Constants
        self.max_torque = 0.7
        self.max_vel = 6.0
        self.max_acc = 5.0

        # Dictionary with motor ids for arm joints
        # Pulled from armnet but last two should be swapped
        self._joint_info = {
            "ARM_BASE": 0x09, 
            "ARM_SHOULDER": 0x0B,
            "ARM_ELBOW": 0x0A,
            "WRIST_ROTATE_1": 0x0C,
            "WRIST_ROTATE_2": 0x0E,
            "WRIST_TILT": 0x0D
        }

        # Dictionary with motor controller constants
        self.joint_motor_info = {
            0: (0x09, 100, 1, 0.0),
            1: (0x0B, 100, -1, 0.0),
            2: (0x0A, 100, 1, 0.0),
            3: (0x0C, 50, 1, 0.0),
            4: (0x0D, 50, 1, 0.0),
            5: (0x0E, 50, 1, 0.0)
        }

        # Create a subscription to the desired joint angles topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_angles/desired',
            self.arm_control_callback,
            10)

        self.calibration_callback = self.create_subscription(
            Float32MultiArray,
            '/joint_angles/offsets',
            self.arm_offset_callback,
            10)

        # Create a subscription to the joint increment topic
        '''self.increment_subscription = self.create_subscription(
            Float32MultiArray,
            '/arm/joint_increment',
            self.joint_increment_callback,
            10)'''

        self.motors_free = True
        self.loop = asyncio.get_event_loop()

        self.loop.run_until_complete(self.__async_initialize_moteus())

        # Initialize current joint angles to zero
        self.curr_positions = np.zeros(6)
        self.set_joints(*self.curr_positions)

        self.get_logger().info('Arm Controller Node has been started.')

    def angle_to_motor_position(self, joint_num, angle):
        ratio = self.joint_motor_info[joint_num][1]
        swap = self.joint_motor_info[joint_num][2]
        offset = self.joint_motor_info[joint_num][3]
        return ((angle+offset) * (ratio/360) * swap)

    def motor_position_to_angle(self, joint_num, position):
        ratio = self.joint_motor_info[joint_num][1]
        swap = self.joint_motor_info[joint_num][2]
        return (position * (360/ratio) * swap)

    def arm_offset_callback(self,msg):
        i = int(msg.data[0])
        offset = msg.data[1]
        curr = self.joint_motor_info[i]
        new = (curr[0], curr[1], curr[2], offset)
        self.joint_motor_info[i] = new
        positions = []
        for i in range(6):
            position = self.angle_to_motor_position(i, 0.0)
            positions.append(position)

        self.curr_positions = np.array(positions)
        self.set_joints(*positions)

    def arm_control_callback(self, msg):
        positions = []
        for i in range(6):
            angle = msg.data[i]
            position = self.angle_to_motor_position(i, angle)
            positions.append(position)

        self.curr_positions = np.array(positions)
        self.set_joints(*positions)

    '''
    def joint_increment_callback(self, msg):
        """
        Callback to handle joint angle increments.

        :param msg: Float32MultiArray containing increments for each joint.
        """
        positions = []
        for i in range(6):
            increment = msg.data[i]
            self.curr_positions[i] += self.angle_to_motor_position(i,increment)
            
        # Send commands to the motors
        self.set_joints(*self.curr_positions.tolist())
    '''

    def set_joints(self, p1, p2, p3, p4, p5, p6):
        return self.loop.run_until_complete(self.__async_set_joints(p1, p2, p3, p4, p5, p6))
    
    async def __async_initialize_moteus(self):
        self.transport = moteus.Fdcanusb()
        self.servos = {
            servo_id : moteus.Controller(id=servo_id, transport=self.transport) for servo_id in [0x09,0x0B,0x0A,0x0C,0x0E,0x0D]
        }

        # reset servo positions
        await self.transport.cycle([x.make_stop() for x in self.servos.values()])
        await self.transport.cycle([x.make_rezero() for x in self.servos.values()])

    async def __async_set_joints(self, p1, p2, p3, p4, p5, p6):
        self.motors_free = False
        self.max_vel = 6.0
        self.max_acc = 5.0
        self.max_torque = 0.7
        
        commands = []
        commands = [
            self.servos[0x09].make_position(
                position = p1,
                query=True,
                accel_limit=self.max_acc,
                velocity_limit=self.max_vel,
                maximum_torque=self.max_torque 
            ),
            self.servos[0x0B].make_position(
                position = p2,
                query=True,
                accel_limit=self.max_acc,
                velocity_limit=self.max_vel,
                maximum_torque=self.max_torque 
            ),
            self.servos[0x0A].make_position(
                position = p3,
                query=True,
                accel_limit=self.max_acc,
                velocity_limit=self.max_vel,
                maximum_torque=self.max_torque 
            ),
            self.servos[0x0C].make_position(
                position = p4,
                query=True,
                accel_limit=self.max_acc,
                velocity_limit=self.max_vel,
                maximum_torque=self.max_torque 
            ),
            self.servos[0x0D].make_position(
                position = p5,
                query=True,
                accel_limit=self.max_acc,
                velocity_limit=self.max_vel,
                maximum_torque=self.max_torque 
            ),
            self.servos[0x0E].make_position(
                position = p6,
                query=True,
                accel_limit=self.max_acc,
                velocity_limit=self.max_vel,
                maximum_torque=self.max_torque 
            )
        ]



        result = await self.transport.cycle(commands)
        #angles = []
        curr_positions = []
        for i in range(6):
            id = self.joint_motor_info[i][0]
            #pos = result[i].values[moteus.Register.POSITION]
            #angles.append(self.motor_position_to_angle(i, pos))
            curr_positions.append(result[i].values[moteus.Register.POSITION])

        #self.curr_positions = np.array(curr_positions)
        

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
