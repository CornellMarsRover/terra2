import rclpy
from rclpy.node import Node

from cmr_msgs.msg import DriveCommand

import asyncio
import math
import moteus

WHEEL_RADIUS = 0.127 # m
WHEEL_CIRCUMFERENCE = WHEEL_RADIUS*2*math.pi
SWERVE_RATIO = 50 # Swerve gearbox ratio
DRIVE_RATIO = 26 # Drive gearbox ratio

class SwerveControllerNode(Node):
    def __init__(self):
        super().__init__('swerve_controller_node')

        # Drive constants
        self.swerve_motor_ids = None
        self.drive_motor_ids = None
        self.swerves_max_torque = 6
        self.swerves_max_vel = 120
        self.swerves_max_acc = 120
        self.drives_max_torque = 6
        self.drives_max_vel = 300
        self.drives_max_acc = 150

        self.angles = {"FRONTLEFT" : 0, "BACKLEFT" : 0, "FRONTRIGHT" : 0, "BACKRIGHT" : 0}
        self.velocities = {"FRONTLEFT" : 0, "BACKLEFT" : 0, "FRONTRIGHT" : 0, "BACKRIGHT" : 0}
        self._id_to_wheel = {5 : "FL_SWERVE",
                             6 : "BL_SWERVE",
                          7 : "FR_SWERVE", 
                          8 : "BR_SWERVE",
                          1 : "FL_DRIVE",
                          2 : "BL_DRIVE",
                          3 : "FR_DRIVE",
                          4  : "BR_DRIVE"}

        self.subscription = self.create_subscription(
            DriveCommand,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.transport = None
        self.servos = None
        self.loop = asyncio.get_event_loop()

        self.loop.run_until_complete(self.__async_initialize_moteus())
        
        self.get_logger().info('Swerve Controller Node has been started.')

    def cmd_vel_callback(self, msg):
        """
        Regular command velocity callback
        """
        scale = msg.speed_rps * WHEEL_CIRCUMFERENCE / DRIVE_RATIO
        s1, s2, s3, s4, a1, a2, a3, a4 = self.wheelAnglesAndSpeeds(
            msg.vx * scale, msg.vy * scale, msg.omega * scale, 1.0, 1.0)
        self.set_drive(s1, s2, s3, s4, a1, a2, a3, a4)

    def stop_wheels(self):
        #self.get_logger().info("STOPPING WHEELS")
        return self.loop.run_until_complete(self.__async_stop_wheels())

    async def __async_initialize_moteus(self):
        self.transport = moteus.Fdcanusb()
        # 1, 2, 3, 4  = drives: front left, back left, front right, back right
        # 5, 6, 7, 8 = swerves: front left, back left, front right, back right
        self.servos = {
            servo_id : moteus.Controller(id=servo_id, transport=self.transport) for servo_id in [1,2,3,4,5,6,7,8]
        }

        # reset servo positions
        await self.transport.cycle([x.make_stop() for x in self.servos.values()])
        #for x in self.servos.values():
        #    await x.set_stop()
        await self.transport.cycle([x.make_rezero() for x in self.servos.values()])

    async def __async_stop_wheels(self):
        '''wheels = {1, 3, 4} # no 2 for now
        rezeros = []
        for x in self.servos.keys():
            if x in wheels:
                rezeros.append(self.servos[x].set_stop())
        self.get_logger().info(f"REZEROING WHEELS {rezeros}")
        await self.transport.cycle(rezeros)
        self.get_logger().info("REZEROS SENT")'''
        await self.transport.cycle([x.make_stop() for x in self.servos.values()])


    def set_drive(self, ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4):
        return self.loop.run_until_complete(self.__async_set_drive(ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4))
    

    async def __async_set_drive(self, ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4):
        #self.get_logger().info(f"{ws1} {ws2} {ws3} {ws4}")

        # Initialize Moteus transport and controllers
        '''self.servos[2].make_position(
            position = math.nan,
            query=False,
            velocity = ws3,
            accel_limit=self.drives_max_acc,
            velocity_limit=self.drives_max_vel,
            maximum_torque=self.drives_max_torque
        ),'''
        commands = [
            self.servos[1].make_position(
                position = math.nan,
                query=False,
                velocity = ws2,
                accel_limit=self.drives_max_acc,
                velocity_limit=self.drives_max_vel,
                maximum_torque=self.drives_max_torque 
            ),
            self.servos[3].make_position(
                position = math.nan,
                query=False,
                velocity = ws4,
                accel_limit=self.drives_max_acc,
                velocity_limit=self.drives_max_vel,
                maximum_torque=self.drives_max_torque
            ),
            self.servos[4].make_position(
                position = math.nan,
                query=False,
                velocity = ws1,
                accel_limit=self.drives_max_acc,
                velocity_limit=self.drives_max_vel,
                maximum_torque=self.drives_max_torque
            ),
            self.servos[5].make_position(
                position = wa2,
                query=False,
                accel_limit=self.swerves_max_acc,
                velocity_limit=self.swerves_max_vel,
                maximum_torque=self.swerves_max_torque
            ),
            self.servos[6].make_position(
                position = wa3,
                query=False,
                accel_limit=self.swerves_max_acc,
                velocity_limit=self.swerves_max_vel,
                maximum_torque=self.swerves_max_torque
            ),
            self.servos[7].make_position(
                position = wa1,
                query=False,
                accel_limit=self.swerves_max_acc,
                velocity_limit=self.swerves_max_vel,
                maximum_torque=self.swerves_max_torque 
            ),
            self.servos[8].make_position(
                position = wa4,
                query=False,
                accel_limit=self.swerves_max_acc,
                velocity_limit=self.swerves_max_vel,
                maximum_torque=self.swerves_max_torque
            )
        ]
        await self.transport.cycle(commands)


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

        # helper to ensure wheels stay between 90 and -90 deg
        def bound_wheels(a, s):
            a_bounded = a
            if abs(a) > 90:
                if a < 0:
                    a_bounded = 180 + a
                else:
                    a_bounded = a - 180
                s *= -1
            return a_bounded, -1*s
        
        a1, s1 = bound_wheels(wa1, ws1)
        a2, s2 = bound_wheels(wa2, ws2)
        a3, s3 = bound_wheels(wa3, ws3)
        a4, s4 = bound_wheels(wa4, ws4)
        self.get_logger().info(f"s1: {s1} a1: {a1}\ns2: {s2} a2: {a2}\ns3: {s3} a3: {a3}\n s4: {s4} a4: {a4}")
        s1 *= DRIVE_RATIO/WHEEL_CIRCUMFERENCE
        s2 *= DRIVE_RATIO/WHEEL_CIRCUMFERENCE
        s3 *= DRIVE_RATIO/WHEEL_CIRCUMFERENCE
        s4 *= DRIVE_RATIO/WHEEL_CIRCUMFERENCE
        
        a1 *= SWERVE_RATIO/360.0
        a2 *= SWERVE_RATIO/360.0
        a3 *= SWERVE_RATIO/360.0
        a4 *= SWERVE_RATIO/360.0
        

        return -1 * s1, s2, s3, -1 * s4, a1, a2, a3, a4

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
