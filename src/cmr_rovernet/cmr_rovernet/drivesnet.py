import math
import logging

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import ControllerReading

import moteus

from cmr_rovernet.rovernet_utils import (
    parse_toml,
    scale_value,
    init_moteus_loop,
    send_moteus_command_sync,
    send_moteus_stop_sync,
    ROVER_LENGTH,
    ROVER_WIDTH,
    L1,
    L2,
    L2_MIN,
    R2,
    TRIANGLE,
)


class DrivesNet(Node):
    """
    Minimal swerve drive node.

    Subscribes:
      /drives_controller/cmd_vel
      /drives_controller/cmd_buttons

    Sends commands directly to 8 moteus controllers:
      Drive:  1 FL, 2 BL, 3 FR, 4 BR
      Swerve: 5 FL, 6 BL, 7 FR, 8 BR
    """

    def __init__(self):
        super().__init__("drivesnet")

        self.create_subscription(
            TwistStamped,
            "/drives_controller/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            self.buttons_callback,
            10,
        )

        self.logger = self.get_logger()
        self.pylogger = logging.getLogger("drivesnet")
        self.pylogger.setLevel(logging.INFO)

        drives_net_table = parse_toml("drivesnet")
        node_cfg = drives_net_table["node"]

        self.CONTROLLER_MAX_SPEED = 2.5
        self.MOTOR_MAX_SPEED = node_cfg["motor_max_speed"]
        self.MAX_ACCELERATION = node_cfg["acceleration_limit"]
        self.MAX_TORQUE = node_cfg["torque_limit"]

        self.velocity = 0.0

        # Reuse one query resolution for all controllers
        qr = moteus.QueryResolution()
        qr.mode = moteus.INT8
        qr.position = moteus.F32
        qr.velocity = moteus.F32
        qr.torque = moteus.F32
        qr.q_current = moteus.F32

        # Create controllers once
        self.drive_fl = moteus.Controller(id=1, query_resolution=qr)
        self.drive_bl = moteus.Controller(id=2, query_resolution=qr)
        self.drive_fr = moteus.Controller(id=3, query_resolution=qr)
        self.drive_br = moteus.Controller(id=4, query_resolution=qr)

        self.swerve_fl = moteus.Controller(id=5, query_resolution=qr)
        self.swerve_bl = moteus.Controller(id=6, query_resolution=qr)
        self.swerve_fr = moteus.Controller(id=7, query_resolution=qr)
        self.swerve_br = moteus.Controller(id=8, query_resolution=qr)

        self.logger.info("drivesnet minimal node started")

    def deadband(self, value: float, threshold: float = 0.1) -> float:
        return 0.0 if abs(value) < threshold else value

    def angle_deg_to_moteus_pos(self, angle_deg: float) -> float:
        """
        Preserve your current scaling behavior:
        degrees -> [-25, 25] style command space via /360 * 50
        """
        return (angle_deg / 360.0) * 50.0

    def wheel_angles_and_speeds(self, vx: float, vy: float, omega: float, L: float, W: float):
        """
        Returns:
          ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4

        Mapping preserved from your original file.
        """
        R = math.sqrt(L**2 + W**2)

        A = Vy = vy - omega * (L / R)
        B = vy + omega * (L / R)
        C = vx - omega * (W / R)
        D = vx + omega * (W / R)

        ws1 = math.sqrt(B**2 + C**2)
        ws2 = math.sqrt(B**2 + D**2)
        ws3 = math.sqrt(A**2 + D**2)
        ws4 = math.sqrt(A**2 + C**2)

        wa1_deg = math.degrees(math.atan2(B, C))
        wa2_deg = math.degrees(math.atan2(B, D))
        wa3_deg = math.degrees(math.atan2(A, D))
        wa4_deg = math.degrees(math.atan2(A, C))

        max_ws = max(ws1, ws2, ws3, ws4)
        if max_ws > 1.0:
            ws1 /= max_ws
            ws2 /= max_ws
            ws3 /= max_ws
            ws4 /= max_ws

        wa1 = self.angle_deg_to_moteus_pos(wa1_deg)
        wa2 = self.angle_deg_to_moteus_pos(wa2_deg)
        wa3 = self.angle_deg_to_moteus_pos(wa3_deg)
        wa4 = self.angle_deg_to_moteus_pos(wa4_deg)

        return ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4

    def send_drive(self, controller, motor_id: int, velocity: float, name: str):
        try:
            send_moteus_command_sync(
                controller=controller,
                motor=motor_id,
                position=math.nan,
                drives_velocity=velocity,
                maximum_torque=self.MAX_TORQUE,
                velocity_limit=self.MOTOR_MAX_SPEED,
                accel_limit=self.MAX_ACCELERATION,
                ff_torque=0,
                logger=self.logger,
            )
        except Exception as e:
            self.logger.error(f"drive send failed: {name} id={motor_id} vel={velocity} err={e}")

    def send_swerve(self, controller, motor_id: int, position: float, name: str):
        try:
            send_moteus_command_sync(
                controller=controller,
                motor=motor_id,
                position=position,
                drives_velocity=None,
                maximum_torque=10,
                velocity_limit=60,
                accel_limit=40,
                ff_torque=None,
                logger=self.logger,
            )
        except Exception as e:
            self.logger.error(f"swerve send failed: {name} id={motor_id} pos={position} err={e}")

    def stop_all_drive_motors(self):
        for controller, motor_id, name in [
            (self.drive_fl, 1, "FL drive"),
            (self.drive_bl, 2, "BL drive"),
            (self.drive_fr, 3, "FR drive"),
            (self.drive_br, 4, "BR drive"),
        ]:
            try:
                send_moteus_stop_sync(controller, motor=motor_id, logger=self.logger)
            except Exception as e:
                self.logger.error(f"stop failed: {name} id={motor_id} err={e}")

    def cmd_vel_callback(self, msg: TwistStamped):
        ly = self.deadband(msg.twist.linear.y)
        lx = self.deadband(msg.twist.linear.x)
        rx = self.deadband(msg.twist.angular.x)

        # Preserve your original convention as closely as possible
        ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4 = self.wheel_angles_and_speeds(
            -ly,   # vx
            lx,    # vy
            rx,    # omega
            ROVER_LENGTH,
            ROVER_WIDTH,
        )

        self.logger.info(
            f"vel={self.velocity:.3f} "
            f"drive_ws=({ws1:.3f}, {ws2:.3f}, {ws3:.3f}, {ws4:.3f}) "
            f"swerve_pos=({wa1:.3f}, {wa2:.3f}, {wa3:.3f}, {wa4:.3f})"
        )

        # Drive mapping preserved from your original code
        self.send_drive(self.drive_fl, 1, -self.velocity * ws2, "FL drive")
        self.send_drive(self.drive_bl, 2, -self.velocity * ws3, "BL drive")
        self.send_drive(self.drive_fr, 3,  self.velocity * ws4, "FR drive")
        self.send_drive(self.drive_br, 4,  self.velocity * ws1, "BR drive")

        # Swerve mapping preserved from your original code
        self.send_swerve(self.swerve_bl, 6, wa3, "BL swerve")
        self.send_swerve(self.swerve_fr, 7, wa1, "FR swerve")
        self.send_swerve(self.swerve_br, 8, wa4, "BR swerve")
        self.send_swerve(self.swerve_fl, 5, wa2, "FL swerve")

    def buttons_callback(self, msg: ControllerReading):
        """
        Expected button_array layout from your remote script:
          [L1, R1, L2, R2, square, cross, circle, triangle]
        """
        if len(msg.button_array) < 8:
            self.logger.warn("button_array too short")
            return

        l1 = msg.button_array[0]
        l2 = msg.button_array[2]
        r2 = msg.button_array[3]
        triangle = msg.button_array[7]

        # Forward velocity from R2
        if 0 <= r2 <= 255:
            self.velocity = scale_value(float(r2), 0.0, 255.0, 0.0, self.MOTOR_MAX_SPEED)

        # Reverse velocity from L2
        if L2_MIN <= l2 <= L2:
            self.velocity = -scale_value(float(l2), float(L2_MIN), float(L2), 0.0, self.MOTOR_MAX_SPEED)

        # Emergency stop: L1 + Triangle
        if l1 == L1 and triangle == TRIANGLE:
            self.velocity = 0.0
            self.stop_all_drive_motors()
            self.logger.info("Drive stop requested")


def main(args=None):
    rclpy.init(args=args)
    init_moteus_loop()

    node = DrivesNet()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()