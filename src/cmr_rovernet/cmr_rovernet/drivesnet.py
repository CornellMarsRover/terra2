import math
import time

import moteus
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from cmr_msgs.msg import AutonomyDrive, ControllerReading
from cmr_rovernet.rovernet_utils import *


class CmdVelSubscriber(Node):
    """
    Subscribes to:
      - /drives_controller/cmd_vel
      - /autonomy_move
      - /drives_controller/cmd_buttons

    Sends drive velocity commands and steer position commands to moteus.
    """

    def __init__(self):
        super().__init__("cmd_vel_subscriber")

        self.create_subscription(
            TwistStamped,
            "/drives_controller/cmd_vel",
            self.listener_callback,
            10,
        )
        self.create_subscription(
            AutonomyDrive,
            "/autonomy_move",
            self.autonomy_callback,
            10,
        )
        self.create_subscription(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            self.listener_button_callback,
            10,
        )

        self.logger = self.get_logger()

        self.current_speed_left = 0.0
        self.current_speed_right = 0.0
        self.turn_thread_lock = False

        self.controller_command_ly = 0.0
        self.controller_command_lx = 0.0
        self.controller_command_ry = 0.0
        self.controller_command_rx = 0.0

        self.last_time_left = time.time()
        self.last_time_right = time.time()

        self.feed_forward_torque = 0.0
        self.velocity = 0.0
        self.swerve = False

        drives_net_table = parse_toml("drivesnet")
        if drives_net_table is None:
            raise RuntimeError("parse_toml('drivesnet') returned None")

        drives_net_node = drives_net_table.get("node")
        if drives_net_node is None:
            raise RuntimeError("drivesnet TOML missing ['node'] section")

        self.CONTROLLER_MAX_SPEED = 2.5
        self.MOTOR_MAX_SPEED = drives_net_node["motor_max_speed"]
        self.GRADUAL_INCREASE_RATE = drives_net_node["gradual_increase_rate"]
        self.BASE_DYNAMIC_RATIO = drives_net_node["base_dynamic_ratio"]
        self.MAX_ACCELERATION = drives_net_node["acceleration_limit"]
        self.MAX_TORQUE = drives_net_node["torque_limit"]
        self.MAX_FEED_FORWARD_TORQUE = drives_net_node["feed_forward_torque_limit"]

        qr = moteus.QueryResolution()
        qr.mode = moteus.INT8
        qr.position = moteus.F32
        qr.velocity = moteus.F32
        qr.torque = moteus.F32
        qr.q_current = moteus.F32


        self.ctrl_drive = {
            "FL": moteus.Controller(id=1, query_resolution=qr),
            "BL": moteus.Controller(id=2, query_resolution=qr),
            "FR": moteus.Controller(id=3, query_resolution=qr),
            "BR": moteus.Controller(id=4, query_resolution=qr),
        }

        self.ctrl_steer = {
            "FL": moteus.Controller(id=5, query_resolution=qr),
            "BL": moteus.Controller(id=6, query_resolution=qr),
            "FR": moteus.Controller(id=7, query_resolution=qr),
            "BR": moteus.Controller(id=8, query_resolution=qr),
        }

        self.logger.info("drivesnet initialized")

    def gradually_increase_speed_left(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time_left
        self.last_time_left = current_time

        base_rate = self.GRADUAL_INCREASE_RATE * time_difference
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35.0)
        dynamic_rate = max(dynamic_rate, base_rate * self.BASE_DYNAMIC_RATIO)

        if self.current_speed_left < target_speed:
            self.current_speed_left = min(self.current_speed_left + dynamic_rate, target_speed)
        elif self.current_speed_left > target_speed:
            self.current_speed_left = max(self.current_speed_left - dynamic_rate, target_speed)

    def gradually_increase_speed_right(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time_right
        self.last_time_right = current_time

        base_rate = self.GRADUAL_INCREASE_RATE * time_difference
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35.0)
        dynamic_rate = max(dynamic_rate, base_rate * self.BASE_DYNAMIC_RATIO)

        if self.current_speed_right < target_speed:
            self.current_speed_right = min(self.current_speed_right + dynamic_rate, target_speed)
        elif self.current_speed_right > target_speed:
            self.current_speed_right = max(self.current_speed_right - dynamic_rate, target_speed)

    def wheelAnglesAndSpeeds(self, Vx, Vy, omega, L, W):
        """
        Calculates normalized wheel speeds and steer angles for a swerve module.
        Returns:
            ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4
        """

        R = math.sqrt(L**2 + W**2)

        Vx = round(Vx, 2)
        Vy = round(Vy, 2)
        omega = round(omega, 2)

        A = Vy - omega * (L / R)
        B = Vy + omega * (L / R)
        C = Vx - omega * (W / R)
        D = Vx + omega * (W / R)

        ws1 = math.sqrt(B**2 + C**2)
        ws2 = math.sqrt(B**2 + D**2)
        ws3 = math.sqrt(A**2 + D**2)
        ws4 = math.sqrt(A**2 + C**2)

        wa1 = math.atan2(B, C) * 180.0 / math.pi
        wa2 = math.atan2(B, D) * 180.0 / math.pi
        wa3 = math.atan2(A, D) * 180.0 / math.pi
        wa4 = math.atan2(A, C) * 180.0 / math.pi

        max_ws = max(ws1, ws2, ws3, ws4)
        if max_ws > 1.0:
            ws1 /= max_ws
            ws2 /= max_ws
            ws3 /= max_ws
            ws4 /= max_ws

        ws1 = round(ws1, 3)
        ws2 = round(ws2, 3)
        ws3 = round(ws3, 3)
        ws4 = round(ws4, 3)

        wa1 = round(wa1, 3) / 360.0 * 50.0
        wa2 = round(wa2, 3) / 360.0 * 50.0
        wa3 = round(wa3, 3) / 360.0 * 50.0
        wa4 = round(wa4, 3) / 360.0 * 50.0

        return ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4

    def send_drive_commands(self, ws1, ws2, ws3, ws4):
        """
        Current mapping preserved from your existing code.
        Change signs or wheel mapping here if a wheel moves backwards.
        """

        send_moteus_command_sync(
            controller=self.ctrl_drive["FL"],
            motor=1,
            position=math.nan,
            drives_velocity=(-self.velocity * ws2),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_drive["BL"],
            motor=2,
            position=math.nan,
            drives_velocity=(-self.velocity * ws3),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_drive["FR"],
            motor=3,
            position=math.nan,
            drives_velocity=(self.velocity * ws4),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_drive["BR"],
            motor=4,
            position=math.nan,
            drives_velocity=(self.velocity * ws1),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )

    def send_steer_commands(self, wa1, wa2, wa3, wa4):
        """
        Current mapping preserved from your existing code:
          wa1 -> FR
          wa2 -> FL
          wa3 -> BL
          wa4 -> BR
        """

        send_moteus_command_sync(
            controller=self.ctrl_steer["FR"],
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
            controller=self.ctrl_steer["BR"],
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
            controller=self.ctrl_steer["BL"],
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
            controller=self.ctrl_steer["FL"],
            motor=5,
            position=wa2,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )

    def listener_callback(self, msg):
        self.controller_command_ly = 0.0 if abs(msg.twist.linear.y) < 0.1 else msg.twist.linear.y
        self.controller_command_lx = 0.0 if abs(msg.twist.linear.x) < 0.1 else msg.twist.linear.x
        self.controller_command_ry = 0.0 if abs(msg.twist.angular.y) < 0.1 else msg.twist.angular.y
        self.controller_command_rx = 0.0 if abs(msg.twist.angular.x) < 0.1 else msg.twist.angular.x

        ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4 = self.wheelAnglesAndSpeeds(
            -self.controller_command_ly,
            self.controller_command_lx,
            self.controller_command_rx,
            ROVER_LENGTH,
            ROVER_WIDTH,
        )

        self.logger.info(
            f"swerve ws=({ws1}, {ws2}, {ws3}, {ws4}) "
            f"wa=({wa1}, {wa2}, {wa3}, {wa4}) vel={self.velocity}"
        )

        self.send_drive_commands(ws1, ws2, ws3, ws4)
        self.send_steer_commands(wa1, wa2, wa3, wa4)

    def autonomy_callback(self, msg):
        """
        Uses autonomy angles directly.
        """

        vel = msg.vel
        fl_angle = scale_value(msg.fl_angle, -360, 360, -100, 100)
        fr_angle = scale_value(msg.fr_angle, -360, 360, -100, 100)
        bl_angle = scale_value(msg.bl_angle, -360, 360, -100, 100)
        br_angle = scale_value(msg.br_angle, -360, 360, -100, 100)

        send_moteus_command_sync(
            controller=self.ctrl_drive["FL"],
            motor=1,
            position=math.nan,
            drives_velocity=vel,
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_drive["BL"],
            motor=2,
            position=math.nan,
            drives_velocity=vel,
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_drive["FR"],
            motor=3,
            position=math.nan,
            drives_velocity=-vel,
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_drive["BR"],
            motor=4,
            position=math.nan,
            drives_velocity=-vel,
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            logger=self.logger,
        )

        send_moteus_command_sync(
            controller=self.ctrl_steer["FL"],
            motor=5,
            position=fl_angle,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_steer["BL"],
            motor=6,
            position=bl_angle,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_steer["FR"],
            motor=7,
            position=fr_angle,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )
        send_moteus_command_sync(
            controller=self.ctrl_steer["BR"],
            motor=8,
            position=br_angle,
            drives_velocity=None,
            maximum_torque=10,
            velocity_limit=60,
            accel_limit=40,
            ff_torque=None,
            logger=self.logger,
        )

    def r2TriggerConverter(self, val):
        hex_value = hex(val & 0xFFFFFFFF)
        first_two_hex = hex_value[2:4]
        return int(first_two_hex, 16)

    def listener_button_callback(self, msg):
        trigger_val = msg.button_array[0]
        button_val = msg.button_array[1]

        r2_trigger = self.r2TriggerConverter(trigger_val)

        if 0 <= r2_trigger <= 255:
            self.velocity = scale_value(r2_trigger, 0.0, 255.0, 0, self.MOTOR_MAX_SPEED)

        if L2_MIN <= trigger_val <= L2:
            self.velocity = -scale_value(trigger_val, L2_MIN, L2, 0, self.MOTOR_MAX_SPEED)

        if trigger_val == L1 and button_val == TRIANGLE:
            self.logger.info("Stopping all drive motors")
            send_moteus_stop_sync(self.ctrl_drive["FL"], motor=1, logger=self.logger)
            send_moteus_stop_sync(self.ctrl_drive["BL"], motor=2, logger=self.logger)
            send_moteus_stop_sync(self.ctrl_drive["FR"], motor=3, logger=self.logger)
            send_moteus_stop_sync(self.ctrl_drive["BR"], motor=4, logger=self.logger)

        if trigger_val == L1 and button_val == CIRCLE:
            self.turn_thread_lock = not self.turn_thread_lock
            if self.turn_thread_lock:
                self.logger.info("Switching to Angular")
            else:
                self.logger.info("Switching to Linear")


def main(args=None):
    rclpy.init(args=args)
    init_moteus_loop()

    node = CmdVelSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
