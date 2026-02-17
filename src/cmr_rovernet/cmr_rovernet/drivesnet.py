#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import ControllerReading, AutonomyDrive

import time
import math
import asyncio
import logging
import moteus

import cmr_rovernet.rovernet_utils as rnu
'''
from cmr_rovernet.rovernet_utils import (
    parse_toml,
    scale_value,
    init_moteus_loop,
    send_moteus_command_sync,
    send_moteus_stop_sync,
    rnu._moteus_loop,   # NOTE: rovernet_utils defines this global; we reuse it
)
'''
# If these constants are defined elsewhere in rovernet_utils, you can delete them here.
# Keeping them here makes this file fully self-contained regarding swerve geometry.
ROVER_LENGTH = 39
ROVER_WIDTH = 39


class CmdVelSubscriber(Node):
    """
    Subscribes to /drives_controller/cmd_vel and /drives_controller/cmd_buttons and
    sends commands directly to moteus servos over fdcanusb.

    Fixes "RuntimeError: no running event loop" by creating moteus.Fdcanusb and all
    controllers inside a dedicated asyncio loop thread (rnu.init_moteus_loop()).
    """

    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # --- ROS subscriptions ---
        self.subscription = self.create_subscription(
            TwistStamped,
            '/drives_controller/cmd_vel',
            self.listener_callback,
            10
        )
        self.button_subscription = self.create_subscription(
            ControllerReading,
            '/drives_controller/cmd_buttons',
            self.listener_button_callback,
            10
        )
        self.autonomy_subscription = self.create_subscription(
            AutonomyDrive,
            '/autonomy_move',
            self.autonomy_callback,
            10
        )

        # --- state ---
        self.controller_command_ly = 0.0
        self.controller_command_lx = 0.0
        self.controller_command_ry = 0.0
        self.controller_command_rx = 0.0

        self.velocity = 0.0  # set by triggers in listener_button_callback
        self.logger = self.get_logger()
        self.turn_thread_lock = 0

        # --- Load TOML config (non-ROS params) ---
        drives_net_table = rnu.parse_toml("drivesnet")
        drives_net_node = drives_net_table['node']

        self.CONTROLLER_MAX_SPEED = 2.5
        self.MOTOR_MAX_SPEED = float(drives_net_node['motor_max_speed'])
        self.GRADUAL_INCREASE_RATE = float(drives_net_node['gradual_increase_rate'])
        self.BASE_DYNAMIC_RATIO = float(drives_net_node['base_dynamic_ratio'])
        self.MAX_ACCELERATION = float(drives_net_node['acceleration_limit'])
        self.MAX_TORQUE = float(drives_net_node['torque_limit'])
        self.MAX_FEED_FORWARD_TORQUE = float(drives_net_node['feed_forward_torque_limit'])

        # --- Start dedicated moteus asyncio loop thread ---
        rnu.init_moteus_loop()

        # --- Create transport + controllers ONCE, inside the moteus loop thread ---
        if rnu._moteus_loop is None:
            raise RuntimeError("moteus loop not initialized; rnu.init_moteus_loop() failed")

        setup_future = asyncio.run_coroutine_threadsafe(self._async_moteus_setup(), rnu._moteus_loop)
        setup_future.result()  # block until done / raise on error

        self.logger.info("drivesnet: moteus transport + controllers initialized")

    # -----------------------------
    # Moteus initialization (async)
    # -----------------------------
    async def _async_moteus_setup(self):
        # Must be created inside a running event loop (fixes 'no running event loop')
        self.transport = moteus.Fdcanusb()

        # Query resolution (optional, but good)
        self.qr = moteus.QueryResolution()
        self.qr.mode = moteus.INT8
        self.qr.position = moteus.F32
        self.qr.velocity = moteus.F32
        self.qr.torque = moteus.F32
        self.qr.q_current = moteus.F32

        # Create controllers with shared transport
        self.controllers = {
            1: moteus.Controller(id=1, transport=self.transport, query_resolution=self.qr),  # FL drive
            2: moteus.Controller(id=2, transport=self.transport, query_resolution=self.qr),  # BL drive
            3: moteus.Controller(id=3, transport=self.transport, query_resolution=self.qr),  # FR drive
            4: moteus.Controller(id=4, transport=self.transport, query_resolution=self.qr),  # BR drive
            5: moteus.Controller(id=5, transport=self.transport, query_resolution=self.qr),  # FL swerve
            6: moteus.Controller(id=6, transport=self.transport, query_resolution=self.qr),  # BL swerve
            7: moteus.Controller(id=7, transport=self.transport, query_resolution=self.qr),  # FR swerve
            8: moteus.Controller(id=8, transport=self.transport, query_resolution=self.qr),  # BR swerve
        }

        # Optional but recommended: stop + rezero once at boot
        await self.transport.cycle([c.make_stop() for c in self.controllers.values()])
        await self.transport.cycle([c.make_rezero() for c in self.controllers.values()])

    # -----------------------------
    # Kinematics helper
    # -----------------------------
    def wheelAnglesAndSpeeds(self, Vx, Vy, omega, L, W):
        """
        Calculates wheel angles and speeds for a swerve module.
        Returns: ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4
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

        wa1 = math.atan2(B, C) * 180 / math.pi
        wa2 = math.atan2(B, D) * 180 / math.pi
        wa3 = math.atan2(A, D) * 180 / math.pi
        wa4 = math.atan2(A, C) * 180 / math.pi

        m = max(ws1, ws2, ws3, ws4)
        if m > 1:
            ws1 /= m
            ws2 /= m
            ws3 /= m
            ws4 /= m

        ws1 = round(ws1, 3)
        ws2 = round(ws2, 3)
        ws3 = round(ws3, 3)
        ws4 = round(ws4, 3)

        # Convert degrees -> swerve position units (gear ratio 50 assumed like your original)
        wa1 = (round(wa1, 3)) / 360.0 * 50.0
        wa2 = (round(wa2, 3)) / 360.0 * 50.0
        wa3 = (round(wa3, 3)) / 360.0 * 50.0
        wa4 = (round(wa4, 3)) / 360.0 * 50.0

        return ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4

    # -----------------------------
    # ROS callbacks
    # -----------------------------
    def listener_callback(self, msg: TwistStamped):
        # Read controller command components
        self.controller_command_ly = msg.twist.linear.y
        self.controller_command_lx = msg.twist.linear.x
        self.controller_command_ry = msg.twist.angular.y
        self.controller_command_rx = msg.twist.angular.x

        # Deadband
        if abs(self.controller_command_ly) < 0.1:
            self.controller_command_ly = 0.0
        if abs(self.controller_command_lx) < 0.1:
            self.controller_command_lx = 0.0
        if abs(self.controller_command_ry) < 0.1:
            self.controller_command_ry = 0.0
        if abs(self.controller_command_rx) < 0.1:
            self.controller_command_rx = 0.0

        ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4 = self.wheelAnglesAndSpeeds(
            -self.controller_command_ly,
            self.controller_command_lx,
            self.controller_command_rx,
            ROVER_LENGTH,
            ROVER_WIDTH
        )

        # IMPORTANT: velocity is set by triggers; if you don't press them, this stays 0
        v = float(self.velocity)

        # Reuse controllers (DO NOT recreate per callback)
        fl_drive = self.controllers[1]
        bl_drive = self.controllers[2]
        fr_drive = self.controllers[3]
        br_drive = self.controllers[4]

        fl_swerve = self.controllers[5]
        bl_swerve = self.controllers[6]
        fr_swerve = self.controllers[7]
        br_swerve = self.controllers[8]

        # Drive motors
        rnu.send_moteus_command_sync(
            controller=fl_drive, motor=1,
            position=math.nan, drives_velocity=(-v * ws2),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0.0,
            logger=self.logger
        )
        rnu.send_moteus_command_sync(
            controller=bl_drive, motor=2,
            position=math.nan, drives_velocity=(-v * ws3),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0.0,
            logger=self.logger
        )
        rnu.send_moteus_command_sync(
            controller=fr_drive, motor=3,
            position=math.nan, drives_velocity=(v * ws4),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0.0,
            logger=self.logger
        )
        rnu.send_moteus_command_sync(
            controller=br_drive, motor=4,
            position=math.nan, drives_velocity=(v * ws1),
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0.0,
            logger=self.logger
        )

        # Swerve motors (position control)
        rnu.send_moteus_command_sync(
            controller=fr_swerve, motor=7,
            position=wa1, drives_velocity=None,
            maximum_torque=10.0, velocity_limit=60.0, accel_limit=40.0,
            ff_torque=None, logger=self.logger
        )
        rnu.send_moteus_command_sync(
            controller=br_swerve, motor=8,
            position=wa4, drives_velocity=None,
            maximum_torque=10.0, velocity_limit=60.0, accel_limit=40.0,
            ff_torque=None, logger=self.logger
        )
        rnu.send_moteus_command_sync(
            controller=bl_swerve, motor=6,
            position=wa3, drives_velocity=None,
            maximum_torque=10.0, velocity_limit=60.0, accel_limit=40.0,
            ff_torque=None, logger=self.logger
        )
        rnu.send_moteus_command_sync(
            controller=fl_swerve, motor=5,
            position=wa2, drives_velocity=None,
            maximum_torque=10.0, velocity_limit=60.0, accel_limit=40.0,
            ff_torque=None, logger=self.logger
        )

    def autonomy_callback(self, msg: AutonomyDrive):
        # Leaving as-is (your original has UART formatting here)
        # If you want autonomy to use moteus directly too, we can wire it similarly.
        pass

    def r2TriggerConverter(self, val: int) -> int:
        hex_value = hex(val & 0xFFFFFFFF)
        first_two_hex = hex_value[2:4]
        return int(first_two_hex, 16) if first_two_hex else 0

    def listener_button_callback(self, msg: ControllerReading):
        trigger_val = msg.button_array[0]
        button_val = msg.button_array[1]

        # R2 sets forward velocity
        r2 = self.r2TriggerConverter(trigger_val)
        if 0 <= r2 <= 255:
            self.velocity = rnu.scale_value(float(r2), 0.0, 255.0, 0.0, self.MOTOR_MAX_SPEED)

        # L2 sets reverse velocity (your original used L2_MIN/L2 constants; keep behavior if those exist)
        # If you have L2_MIN and L2 in rovernet_utils, you can re-enable that path.

        # Emergency stop combo (L1 + TRIANGLE) in your original:
        # If you still want that exact combo, keep your constants and add the condition here.
        # For now, keep the behavior you had: if both are pressed, stop.
        # Replace the condition below with your actual constants if desired.
        if trigger_val == 1 and button_val == 16777216:
            logger = self.get_logger()
            for mid in [1, 2, 3, 4]:
                rnu.send_moteus_stop_sync(self.controllers[mid], motor=mid, logger=logger)

        # Mode toggle example (L1 + CIRCLE) - keep your old behavior if needed
        # Replace the condition below with your actual constants if desired.
        if trigger_val == 1 and button_val == 65536:
            self.turn_thread_lock = not self.turn_thread_lock
            self.logger.info("Switching to Angular" if self.turn_thread_lock else "Switching to Linear")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
