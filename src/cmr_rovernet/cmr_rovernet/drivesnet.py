import math

import moteus
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from cmr_msgs.msg import AutonomyDrive, ControllerReading
from cmr_rovernet.rovernet_utils import (
    CIRCLE,
    L1,
    R1,
    ROVER_LENGTH,
    ROVER_WIDTH,
    TRIANGLE,
    init_moteus_loop,
    parse_toml,
    send_moteus_command_sync,
    send_moteus_stop_sync,
)


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__("drivesnet")

        self.create_subscription(
            TwistStamped,
            "/drives_controller/cmd_vel",
            self.listener_callback,
            10,
        )
        self.create_subscription(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            self.listener_button_callback,
            10,
        )
        self.create_subscription(
            AutonomyDrive,
            "/autonomy_move",
            self.autonomy_callback,
            10,
        )

        self.logger = self.get_logger()
        self.turn_thread_lock = False
        self.controller_command_lx = 0.0
        self.controller_command_ly = 0.0
        self.controller_command_rx = 0.0
        self.controller_command_ry = 0.0
        self.velocity_scale = 1.0
        self.deadband = 0.1
        self.last_motion_log = None
        self.last_cmd_vel_log = None
        self.last_buttons_log = None

        drives_net_table = parse_toml("drivesnet")
        drives_net_node = drives_net_table["node"]
        self.CONTROLLER_MAX_SPEED = 2.5
        self.MOTOR_MAX_SPEED = float(drives_net_node["motor_max_speed"])
        self.MAX_ACCELERATION = float(drives_net_node["acceleration_limit"])
        self.MAX_TORQUE = float(drives_net_node["torque_limit"])

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

        self.logger.info("Drivesnet teleop ready")
        self.logger.info(
            f"deadband={self.deadband}, controller_max={self.CONTROLLER_MAX_SPEED}, "
            f"motor_max={self.MOTOR_MAX_SPEED}"
        )
        self.logger.info("Teleop input topic: /drives_controller/cmd_vel")
        self.logger.info("Buttons topic: /drives_controller/cmd_buttons")
        self.logger.info("L1 + Triangle -> stop all motors")
        self.logger.info("R1 -> full speed mode, L1 -> slow mode")

    def apply_deadband(self, value: float) -> float:
        if abs(value) < self.deadband:
            return 0.0
        return value

    def compute_drive_scale(self) -> float:
        max_input = max(
            abs(self.controller_command_lx),
            abs(self.controller_command_ly),
            abs(self.controller_command_rx),
        )
        normalized = min(1.0, max_input / self.CONTROLLER_MAX_SPEED)
        return normalized * self.velocity_scale * self.MOTOR_MAX_SPEED

    def log_motion_summary(self, drive_speed, wheel_speeds, wheel_angles):
        summary = (
            f"sticks raw: lx={self.controller_command_lx:.2f} "
            f"ly={self.controller_command_ly:.2f} "
            f"rx={self.controller_command_rx:.2f} "
            f"ry={self.controller_command_ry:.2f} | "
            f"drive_scale={drive_speed:.2f} vel_scale={self.velocity_scale:.2f} | "
            f"wheel_speeds={wheel_speeds} | swerve={wheel_angles}"
        )
        if summary != self.last_motion_log:
            self.logger.info(summary)
            self.last_motion_log = summary

    def log_cmd_vel(self, msg: TwistStamped):
        summary = (
            f"cmd_vel rx: lx={msg.twist.linear.x:.2f} ly={msg.twist.linear.y:.2f} "
            f"rx={msg.twist.angular.x:.2f} ry={msg.twist.angular.y:.2f}"
        )
        if summary != self.last_cmd_vel_log:
            self.logger.info(summary)
            self.last_cmd_vel_log = summary

    def log_buttons(self, msg: ControllerReading, trigger_val: int, button_val: int):
        summary = (
            "buttons rx: "
            f"raw={list(msg.button_array)} trigger={trigger_val} button={button_val} dpad={msg.dpad}"
        )
        if summary != self.last_buttons_log:
            self.logger.info(summary)
            self.last_buttons_log = summary

    def stop_all_motors(self):
        for motor_id, controller in self.drive_controllers.items():
            send_moteus_stop_sync(controller, motor=motor_id, logger=self.logger)
        for motor_id, controller in self.swerve_controllers.items():
            send_moteus_stop_sync(controller, motor=motor_id, logger=self.logger)

    def wheelAnglesAndSpeeds(self, vx, vy, omega, length, width):
        r = math.sqrt(length**2 + width**2)

        vx = round(vx, 2)
        vy = round(vy, 2)
        omega = round(omega, 2)

        a = vy - omega * (length / r)
        b = vy + omega * (length / r)
        c = vx - omega * (width / r)
        d = vx + omega * (width / r)

        ws1 = math.sqrt(b**2 + c**2)
        ws2 = math.sqrt(b**2 + d**2)
        ws3 = math.sqrt(a**2 + d**2)
        ws4 = math.sqrt(a**2 + c**2)

        wa1 = math.atan2(b, c) * 180 / math.pi
        wa2 = math.atan2(b, d) * 180 / math.pi
        wa3 = math.atan2(a, d) * 180 / math.pi
        wa4 = math.atan2(a, c) * 180 / math.pi

        max_speed = max(ws1, ws2, ws3, ws4)
        if max_speed > 1:
            ws1 /= max_speed
            ws2 /= max_speed
            ws3 /= max_speed
            ws4 /= max_speed

        ws1 = round(ws1, 3)
        ws2 = round(ws2, 3)
        ws3 = round(ws3, 3)
        ws4 = round(ws4, 3)

        wa1 = round(wa1, 3) / 360 * 50
        wa2 = round(wa2, 3) / 360 * 50
        wa3 = round(wa3, 3) / 360 * 50
        wa4 = round(wa4, 3) / 360 * 50

        return ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4

    def listener_callback(self, msg: TwistStamped):
        try:
            self.handle_drive_command(msg)
        except Exception as exc:
            self.logger.error(f"Drive callback failed: {exc!r}")
            self.stop_all_motors()

    def handle_drive_command(self, msg: TwistStamped):
        self.log_cmd_vel(msg)

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

        self.log_motion_summary(
            drive_speed,
            {
                "br": round(ws1, 3),
                "fl": round(ws2, 3),
                "bl": round(ws3, 3),
                "fr": round(ws4, 3),
            },
            {
                "fr": round(wa1, 3),
                "fl": round(wa2, 3),
                "bl": round(wa3, 3),
                "br": round(wa4, 3),
            },
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
            logger=None,
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
            logger=None,
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
            logger=None,
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
            logger=None,
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
            logger=None,
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
            logger=None,
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
            logger=None,
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
            logger=None,
        )

    def autonomy_callback(self, msg: AutonomyDrive):
        self.logger.info(
            "Ignoring legacy /autonomy_move command; teleop uses /drives_controller/cmd_vel"
        )

    def listener_button_callback(self, msg: ControllerReading):
        trigger_val = int(msg.button_array[0]) if len(msg.button_array) > 0 else 0
        button_val = int(msg.button_array[1]) if len(msg.button_array) > 1 else 0
        self.log_buttons(msg, trigger_val, button_val)

        if trigger_val == L1 and button_val == TRIANGLE:
            self.velocity_scale = 0.0
            self.logger.info("L1+Triangle detected -> STOP ALL")
            self.stop_all_motors()
            return

        if trigger_val == R1:
            self.velocity_scale = 1.0
            self.logger.info("R1 held -> full speed mode")
        elif trigger_val == L1:
            self.velocity_scale = 0.4
            self.logger.info("L1 held -> slow mode")

        if trigger_val == L1 and button_val == CIRCLE:
            self.turn_thread_lock = not self.turn_thread_lock
            if self.turn_thread_lock:
                self.logger.info("Switching to Angular")
            else:
                self.logger.info("Switching to Linear")
        elif trigger_val != 0 or button_val != 0:
            self.logger.info("Button combo did not match a special action")


def main(args=None):
    rclpy.init(args=args)
    init_moteus_loop()
    node = CmdVelSubscriber()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.logger.info("drivesnet shutting down -> stopping all motors")
            node.stop_all_motors()
        except Exception as exc:
            node.logger.error(f"Failed to stop motors during shutdown: {exc!r}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
