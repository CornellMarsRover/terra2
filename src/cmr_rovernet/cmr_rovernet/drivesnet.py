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
    X,
    init_moteus_loop,
    parse_toml,
    query_moteus_sync,
    send_moteus_command_sync,
    send_moteus_stop_sync,
)


STEER_GEAR_RATIO = 50.0
STEER_MAX_TORQUE = 10.0
STEER_VELOCITY_LIMIT = 60.0
STEER_ACCEL_LIMIT = 40.0
IDLE_COMMAND_EPSILON = 0.03
WHEEL_SPEED_EPSILON = 0.02
DRIVE_WATCHDOG_TIMEOUT = 0.25
STEER_WATCHDOG_TIMEOUT = 0.25

MODULES = {
    "FL": {"drive": 1, "steer": 5, "drive_sign": -1.0},
    "BL": {"drive": 2, "steer": 6, "drive_sign": -1.0},
    "FR": {"drive": 3, "steer": 7, "drive_sign": 1.0},
    "BR": {"drive": 4, "steer": 8, "drive_sign": 1.0},
}


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
        self.last_module_command_log = None

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

        self.steer_motor_pos_est = {
            side: self.query_steer_position(side)
            for side in MODULES
        }
        self.logger.info(
            "Initial steer estimates radians: "
            + ", ".join(
                f"{side}={self.motor_rot_to_wheel_rad(pos):.2f}"
                for side, pos in self.steer_motor_pos_est.items()
            )
        )

    def apply_deadband(self, value: float) -> float:
        if abs(value) < self.deadband:
            return 0.0
        return value

    def normalize_angle_rad(self, angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def motor_rot_to_wheel_rad(self, motor_rot: float) -> float:
        return self.normalize_angle_rad((motor_rot / STEER_GEAR_RATIO) * (2 * math.pi))

    def wheel_rad_delta_to_motor_rot(self, wheel_rad: float) -> float:
        return (wheel_rad / (2 * math.pi)) * STEER_GEAR_RATIO

    def query_steer_position(self, side: str) -> float:
        motor_id = MODULES[side]["steer"]
        try:
            result = query_moteus_sync(self.swerve_controllers[motor_id])
            return float(result.values.get(moteus.Register.POSITION, 0.0))
        except Exception as exc:
            self.logger.warn(f"Could not query {side} steer position on startup: {exc!r}")
            return 0.0

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

    def calculate_swerve(self, vx: float, vy: float, omega: float):
        r = math.sqrt(ROVER_LENGTH**2 + ROVER_WIDTH**2)
        x_term = omega * (ROVER_LENGTH / r)
        y_term = omega * (ROVER_WIDTH / r)

        vectors = {
            "FL": (vx - x_term, vy + y_term),
            "FR": (vx - x_term, vy - y_term),
            "BL": (vx + x_term, vy + y_term),
            "BR": (vx + x_term, vy - y_term),
        }
        speeds = {side: math.hypot(x, y) for side, (x, y) in vectors.items()}
        angles = {side: math.atan2(y, x) for side, (x, y) in vectors.items()}

        max_speed = max(speeds.values())
        if max_speed > 1.0:
            speeds = {side: speed / max_speed for side, speed in speeds.items()}

        return speeds, angles

    def optimize_swerve(self, target_angle: float, target_speed: float, current_angle: float):
        diff = self.normalize_angle_rad(target_angle - current_angle)
        if abs(diff) > math.pi / 2:
            diff = self.normalize_angle_rad(diff - math.copysign(math.pi, diff))
            target_speed *= -1.0
        return current_angle + diff, target_speed

    def stop_drive_motors(self):
        for side, module in MODULES.items():
            motor_id = module["drive"]
            send_moteus_command_sync(
                controller=self.drive_controllers[motor_id],
                motor=motor_id,
                position=math.nan,
                drives_velocity=0.0,
                maximum_torque=self.MAX_TORQUE,
                velocity_limit=self.MOTOR_MAX_SPEED,
                accel_limit=self.MAX_ACCELERATION,
                ff_torque=0,
                watchdog_timeout=DRIVE_WATCHDOG_TIMEOUT,
                logger=None,
            )

    def log_module_commands(self, commands):
        summary = "module commands: " + " | ".join(
            f"{side} drv={cmd['drive_velocity']:.2f} "
            f"steer={cmd['steer_target']:.2f} "
            f"delta={cmd['steer_delta']:.2f}"
            for side, cmd in commands.items()
        )
        if summary != self.last_module_command_log:
            self.logger.info(summary)
            self.last_module_command_log = summary

    def command_module(self, side: str, drive_speed: float, wheel_speed: float, wheel_angle: float):
        module = MODULES[side]
        drive_id = module["drive"]
        steer_id = module["steer"]
        current_motor_pos = self.steer_motor_pos_est[side]
        current_angle = self.motor_rot_to_wheel_rad(current_motor_pos)

        if abs(wheel_speed) < WHEEL_SPEED_EPSILON:
            target_angle = current_angle
            optimized_speed = 0.0
        else:
            target_angle, optimized_speed = self.optimize_swerve(
                wheel_angle,
                wheel_speed,
                current_angle,
            )

        angle_delta = target_angle - current_angle
        target_motor_pos = current_motor_pos + self.wheel_rad_delta_to_motor_rot(angle_delta)
        self.steer_motor_pos_est[side] = target_motor_pos
        drive_velocity = module["drive_sign"] * drive_speed * optimized_speed

        send_moteus_command_sync(
            controller=self.drive_controllers[drive_id],
            motor=drive_id,
            position=math.nan,
            drives_velocity=drive_velocity,
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0,
            watchdog_timeout=DRIVE_WATCHDOG_TIMEOUT,
            logger=None,
        )
        send_moteus_command_sync(
            controller=self.swerve_controllers[steer_id],
            motor=steer_id,
            position=target_motor_pos,
            drives_velocity=None,
            maximum_torque=STEER_MAX_TORQUE,
            velocity_limit=STEER_VELOCITY_LIMIT,
            accel_limit=STEER_ACCEL_LIMIT,
            ff_torque=None,
            watchdog_timeout=STEER_WATCHDOG_TIMEOUT,
            logger=None,
        )
        return {
            "drive_velocity": drive_velocity,
            "steer_target": target_motor_pos,
            "steer_delta": target_motor_pos - current_motor_pos,
        }

    def listener_callback(self, msg: TwistStamped):
        self.log_cmd_vel(msg)

        self.controller_command_ly = self.apply_deadband(msg.twist.linear.y)
        self.controller_command_lx = self.apply_deadband(msg.twist.linear.x)
        self.controller_command_ry = self.apply_deadband(msg.twist.angular.y)
        self.controller_command_rx = self.apply_deadband(msg.twist.angular.x)

        drive_speed = self.compute_drive_scale()
        command_mag = max(
            abs(self.controller_command_lx),
            abs(self.controller_command_ly),
            abs(self.controller_command_rx),
        )

        if command_mag < IDLE_COMMAND_EPSILON or drive_speed == 0.0:
            self.stop_drive_motors()
            return

        wheel_speeds, wheel_angles = self.calculate_swerve(
            -self.controller_command_ly,
            self.controller_command_lx,
            self.controller_command_rx,
        )

        self.log_motion_summary(
            drive_speed,
            {side: round(speed, 3) for side, speed in wheel_speeds.items()},
            {side: round(angle, 3) for side, angle in wheel_angles.items()},
        )

        module_commands = {}
        for side in MODULES:
            module_commands[side] = self.command_module(
                side,
                drive_speed,
                wheel_speeds[side],
                wheel_angles[side],
            )
        self.log_module_commands(module_commands)

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
