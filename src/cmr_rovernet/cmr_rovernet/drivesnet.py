import math

import moteus
import rclpy
from rclpy.node import Node

from cmr_msgs.msg import ControllerReading
from cmr_rovernet.rovernet_utils import *


class DrivesnetDiagnostic(Node):
    def __init__(self):
        super().__init__("drivesnet_diagnostic")

        self.create_subscription(
            ControllerReading,
            "/drives_controller/cmd_buttons",
            self.listener_button_callback,
            10,
        )

        self.logger = self.get_logger()

        drives_net_table = parse_toml("drivesnet")
        if drives_net_table is None:
            raise RuntimeError("parse_toml('drivesnet') returned None")

        drives_net_node = drives_net_table.get("node")
        if drives_net_node is None:
            raise RuntimeError("drivesnet TOML missing ['node'] section")

        self.MOTOR_MAX_SPEED = drives_net_node["motor_max_speed"]
        self.MAX_ACCELERATION = drives_net_node["acceleration_limit"]
        self.MAX_TORQUE = drives_net_node["torque_limit"]

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

        self.last_button_combo = None

        self.logger.info("Drivesnet diagnostic ready")
        self.logger.info("L1+Triangle = STOP ALL")
        self.logger.info("L1+Square   = Drive FL  (id 1)")
        self.logger.info("L1+X        = Drive BL  (id 2)")
        self.logger.info("L1+Circle   = Drive FR  (id 3)")
        self.logger.info("L1+R1       = Drive BR  (id 4)")
        self.logger.info("R1+Square   = Steer FL  (id 5)")
        self.logger.info("R1+X        = Steer BL  (id 6)")
        self.logger.info("R1+Circle   = Steer FR  (id 7)")
        self.logger.info("R1+Triangle = Steer BR  (id 8)")
        self.logger.info("L2 held     = negative direction for selected test")
        self.logger.info("Default test values: drive velocity=2.0, steer position=0.05")

    def stop_all(self):
        self.logger.info("STOP ALL")

        for name, motor_id in [("FL", 1), ("BL", 2), ("FR", 3), ("BR", 4)]:
            send_moteus_stop_sync(self.ctrl_drive[name], motor=motor_id, logger=self.logger)

        for name, motor_id in [("FL", 5), ("BL", 6), ("FR", 7), ("BR", 8)]:
            send_moteus_stop_sync(self.ctrl_steer[name], motor=motor_id, logger=self.logger)

    def send_drive_test(self, wheel_name, motor_id, velocity):
        self.logger.info(f"DRIVE TEST -> {wheel_name} id={motor_id} vel={velocity}")
        send_moteus_command_sync(
            controller=self.ctrl_drive[wheel_name],
            motor=motor_id,
            position=math.nan,
            drives_velocity=velocity,
            maximum_torque=self.MAX_TORQUE,
            velocity_limit=self.MOTOR_MAX_SPEED,
            accel_limit=self.MAX_ACCELERATION,
            ff_torque=0.0,
            logger=self.logger,
        )

    def send_steer_test(self, wheel_name, motor_id, position):
        self.logger.info(f"STEER TEST -> {wheel_name} id={motor_id} pos={position}")
        send_moteus_command_sync(
            controller=self.ctrl_steer[wheel_name],
            motor=motor_id,
            position=position,
            drives_velocity=None,
            maximum_torque=10.0,
            velocity_limit=20.0,
            accel_limit=20.0,
            ff_torque=None,
            logger=self.logger,
        )

    def listener_button_callback(self, msg):
        trigger_val = msg.button_array[0]
        button_val = msg.button_array[1]

        combo = (trigger_val, button_val)

        # Prevent repeat-spamming the same command while button is held
        if combo == self.last_button_combo:
            return
        self.last_button_combo = combo

        negative = L2_MIN <= trigger_val <= L2

        drive_velocity = -2.0 if negative else 2.0
        steer_position = -0.05 if negative else 0.05

        # STOP ALL
        if trigger_val == L1 and button_val == TRIANGLE:
            self.stop_all()
            return

        # DRIVE TESTS
        if trigger_val == L1 and button_val == SQUARE:
            self.send_drive_test("FL", 1, drive_velocity)
            return

        if trigger_val == L1 and button_val == X:
            self.send_drive_test("BL", 2, drive_velocity)
            return

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