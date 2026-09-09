import time

import rclpy
from cmr_msgs.msg import DriveCommand
from rclpy.node import Node
from std_msgs.msg import Bool, String

from cmr_rovernet.command_mux_core import CommandMux


class DriveCommandMux(Node):
    def __init__(self):
        super().__init__("drive_command_mux")
        self.declare_parameter("active_source", "teleop")
        self.declare_parameter("command_timeout_s", 0.5)
        self.mux = CommandMux(float(self.get_parameter("command_timeout_s").value))
        self.mux.select(str(self.get_parameter("active_source").value))
        self.estop = False
        self.last_status = None
        self.output = self.create_publisher(DriveCommand, "/cmd_vel", 10)
        self.create_subscription(DriveCommand, "/cmd_vel/teleop",
                                 lambda msg: self.receive("teleop", msg), 10)
        self.create_subscription(DriveCommand, "/cmd_vel/autonomy",
                                 lambda msg: self.receive("autonomy", msg), 10)
        self.create_subscription(Bool, "/cmd_vel/estop", self.set_estop, 10)
        self.create_subscription(String, "/cmd_vel/source", self.set_source, 10)
        self.create_timer(0.05, self.publish_selected)

    def receive(self, source, msg):
        values = (msg.vx, msg.vy, msg.omega, msg.speed_rps)
        if not self.mux.receive(source, values, time.monotonic()):
            self.get_logger().error(f"Rejected non-finite {source} command")

    def set_estop(self, msg):
        if msg.data and not self.estop:
            self.mux.emergency_stop()
        elif not msg.data and self.estop:
            self.mux.reset()
        self.estop = msg.data

    def set_source(self, msg):
        try:
            self.mux.select(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.get_logger().info(f"Drive source selected: {msg.data}")

    def publish_selected(self):
        selected = self.mux.output(time.monotonic())
        status = "estop" if self.estop else self.mux.source if selected else "timeout"
        if status != self.last_status:
            self.get_logger().info(f"Drive mux status: {status}")
            self.last_status = status
        values = selected or (0.0,) * 4
        self.output.publish(DriveCommand(
            vx=values[0], vy=values[1], omega=values[2], speed_rps=values[3]))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(DriveCommandMux())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
