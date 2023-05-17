import rclpy
from rclpy.node import Node

from cmr_msgs.msg import MotorWriteBatch

MOTOR_IDS = [64, 65, 66, 67, 68, 69]

class DrivesCalibrator(Node):
    """A node that calls the ODrive calibration routine of the CCB. By the time
    this node finishes running, all ODrives will be calibrated."""

    def __init__(self):
        super().__init__("drives_calibrator")

        self.motors = self.create_publisher(
            MotorWriteBatch, "/ccb/motors", 10
        )

        for id in MOTOR_IDS:
            print("Calibrating motor", hex(id), "...")
            self.calibrate_motor(id)
            input("Press any key to continue once the axis has finished calibrating.")            

    def calibrate_motor(self, motor_id):
        msg = MotorWriteBatch()
        msg.motor_ids = [motor_id]
        msg.control_modes = [3]
        msg.values = [1]
        msg.size = 1
        self.motors.publish(msg)

def main():
    rclpy.init(args=None)

    node = DrivesCalibrator()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
