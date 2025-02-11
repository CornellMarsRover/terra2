import rclpy
from rclpy.node import Node
from cmr_msgs.msg import IMUSensorData
import numpy as np

class IMUCalibrationNode(Node):
    def __init__(self):
        super().__init__('imu_calibration')
        self.subscription = self.create_subscription(
            IMUSensorData,
            '/imu',
            self.imu_callback,
            10  # QoS Profile
        )
        self.data = {'accx': [], 'accy': [], 'gyroz': [], 'anglez': []}
        self.counter = 0
        self.batch_size = 1000

    def imu_callback(self, msg):
        # Collect IMU data
        self.data['accx'].append(msg.accx)
        self.data['accy'].append(msg.accy)
        self.data['gyroz'].append(msg.gyroz)
        self.data['anglez'].append(msg.anglez)
        self.counter += 1

        # Compute standard deviation every 1000 messages
        if self.counter % self.batch_size == 0:
            self.compute_std_dev()

    def compute_std_dev(self):
        std_dev = {key: np.std(values, ddof=1) for key, values in self.data.items()}
        self.get_logger().info(f"Standard Deviations: {std_dev}")

        # Keep accumulating data (no reset of self.data to maintain full dataset)

def main(args=None):
    rclpy.init(args=args)
    imu_calibration_node = IMUCalibrationNode()
    rclpy.spin(imu_calibration_node)
    imu_calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
