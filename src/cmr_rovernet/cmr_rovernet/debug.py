import rclpy
from rclpy.node import Node
from cmr_msgs.msg import MotorReadData

class DebugSubscriber(Node):
    
    def __init__(self):
        super().__init__('debug_subscriber')
        self.subscription = self.create_subscription(
            MotorReadData,
            '/ccb/read',
            self.listener_callback,
            10)
        self.velocity_total = 0
        self.torque_total = 0
        self.current_total = 0
        self.count = 0
        self.logger = self.get_logger()
        
    def listener_callback(self, msg):
        # Update totals and count
        motors = [msg.back_left, msg.front_right, msg.back_right, msg.front_left]
        for motor in motors:
            self.velocity_total += motor.velocity
            self.torque_total += motor.torque
            self.current_total += motor.current
            self.count += 1

        # Calculate averages and totals
        if self.count > 0:  # Avoid division by zero
            average_velocity = self.velocity_total / self.count
            average_torque = self.torque_total / self.count
            average_current = self.current_total / self.count
            total_torque = self.torque_total
            total_current = self.current_total

            # Print the calculated values
            #self.logger.info(f'Average Velocity: {average_velocity}, Average Torque: {average_torque}, Total Torque: {total_torque}, Average Current: {average_current}, Total Current: {total_current}')

            # Reset totals and count for next calculation
            self.velocity_total = 0
            self.torque_total = 0
            self.current_total = 0
            self.count = 0

def main(args=None):
    rclpy.init(args=args)
    debug_subscriber = DebugSubscriber()
    rclpy.spin(debug_subscriber)

    # Clean up before shutting down
    debug_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
