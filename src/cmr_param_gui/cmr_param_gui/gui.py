import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys

class InfoSubscriber(Node):
    def __init__(self, display_window):
        super().__init__('info_subscriber')
        self.display_window = display_window
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/gui/display',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Check if the message contains only one value
        if len(msg.data) == 1:
            # Assuming the single value is speed for simplicity
            speed = msg.data[0]
            avg_accel = avg_torque = avg_current = 0  # Placeholder values
            # Directly calling the update method (note: not thread-safe, see below for solution)
            QtCore.QMetaObject.invokeMethod(self.display_window, "update_info", 
                                            QtCore.Qt.QueuedConnection,
                                            QtCore.Q_ARG(float, speed),
                                            QtCore.Q_ARG(float, avg_accel),
                                            QtCore.Q_ARG(float, avg_torque),
                                            QtCore.Q_ARG(float, avg_current))


class DisplayWindow(QMainWindow):
    def __init__(self):
        super(DisplayWindow, self).__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("ROS2 Info Display")
        self.setGeometry(100, 100, 400, 300)  # x, y, width, height

        # Speed Label
        self.label_speed = QtWidgets.QLabel(self)
        self.label_speed.setText("Speed: ")
        self.label_speed.move(50, 50)

    @QtCore.pyqtSlot(float, float, float, float)
    def update_info(self, speed, avg_accel, avg_torque, avg_current):
        self.label_speed.setText(f"Speed: {speed}")
        # Update other labels similarly

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    display_window = DisplayWindow()
    info_subscriber = InfoSubscriber(display_window)
    display_window.show()

    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(info_subscriber)
            # Consider adding a sleep here if necessary

    import threading
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
