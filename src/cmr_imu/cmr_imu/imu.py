#!/usr/bin/env python3

import threading
import time
from queue import Queue

import rclpy
import serial
from cmr_msgs.msg import IMUSensorData
from rclpy.node import Node

from cmr_imu.hwt905_parser import HWT905Parser, HWT905Sample, normalize_degrees


DEFAULT_SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


class HWT905Publisher(Node):
    def __init__(self):
        super().__init__("hwt905_imu_node")

        self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT)
        self.declare_parameter("baud", 9600)
        self.declare_parameter("frame_topic", "/imu")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("yaw_offset_degrees", 0.0)
        self.declare_parameter("auto_zero_yaw", False)

        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.frame_topic = self.get_parameter("frame_topic").get_parameter_value().string_value
        self.yaw_offset_degrees = self.get_parameter("yaw_offset_degrees").get_parameter_value().double_value
        self.auto_zero_yaw = self.get_parameter("auto_zero_yaw").get_parameter_value().bool_value
        publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._samples = Queue()
        self._stop_event = threading.Event()
        self._yaw_zero = None
        self._reader_thread = threading.Thread(target=self._read_serial, daemon=True)

        self.publisher_ = self.create_publisher(IMUSensorData, self.frame_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_latest)

        self._reader_thread.start()
        self.get_logger().info(
            f"Reading HWT905-RS232 from {self.serial_port} at {self.baud} baud; "
            f"publishing {self.frame_topic}"
        )

    def publish_latest(self):
        sample = self._get_latest_sample()
        if sample is None:
            return

        msg = IMUSensorData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.temp = float(sample.temp)
        msg.accx = float(sample.accx)
        msg.accy = float(sample.accy)
        msg.accz = float(sample.accz)
        msg.gyrox = float(sample.gyrox)
        msg.gyroy = float(sample.gyroy)
        msg.gyroz = float(sample.gyroz)
        msg.anglex = float(sample.anglex)
        msg.angley = float(sample.angley)
        msg.anglez = self._correct_yaw(sample.anglez)
        msg.magx = int(sample.magx)
        msg.magy = int(sample.magy)
        msg.magz = int(sample.magz)
        self.publisher_.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        self._reader_thread.join(timeout=1.0)
        super().destroy_node()

    def _get_latest_sample(self):
        sample = None
        while not self._samples.empty():
            sample = self._samples.get()
        return sample

    def _correct_yaw(self, yaw_degrees: float) -> float:
        if self.auto_zero_yaw and self._yaw_zero is None:
            self._yaw_zero = yaw_degrees

        offset = self.yaw_offset_degrees
        if self._yaw_zero is not None:
            offset += self._yaw_zero

        return normalize_degrees(yaw_degrees - offset)

    def _read_serial(self):
        parser = HWT905Parser()

        while not self._stop_event.is_set():
            try:
                with serial.Serial(self.serial_port, self.baud, timeout=1) as serial_port:
                    self.get_logger().info(f"Opened {self.serial_port}")
                    while not self._stop_event.is_set():
                        data = serial_port.read(128)
                        if not data:
                            continue
                        for sample in parser.parse(data):
                            if is_publishable(sample):
                                self._samples.put(sample)
            except serial.SerialException as exc:
                self.get_logger().error(
                    f"Failed to read HWT905 on {self.serial_port}: {exc}; retrying in 1s",
                    throttle_duration_sec=5.0,
                )
                time.sleep(1.0)


def is_publishable(sample: HWT905Sample) -> bool:
    return sample.has_accel and sample.has_gyro and sample.has_angle and sample.has_mag


def main(args=None):
    rclpy.init(args=args)
    node = HWT905Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
