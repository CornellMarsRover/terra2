"""Read-only live ROS telemetry, displayed beside Gazebo and at localhost:8765."""
import collections
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Dashboard(Node):
    def __init__(self):
        super().__init__('course_dashboard')
        self.camera = None
        self.costs = np.empty((0, 3))
        self.path = np.empty((0, 2))
        self.trail = collections.deque(maxlen=20000)
        self.target = None
        self.seen = {}
        self.jpeg = b''
        self.events = open(os.path.join(os.environ['SESSION_DIR'], 'telemetry.jsonl'), 'w')
        self.create_subscription(Image, '/camera/image_raw', self.on_camera, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/drives/odom', self.on_pose, qos_profile_sensor_data)
        for topic, key, width in [('/autonomy/costmap', 'costs', 3),
                                  ('/autonomy/path/plan', 'path', 2),
                                  ('/autonomy/target/global', 'target', 2)]:
            self.create_subscription(Float32MultiArray, topic,
                lambda msg, k=key, w=width: self.on_array(msg, k, w), 10)

    def on_array(self, msg, key, width):
        if len(msg.data) % width:
            return
        value = np.asarray(msg.data).reshape(-1, width)
        if not np.isfinite(value).all():
            return
        setattr(self, key, value)
        self.seen[key] = time.monotonic()
        self.events.write(json.dumps({'time_ns': self.get_clock().now().nanoseconds,
                                      'topic': key, 'data': list(msg.data)}) + '\n')

    def on_pose(self, msg):
        p = msg.pose.pose.position
