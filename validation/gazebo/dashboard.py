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
        self.trail.append((p.x, p.y))
        self.seen['pose'] = time.monotonic()

    def on_camera(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        self.camera = data[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        if msg.encoding == 'rgb8':
            self.camera = cv2.cvtColor(self.camera, cv2.COLOR_RGB2BGR)
        self.seen['camera'] = time.monotonic()

    def render(self):
        canvas = np.full((720, 500, 3), 24, np.uint8)
        def label(text, y, color=(240, 240, 240)):
            cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
        label('LIVE ROS CAMERA /camera/image_raw', 20)
        if self.camera is not None:
            canvas[30:210, 130:370] = cv2.resize(self.camera, (240, 180), interpolation=cv2.INTER_NEAREST)
        label('COSTMAP + PLAN + ACTUAL TRAJECTORY', 235)
        def pixel(point):
            return (int(25 + (point[0] + 5) * 17), int(660 - (point[1] + 5) * 16))
        for x, y, cost in self.costs:
            u, v = pixel((x, y))
            if 0 <= u < 500 and 250 <= v < 670:
                cv2.rectangle(canvas, (u-2, v-2), (u+2, v+2), (0, 60, int(min(255, max(40, cost * 10)))), -1)
        for points, color in [(self.trail, (255, 220, 0)), (self.path, (60, 255, 60))]:
            if len(points) > 1:
                cv2.polylines(canvas, [np.array([pixel(p) for p in points])], False, color, 2)
        if self.target is not None and len(self.target):
            cv2.drawMarker(canvas, pixel(self.target[0]), (255, 255, 255), cv2.MARKER_CROSS, 12, 2)
        if self.trail:
            cv2.circle(canvas, pixel(self.trail[-1]), 5, (255, 255, 255), -1)
        label('Red: cost | Green: plan | Cyan: actual | +: target', 685)
        ages = ' '.join(f'{k}:{time.monotonic()-self.seen[k]:.1f}s' if k in self.seen else f'{k}:WAIT' for k in ('camera','costs','path'))
        label(ages, 710)
        self.jpeg = cv2.imencode('.jpg', canvas)[1].tobytes()
        cv2.imshow('Rover telemetry', canvas)
        cv2.moveWindow('Rover telemetry', 780, 0)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = Dashboard()
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            frame = self.path.startswith('/frame')
            body = node.jpeg if frame else b'<html><body style="background:#181818;color:white"><h3>Live ROS telemetry</h3><img id="feed" src="/frame"><script>setInterval(()=>feed.src="/frame?t="+Date.now(),500)</script></body></html>'
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg' if frame else 'text/html')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        def log_message(self, *_):
            pass
    server = ThreadingHTTPServer(('0.0.0.0', 8765), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        last = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() - last > 0.5:
                node.render()
                last = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        node.events.close()
        server.shutdown()
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
