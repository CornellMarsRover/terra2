#!/usr/bin/env python3
"""
MJPEG web streamer for viewing ROS image topics from a browser.

Subscribes to configurable image topics and serves them as MJPEG streams
over HTTP. Designed for headless Jetson — view from a host laptop via
SSH port forwarding:

    ssh -L 8000:localhost:8000 user@jetson
    open http://localhost:8000
"""

import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class WebStreamNode(Node):
    def __init__(self):
        super().__init__("web_stream_node")

        self.declare_parameter("port", 8000)
        self.declare_parameter("quality", 70)

        self.port = self.get_parameter("port").value
        self.quality = self.get_parameter("quality").value
        self.bridge = CvBridge()

        self.lock = threading.Lock()
        self.frames = {}

        topics = [
            ("/object_detection/annotated_image", "detections"),
            ("/zed/image_left", "raw"),
        ]

        for topic, name in topics:
            self.create_subscription(
                Image, topic,
                lambda msg, n=name: self._image_cb(msg, n),
                10,
            )

        self._start_server()
        self.get_logger().info(
            f"Web stream on port {self.port} — "
            f"/detections (annotated) and /raw (ZED left)"
        )

    def _image_cb(self, msg: Image, stream_name: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        except Exception:
            return

        _, jpg = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        )
        with self.lock:
            self.frames[stream_name] = jpg.tobytes()

    def get_frame(self, stream_name: str):
        with self.lock:
            return self.frames.get(stream_name)

    def _start_server(self):
        node_ref = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def do_GET(self):
                path = self.path.strip("/")

                if path in ("detections", "raw"):
                    self._stream(path)
                else:
                    self._index()

            def _index(self):
                html = (
                    "<!doctype html><html><head>"
                    "<title>Rover Camera Streams</title>"
                    "<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center;}"
                    "img{max-width:90%;border:2px solid #0f0;margin:10px;}</style></head><body>"
                    "<h2>Rover Camera Streams</h2>"
                    "<p><b>Detections</b> (YOLO annotated):</p>"
                    '<img src="/detections" />'
                    "<p><b>Raw</b> (ZED left camera):</p>"
                    '<img src="/raw" />'
                    "</body></html>"
                )
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(html.encode())

            def _stream(self, name):
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self.end_headers()

                import time

                while True:
                    jpg = node_ref.get_frame(name)
                    if jpg is None:
                        time.sleep(0.05)
                        continue
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    except BrokenPipeError:
                        break
                    time.sleep(0.033)

        server = HTTPServer(("0.0.0.0", self.port), Handler)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()


def main(args=None):
    rclpy.init(args=args)
    node = WebStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
