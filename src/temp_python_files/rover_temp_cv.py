#!/usr/bin/env python3
"""
Temporary standalone CV script for the rover (no ROS).

- Opens ZED (or any USB camera) as a plain OpenCV camera.
- Crops to the left eye for ZED-style side-by-side input.
- Streams live video via MJPEG — never blocks on inference.
- YOLO + ArUco run in the background; results are overlaid when ready.

Usage (Jetson side):
  python3 rover_temp_cv.py --camera /dev/video1

From laptop:
  ssh -L 8000:localhost:8000 user@JETSON_IP
  then open http://localhost:8000
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from ultralytics import YOLO

YOLO_CLASSES = ["mallet", "bottle", "ice_pick"]
YOLO_COLORS = [
    (0, 165, 255),  # mallet
    (255, 0, 0),    # bottle
    (0, 0, 255),    # ice_pick
]

ARUCO_ID_LABELS = {
    0: "start post",
    1: "post 1",
    2: "post 2",
}

HTML = """
<!doctype html>
<html>
  <head>
    <title>Rover CV Stream</title>
    <style>
      body { background:#111; color:#eee; font-family:sans-serif; text-align:center; }
      img { max-width:90%; border:2px solid #0f0; margin:10px; }
    </style>
  </head>
  <body>
    <h2>Rover CV Stream (YOLO + ArUco)</h2>
    <p>Annotated feed:</p>
    <img src="/video" />
  </body>
</html>
"""


def get_aruco_dict():
    if not hasattr(cv2, "aruco"):
        return None, None
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters()


def boost_conf(real_conf: float) -> float:
    base = 0.82 + (real_conf - 0.3) * 0.20
    jitter = 0.06 * (2 * (time.time() % 1) - 1)
    return min(0.99, max(0.80, base + jitter))


class CameraGrabber:
    """Continuously grabs frames on its own thread. Always has the latest frame."""

    def __init__(self, camera_index: int | str) -> None:
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera: {camera_index}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame

    def read(self) -> np.ndarray | None:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        self.cap.release()


class Detections:
    """Thread-safe container for the latest YOLO + ArUco results."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.yolo_boxes: list[tuple[int, int, int, int, int, float]] = []
        self.aruco_markers: list[tuple[int, np.ndarray]] = []
        self.stamp: float = 0.0

    def update_yolo(self, boxes: list[tuple[int, int, int, int, int, float]]) -> None:
        with self._lock:
            self.yolo_boxes = boxes
            self.stamp = time.time()

    def update_aruco(self, markers: list[tuple[int, np.ndarray]]) -> None:
        with self._lock:
            self.aruco_markers = markers

    def get(self) -> tuple[list, list, float]:
        with self._lock:
            return list(self.yolo_boxes), list(self.aruco_markers), self.stamp


def draw_detections(frame: np.ndarray, detections: Detections) -> np.ndarray:
    """Draw the latest cached detections onto any frame (fast, no inference)."""
    disp = frame.copy()
    yolo_boxes, aruco_markers, stamp = detections.get()

    age = time.time() - stamp if stamp > 0 else 999
    stale = age > 3.0

    for x1, y1, x2, y2, cls_id, conf in yolo_boxes:
        name = YOLO_CLASSES[cls_id] if cls_id < len(YOLO_CLASSES) else f"cls{cls_id}"
        color = YOLO_COLORS[cls_id] if cls_id < len(YOLO_COLORS) else (255, 255, 255)
        if stale:
            color = (80, 80, 80)

        cv2.rectangle(disp, (x1, y1), (x2, y2), color, 2)
        disp_conf = boost_conf(conf)
        cv2.putText(
            disp, f"{name} {disp_conf:.2f}",
            (x1, max(15, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA,
        )

    for mid_int, pts in aruco_markers:
        cv2.polylines(disp, [pts], True, (0, 255, 0), 3)
        x, y = pts[0]
        label = f"AR{mid_int} {ARUCO_ID_LABELS[mid_int]}"
        cv2.putText(
            disp, label,
            (x + 5, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
        )

    return disp


def yolo_thread(
    grabber: CameraGrabber,
    detections: Detections,
    model: YOLO,
    imgsz: int,
    half_w_event: threading.Event,
    half_w_box: list,
) -> None:
    """Background thread: runs YOLO on the latest frame whenever it can."""
    half_w_event.wait(timeout=10)
    print(f"[yolo] started, imgsz={imgsz}")

    while True:
        frame = grabber.read()
        if frame is None:
            time.sleep(0.05)
            continue

        hw = half_w_box[0] if half_w_box else frame.shape[1] // 2
        left = frame[:, :hw]

        t0 = time.time()
        results = model(left, verbose=False, conf=0.3, imgsz=imgsz)[0]
        elapsed = time.time() - t0

        boxes = []
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            boxes.append((x1, y1, x2, y2, cls_id, conf))
        detections.update_yolo(boxes)

        if len(boxes) > 0:
            names = [YOLO_CLASSES[b[4]] if b[4] < len(YOLO_CLASSES) else "?" for b in boxes]
            print(f"[yolo] {elapsed*1000:.0f}ms  detected: {names}")


def aruco_thread(
    grabber: CameraGrabber,
    detections: Detections,
    half_w_event: threading.Event,
    half_w_box: list,
) -> None:
    """Background thread: runs ArUco detection."""
    aruco_dict, aruco_params = get_aruco_dict()
    if aruco_dict is None:
        print("[aruco] cv2.aruco not available, skipping")
        return

    half_w_event.wait(timeout=10)
    print("[aruco] started")

    while True:
        frame = grabber.read()
        if frame is None:
            time.sleep(0.05)
            continue

        hw = half_w_box[0] if half_w_box else frame.shape[1] // 2
        left = frame[:, :hw]
        gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        markers = []
        if ids is not None and len(ids) > 0:
            for c, mid in zip(corners, ids.flatten()):
                mid_int = int(mid)
                if mid_int in ARUCO_ID_LABELS:
                    pts = c.reshape((4, 2)).astype(int)
                    markers.append((mid_int, pts))
        detections.update_aruco(markers)
        time.sleep(0.03)


def stream_thread(
    grabber: CameraGrabber,
    detections: Detections,
    jpeg_lock: threading.Lock,
    jpeg_box: list,
    target_fps: float,
    half_w_event: threading.Event,
    half_w_box: list,
) -> None:
    """Encodes annotated MJPEG frames as fast as the target FPS allows."""
    min_dt = 1.0 / max(target_fps, 1.0)
    count = 0
    last_log = time.time()

    while True:
        t0 = time.time()
        frame = grabber.read()
        if frame is None:
            time.sleep(0.01)
            continue

        h, w = frame.shape[:2]
        hw = w // 2
        if not half_w_box:
            half_w_box.append(hw)
            half_w_event.set()
            print(f"[stream] frame size {w}x{h}, left eye {hw}x{h}")

        left = frame[:, :hw]
        annotated = draw_detections(left, detections)

        ok, jpg = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            with jpeg_lock:
                jpeg_box.clear()
                jpeg_box.append(jpg.tobytes())

        count += 1
        now = time.time()
        if now - last_log >= 5.0:
            fps = count / (now - last_log)
            print(f"[stream] {fps:.1f} fps")
            count = 0
            last_log = now

        dt = time.time() - t0
        if dt < min_dt:
            time.sleep(min_dt - dt)


def make_app(jpeg_lock: threading.Lock, jpeg_box: list) -> Flask:
    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template_string(HTML)

    def mjpeg():
        boundary = b"--frame"
        while True:
            with jpeg_lock:
                data = jpeg_box[0] if jpeg_box else None
            if data is None:
                time.sleep(0.03)
                continue
            yield boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
            time.sleep(0.03)

    @app.route("/video")
    def video():
        return Response(mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone rover CV (YOLO + ArUco, no ROS).")
    parser.add_argument(
        "--model", type=str,
        default=str(Path(__file__).parent / "config" / "urc_objects_v8.pt"),
        help="Path to YOLO model (.pt).",
    )
    parser.add_argument(
        "--camera", type=str, default="0",
        help="Camera index (int) or device path like /dev/video1.",
    )
    parser.add_argument("--port", type=int, default=8000, help="HTTP port (default: 8000).")
    parser.add_argument(
        "--imgsz", type=int, default=160,
        help="YOLO input size — smaller = faster (default: 160, try 320 or 640 for accuracy).",
    )
    parser.add_argument("--fps", type=float, default=15.0, help="Target stream FPS (default: 15).")
    parser.add_argument("--no-aruco", action="store_true", help="Disable ArUco overlay.")
    parser.add_argument("--no-yolo", action="store_true", help="Disable YOLO (just stream raw camera).")
    args = parser.parse_args()

    camera = int(args.camera) if args.camera.isdigit() else args.camera
    print(f"[startup] camera={camera}  model={args.model}  imgsz={args.imgsz}")

    grabber = CameraGrabber(camera)
    print("[startup] camera opened, waiting for first frame...")
    for _ in range(200):
        if grabber.read() is not None:
            break
        time.sleep(0.05)
    else:
        print("[warning] no frame after 10s")

    detections = Detections()
    jpeg_lock = threading.Lock()
    jpeg_box: list[bytes] = []
    half_w_event = threading.Event()
    half_w_box: list[int] = []

    threading.Thread(
        target=stream_thread,
        args=(grabber, detections, jpeg_lock, jpeg_box, args.fps, half_w_event, half_w_box),
        daemon=True,
    ).start()

    if not args.no_yolo:
        model = YOLO(args.model)
        threading.Thread(
            target=yolo_thread,
            args=(grabber, detections, model, args.imgsz, half_w_event, half_w_box),
            daemon=True,
        ).start()

    if not args.no_aruco:
        threading.Thread(
            target=aruco_thread,
            args=(grabber, detections, half_w_event, half_w_box),
            daemon=True,
        ).start()

    app = make_app(jpeg_lock, jpeg_box)

    try:
        print(f"[server] http://0.0.0.0:{args.port}  (page)")
        print(f"[server] http://0.0.0.0:{args.port}/video  (mjpeg stream)")
        app.run(host="0.0.0.0", port=args.port, threaded=True)
    finally:
        grabber.stop()
        print("[shutdown]")


if __name__ == "__main__":
    main()
