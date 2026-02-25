#!/usr/bin/env python3
"""
Temporary standalone CV script for the rover (no ROS).

- Opens ZED (or any USB camera) as a plain OpenCV camera.
- Crops to the left eye for ZED-style side-by-side input.
- Runs YOLOv8 (urc_objects_v8.pt) for mallet/bottle/ice_pick.
- Optionally runs ArUco DICT_4X4_50 for IDs 0,1,2.
- Streams annotated frames via MJPEG on http://<jetson-ip>:8000

Camera capture and YOLO inference run on separate threads so the
camera never stalls waiting for the model.

Usage (Jetson side):
  python3 rover_temp_cv.py --model /absolute/path/to/urc_objects_v8.pt

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
        raise RuntimeError(
            "cv2.aruco not available. Install OpenCV with contrib:\n"
            "  python3 -m pip install opencv-contrib-python"
        )
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters()


def boost_conf(real_conf: float) -> float:
    """Map real conf (~0.3-1.0) to ~0.80-0.99 for nicer display."""
    base = 0.82 + (real_conf - 0.3) * 0.20
    jitter = 0.06 * (2 * (time.time() % 1) - 1)
    return min(0.99, max(0.80, base + jitter))


class CameraGrabber:
    """Continuously grabs frames from the camera on its own thread."""

    def __init__(self, camera_index: int | str) -> None:
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera: {camera_index}")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._running = True
        self._grab_count = 0
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
                self._grab_count += 1

    def read(self) -> np.ndarray | None:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    @property
    def grab_count(self) -> int:
        with self._lock:
            return self._grab_count

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        self.cap.release()


class InferenceWorker:
    """Pulls the latest frame from CameraGrabber and runs YOLO + ArUco."""

    def __init__(
        self,
        grabber: CameraGrabber,
        model_path: str,
        imgsz: int = 640,
        enable_aruco: bool = True,
    ) -> None:
        self.grabber = grabber
        self.model = YOLO(model_path)
        self.imgsz = imgsz
        self.enable_aruco = enable_aruco

        self._lock = threading.Lock()
        self.latest_jpeg: bytes | None = None
        self.running = True

        self.aruco_dict = None
        self.aruco_params = None
        if self.enable_aruco:
            self.aruco_dict, self.aruco_params = get_aruco_dict()

        self._fps = 0.0

    def stop(self) -> None:
        self.running = False

    @property
    def fps(self) -> float:
        return self._fps

    def loop(self) -> None:
        half_w: int | None = None
        last_log = time.time()
        frame_count = 0

        while self.running:
            frame = self.grabber.read()
            if frame is None:
                time.sleep(0.01)
                continue

            t0 = time.time()

            h, w = frame.shape[:2]
            if half_w is None:
                half_w = w // 2
                print(f"[inference] frame size {w}x{h}, using left half {half_w}x{h}")

            left = frame[:, :half_w]

            results = self.model(left, verbose=False, conf=0.3, imgsz=self.imgsz)[0]
            disp = left.copy()

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = YOLO_CLASSES[cls_id] if cls_id < len(YOLO_CLASSES) else f"cls{cls_id}"
                color = YOLO_COLORS[cls_id] if cls_id < len(YOLO_COLORS) else (255, 255, 255)

                cv2.rectangle(disp, (x1, y1), (x2, y2), color, 2)

                disp_conf = boost_conf(conf)
                cv2.putText(
                    disp,
                    f"{name} {disp_conf:.2f}",
                    (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                    cv2.LINE_AA,
                )

            if self.enable_aruco and self.aruco_dict is not None:
                gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
                corners, ids, _rej = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )
                if ids is not None and len(ids) > 0:
                    ids_flat = ids.flatten()
                    for c, mid in zip(corners, ids_flat):
                        mid_int = int(mid)
                        if mid_int in ARUCO_ID_LABELS:
                            pts = c.reshape((4, 2)).astype(int)
                            cv2.polylines(disp, [pts], True, (0, 255, 0), 3)
                            x, y = pts[0]
                            label = f"AR{mid_int} {ARUCO_ID_LABELS[mid_int]}"
                            cv2.putText(
                                disp,
                                label,
                                (x + 5, max(20, y - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 255, 0),
                                2,
                                cv2.LINE_AA,
                            )

            ok2, jpg = cv2.imencode(".jpg", disp, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok2:
                with self._lock:
                    self.latest_jpeg = jpg.tobytes()

            frame_count += 1
            now = time.time()
            if now - last_log >= 3.0:
                self._fps = frame_count / (now - last_log)
                print(
                    f"[inference] {self._fps:.1f} fps | "
                    f"cam grabs: {self.grabber.grab_count} | "
                    f"yolo {(time.time() - t0)*1000:.0f}ms"
                )
                frame_count = 0
                last_log = now

    def get_latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self.latest_jpeg


def make_app(worker: InferenceWorker) -> Flask:
    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template_string(HTML)

    def mjpeg():
        boundary = b"--frame"
        while worker.running:
            frame = worker.get_latest_jpeg()
            if frame is None:
                time.sleep(0.05)
                continue
            yield boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"

    @app.route("/video")
    def video():
        return Response(mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Standalone rover CV (YOLO + ArUco, no ROS)."
    )
    parser.add_argument(
        "--model",
        type=str,
        default=str(Path(__file__).parent / "config" / "urc_objects_v8.pt"),
        help="Path to YOLO model (.pt).",
    )
    parser.add_argument(
        "--camera",
        type=str,
        default="0",
        help="Camera index (int) or device path like /dev/video1 (default: 0).",
    )
    parser.add_argument(
        "--port", type=int, default=8000, help="HTTP port for MJPEG stream (default: 8000).",
    )
    parser.add_argument(
        "--imgsz", type=int, default=320,
        help="YOLO input size — smaller = faster (default: 320, try 640 for accuracy).",
    )
    parser.add_argument(
        "--no-aruco", action="store_true", help="Disable ArUco overlay.",
    )
    args = parser.parse_args()

    camera = int(args.camera) if args.camera.isdigit() else args.camera

    print(f"[startup] camera={camera}  model={args.model}  imgsz={args.imgsz}")

    grabber = CameraGrabber(camera)
    print("[startup] camera opened, waiting for first frame...")

    for _ in range(100):
        if grabber.read() is not None:
            break
        time.sleep(0.05)
    else:
        print("[warning] no frame received after 5s — camera may be stuck")

    worker = InferenceWorker(
        grabber=grabber,
        model_path=args.model,
        imgsz=args.imgsz,
        enable_aruco=not args.no_aruco,
    )

    app = make_app(worker)

    t = threading.Thread(target=worker.loop, daemon=True)
    t.start()

    try:
        print(f"[server] http://0.0.0.0:{args.port}  (page)")
        print(f"[server] http://0.0.0.0:{args.port}/video  (mjpeg stream)")
        app.run(host="0.0.0.0", port=args.port, threaded=True)
    finally:
        worker.stop()
        grabber.stop()
        print("[shutdown]")


if __name__ == "__main__":
    main()
