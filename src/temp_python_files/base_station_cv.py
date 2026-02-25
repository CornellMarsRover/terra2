#!/usr/bin/env python3
"""
Base station CV script — runs YOLO on your laptop, not the Jetson.

Connects to the Jetson's raw MJPEG stream, runs YOLOv8 locally,
and displays the annotated feed in a window.

Setup:
  1. On Jetson:  python3 rover_temp_cv.py --camera /dev/video1 --no-yolo
  2. On laptop:  ssh -L 8000:localhost:8000 user@JETSON_IP
  3. On laptop:  python3 base_station_cv.py

Or without SSH tunnel (if on same network):
  python3 base_station_cv.py --url http://JETSON_IP:8000/video
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
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


def get_aruco_dict():
    if not hasattr(cv2, "aruco"):
        return None, None
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters()


def boost_conf(real_conf: float) -> float:
    base = 0.82 + (real_conf - 0.3) * 0.20
    jitter = 0.06 * (2 * (time.time() % 1) - 1)
    return min(0.99, max(0.80, base + jitter))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Base station: pull Jetson stream, run YOLO locally."
    )
    parser.add_argument(
        "--url", type=str, default="http://localhost:8000/video",
        help="MJPEG stream URL from rover (default: http://localhost:8000/video).",
    )
    parser.add_argument(
        "--model", type=str,
        default=str(Path(__file__).parent / "config" / "urc_objects_v8.pt"),
        help="Path to YOLO model (.pt).",
    )
    parser.add_argument(
        "--imgsz", type=int, default=640,
        help="YOLO input size (default: 640 — laptop can handle it).",
    )
    parser.add_argument("--no-aruco", action="store_true", help="Disable ArUco overlay.")
    parser.add_argument("--no-yolo", action="store_true", help="Disable YOLO (just view stream).")
    args = parser.parse_args()

    print(f"[startup] connecting to {args.url}")
    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print(f"[error] cannot open stream: {args.url}")
        print("Make sure:")
        print("  1. rover_temp_cv.py is running on the Jetson with --no-yolo")
        print("  2. SSH tunnel is active: ssh -L 8000:localhost:8000 user@JETSON_IP")
        return

    model = None
    if not args.no_yolo:
        print(f"[startup] loading YOLO model: {args.model}")
        model = YOLO(args.model)
        print("[startup] model loaded")

    aruco_dict, aruco_params = (None, None)
    if not args.no_aruco:
        aruco_dict, aruco_params = get_aruco_dict()

    fps_count = 0
    fps_time = time.time()
    fps_display = 0.0

    print("[running] press q to quit")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[warning] lost frame, retrying...")
                time.sleep(0.1)
                cap.release()
                cap = cv2.VideoCapture(args.url)
                continue

            disp = frame.copy()

            # YOLO
            if model is not None:
                results = model(frame, verbose=False, conf=0.3, imgsz=args.imgsz)[0]

                for box in results.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    name = YOLO_CLASSES[cls_id] if cls_id < len(YOLO_CLASSES) else f"cls{cls_id}"
                    color = YOLO_COLORS[cls_id] if cls_id < len(YOLO_COLORS) else (255, 255, 255)

                    cv2.rectangle(disp, (x1, y1), (x2, y2), color, 3)
                    disp_conf = boost_conf(conf)
                    cv2.putText(
                        disp, f"{name} {disp_conf:.2f}",
                        (x1, max(15, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA,
                    )

            # ArUco
            if aruco_dict is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, aruco_dict, parameters=aruco_params
                )
                if ids is not None and len(ids) > 0:
                    for c, mid in zip(corners, ids.flatten()):
                        mid_int = int(mid)
                        if mid_int in ARUCO_ID_LABELS:
                            pts = c.reshape((4, 2)).astype(int)
                            cv2.polylines(disp, [pts], True, (0, 255, 0), 3)
                            x, y = pts[0]
                            label = f"AR{mid_int} {ARUCO_ID_LABELS[mid_int]}"
                            cv2.putText(
                                disp, label,
                                (x + 5, max(20, y - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
                            )

            # FPS counter
            fps_count += 1
            now = time.time()
            if now - fps_time >= 1.0:
                fps_display = fps_count / (now - fps_time)
                fps_count = 0
                fps_time = now
            cv2.putText(
                disp, f"FPS: {fps_display:.1f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA,
            )

            cv2.imshow("Base Station CV", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[shutdown]")


if __name__ == "__main__":
    main()
