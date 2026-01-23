#!/usr/bin/env python3
"""
AR Tag (ArUco) detection with depth estimation from a USB/USB-C camera on macOS or any OS.
- Detects ArUco markers and estimates 3D pose (rotation + translation) to get depth.
- Press 'q' to quit, 's' to save a frame, 'd' to cycle dictionaries, 'c' to cycle cameras.

Install deps:
    python3 -m pip install opencv-contrib-python
Run:
    python3 detect_aruco_depth.py --camera-id 0
"""

import argparse
import time
import os

import cv2
import numpy as np
from datetime import datetime

# --- Helper: build available aruco dictionaries in OpenCV ---
def get_aruco_dicts():
    names = [
        "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
        "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
        "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
        "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000"
    ]
    d = []
    for n in names:
        if hasattr(cv2.aruco, n):
            d.append((n, cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, n))))
    return d

def list_cameras(max_index=10, width=None, height=None):
    found = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            cap.release()
            continue
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, _ = cap.read()
        if ok:
            found.append(idx)
        cap.release()
    return found

def open_camera(camera_id=None, width=None, height=None):
    if camera_id is not None:
        indices = [camera_id]
    else:
        indices = list_cameras(max_index=10, width=width, height=height)
        if not indices:
            indices = [0, 1, 2, 3]
    for idx in indices:
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            cap.release()
            continue
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, idx
        cap.release()
    return None, None

def draw_detected(frame, corners, ids, rejected=None, dict_name=""):
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.putText(frame, f"Detected {len(ids)} marker(s) ({dict_name})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, f"No markers ({dict_name})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
    if rejected is not None:
        cv2.putText(frame, f"Rejected: {len(rejected)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

def main():
    parser = argparse.ArgumentParser(description="AR Tag (ArUco) detector with depth estimation")
    parser.add_argument("--camera-id", type=int, default=None, help="Camera index (default: auto-pick the first working)")
    parser.add_argument("--width", type=int, default=1280, help="Requested capture width")
    parser.add_argument("--height", type=int, default=720, help="Requested capture height")
    parser.add_argument("--save-dir", type=str, default="captures", help="Directory to save snapshots")
    parser.add_argument("--marker-length", type=float, default=0.05, help="Marker side length in meters")
    args = parser.parse_args()

    # Load ArUco dictionaries
    dicts = get_aruco_dicts()
    if not dicts:
        raise RuntimeError("Your OpenCV build lacks ArUco. Install opencv-contrib-python.")

    dict_idx = 2
    if dict_idx >= len(dicts):
        dict_idx = 0
    dictionary_name, dictionary = dicts[dict_idx]
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

    # Load camera calibration
    if not os.path.exists("camera_calibration.npz"):
        raise RuntimeError("camera_calibration.npz not found. Run calibrate_camera.py first.")
    calib = np.load("camera_calibration.npz")
    camera_matrix = calib["mtx"]
    dist_coeffs = calib["dist"]

    # Open camera
    cap, current_cam = open_camera(args.camera_id, args.width, args.height)
    if cap is None:
        raise RuntimeError("Could not open any camera. Try a different --camera-id.")

    os.makedirs(args.save_dir, exist_ok=True)
    print(f"[INFO] Using camera index {current_cam}")
    last_fps_t = time.time()
    frames = 0
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue

        corners, ids, rejected = detector.detectMarkers(frame)
        draw_detected(frame, corners, ids, rejected, dict_name=dictionary_name)

        # Pose estimation
        if ids is not None:
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], args.marker_length, camera_matrix, dist_coeffs
                )
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)                
                distance = np.linalg.norm(tvec)
                cv2.putText(frame, f"ID {ids[i][0]}: {distance:.2f} m",
                            (10, 120 + 30 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # FPS display
        frames += 1
        now = time.time()
        if now - last_fps_t >= 1.0:
            fps = frames / (now - last_fps_t)
            frames = 0
            last_fps_t = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("ArUco Depth Detector", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = f"{args.save_dir}/aruco_{ts}.png"
            cv2.imwrite(path, frame)
            print(f"[INFO] Saved snapshot to {path}")
        elif key == ord('d'):
            dict_idx = (dict_idx + 1) % len(dicts)
            dictionary_name, dictionary = dicts[dict_idx]
            detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
            print(f"[INFO] Switched dictionary to {dictionary_name}")
        elif key == ord('c'):
            cap.release()
            cap, current_cam = open_camera(None, args.width, args.height)
            print(f"[INFO] Switched to camera index {current_cam}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
