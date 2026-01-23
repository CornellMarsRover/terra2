#!/usr/bin/env python3
"""
AR Tag (ArUco) detection with depth estimation from a USB/USB-C camera on macOS or any OS.
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
    parser.add_argument("--camera-id", type=int, default=None)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--save-dir", type=str, default="captures")
    parser.add_argument("--marker-length", type=float, default=0.035)  # 3.5 cm
    args = parser.parse_args()

    # --- Load ArUco dictionaries ---
    dicts = get_aruco_dicts()
    if not dicts:
        raise RuntimeError("OpenCV ArUco not found.")

    # --- FIXED: Use DICT_4X4_50 (correct dictionary for your printed markers) ---
    dict_idx = 0  # *** This is the correct dictionary index ***
    dictionary_name, dictionary = dicts[dict_idx]

    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

    # Load camera calibration
    if not os.path.exists("camera_calibration.npz"):
        raise RuntimeError("camera_calibration.npz missing.")
    calib = np.load("camera_calibration.npz")
    camera_matrix = calib["mtx"]
    dist_coeffs = calib["dist"]

    # Open camera
    cap, current_cam = open_camera(args.camera_id, args.width, args.height)
    if cap is None:
        raise RuntimeError("Could not open any camera.")

    os.makedirs(args.save_dir, exist_ok=True)
    print(f"[INFO] Using camera index {current_cam}")
    print(f"[INFO] Using dictionary: {dictionary_name}")
    
    last_fps_t = time.time()
    frames = 0
    fps = 0.0

    # --- STATE VARIABLES ---
    four_detected = False
    initialized = False
    top_left_id = None  # store ID of top-left marker forever

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        corners, ids, rejected = detector.detectMarkers(frame)
        draw_detected(frame, corners, ids, rejected, dict_name=dictionary_name)

        if ids is not None:

            # --- Detect 4 tags THE FIRST TIME ---
            if not four_detected and len(ids) == 4:
                print("4 detected")
                four_detected = True

                # Compute centroids
                centroids = [np.mean(c.reshape(4, 2), axis=0) for c in corners]

                # Identify top-left marker based on smallest x+y
                top_left_index = np.argmin([pt[0] + pt[1] for pt in centroids])
                top_left_id = int(ids[top_left_index][0])

                print(f"Top-left marker ID locked as: {top_left_id}")

            # Pose estimation for each marker
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], args.marker_length, camera_matrix, dist_coeffs
                )

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
                distance = float(np.linalg.norm(tvec))
                this_id = int(ids[i][0])

                cv2.putText(frame, f"ID {this_id}: {distance:.2f} m",
                            (10, 120 + 30 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # --- INITIALIZATION BASED ON LOCKED ID ---
                if four_detected and not initialized and top_left_id is not None:
                    if this_id == top_left_id and distance <= 0.2:
                        print("initialized")
                        initialized = True

        # FPS overlay
        frames += 1
        now = time.time()
        if now - last_fps_t >= 1.0:
            fps = frames / (now - last_fps_t)
            frames = 0
            last_fps_t = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

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
