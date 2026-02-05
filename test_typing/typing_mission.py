#!/usr/bin/env python3
"""
AR Tag (ArUco) detection with depth estimation from a USB/USB-C camera on macOS or any OS.
"""

import argparse
import platform
import time
import os
import cv2
import numpy as np
from datetime import datetime

# Camera access backend system
if platform.system() == "Windows":
    BACKEND = cv2.CAP_DSHOW
elif platform.system() == "Darwin":
    BACKEND = cv2.CAP_AVFOUNDATION
else:
    BACKEND = 0  # default for Linux


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

def make_board(dictionary):
    # Define marker corners given top left position and size
    def marker_corners(x, y, size):
        return np.array([
            [x, y, 0], [x + size, y, 0],
            [x + size, y + size, 0], [x, y + size, 0] ], dtype=np.float32)
    # Create keyboard positions and ids
    marker_length = 0.052
    # TODO Put in Aruco Tag Measurements
    boardCorners = [
        marker_corners(0.00, 0.00, marker_length),
        marker_corners(0.17, 0.00, marker_length),
        marker_corners(0.00, 0.106, marker_length),
        marker_corners(0.17, 0.106, marker_length)]
    boardIds = np.array([[0], [1], [2], [3]], dtype=np.int32)
    return cv2.aruco.Board(boardCorners, dictionary, boardIds)

def list_cameras(max_index=10, width=None, height=None):
    found = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx, BACKEND)
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
        cap = cv2.VideoCapture(idx, BACKEND)
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-id", type=int, default=None)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--save-dir", type=str, default="captures")
    parser.add_argument("--marker-length", type=float, default=0.035)
    args = parser.parse_args()

    dicts = get_aruco_dicts()
    dict_idx = 0  # DICT_4X4_50
    dictionary_name, dictionary = dicts[dict_idx]
    keyboardBoard = make_board(dictionary)

    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

    if not os.path.exists("camera_calibration.npz"):
        raise RuntimeError("camera_calibration.npz missing.")
    calib = np.load("camera_calibration.npz")
    camera_matrix = calib["mtx"]
    dist_coeffs = calib["dist"]

    # TODO replace 1 with args.camera-id
    cap, current_cam = open_camera(1, args.width, args.height)
    if cap is None:
        raise RuntimeError("Could not open camera")

    os.makedirs(args.save_dir, exist_ok=True)
    print(f"[INFO] Using camera index {current_cam}")
    print(f"[INFO] Using dictionary: {dictionary_name}")

    four_detected = False
    initialized = False
    top_left_id = None

    # NEW SEQUENCE LOGIC
    key_stage = 0
    mission_done = False

    last_fps_t = time.time()
    frames = 0
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        corners, ids, rejected = detector.detectMarkers(frame)
        draw_detected(frame, corners, ids, rejected, dict_name=dictionary_name)

        if ids is not None:
            rvec_init = np.zeros((3, 1), dtype=np.float32)
            tvec_init = np.zeros((3, 1), dtype=np.float32)
            retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
                corners, ids,keyboardBoard, camera_matrix, dist_coeffs,
                rvec_init, tvec_init)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

        # FPS calculation
        frames += 1
        now = time.time()
        if now - last_fps_t >= 1.0:
            fps = frames / (now - last_fps_t)
            frames = 0
            last_fps_t = now

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        # SHOW THE CAMERA FEED (this was missing)
        cv2.imshow("ArUco Depth Detector", frame)

        # ----- KEY HANDLING -----
        key = cv2.waitKey(1) & 0xFF

        if initialized:
            # wait for C
            if key_stage == 0 and key == ord('c'):
                print("C typed")
                key_stage = 1

            # wait for M
            elif key_stage == 1 and key == ord('m'):
                print("M typed")
                key_stage = 2

            # wait for R
            elif key_stage == 2 and key == ord('r'):
                print("R typed")
                key_stage = 3

            # complete
            if key_stage == 3 and not mission_done:
                print("mission complete")
                mission_done = True

        # quit/save/etc
        if key == ord('q'):
            break
        elif key == ord('s'):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"{args.save_dir}/aruco_{ts}.png", frame)
            print(f"[INFO] Saved snapshot to {args.save_dir}/aruco_{ts}.png")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
