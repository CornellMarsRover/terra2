#!/usr/bin/env python3
"""
Camera calibration script for depth estimation with ArUco markers.
1. Capture several images of a printed 7x6 chessboard at different angles.
2. Place all images (in .png format) in a folder called 'calib_images/'.
3. Run this script to compute the camera matrix and distortion coefficients.

Output: camera_calibration.npz
"""

import cv2
import numpy as np
import glob

# --- Prepare object points like (0,0,0), (1,0,0), (2,0,0), ...,(6,5,0)
chessboard_size = (7, 6)
square_size = 1.0  # arbitrary units (scaling cancels out later)

objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

objpoints = []  # 3d points in real-world space
imgpoints = []  # 2d points in image plane

# --- Load calibration images (now .png instead of .jpg)
images = glob.glob('calib_images/*.png')
print(f"[INFO] Found {len(images)} PNG calibration images")

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"[WARN] Could not read {fname}, skipping.")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Calibration', img)
        cv2.waitKey(200)

cv2.destroyAllWindows()

# --- Calibrate camera
if len(objpoints) < 5:
    raise RuntimeError("Not enough valid calibration images found. Take more chessboard pictures.")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)
np.savez('camera_calibration.npz', mtx=mtx, dist=dist)
print("[INFO] Calibration successful. Saved to camera_calibration.npz")
