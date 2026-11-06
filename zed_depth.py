#!/usr/bin/env python3
"""ZED SDK and explicitly simulated depth sources for gesture_test.py.

The official SDK is imported lazily so webcam and mock modes continue to work
on macOS and on machines where ``pyzed.sl`` is not installed.
"""

from __future__ import annotations

import importlib
import math
import platform
import time
from dataclasses import dataclass

import numpy as np


class ZedDepthError(RuntimeError):
    """Base error for ZED SDK setup and capture failures."""


class ZedSdkUnavailable(ZedDepthError):
    """Raised when the official ZED Python API cannot be imported."""


@dataclass(frozen=True)
class DepthFrame:
    """One image-aligned depth observation in camera coordinates and meters."""

    image_bgr: np.ndarray
    xyz_m: np.ndarray | None
    confidence: np.ndarray | None
    timestamp: float
    source_status: str
    simulated: bool = False


def load_zed_sdk():
    """Load ``pyzed.sl`` only when real SDK depth was explicitly requested."""
    if platform.system() == "Darwin":
        raise ZedSdkUnavailable(
            "The official Stereolabs ZED SDK does not run natively on macOS. "
            "Use --no-zed for gesture-only webcam mode or --mock-depth for "
            "clearly labeled simulated depth."
        )
    try:
        return importlib.import_module("pyzed.sl")
    except ImportError as exc:
        raise ZedSdkUnavailable(
            "pyzed.sl is not installed. Install the Stereolabs ZED SDK and its "
            "Python API on the supported NVIDIA Jetson/Linux host."
        ) from exc


class ZedDepthCamera:
    """Synchronized ZED left image, XYZ point cloud, and confidence capture."""

    def __init__(
        self,
        resolution: str = "HD720",
        fps: int = 30,
        depth_mode: str = "NEURAL",
        confidence_threshold: int = 80,
    ):
        self.resolution = resolution.upper()
        self.fps = int(fps)
        self.depth_mode = depth_mode.upper()
        self.confidence_threshold = int(confidence_threshold)
        self.sl = None
        self.camera = None
        self.runtime = None
        self.image = None
        self.xyz = None
        self.confidence = None

    def open(self) -> None:
        sl = load_zed_sdk()
        try:
            resolution = getattr(sl.RESOLUTION, self.resolution)
            depth_mode = getattr(sl.DEPTH_MODE, self.depth_mode)
        except AttributeError as exc:
            raise ZedDepthError(
                f"Unsupported ZED configuration: resolution={self.resolution}, "
                f"depth_mode={self.depth_mode}"
            ) from exc

        init = sl.InitParameters()
        init.camera_resolution = resolution
        init.camera_fps = self.fps
        init.depth_mode = depth_mode
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE

        camera = sl.Camera()
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            camera.close()
            raise ZedDepthError(f"Unable to open ZED camera through SDK: {status!r}")

        runtime = sl.RuntimeParameters()
        runtime.confidence_threshold = self.confidence_threshold
        self.sl = sl
        self.camera = camera
        self.runtime = runtime
        self.image = sl.Mat()
        self.xyz = sl.Mat()
        self.confidence = sl.Mat()

    def grab(self) -> DepthFrame:
        if self.camera is None or self.sl is None:
            raise ZedDepthError("ZED camera is not open")

        sl = self.sl
        status = self.camera.grab(self.runtime)
        if status != sl.ERROR_CODE.SUCCESS:
            raise ZedDepthError(f"ZED frame grab failed: {status!r}")

        image_status = self.camera.retrieve_image(self.image, sl.VIEW.LEFT)
        xyz_status = self.camera.retrieve_measure(self.xyz, sl.MEASURE.XYZ)
        confidence_status = self.camera.retrieve_measure(
            self.confidence,
            sl.MEASURE.CONFIDENCE,
        )
        # Some pyzed releases return ERROR_CODE here; others mutate the Mat and
        # return None. Treat only an explicit non-success code as a failure.
        if image_status is not None and image_status != sl.ERROR_CODE.SUCCESS:
            raise ZedDepthError(f"ZED left-image retrieval failed: {image_status!r}")
        if xyz_status is not None and xyz_status != sl.ERROR_CODE.SUCCESS:
            raise ZedDepthError(f"ZED XYZ retrieval failed: {xyz_status!r}")

        image_data = self.image.get_data()
        xyz_data = self.xyz.get_data()
        if image_data is None or xyz_data is None:
            raise ZedDepthError("ZED SDK returned an empty image or XYZ map")
        if image_data.ndim != 3 or image_data.shape[2] < 3:
            raise ZedDepthError(f"Unexpected ZED image shape: {image_data.shape}")
        if xyz_data.ndim != 3 or xyz_data.shape[2] < 3:
            raise ZedDepthError(f"Unexpected ZED XYZ shape: {xyz_data.shape}")

        # ZED VIEW.LEFT is BGRA; OpenCV and MediaPipe need a contiguous BGR image.
        image_bgr = np.ascontiguousarray(image_data[:, :, :3]).copy()
        xyz_m = xyz_data[:, :, :3]
        confidence = None
        if confidence_status is None or confidence_status == sl.ERROR_CODE.SUCCESS:
            confidence_data = self.confidence.get_data()
            if confidence_data is not None:
                confidence = np.asarray(confidence_data).squeeze()

        return DepthFrame(
            image_bgr=image_bgr,
            xyz_m=xyz_m,
            confidence=confidence,
            timestamp=time.monotonic(),
            source_status="SDK",
            simulated=False,
        )

    def close(self) -> None:
        if self.camera is not None:
            self.camera.close()
        self.camera = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()


class MockDepthProvider:
    """Generate a pinhole-like constant-depth map, always labeled simulated."""

    def __init__(self, distance_m: float = 3.0):
        if not math.isfinite(distance_m) or distance_m <= 0.0:
            raise ValueError("mock distance must be a finite positive value")
        self.distance_m = float(distance_m)
        self._shape = None
        self._xyz = None
        self._confidence = None

    def frame_for(self, image_bgr: np.ndarray, timestamp: float) -> DepthFrame:
        height, width = image_bgr.shape[:2]
        shape = (height, width)
        if shape != self._shape:
            # This is intentionally not camera calibration. It exists only to
            # exercise mapping, visualization, smoothing, and reporting locally.
            focal = float(max(width, height))
            x_axis = (
                (np.arange(width, dtype=np.float32) - (width - 1) / 2.0)
                / focal
                * self.distance_m
            )
            y_axis = (
                (np.arange(height, dtype=np.float32) - (height - 1) / 2.0)
                / focal
                * self.distance_m
            )
            x_map, y_map = np.meshgrid(x_axis, y_axis)
            z_map = np.full(shape, self.distance_m, dtype=np.float32)
            self._xyz = np.dstack((x_map, y_map, z_map))
            self._confidence = np.zeros(shape, dtype=np.float32)
            self._shape = shape
        return DepthFrame(
            image_bgr=image_bgr,
            xyz_m=self._xyz,
            confidence=self._confidence,
            timestamp=timestamp,
            source_status="SIMULATED",
            simulated=True,
        )
