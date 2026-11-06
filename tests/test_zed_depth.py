import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from zed_depth import (
    MockDepthProvider,
    ZedDepthCamera,
    ZedDepthError,
    ZedSdkUnavailable,
    load_zed_sdk,
)


class ZedDepthTests(unittest.TestCase):
    def test_macos_sdk_request_fails_clearly(self):
        with mock.patch("zed_depth.platform.system", return_value="Darwin"):
            with self.assertRaisesRegex(ZedSdkUnavailable, "does not run natively"):
                load_zed_sdk()

    def test_missing_sdk_fails_without_webcam_fallback(self):
        with mock.patch("zed_depth.platform.system", return_value="Linux"):
            with mock.patch(
                "zed_depth.importlib.import_module",
                side_effect=ImportError("missing"),
            ):
                with self.assertRaisesRegex(ZedSdkUnavailable, "pyzed.sl"):
                    load_zed_sdk()

    def test_missing_camera_reports_sdk_open_failure(self):
        success = object()
        failure = object()

        class FakeCamera:
            def open(self, _init):
                return failure

            def close(self):
                pass

        fake_sdk = SimpleNamespace(
            RESOLUTION=SimpleNamespace(HD720=object()),
            DEPTH_MODE=SimpleNamespace(NEURAL=object()),
            UNIT=SimpleNamespace(METER=object()),
            COORDINATE_SYSTEM=SimpleNamespace(IMAGE=object()),
            ERROR_CODE=SimpleNamespace(SUCCESS=success),
            InitParameters=SimpleNamespace,
            Camera=FakeCamera,
        )
        with mock.patch("zed_depth.load_zed_sdk", return_value=fake_sdk):
            with self.assertRaisesRegex(ZedDepthError, "Unable to open ZED"):
                ZedDepthCamera().open()

    def test_mock_depth_is_explicitly_simulated(self):
        image = np.zeros((12, 20, 3), dtype=np.uint8)
        frame = MockDepthProvider(2.5).frame_for(image, timestamp=4.0)
        self.assertTrue(frame.simulated)
        self.assertEqual(frame.source_status, "SIMULATED")
        self.assertEqual(frame.xyz_m.shape, (12, 20, 3))
        self.assertTrue(np.allclose(frame.xyz_m[:, :, 2], 2.5))

    def test_capture_accepts_pyzed_retrieval_methods_returning_none(self):
        success = object()
        fake_sdk = SimpleNamespace(
            ERROR_CODE=SimpleNamespace(SUCCESS=success),
            VIEW=SimpleNamespace(LEFT=object()),
            MEASURE=SimpleNamespace(XYZ=object(), CONFIDENCE=object()),
        )

        class FakeCamera:
            def grab(self, _runtime):
                return success

            def retrieve_image(self, _mat, _view):
                return None

            def retrieve_measure(self, _mat, _measure):
                return None

        class FakeMat:
            def __init__(self, data):
                self.data = data

            def get_data(self):
                return self.data

        source = ZedDepthCamera()
        source.sl = fake_sdk
        source.camera = FakeCamera()
        source.runtime = object()
        source.image = FakeMat(np.zeros((4, 6, 4), dtype=np.uint8))
        source.xyz = FakeMat(np.ones((4, 6, 4), dtype=np.float32))
        source.confidence = FakeMat(np.zeros((4, 6), dtype=np.float32))
        frame = source.grab()
        self.assertEqual(frame.image_bgr.shape, (4, 6, 3))
        self.assertEqual(frame.xyz_m.shape, (4, 6, 3))
        self.assertFalse(frame.simulated)


if __name__ == "__main__":
    unittest.main()
