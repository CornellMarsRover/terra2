import copy
import unittest
from types import SimpleNamespace

import gesture_test as gestures
import numpy as np
from voice_recognizer import VoiceCommand


def point(x=0.5, y=0.5, z=0.0, visibility=0.99):
    return SimpleNamespace(x=x, y=y, z=z, visibility=visibility)


def pose(values):
    landmarks = [point() for _ in range(33)]
    for index, value in values.items():
        landmarks[index] = value
    return landmarks


def classify(landmarks):
    wrapped = SimpleNamespace(landmark=landmarks)
    world = SimpleNamespace(landmark=copy.deepcopy(landmarks))
    return gestures.GestureRecognizer().update(wrapped, world).label


class GestureRecognitionTests(unittest.TestCase):
    def test_recent_voice_command_overrides_gesture(self):
        gesture = gestures.Detection("STAY", 0.8)
        voice = VoiceCommand("COME", 0.9, "come", 10.0)
        command = gestures.select_command(gesture, voice, 10.5, 2.0)
        self.assertEqual(command.label, "COME")
        self.assertEqual(command.source, "VOICE")

    def test_expired_voice_command_falls_back_to_gesture(self):
        gesture = gestures.Detection("STAY", 0.8)
        voice = VoiceCommand("COME", 0.9, "come", 10.0)
        command = gestures.select_command(gesture, voice, 12.1, 2.0)
        self.assertEqual(command.label, "STAY")
        self.assertEqual(command.source, "GESTURE")

    def test_sdk_left_image_does_not_require_webcam_session(self):
        frame = np.zeros((12, 20, 3), dtype=np.uint8)
        self.assertIs(gestures.prepare_camera_frame(None, frame), frame)

    def test_follow_hands_on_hips(self):
        landmarks = pose(
            {
                gestures.LS: point(0.35, 0.25),
                gestures.RS: point(0.65, 0.25),
                gestures.LE: point(0.15, 0.45),
                gestures.RE: point(0.85, 0.45),
                gestures.LW: point(0.33, 0.60),
                gestures.RW: point(0.67, 0.60),
                gestures.LH: point(0.40, 0.65),
                gestures.RH: point(0.60, 0.65),
            }
        )
        self.assertEqual(classify(landmarks), "FOLLOW")

    def test_stay_t_pose(self):
        landmarks = pose(
            {
                gestures.LS: point(0.40, 0.40),
                gestures.RS: point(0.60, 0.40),
                gestures.LE: point(0.25, 0.40),
                gestures.RE: point(0.75, 0.40),
                gestures.LW: point(0.05, 0.40),
                gestures.RW: point(0.95, 0.40),
                gestures.LH: point(0.45, 0.75),
                gestures.RH: point(0.55, 0.75),
            }
        )
        self.assertEqual(classify(landmarks), "STAY")

    def test_fetch_left_hand_on_neck(self):
        landmarks = pose(
            {
                gestures.LS: point(0.40, 0.30),
                gestures.RS: point(0.60, 0.30),
                gestures.LE: point(0.43, 0.30),
                gestures.LW: point(0.50, 0.23),
                gestures.RE: point(0.72, 0.48),
                gestures.RW: point(0.98, 0.43),
                gestures.LH: point(0.45, 0.70),
                gestures.RH: point(0.55, 0.70),
            }
        )
        self.assertEqual(classify(landmarks), "FETCH")

    def test_come_relaxed_roof(self):
        landmarks = pose(
            {
                gestures.NOSE: point(0.50, 0.23),
                gestures.LEFT_EYE: point(0.47, 0.21),
                gestures.RIGHT_EYE: point(0.53, 0.21),
                gestures.LS: point(0.40, 0.40),
                gestures.RS: point(0.60, 0.40),
                gestures.LE: point(0.30, 0.40),
                gestures.RE: point(0.70, 0.45),
                gestures.LW: point(0.49, 0.12),
                gestures.RW: point(0.52, 0.13),
                gestures.LH: point(0.45, 0.75),
                gestures.RH: point(0.55, 0.75),
            }
        )
        self.assertEqual(classify(landmarks), "COME")

    def test_give_crossed_forearms(self):
        landmarks = pose(
            {
                gestures.LS: point(0.40, 0.35),
                gestures.RS: point(0.60, 0.35),
                gestures.LE: point(0.25, 0.50),
                gestures.RE: point(0.75, 0.50),
                gestures.LW: point(0.58, 0.42),
                gestures.RW: point(0.42, 0.42),
                gestures.LH: point(0.45, 0.75),
                gestures.RH: point(0.55, 0.75),
            }
        )
        self.assertEqual(classify(landmarks), "GIVE")


if __name__ == "__main__":
    unittest.main()
