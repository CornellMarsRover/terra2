import json
import tempfile
import unittest

import numpy as np

from voice_recognizer import (
    VoiceCommandRecognizer,
    VoiceRecognitionError,
    amplify_pcm16,
    command_from_vosk_result,
    normalize_audio_device,
)


def result(text, *confidences):
    words = text.split()
    return json.dumps(
        {
            "text": text,
            "result": [
                {"word": word, "conf": confidence}
                for word, confidence in zip(words, confidences)
            ],
        }
    )


class VoiceRecognitionTests(unittest.TestCase):
    def test_all_five_commands_are_recognized(self):
        expected = {
            "follow": "FOLLOW",
            "stay": "STAY",
            "fetch": "FETCH",
            "come": "COME",
            "give": "GIVE",
        }
        for spoken, label in expected.items():
            with self.subTest(spoken=spoken):
                command = command_from_vosk_result(result(spoken, 0.91), 0.65)
                self.assertIsNotNone(command)
                self.assertEqual(command.label, label)

    def test_supported_natural_phrase_is_recognized(self):
        command = command_from_vosk_result(
            result("come here", 0.90, 0.80),
            0.65,
        )
        self.assertEqual(command.label, "COME")
        self.assertAlmostEqual(command.confidence, 0.85)

    def test_quiet_audio_is_amplified_and_clipped_safely(self):
        samples = np.array([1000, -1000, 20000, -20000], dtype="<i2")
        amplified, level_dbfs = amplify_pcm16(samples.tobytes(), 2.0)
        output = np.frombuffer(amplified, dtype="<i2")
        np.testing.assert_array_equal(output, [2000, -2000, 32767, -32768])
        self.assertLess(level_dbfs, 0.0)

    def test_silent_audio_level_is_negative_infinity(self):
        samples = np.zeros(8, dtype="<i2")
        amplified, level_dbfs = amplify_pcm16(samples.tobytes(), 1.75)
        self.assertEqual(amplified, samples.tobytes())
        self.assertEqual(level_dbfs, -float("inf"))

    def test_low_confidence_and_unknown_speech_are_rejected(self):
        self.assertIsNone(command_from_vosk_result(result("stay", 0.40), 0.65))
        self.assertIsNone(command_from_vosk_result(result("drive", 0.99), 0.65))
        self.assertIsNone(command_from_vosk_result("not json", 0.65))

    def test_audio_device_accepts_index_or_name(self):
        self.assertEqual(normalize_audio_device(" 3 "), 3)
        self.assertEqual(normalize_audio_device("USB microphone"), "USB microphone")
        self.assertIsNone(normalize_audio_device(""))

    def test_missing_model_fails_before_opening_microphone(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = f"{directory}/missing-model"
            with self.assertRaisesRegex(VoiceRecognitionError, "model directory"):
                VoiceCommandRecognizer(missing).start()


if __name__ == "__main__":
    unittest.main()
