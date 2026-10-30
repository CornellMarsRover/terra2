import json
import struct
import tempfile
import unittest
import wave
from pathlib import Path

from astronaut_autonomy.voice_core import (
    VoiceCommandGate,
    VoiceRecognition,
    VoiceRecognitionError,
    VoskOfflineEngine,
    read_pcm16_wav,
    recognition_from_vosk_result,
)


def vosk_result(text, confidence):
    return json.dumps(
        {
            "text": text,
            "result": [
                {"word": word, "conf": confidence}
                for word in text.split()
            ],
        }
    )


class MarkerWavEngine:
    """Deterministic test double; markers stand in for unavailable recordings."""

    labels = {1: "FOLLOW", 2: "STAY", 3: "FETCH", 4: "COME", 5: "GIVE"}

    def recognize_wav(self, path, *, minimum_confidence, ambiguity_margin):
        _rate, chunks = read_pcm16_wav(path)
        marker = struct.unpack("<h", chunks[0][:2])[0]
        label = self.labels[marker]
        return [VoiceRecognition(label, 0.92, label.lower())]


def write_marker_wav(path, marker):
    with wave.open(str(path), "wb") as audio:
        audio.setnchannels(1)
        audio.setsampwidth(2)
        audio.setframerate(16000)
        audio.writeframes(struct.pack("<h", marker) * 320)


class VoiceCoreTests(unittest.TestCase):
    def test_all_five_commands_pass_through_wav_engine_boundary(self):
        engine = MarkerWavEngine()
        with tempfile.TemporaryDirectory() as directory:
            for marker, expected in engine.labels.items():
                with self.subTest(command=expected):
                    path = Path(directory) / f"{expected.lower()}.wav"
                    write_marker_wav(path, marker)
                    recognized = engine.recognize_wav(
                        path,
                        minimum_confidence=0.70,
                        ambiguity_margin=0.12,
                    )
                    self.assertEqual(recognized[0].command, expected)

    def test_supported_phrase_and_word_confidence(self):
        result = recognition_from_vosk_result(
            vosk_result("come here", 0.84),
            minimum_confidence=0.70,
        )
        self.assertEqual(result.command, "COME")
        self.assertAlmostEqual(result.confidence, 0.84)

    def test_low_confidence_unknown_and_malformed_results_are_rejected(self):
        self.assertIsNone(
            recognition_from_vosk_result(vosk_result("follow", 0.45), 0.70)
        )
        self.assertIsNone(
            recognition_from_vosk_result(vosk_result("drive", 0.99), 0.70)
        )
        self.assertIsNone(recognition_from_vosk_result("not json", 0.70))

    def test_ambiguous_different_commands_are_rejected(self):
        payload = {
            "alternatives": [
                {"text": "come", "confidence": 0.82},
                {"text": "give", "confidence": 0.76},
            ]
        }
        self.assertIsNone(
            recognition_from_vosk_result(
                payload,
                minimum_confidence=0.70,
                ambiguity_margin=0.10,
            )
        )

    def test_clear_alternative_wins(self):
        payload = {
            "alternatives": [
                {"text": "stay", "confidence": 0.93},
                {"text": "fetch", "confidence": 0.60},
            ]
        }
        result = recognition_from_vosk_result(payload, 0.70, 0.12)
        self.assertEqual(result.command, "STAY")

    def test_duplicate_gate_rejects_only_recent_same_command(self):
        gate = VoiceCommandGate(duplicate_cooldown_s=1.5)
        follow = VoiceRecognition("FOLLOW", 0.9, "follow")
        stay = VoiceRecognition("STAY", 0.9, "stay")
        self.assertTrue(gate.accept(follow, 10.0))
        self.assertFalse(gate.accept(follow, 10.5))
        self.assertTrue(gate.accept(stay, 10.6))
        self.assertTrue(gate.accept(follow, 10.7))

    def test_invalid_wav_format_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stereo.wav"
            with wave.open(str(path), "wb") as audio:
                audio.setnchannels(2)
                audio.setsampwidth(2)
                audio.setframerate(16000)
                audio.writeframes(b"\0" * 32)
            with self.assertRaisesRegex(VoiceRecognitionError, "mono"):
                read_pcm16_wav(path)

    def test_missing_model_fails_without_importing_microphone_support(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(VoiceRecognitionError, "model directory"):
                VoskOfflineEngine(Path(directory) / "missing")


if __name__ == "__main__":
    unittest.main()
