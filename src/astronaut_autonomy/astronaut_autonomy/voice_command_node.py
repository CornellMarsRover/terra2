#!/usr/bin/env python3
"""ROS 2 offline voice-command source using a modular Vosk engine."""

from __future__ import annotations

import queue

import rclpy
from cmr_msgs.msg import AstronautVoiceCommand
from rclpy.node import Node
from std_msgs.msg import String

from astronaut_autonomy.voice_core import (
    VoiceCommandGate,
    VoiceRecognitionError,
    VoskOfflineEngine,
)


class VoiceCommandNode(Node):
    def __init__(self, engine_factory=VoskOfflineEngine):
        super().__init__("astronaut_voice_command")
        self._declare_parameters()
        self.minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        self.ambiguity_margin = float(
            self.get_parameter("ambiguity_margin").value
        )
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if not 0.0 <= self.ambiguity_margin <= 1.0:
            raise ValueError("ambiguity_margin must be in [0, 1]")
        self.input_mode = str(self.get_parameter("input_mode").value).lower()
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.gate = VoiceCommandGate(
            float(self.get_parameter("duplicate_cooldown_s").value)
        )
        self.engine = engine_factory(
            str(self.get_parameter("model_path").value)
        )
        self.publisher = self.create_publisher(
            AstronautVoiceCommand,
            "/autonomy/astronaut/voice_command",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/autonomy/astronaut/voice_status",
            10,
        )
        self.audio_queue = queue.Queue(maxsize=64)
        self.pending_wav_results = []
        self.stream = None

        if self.input_mode == "wav":
            wav_file = str(self.get_parameter("wav_file").value)
            if not wav_file:
                raise VoiceRecognitionError("wav_file is required in WAV mode")
            self.pending_wav_results = list(
                self.engine.recognize_wav(
                    wav_file,
                    minimum_confidence=self.minimum_confidence,
                    ambiguity_margin=self.ambiguity_margin,
                )
            )
            self.timer = self.create_timer(
                float(self.get_parameter("wav_publish_interval_s").value),
                self._publish_next_wav_result,
            )
            self._status(
                f"WAV_READY:{len(self.pending_wav_results)}_COMMANDS"
            )
        elif self.input_mode == "microphone":
            self._start_microphone()
            self.timer = self.create_timer(0.05, self._process_microphone)
            self._status("LISTENING")
        else:
            raise ValueError("input_mode must be 'microphone' or 'wav'")

    def _declare_parameters(self):
        defaults = {
            "model_path": "",
            "input_mode": "microphone",
            "wav_file": "",
            "audio_device": "",
            "sample_rate": 16000,
            "minimum_confidence": 0.70,
            "ambiguity_margin": 0.12,
            "duplicate_cooldown_s": 1.50,
            "wav_publish_interval_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _start_microphone(self):
        try:
            import sounddevice as sd
        except ImportError as exc:
            raise VoiceRecognitionError(
                "sounddevice is not installed; run ./run voice-setup"
            ) from exc
        device_value = str(self.get_parameter("audio_device").value).strip()
        device = (
            int(device_value)
            if device_value.isdigit()
            else device_value or None
        )
        self.stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=2000,
            device=device,
            dtype="int16",
            channels=1,
            callback=self._audio_callback,
        )
        self.stream.start()

    def _audio_callback(self, indata, _frames, _time_info, status):
        if status:
            self._status(f"AUDIO_WARNING:{status}")
        try:
            self.audio_queue.put_nowait(bytes(indata))
        except queue.Full:
            try:
                self.audio_queue.get_nowait()
                self.audio_queue.put_nowait(bytes(indata))
            except (queue.Empty, queue.Full):
                pass

    def _process_microphone(self):
        while True:
            try:
                chunk = self.audio_queue.get_nowait()
            except queue.Empty:
                return
            recognition = self.engine.accept_audio(
                chunk,
                sample_rate=self.sample_rate,
                minimum_confidence=self.minimum_confidence,
                ambiguity_margin=self.ambiguity_margin,
            )
            if recognition is not None:
                self._publish_recognition(recognition)

    def _publish_next_wav_result(self):
        if not self.pending_wav_results:
            self._status("WAV_COMPLETE")
            self.timer.cancel()
            return
        self._publish_recognition(self.pending_wav_results.pop(0))

    def _publish_recognition(self, recognition):
        now = self._now_s()
        if not self.gate.accept(recognition, now):
            self._status("DUPLICATE_REJECTED")
            return
        message = AstronautVoiceCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "offline_voice"
        message.command = recognition.command
        message.confidence = float(recognition.confidence)
        message.transcript = recognition.transcript
        message.source = "VOICE"
        self.publisher.publish(message)
        self._status(
            f"ACCEPTED:{recognition.command}:{recognition.confidence:.3f}"
        )

    def _status(self, value):
        if hasattr(self, "status_publisher"):
            self.status_publisher.publish(String(data=str(value)))

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def destroy_node(self):
        if self.stream is not None:
            try:
                self.stream.stop()
                self.stream.close()
            except Exception:
                pass
            self.stream = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceCommandNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, VoiceRecognitionError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        elif not isinstance(exc, KeyboardInterrupt):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
