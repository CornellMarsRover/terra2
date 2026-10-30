#!/usr/bin/env python3
"""Offline five-command speech recognition for the astronaut demo.

Vosk and sounddevice are imported only when voice input is requested, so the
existing camera/gesture modes remain usable without a microphone or model.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


COMMAND_PHRASES = {
    "follow": "FOLLOW",
    "follow me": "FOLLOW",
    "follow us": "FOLLOW",
    "please follow": "FOLLOW",
    "stay": "STAY",
    "stay there": "STAY",
    "please stay": "STAY",
    "fetch": "FETCH",
    "go fetch": "FETCH",
    "please fetch": "FETCH",
    "come": "COME",
    "come here": "COME",
    "come on": "COME",
    "please come": "COME",
    "give": "GIVE",
    "give it": "GIVE",
    "give me": "GIVE",
    "please give": "GIVE",
}


class VoiceRecognitionError(RuntimeError):
    """Raised when offline speech recognition cannot be started."""


@dataclass(frozen=True)
class VoiceCommand:
    label: str
    confidence: float
    text: str
    timestamp: float


def command_from_vosk_result(
    payload: str | dict,
    minimum_confidence: float = 0.50,
) -> VoiceCommand | None:
    """Convert a final Vosk result into one of the five accepted commands."""
    try:
        result = json.loads(payload) if isinstance(payload, str) else payload
    except (TypeError, json.JSONDecodeError):
        return None

    text = " ".join(str(result.get("text", "")).lower().split())
    label = COMMAND_PHRASES.get(text)
    if label is None:
        return None

    words = result.get("result") or []
    confidences = []
    for word in words:
        try:
            value = float(word.get("conf", 0.0))
        except (AttributeError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            confidences.append(value)
    confidence = sum(confidences) / len(confidences) if confidences else 0.0
    if confidence < minimum_confidence:
        return None
    return VoiceCommand(
        label=label,
        confidence=confidence,
        text=text,
        timestamp=time.monotonic(),
    )


def amplify_pcm16(chunk: bytes, gain: float) -> tuple[bytes, float]:
    """Apply software gain to mono PCM16 and return its original dBFS level."""
    if not math.isfinite(gain) or gain <= 0.0:
        raise ValueError("audio gain must be a finite positive value")
    usable_bytes = len(chunk) - len(chunk) % 2
    if usable_bytes == 0:
        return b"", -math.inf
    samples = np.frombuffer(chunk[:usable_bytes], dtype="<i2").astype(np.float32)
    rms = float(np.sqrt(np.mean(np.square(samples, dtype=np.float32))))
    level_dbfs = -math.inf if rms <= 0.0 else 20.0 * math.log10(rms / 32768.0)
    if gain == 1.0:
        return chunk[:usable_bytes], level_dbfs
    boosted = np.clip(samples * gain, -32768.0, 32767.0).astype("<i2")
    return boosted.tobytes(), level_dbfs


def normalize_audio_device(device: str | int | None) -> str | int | None:
    """Allow a sounddevice index or a case-insensitive device-name fragment."""
    if device is None or isinstance(device, int):
        return device
    value = device.strip()
    if not value:
        return None
    try:
        return int(value)
    except ValueError:
        return value


def list_audio_input_devices() -> list[str]:
    """Return printable input-device descriptions without importing at startup."""
    try:
        import sounddevice as sd
    except ImportError as exc:
        raise VoiceRecognitionError(
            "sounddevice is not installed; install requirements-gesture-test.txt"
        ) from exc

    lines = []
    try:
        devices = sd.query_devices()
    except Exception as exc:
        raise VoiceRecognitionError(f"unable to enumerate audio devices: {exc}") from exc
    for index, device in enumerate(devices):
        channels = int(device.get("max_input_channels", 0))
        if channels <= 0:
            continue
        rate = float(device.get("default_samplerate", 0.0))
        lines.append(
            f"Audio[{index}] {device.get('name', 'unknown')} "
            f"(inputs={channels}, default_rate={rate:g} Hz)"
        )
    return lines


class VoiceCommandRecognizer:
    """Background offline recognizer using a constrained Vosk vocabulary."""

    def __init__(
        self,
        model_path: str | Path,
        device: str | int | None = None,
        minimum_confidence: float = 0.50,
        cooldown_seconds: float = 1.0,
        sample_rate: int | None = None,
        input_gain: float = 1.75,
    ):
        self.model_path = Path(model_path).expanduser()
        self.device = normalize_audio_device(device)
        self.minimum_confidence = float(minimum_confidence)
        self.cooldown_seconds = float(cooldown_seconds)
        self.input_gain = float(input_gain)
        self.sample_rate = sample_rate
        self.device_name = ""
        self.status = "DISABLED"
        self.last_error = ""
        self.audio_level_dbfs = -math.inf
        self._audio_queue: queue.Queue[bytes | None] = queue.Queue(maxsize=32)
        self._events: queue.Queue[VoiceCommand] = queue.Queue()
        self._stop_event = threading.Event()
        self._stream = None
        self._worker = None
        self._recognizer = None
        self._last_emitted_at = -math.inf

    def start(self) -> None:
        if not self.model_path.is_dir():
            raise VoiceRecognitionError(
                f"Vosk model directory not found: {self.model_path}. "
                "Download and extract an English model, then pass --voice-model PATH."
            )
        try:
            import sounddevice as sd
            from vosk import KaldiRecognizer, Model, SetLogLevel
        except ImportError as exc:
            raise VoiceRecognitionError(
                f"missing voice dependency {exc.name}; install "
                "requirements-gesture-test.txt"
            ) from exc

        try:
            device_info = sd.query_devices(self.device, "input")
            rate = self.sample_rate or int(device_info["default_samplerate"])
            if rate <= 0:
                raise ValueError("input device reports an invalid sample rate")
            SetLogLevel(-1)
            model = Model(str(self.model_path))
            grammar = json.dumps([*COMMAND_PHRASES, "[unk]"])
            recognizer = KaldiRecognizer(model, rate, grammar)
            recognizer.SetWords(True)
            stream = sd.RawInputStream(
                samplerate=rate,
                blocksize=2000,
                device=self.device,
                dtype="int16",
                channels=1,
                callback=self._audio_callback,
            )
            self.sample_rate = rate
            self.device_name = str(device_info["name"])
            self._recognizer = recognizer
            self._stream = stream
            self._stop_event.clear()
            self._worker = threading.Thread(
                target=self._recognition_loop,
                name="voice-command-recognizer",
                daemon=True,
            )
            self._worker.start()
            stream.start()
            self.status = "LISTENING"
        except Exception as exc:
            self.stop()
            raise VoiceRecognitionError(f"unable to start microphone recognition: {exc}") from exc

    def _audio_callback(self, indata, _frames, _time_info, status) -> None:
        if status:
            self.status = "AUDIO_WARNING"
            self.last_error = str(status)
        elif self.status == "AUDIO_WARNING":
            self.status = "LISTENING"
            self.last_error = ""
        chunk = bytes(indata)
        try:
            self._audio_queue.put_nowait(chunk)
        except queue.Full:
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._audio_queue.put_nowait(chunk)
            except queue.Full:
                pass

    def _recognition_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                chunk = self._audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if chunk is None:
                break
            try:
                processed, self.audio_level_dbfs = amplify_pcm16(
                    chunk,
                    self.input_gain,
                )
                is_final = self._recognizer.AcceptWaveform(processed)
                if not is_final:
                    continue
                command = command_from_vosk_result(
                    self._recognizer.Result(),
                    self.minimum_confidence,
                )
                if command is None:
                    continue
                now = command.timestamp
                if now - self._last_emitted_at < self.cooldown_seconds:
                    continue
                self._last_emitted_at = now
                self._events.put(command)
                self.status = "LISTENING"
            except Exception as exc:
                self.status = "ERROR"
                self.last_error = str(exc)
                self._stop_event.set()
                break

    def poll(self) -> VoiceCommand | None:
        """Return the newest recognized command without blocking."""
        newest = None
        while True:
            try:
                newest = self._events.get_nowait()
            except queue.Empty:
                return newest

    def stop(self) -> None:
        self._stop_event.set()
        if self._stream is not None:
            try:
                self._stream.stop()
            except Exception:
                pass
            try:
                self._stream.close()
            except Exception:
                pass
            self._stream = None
        try:
            self._audio_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        self._worker = None
        if self.status != "ERROR":
            self.status = "STOPPED"

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.stop()
