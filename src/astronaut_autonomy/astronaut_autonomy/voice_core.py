"""Offline voice-command recognition with a replaceable engine boundary."""

from __future__ import annotations

import json
import math
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


VOICE_COMMANDS = frozenset(("FOLLOW", "STAY", "FETCH", "COME", "GIVE"))
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
    """Raised when an offline recognition engine cannot process audio."""


@dataclass(frozen=True)
class VoiceRecognition:
    command: str
    confidence: float
    transcript: str


class OfflineVoiceEngine(Protocol):
    """Minimal boundary for Vosk or a future offline recognition engine."""

    def recognize_wav(
        self,
        path: str | Path,
        *,
        minimum_confidence: float,
        ambiguity_margin: float,
    ) -> list[VoiceRecognition]: ...


def normalize_transcript(text: object) -> str:
    return " ".join(str(text or "").lower().split())


def _candidate_confidence(candidate: dict) -> float:
    words = candidate.get("result") or []
    values = []
    for word in words:
        try:
            value = float(word.get("conf", math.nan))
        except (AttributeError, TypeError, ValueError):
            continue
        if math.isfinite(value) and 0.0 <= value <= 1.0:
            values.append(value)
    if values:
        return sum(values) / len(values)
    try:
        value = float(candidate.get("confidence", math.nan))
    except (AttributeError, TypeError, ValueError):
        return 0.0
    return value if math.isfinite(value) and 0.0 <= value <= 1.0 else 0.0


def recognition_from_vosk_result(
    payload: str | dict,
    minimum_confidence: float = 0.70,
    ambiguity_margin: float = 0.12,
) -> VoiceRecognition | None:
    """Validate one final Vosk result and reject weak or ambiguous commands."""
    if not 0.0 <= minimum_confidence <= 1.0:
        raise ValueError("minimum_confidence must be in [0, 1]")
    if not 0.0 <= ambiguity_margin <= 1.0:
        raise ValueError("ambiguity_margin must be in [0, 1]")
    try:
        result = json.loads(payload) if isinstance(payload, str) else payload
    except (TypeError, json.JSONDecodeError):
        return None
    if not isinstance(result, dict):
        return None

    alternatives = result.get("alternatives")
    candidates = alternatives if isinstance(alternatives, list) else [result]
    recognized = []
    for candidate in candidates:
        if not isinstance(candidate, dict):
            continue
        transcript = normalize_transcript(candidate.get("text", ""))
        command = COMMAND_PHRASES.get(transcript)
        if command is None:
            continue
        recognized.append(
            VoiceRecognition(
                command=command,
                confidence=_candidate_confidence(candidate),
                transcript=transcript,
            )
        )
    if not recognized:
        return None

    recognized.sort(key=lambda item: item.confidence, reverse=True)
    best = recognized[0]
    if best.confidence < minimum_confidence:
        return None
    distinct_runner_up = next(
        (item for item in recognized[1:] if item.command != best.command),
        None,
    )
    if (
        distinct_runner_up is not None
        and best.confidence - distinct_runner_up.confidence < ambiguity_margin
    ):
        return None
    return best


def read_pcm16_wav(path: str | Path, chunk_frames: int = 4000):
    """Return ``(sample_rate, chunks)`` for a mono, uncompressed PCM16 WAV."""
    wav_path = Path(path).expanduser()
    if chunk_frames < 1:
        raise ValueError("chunk_frames must be positive")
    if not wav_path.is_file():
        raise VoiceRecognitionError(f"WAV file not found: {wav_path}")
    try:
        audio = wave.open(str(wav_path), "rb")
    except (OSError, wave.Error) as exc:
        raise VoiceRecognitionError(f"cannot open WAV file {wav_path}: {exc}") from exc
    with audio:
        if audio.getnchannels() != 1:
            raise VoiceRecognitionError("voice WAV must contain one mono channel")
        if audio.getsampwidth() != 2:
            raise VoiceRecognitionError("voice WAV must use 16-bit PCM samples")
        if audio.getcomptype() != "NONE":
            raise VoiceRecognitionError("voice WAV must be uncompressed PCM")
        sample_rate = int(audio.getframerate())
        if sample_rate <= 0:
            raise VoiceRecognitionError("voice WAV has an invalid sample rate")
        chunks = []
        while True:
            chunk = audio.readframes(chunk_frames)
            if not chunk:
                break
            chunks.append(chunk)
    return sample_rate, chunks


class VoskOfflineEngine:
    """Small-vocabulary, CPU-only Vosk engine for WAV and microphone audio."""

    def __init__(self, model_path: str | Path):
        self.model_path = Path(model_path).expanduser()
        if not self.model_path.is_dir():
            raise VoiceRecognitionError(
                f"Vosk model directory not found: {self.model_path}"
            )
        try:
            from vosk import KaldiRecognizer, Model, SetLogLevel
        except ImportError as exc:
            raise VoiceRecognitionError(
                "vosk is not installed; run ./run voice-setup"
            ) from exc
        SetLogLevel(-1)
        self._recognizer_class = KaldiRecognizer
        self._model = Model(str(self.model_path))
        self._recognizer = None
        self._sample_rate = None

    def _new_recognizer(self, sample_rate: int):
        grammar = json.dumps([*COMMAND_PHRASES.keys(), "[unk]"])
        recognizer = self._recognizer_class(
            self._model,
            float(sample_rate),
            grammar,
        )
        recognizer.SetWords(True)
        if hasattr(recognizer, "SetMaxAlternatives"):
            recognizer.SetMaxAlternatives(3)
        return recognizer

    def reset_stream(self, sample_rate: int) -> None:
        if sample_rate <= 0:
            raise ValueError("sample_rate must be positive")
        self._sample_rate = int(sample_rate)
        self._recognizer = self._new_recognizer(self._sample_rate)

    def accept_audio(
        self,
        chunk: bytes,
        *,
        sample_rate: int,
        minimum_confidence: float,
        ambiguity_margin: float,
    ) -> VoiceRecognition | None:
        if self._recognizer is None or self._sample_rate != int(sample_rate):
            self.reset_stream(sample_rate)
        if not self._recognizer.AcceptWaveform(chunk):
            return None
        return recognition_from_vosk_result(
            self._recognizer.Result(),
            minimum_confidence,
            ambiguity_margin,
        )

    def recognize_wav(
        self,
        path: str | Path,
        *,
        minimum_confidence: float,
        ambiguity_margin: float,
    ) -> list[VoiceRecognition]:
        sample_rate, chunks = read_pcm16_wav(path)
        recognizer = self._new_recognizer(sample_rate)
        results = []
        for chunk in chunks:
            if recognizer.AcceptWaveform(chunk):
                recognition = recognition_from_vosk_result(
                    recognizer.Result(),
                    minimum_confidence,
                    ambiguity_margin,
                )
                if recognition is not None:
                    results.append(recognition)
        final = recognition_from_vosk_result(
            recognizer.FinalResult(),
            minimum_confidence,
            ambiguity_margin,
        )
        if final is not None:
            results.append(final)
        return results


class VoiceCommandGate:
    """Reject repeated recognitions without hiding later different commands."""

    def __init__(self, duplicate_cooldown_s: float = 1.5):
        if not math.isfinite(duplicate_cooldown_s) or duplicate_cooldown_s < 0.0:
            raise ValueError("duplicate_cooldown_s must be finite and nonnegative")
        self.duplicate_cooldown_s = float(duplicate_cooldown_s)
        self._last_command = None
        self._last_time = -math.inf

    def accept(self, recognition: VoiceRecognition, now_s: float) -> bool:
        if not math.isfinite(now_s):
            return False
        if recognition.command not in VOICE_COMMANDS:
            return False
        if (
            recognition.command == self._last_command
            and now_s - self._last_time < self.duplicate_cooldown_s
        ):
            return False
        self._last_command = recognition.command
        self._last_time = float(now_s)
        return True
