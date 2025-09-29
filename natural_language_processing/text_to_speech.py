"""Utilities for converting LLM output into speech using OpenAI's GPT-4o TTS."""
from __future__ import annotations

import datetime as _dt
import logging
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

try:  # pragma: no-cover - optional dependency in offline environments
    from openai import OpenAI
    from openai import OpenAIError  # type: ignore
except Exception:  # noqa: BLE001 - fallback when OpenAI SDK is unavailable
    OpenAI = None  # type: ignore[assignment]
    OpenAIError = Exception  # type: ignore[assignment]


LOGGER = logging.getLogger(__name__)


class TextToSpeechError(RuntimeError):
    """Raised when speech synthesis fails."""


@dataclass(slots=True)
class TextToSpeechSettings:
    """Configuration for text-to-speech synthesis."""

    enabled: bool = False
    provider: str = "openai"
    model: str = "gpt-4o-mini-tts"
    voice: str = "ballad"
    audio_format: str = "mp3"
    output_dir: str = "audio"
    instructions: Optional[str] = None

    def resolved_output_dir(self, base: Optional[Path] = None) -> Path:
        base_path = base or Path(__file__).resolve().parent
        path = Path(self.output_dir)
        if not path.is_absolute():
            path = base_path / path
        return path

    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]]) -> "TextToSpeechSettings":
        if not isinstance(config, dict):
            return cls()
        return cls(
            enabled=bool(config.get("enabled", False)),
            provider=str(config.get("provider", "openai")),
            model=str(config.get("model", "gpt-4o-mini-tts")),
            voice=str(config.get("voice", "ballad")),
            audio_format=str(config.get("audio_format", "mp3")),
            output_dir=str(config.get("output_dir", "audio")),
            instructions=(
                str(config["instructions"])
                if isinstance(config.get("instructions"), str)
                else None
            ),
        )


class TextToSpeechService:
    """High level helper that turns text into speech audio files."""

    def __init__(
        self,
        settings: Optional[TextToSpeechSettings] = None,
        *,
        client: Optional[Any] = None,
    ) -> None:
        self._settings = settings or TextToSpeechSettings()
        self._client = client
        if self._client is None and self._settings.enabled:
            self._client = self._instantiate_client()

    @property
    def settings(self) -> TextToSpeechSettings:
        return self._settings

    def is_enabled(self) -> bool:
        return self._settings.enabled and self._client is not None

    def synthesise(self, text: str) -> Optional[Path]:
        """Generate speech for ``text`` and return the saved audio path."""
        if not self.is_enabled():
            return None

        message = (text or "").strip()
        if not message:
            LOGGER.debug("TTS skipped: empty input")
            return None

        if self._client is None:
            raise TextToSpeechError("OpenAI client unavailable for TTS")

        output_dir = self._settings.resolved_output_dir()
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:  # pragma: no-cover - filesystem issues are environment-specific
            raise TextToSpeechError(f"Unable to create audio directory '{output_dir}': {exc}") from exc

        extension = self._normalise_extension(self._settings.audio_format)
        if extension != ".mp3":
            LOGGER.warning(
                "Audio format '%s' is not supported for streaming output; defaulting to mp3.",
                self._settings.audio_format,
            )
            extension = ".mp3"

        filename = self._build_filename(extension)
        output_path = output_dir / filename

        request_kwargs = {
            "model": self._settings.model,
            "voice": self._settings.voice,
            "input": message,
        }
        instructions = self._compose_instructions()
        if instructions:
            request_kwargs["instructions"] = instructions

        try:
            return self._stream_to_file(request_kwargs, output_path)
        except OpenAIError as exc:  # type: ignore[arg-type]
            raise TextToSpeechError(f"OpenAI TTS synthesis failed: {exc}") from exc
        except Exception as exc:  # noqa: BLE001
            raise TextToSpeechError(f"Unexpected TTS failure: {exc}") from exc

    # Internal helpers -----------------------------------------------------

    def _instantiate_client(self) -> Optional[Any]:
        if self._settings.provider.lower() != "openai":
            LOGGER.warning("Unsupported TTS provider '%s'", self._settings.provider)
            return None
        if OpenAI is None:
            LOGGER.error("OpenAI SDK is not installed; TTS disabled")
            return None
        try:
            return OpenAI()
        except Exception as exc:  # noqa: BLE001
            LOGGER.error("Failed to initialise OpenAI client: %s", exc)
            return None

    def _stream_to_file(self, request: Dict[str, Any], output_path: Path) -> Path:
        LOGGER.debug(
            "Requesting TTS synthesis via OpenAI: voice=%s model=%s",
            request.get("voice"),
            request.get("model"),
        )
        with self._client.audio.speech.with_streaming_response.create(**request) as response:  # type: ignore[union-attr]
            response.stream_to_file(output_path)
        return output_path

    @staticmethod
    def _normalise_extension(audio_format: str) -> str:
        ext = (audio_format or "mp3").lower().strip()
        if not ext:
            return "mp3"
        if not ext.startswith("."):
            ext = f".{ext}"
        return ext

    @staticmethod
    def _build_filename(extension: str) -> str:
        timestamp = _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%S")
        unique = uuid.uuid4().hex[:8]
        return f"tts_{timestamp}_{unique}{extension}"

    def _compose_instructions(self) -> Optional[str]:
        if not self._settings.instructions:
            return None
        instructions = self._settings.instructions.strip()
        return instructions or None


__all__ = [
    "TextToSpeechService",
    "TextToSpeechSettings",
    "TextToSpeechError",
]
