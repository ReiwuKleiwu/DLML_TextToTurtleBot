from __future__ import annotations

import os
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional


class BaseLLMAdapter(ABC):
    """Abstract base class for LLM provider adapters."""

    def __init__(self, model_name: str, temperature: float = 0.1) -> None:
        self._model_name = model_name
        self._temperature = temperature

    @abstractmethod
    def create_llm(self) -> Any:
        """Return an instantiated LangChain chat model."""

    @property
    def model_name(self) -> str:
        return self._model_name

    @property
    def temperature(self) -> float:
        return self._temperature


class GeminiLLMAdapter(BaseLLMAdapter):
    """Adapter for Google Gemini chat models."""

    def __init__(
        self,
        model_name: str,
        temperature: float = 0.1,
        api_key: Optional[str] = None,
    ) -> None:
        super().__init__(model_name=model_name, temperature=temperature)
        self._configured_api_key = api_key

    def create_llm(self) -> Any:
        api_key = self._configured_api_key or os.getenv("GOOGLE_API_KEY")
        if not api_key:
            raise RuntimeError(
                "GOOGLE_API_KEY environment variable is not set; cannot initialise Gemini LLM."
            )
        try:
            from langchain_google_genai import ChatGoogleGenerativeAI
        except ImportError as exc:  # pragma: no cover - executed only when dependency missing
            raise RuntimeError(
                "langchain-google-genai is not installed; cannot initialise Gemini LLM."
            ) from exc

        return ChatGoogleGenerativeAI(
            model=self.model_name,
            temperature=self.temperature,
            google_api_key=api_key,
        )


class OpenAILLMAdapter(BaseLLMAdapter):
    """Adapter for OpenAI-hosted chat models."""

    def __init__(
        self,
        model_name: str,
        temperature: float = 0.1,
        api_key: Optional[str] = None,
        base_url: Optional[str] = None,
        organization: Optional[str] = None,
        additional_kwargs: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(model_name=model_name, temperature=temperature)
        self._configured_api_key = api_key
        self._base_url = base_url or os.getenv("OPENAI_BASE_URL") or os.getenv("OPENAI_API_BASE")
        self._organization = organization or os.getenv("OPENAI_ORGANIZATION")
        self._additional_kwargs = dict(additional_kwargs or {})

    def create_llm(self) -> Any:
        api_key = self._configured_api_key or os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError(
                "OPENAI_API_KEY environment variable is not set; cannot initialise OpenAI LLM."
            )
        try:
            from langchain_openai import ChatOpenAI
        except ImportError as exc:  # pragma: no cover - executed only when dependency missing
            raise RuntimeError(
                "langchain-openai is not installed; cannot initialise OpenAI LLM."
            ) from exc

        kwargs: Dict[str, Any] = {
            "model": self.model_name,
            "temperature": self.temperature,
            "api_key": api_key,
            **self._additional_kwargs,
        }
        if self._base_url:
            kwargs.setdefault("base_url", self._base_url)
        if self._organization:
            kwargs.setdefault("organization", self._organization)

        return ChatOpenAI(**kwargs)


class OllamaLLMAdapter(BaseLLMAdapter):
    """Adapter for locally hosted Ollama chat models."""

    def __init__(
        self,
        model_name: str,
        temperature: float = 0.1,
        base_url: Optional[str] = None,
        num_ctx: Optional[int] = None,
        additional_kwargs: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(model_name=model_name, temperature=temperature)
        self._base_url = base_url or os.getenv("OLLAMA_BASE_URL")
        self._num_ctx = self._validate_num_ctx(num_ctx)
        self._additional_kwargs = dict(additional_kwargs or {})

    @staticmethod
    def _validate_num_ctx(value: Optional[int]) -> Optional[int]:
        if value is not None:
            return value
        env_value = os.getenv("OLLAMA_NUM_CTX")
        if env_value is None:
            return None
        try:
            return int(env_value)
        except ValueError as exc:
            raise RuntimeError(
                "OLLAMA_NUM_CTX must be an integer if provided."
            ) from exc

    def create_llm(self) -> Any:
        try:
            from langchain_ollama import ChatOllama
        except ImportError as exc:  # pragma: no cover - executed only when dependency missing
            raise RuntimeError(
                "langchain-ollama is not installed; install it to use Ollama models."
            ) from exc

        kwargs: Dict[str, Any] = {
            "model": self.model_name,
            "temperature": self.temperature,
            **self._additional_kwargs,
        }
        if self._base_url:
            kwargs.setdefault("base_url", self._base_url)
        if self._num_ctx is not None:
            kwargs.setdefault("num_ctx", self._num_ctx)

        return ChatOllama(**kwargs)


def create_llm_adapter(
    provider: str,
    model_name: str,
    temperature: float = 0.1,
    **adapter_kwargs: Any,
) -> BaseLLMAdapter:
    """Factory function that returns an adapter for the requested provider."""

    provider_normalised = provider.strip().lower()

    if provider_normalised in {"gemini", "google", "google_gemini"}:
        return GeminiLLMAdapter(
            model_name=model_name,
            temperature=temperature,
            **adapter_kwargs,
        )

    if provider_normalised in {"openai", "gpt", "gpt4", "gpt-4"}:
        return OpenAILLMAdapter(
            model_name=model_name,
            temperature=temperature,
            **adapter_kwargs,
        )

    if provider_normalised in {"ollama", "local_ollama", "local"}:
        return OllamaLLMAdapter(
            model_name=model_name,
            temperature=temperature,
            **adapter_kwargs,
        )

    raise ValueError(f"Unsupported LLM provider '{provider}'.")


__all__ = [
    "BaseLLMAdapter",
    "GeminiLLMAdapter",
    "OpenAILLMAdapter",
    "OllamaLLMAdapter",
    "create_llm_adapter",
]
