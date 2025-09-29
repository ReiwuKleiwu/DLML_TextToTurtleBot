from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.tools import BaseTool

from natural_language_processing.langchain_tools import LANGCHAIN_TOOLS
from natural_language_processing.llm_adapters import (
    BaseLLMAdapter,
    create_llm_adapter,
)
from natural_language_processing.text_to_speech import TextToSpeechSettings


DEFAULT_SYSTEM_PROMPT = (
    "You are an autonomous robotics assistant that plans high-level actions for a TurtleBot. "
    "Always inspect the robot state using the available diagnostic tools before enqueueing commands. "
    "Prefer conservative actions, avoid redundant commands, and clearly explain any tool usage."
)

DEFAULT_PROMPT_PATH = Path(__file__).resolve().parent.parent / "system_prompt.txt"
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "llm_config.json"
DEFAULT_MAX_ITERATIONS = 40


HistoryInput = Iterable[Union[BaseMessage, Tuple[str, str]]]


class LLMAPI:
    def __init__(
        self,
        model_name: Optional[str] = None,
        temperature: Optional[float] = None,
        system_prompt_path: Optional[Path] = None,
        provider: Optional[str] = None,
        config_path: Optional[Path] = None,
        adapter: Optional[BaseLLMAdapter] = None,
        adapter_options: Optional[dict] = None,
    ) -> None:
        self._config = self._load_config(config_path)
        provider_name = self._resolve_provider(provider, adapter)
        provider_config = self._get_provider_config(provider_name)
        self._tts_settings = TextToSpeechSettings.from_config(
            self._config.get("text_to_speech") if isinstance(self._config, dict) else None
        )

        resolved_model_name = (
            model_name
            or provider_config.get("model")
            or self._default_model_for(provider_name)
        )
        resolved_temperature = (
            temperature
            if temperature is not None
            else provider_config.get("temperature", 0.1)
        )
        merged_adapter_options = self._merge_adapter_options(
            provider_config,
            adapter_options,
        )

        self._provider_name = provider_name
        self._model_name = resolved_model_name
        self._temperature = resolved_temperature
        self._system_prompt_path = system_prompt_path or DEFAULT_PROMPT_PATH
        self._tools: Sequence[BaseTool] = LANGCHAIN_TOOLS
        self._executor: Optional[AgentExecutor] = None
        self._adapter = self._initialise_adapter(
            provider_name,
            adapter,
            merged_adapter_options,
        )
        self._max_iterations = self._resolve_max_iterations(provider_config)

    # Public API -----------------------------------------------------------------

    def get_tools(self) -> Sequence[BaseTool]:
        return self._tools

    def get_tts_settings(self) -> TextToSpeechSettings:
        """Return the configured TTS settings."""
        return self._tts_settings

    def run(self, user_input: str, history: Optional[HistoryInput] = None) -> dict:
        """Execute the LLM agent with the provided input and optional history."""

        executor = self._ensure_executor()
        chat_history = list(self._normalise_history(history or []))

        result = executor.invoke({
            "input": user_input,
            "chat_history": chat_history,
        })

        output_text = result.get("output", "")
        chat_history.extend([
            HumanMessage(content=user_input),
            AIMessage(content=output_text),
        ])

        return {
            "output": output_text,
            "chat_history": chat_history,
        }

    # Internal helpers -----------------------------------------------------------

    def _ensure_executor(self) -> AgentExecutor:
        if self._executor is not None:
            return self._executor
        llm = self._adapter.create_llm()

        prompt = ChatPromptTemplate.from_messages([
            ("system", self._load_system_prompt()),
            MessagesPlaceholder(variable_name="chat_history"),
            ("human", "{input}"),
            MessagesPlaceholder(variable_name="agent_scratchpad"),
        ])

        agent = create_tool_calling_agent(llm, list(self._tools), prompt)
        self._executor = AgentExecutor(
            agent=agent,
            tools=list(self._tools),
            verbose=False,
            max_iterations=self._max_iterations,
        )
        return self._executor

    def _initialise_adapter(
        self,
        provider_name: str,
        adapter: Optional[BaseLLMAdapter],
        adapter_options: Optional[dict],
    ) -> BaseLLMAdapter:
        if adapter is not None:
            return adapter
        options = adapter_options or {}

        try:
            return create_llm_adapter(
                provider=provider_name,
                model_name=self._model_name,
                temperature=self._temperature,
                **options,
            )
        except Exception as exc:
            raise RuntimeError(
                f"Failed to initialise LLM adapter for provider '{provider_name}'."
            ) from exc

    def _resolve_provider(
        self,
        provider: Optional[str],
        adapter: Optional[BaseLLMAdapter],
    ) -> str:
        if adapter is not None:
            return (provider or "custom").strip()
        if provider and provider.strip():
            return provider.strip()
        config_default = self._config.get("default_provider") if isinstance(self._config, dict) else None
        if isinstance(config_default, str) and config_default.strip():
            return config_default.strip()
        env_provider = os.getenv("LLM_PROVIDER")
        if env_provider and env_provider.strip():
            return env_provider.strip()
        return "ollama"

    def _default_model_for(self, provider_name: str) -> str:
        model_from_env = os.getenv("LLM_MODEL_NAME")
        if model_from_env:
            return model_from_env

        provider_key = provider_name.lower()
        if provider_key in {"ollama", "local_ollama", "local"}:
            return os.getenv("OLLAMA_MODEL", "llama3")
        if provider_key in {"gemini", "google", "google_gemini"}:
            return os.getenv("GOOGLE_MODEL", "gemini-2.5-flash")
        return os.getenv("LLM_FALLBACK_MODEL", "llama3")

    def _resolve_max_iterations(self, provider_config: Dict[str, Any]) -> Optional[int]:
        env_value = os.getenv("LLM_MAX_ITERATIONS")
        if env_value:
            try:
                parsed = int(env_value)
            except ValueError as exc:
                raise RuntimeError(
                    "LLM_MAX_ITERATIONS must be an integer if set."
                ) from exc
            if parsed > 0:
                return parsed
            raise RuntimeError("LLM_MAX_ITERATIONS must be > 0 if provided.")

        provider_value = provider_config.get("max_iterations") if isinstance(provider_config, dict) else None
        if isinstance(provider_value, int) and provider_value > 0:
            return provider_value

        global_value = None
        if isinstance(self._config, dict):
            global_value = self._config.get("max_iterations")
        if isinstance(global_value, int) and global_value > 0:
            return global_value

        return DEFAULT_MAX_ITERATIONS

    def _load_config(self, config_path: Optional[Path]) -> Dict[str, Any]:
        resolved_path = self._resolve_config_path(config_path)
        self._config_path: Optional[Path] = resolved_path
        if not resolved_path:
            return {}
        try:
            with resolved_path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except FileNotFoundError:
            return {}
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"Failed to parse LLM config at '{resolved_path}': {exc}"
            ) from exc
        except OSError as exc:
            raise RuntimeError(
                f"Unable to read LLM config at '{resolved_path}': {exc}"
            ) from exc

        if not isinstance(data, dict):
            raise RuntimeError(
                f"LLM config at '{resolved_path}' must contain a JSON object."
            )
        return data

    def _resolve_config_path(self, config_path: Optional[Path]) -> Optional[Path]:
        if config_path is not None:
            path = Path(config_path)
            if not path.exists():
                raise RuntimeError(
                    f"Specified LLM config file '{path}' does not exist."
                )
            return path

        env_path = os.getenv("LLM_CONFIG_PATH")
        if env_path:
            path = Path(env_path)
            if not path.exists():
                raise RuntimeError(
                    f"LLM_CONFIG_PATH points to '{path}', which does not exist."
                )
            return path

        if DEFAULT_CONFIG_PATH.exists():
            return DEFAULT_CONFIG_PATH
        return None

    def _get_provider_config(self, provider_name: str) -> Dict[str, Any]:
        adapters = self._config.get("adapters") if isinstance(self._config, dict) else None
        if not isinstance(adapters, dict):
            return {}
        provider_key = provider_name.lower()
        if provider_key in adapters and isinstance(adapters[provider_key], dict):
            return adapters[provider_key]  # type: ignore[return-value]
        for key, value in adapters.items():
            if isinstance(key, str) and key.lower() == provider_key and isinstance(value, dict):
                return value
        return {}

    @staticmethod
    def _merge_adapter_options(
        provider_config: Dict[str, Any],
        override_options: Optional[dict],
    ) -> Dict[str, Any]:
        merged: Dict[str, Any] = {}
        config_options = provider_config.get("options") if isinstance(provider_config, dict) else None
        if isinstance(config_options, dict):
            merged.update(config_options)
        if override_options:
            merged.update(override_options)
        return merged

    def _load_system_prompt(self) -> str:
        prompt_path = self._system_prompt_path
        if prompt_path and prompt_path.exists():
            text = prompt_path.read_text(encoding="utf-8").strip()
            if text:
                return text
        return DEFAULT_SYSTEM_PROMPT

    @staticmethod
    def _normalise_history(history: HistoryInput) -> List[BaseMessage]:
        normalised: List[BaseMessage] = []
        for item in history:
            if isinstance(item, BaseMessage):
                normalised.append(item)
                continue
            if isinstance(item, tuple) and len(item) == 2:
                role, content = item
                if role.lower() in {"user", "human"}:
                    normalised.append(HumanMessage(content=content))
                elif role.lower() in {"assistant", "ai"}:
                    normalised.append(AIMessage(content=content))
            # Ignore unrecognised history entries silently to remain robust.
        return normalised
