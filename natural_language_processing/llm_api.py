from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple, Union

from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.tools import BaseTool
from langchain_google_genai import ChatGoogleGenerativeAI

from natural_language_processing.langchain_tools import LANGCHAIN_TOOLS


DEFAULT_SYSTEM_PROMPT = (
    "You are an autonomous robotics assistant that plans high-level actions for a TurtleBot. "
    "Always inspect the robot state using the available diagnostic tools before enqueueing commands. "
    "Prefer conservative actions, avoid redundant commands, and clearly explain any tool usage."
)

DEFAULT_PROMPT_PATH = Path(__file__).with_name("system_prompt.txt")


HistoryInput = Iterable[Union[BaseMessage, Tuple[str, str]]]


class LLMAPI:
    def __init__(
        self,
        model_name: str = "gemini-2.5-flash",
        temperature: float = 0.1,
        system_prompt_path: Optional[Path] = None,
    ) -> None:
        self._model_name = model_name
        self._temperature = temperature
        self._system_prompt_path = system_prompt_path or DEFAULT_PROMPT_PATH
        self._tools: Sequence[BaseTool] = LANGCHAIN_TOOLS
        self._executor: Optional[AgentExecutor] = None

    # Public API -----------------------------------------------------------------

    def get_tools(self) -> Sequence[BaseTool]:
        return self._tools

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

        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            raise RuntimeError(
                "GOOGLE_API_KEY environment variable is not set; cannot initialise Gemini LLM."
            )

        llm = ChatGoogleGenerativeAI(
            model=self._model_name,
            temperature=self._temperature,
        )

        prompt = ChatPromptTemplate.from_messages([
            ("system", self._load_system_prompt()),
            MessagesPlaceholder(variable_name="chat_history"),
            ("human", "{input}"),
            MessagesPlaceholder(variable_name="agent_scratchpad"),
        ])

        agent = create_tool_calling_agent(llm, list(self._tools), prompt)
        self._executor = AgentExecutor(agent=agent, tools=list(self._tools), verbose=False)
        return self._executor

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
