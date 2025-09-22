"""Backend controller wiring ROS, LangChain, and FastAPI."""
from __future__ import annotations

import asyncio
import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from langchain.agents import AgentType
from langchain_google_genai import ChatGoogleGenerativeAI

from code.application.llm_interface import create_langchain_agent, describe_robot_tools
from code.application.runtime.control_loop import ControlLoop
from code.infrastructure.event.threaded_event_bus import ThreadedEventBus
from code.infrastructure.ros.node import TurtleBotNode


class AgentBackend:
    """Owns the ROS control stack and the Gemini-powered LangChain agent."""

    def __init__(self, *, model: str, temperature: float, system_prompt_path: Optional[Path]) -> None:
        rclpy.init()

        self._shutdown_event = threading.Event()

        self._event_bus = ThreadedEventBus()
        self._event_bus.start()

        self._node = TurtleBotNode(event_bus=self._event_bus, namespace="", use_turtlebot_sim=True)
        self._control_loop = ControlLoop(rate_hz=2.0, tick_callback=self._node.orchestrator.perform_control_step)
        self._control_loop.start()

        self._spin_thread = threading.Thread(target=self._spin_node, daemon=True)
        self._spin_thread.start()

        if system_prompt_path:
            system_prompt = system_prompt_path.read_text(encoding="utf-8")
        else:
            default_prompt = Path(__file__).resolve().parents[1] / "application" / "llm_interface" / "default_system_prompt.txt"
            system_prompt = default_prompt.read_text(encoding="utf-8")

        llm = ChatGoogleGenerativeAI(model=model, temperature=temperature)
        self._agent = create_langchain_agent(
            llm,
            self._node.control_api,
            agent_type=AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION,
            system_prompt=system_prompt,
        )

        self._tool_descriptions = list(describe_robot_tools(self._node.control_api))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _spin_node(self) -> None:
        try:
            rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass

    @property
    def control_api(self):
        return self._node.control_api

    @property
    def tools(self) -> List[str]:
        return self._tool_descriptions

    # ------------------------------------------------------------------
    # Public API surface for the web layer
    # ------------------------------------------------------------------
    async def invoke_agent(self, prompt: str) -> Dict[str, Any]:
        start = time.time()
        result = await self._agent.ainvoke({"input": prompt})
        elapsed = time.time() - start

        output = result.get("output") if isinstance(result, dict) else None
        if not output:
            try:
                output = json.dumps(result, indent=2, sort_keys=True, default=str)
            except TypeError:
                output = str(result)

        snapshot = await asyncio.to_thread(self.control_api.get_snapshot)

        return {
            "output": output,
            "elapsed": elapsed,
            "snapshot": snapshot,
        }

    async def get_snapshot(self) -> Dict[str, Any]:
        return await asyncio.to_thread(self.control_api.get_snapshot)

    async def get_map_view(self) -> Dict[str, Any]:
        return await asyncio.to_thread(self.control_api.get_visualization_snapshot)

    async def get_camera_frame(self) -> Optional[bytes]:
        return await asyncio.to_thread(self.control_api.get_camera_frame)

    async def clear_map(self) -> None:
        await asyncio.to_thread(self.control_api.clear_map)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        if self._shutdown_event.is_set():
            return
        self._shutdown_event.set()

        try:
            rclpy.shutdown()
        except Exception:
            pass

        self._spin_thread.join(timeout=2.0)

        self._control_loop.stop()
        self._control_loop.join(timeout=2.0)

        try:
            self._node.destroy_node()
        except Exception:
            pass

        self._event_bus.stop()


__all__ = ["AgentBackend"]
