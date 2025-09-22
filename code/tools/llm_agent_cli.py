"""CLI for driving the TurtleBot via a Google Gemini powered LangChain agent."""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from code.application.llm_interface import create_langchain_agent, describe_robot_tools
from code.application.runtime.control_loop import ControlLoop
from code.infrastructure.event.threaded_event_bus import ThreadedEventBus
from code.infrastructure.ros.node import TurtleBotNode

import rclpy
from langchain.agents import AgentType
from langchain_google_genai import ChatGoogleGenerativeAI


def _print_snapshot(snapshot: Dict[str, Any]) -> None:
    state_stack = snapshot.get('state_stack', [])
    nav_info = snapshot.get('navigation', {})
    print("\n--- Robot Snapshot ---")
    print(json.dumps(
        {
            'current_state': snapshot.get('current_state'),
            'target_queue': snapshot.get('target_queue'),
            'navigation': nav_info,
        },
        indent=2,
        sort_keys=True,
    ))
    print("----------------------\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive the TurtleBot with a LangChain agent powered by Google Gemini.")
    parser.add_argument(
        "--model",
        default="gemini-2.5-flash",
        help="Google Gemini model identifier (default: gemini-2.5-flash)",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.1,
        help="Sampling temperature for the chat model (default: 0.1)",
    )
    parser.add_argument(
        "--initial-prompt",
        help="Optional initial instruction to send to the agent.",
    )
    parser.add_argument(
        "--system-prompt-file",
        help="Path to a file containing system prompt instructions.",
    )
    args = parser.parse_args()

    rclpy.init()
    event_bus = ThreadedEventBus()
    event_bus.start()

    node = TurtleBotNode(event_bus=event_bus, namespace="", use_turtlebot_sim=True)
    control_loop = ControlLoop(rate_hz=2.0, tick_callback=node.orchestrator.perform_control_step)
    control_loop.start()

    def _spin_node() -> None:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=_spin_node, daemon=True)
    spin_thread.start()

    llm = ChatGoogleGenerativeAI(model=args.model, temperature=args.temperature)
    print("Using Google Gemini via langchain-google-genai. Ensure GOOGLE_API_KEY is set.")
    agent_type = AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION

    if args.system_prompt_file:
        system_prompt = Path(args.system_prompt_file).read_text(encoding='utf-8')
    else:
        default_prompt_path = REPO_ROOT / "code" / "application" / "llm_interface" / "default_system_prompt.txt"
        system_prompt = default_prompt_path.read_text(encoding='utf-8')

    agent = create_langchain_agent(
        llm,
        node.control_api,
        agent_type=agent_type,
        system_prompt=system_prompt,
    )

    print("Available tools:")
    for line in describe_robot_tools(node.control_api):
        print(f"  - {line}")
    print("\nType natural-language instructions. Use 'snapshot' to view state, 'quit' to exit.\n")

    async def interactive_loop() -> None:
        loop = asyncio.get_running_loop()

        async def snap() -> Dict[str, Any]:
            return await loop.run_in_executor(None, node.control_api.get_snapshot)

        try:
            if args.initial_prompt:
                start = time.time()
                result = await agent.ainvoke({"input": args.initial_prompt})
                duration = time.time() - start
                print("Initial response:", result)
                print(f"(Elapsed {duration:.1f}s)")
                _print_snapshot(await snap())

            while True:
                user_input = await loop.run_in_executor(None, input, "LLM command> ")
                user_input = user_input.strip()
                if user_input.lower() in {"quit", "exit"}:
                    break
                if user_input.lower() == "snapshot":
                    _print_snapshot(await snap())
                    continue
                if not user_input:
                    continue

                start = time.time()
                result = await agent.ainvoke({"input": user_input})
                duration = time.time() - start
                print("Response:", result)
                print(f"(Elapsed {duration:.1f}s)")
                _print_snapshot(await snap())
        except KeyboardInterrupt:
            pass

    try:
        asyncio.run(interactive_loop())
    finally:
        rclpy.shutdown()
        spin_thread.join()
        control_loop.stop()
        control_loop.join()
        node.destroy_node()
        event_bus.stop()



if __name__ == "__main__":
    main()
