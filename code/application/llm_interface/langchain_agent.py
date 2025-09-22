"""Helper utilities to integrate the robot control API with LangChain."""
from __future__ import annotations

import json
from typing import Iterable, List

from langchain.agents import AgentExecutor, AgentType, initialize_agent
from langchain_core.language_models import BaseLanguageModel
from langchain.tools import Tool

from code.application.llm_interface.control_api import RobotControlAPI


def build_robot_tools(control_api: RobotControlAPI) -> List[Tool]:
    """Create LangChain tool definitions bound to the robot control API."""

    def _request_target(target: str) -> str:
        control_api.request_target(target)
        return json.dumps({'result': 'target_requested', 'snapshot': control_api.get_snapshot()})

    def _cancel_target(_: str = "") -> str:
        cancelled = control_api.cancel_current_target()
        return json.dumps({'result': 'target_cancelled' if cancelled else 'no_target', 'snapshot': control_api.get_snapshot()})

    def _clear_targets(_: str = "") -> str:
        control_api.clear_all_targets()
        return json.dumps({'result': 'targets_cleared', 'snapshot': control_api.get_snapshot()})

    def _reset_state_stack(_: str = "") -> str:
        control_api.reset_state_stack()
        return json.dumps({'result': 'state_reset', 'snapshot': control_api.get_snapshot()})

    def _pause_execution(_: str = "") -> str:
        control_api.pause_execution()
        return json.dumps({'result': 'paused', 'snapshot': control_api.get_snapshot()})

    def _resume_execution(_: str = "") -> str:
        control_api.resume_execution()
        return json.dumps({'result': 'resumed', 'snapshot': control_api.get_snapshot()})

    def _queue_nav_goal(*args, **kwargs) -> str:
        x = kwargs.get('x')
        y = kwargs.get('y')
        yaw = kwargs.get('yaw')

        if args:
            if len(args) == 1 and isinstance(args[0], str):
                text = args[0].strip()
                if text:
                    payload: Dict[str, Any] = {}
                    if text.startswith('{'):
                        try:
                            payload = json.loads(text)
                        except json.JSONDecodeError:
                            pass
                    if not payload:
                        pieces = [piece.strip() for piece in text.replace(';', ',').split(',') if piece.strip()]
                        for piece in pieces:
                            if '=' in piece:
                                key, value = piece.split('=', 1)
                                payload[key.strip()] = value.strip()
                    if 'x' in payload and 'y' in payload:
                        x = payload.get('x', x)
                        y = payload.get('y', y)
                        yaw = payload.get('yaw', yaw)
            elif len(args) >= 2:
                x = args[0]
                y = args[1]
                if len(args) >= 3:
                    yaw = args[2]

        if x is None or y is None:
            return json.dumps({'error': 'queue_nav_goal requires x and y values.'})

        try:
            x_val = float(x)
            y_val = float(y)
            yaw_val = float(yaw) if yaw is not None else None
        except (TypeError, ValueError) as exc:
            return json.dumps({'error': f'Invalid nav goal parameters: {exc}'})

        success = control_api.queue_nav_goal(x_val, y_val, yaw_val)
        return json.dumps({'result': 'nav_goal_sent' if success else 'nav_goal_failed', 'snapshot': control_api.get_snapshot()})

    def _cancel_nav(_: str = "") -> str:
        control_api.cancel_nav_goal()
        return json.dumps({'result': 'nav_goal_cancelled', 'snapshot': control_api.get_snapshot()})

    def _get_snapshot(_: str = "") -> str:
        return json.dumps(control_api.get_snapshot())

    tools: List[Tool] = [
        Tool.from_function(
            func=_request_target,
            name="request_target",
            description="Push a new explore target onto the stack. Input: object class (string).",
        ),
        Tool.from_function(
            func=_cancel_target,
            name="cancel_current_target",
            description="Cancel the current user target if one is active.",
        ),
        Tool.from_function(
            func=_clear_targets,
            name="clear_all_targets",
            description="Clear all pending user targets and stop navigation.",
        ),
        Tool.from_function(
            func=_reset_state_stack,
            name="reset_state_stack",
            description="Reset the state machine to IDLE, cancelling navigation and clearing targets.",
        ),
        Tool.from_function(
            func=_pause_execution,
            name="pause_execution",
            description="Pause the orchestrator. Call before major stack edits to halt behaviour execution.",
        ),
        Tool.from_function(
            func=_resume_execution,
            name="resume_execution",
            description="Resume the orchestrator after you have finished editing the state stack.",
        ),
        Tool.from_function(
            func=_queue_nav_goal,
            name="queue_nav_goal",
            description="Send a manual Nav2 goal. Accepts keyword arguments (x=..., y=..., yaw=...) or strings like 'x=0, y=0'.",
        ),
        Tool.from_function(
            func=_cancel_nav,
            name="cancel_nav_goal",
            description="Cancel the active Nav2 navigation goal, if any.",
        ),
        Tool.from_function(
            func=_get_snapshot,
            name="get_robot_snapshot",
            description="Retrieve the current robot state snapshot as JSON.",
        ),
    ]

    return tools


def create_langchain_agent(
    llm: BaseLanguageModel,
    control_api: RobotControlAPI,
    *,
    agent_type: AgentType = AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION,
    verbose: bool = True,
    system_prompt: str | None = None,
) -> AgentExecutor:
    """Initialise a LangChain agent bound to the robot control API."""
    tools = build_robot_tools(control_api)
    agent_kwargs = {'system_message': system_prompt} if system_prompt else None
    return initialize_agent(
        tools=tools,
        llm=llm,
        agent=agent_type,
        verbose=verbose,
        agent_kwargs=agent_kwargs,
    )


def describe_robot_tools(control_api: RobotControlAPI) -> Iterable[str]:
    """Yield human-readable descriptions of the available tools."""
    for tool in build_robot_tools(control_api):
        yield f"{tool.name}: {tool.description}"
