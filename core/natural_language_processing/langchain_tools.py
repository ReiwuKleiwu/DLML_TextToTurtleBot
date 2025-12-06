from __future__ import annotations

import json
from typing import Any, Dict, Optional

from langchain.tools import tool

from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from core.commands.user_command import UserCommand
from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType

_blackboard = Blackboard()
_event_bus = EventBus()


@tool("queue_drive_command")
def queue_drive_command(distance_m: Any, direction: str = "forward") -> str:
    """Queue a drive command for execution by the behaviour tree."""

    try:
        distance_val = float(distance_m)
    except (TypeError, ValueError):
        return "Failed to queue drive command: distance_m must be a number."

    if distance_val <= 0.0:
        return "Failed to queue drive command: distance_m must be > 0."

    direction_norm = str(direction).lower()
    if direction_norm not in {"forward", "backward"}:
        return "Failed to queue drive command: direction must be 'forward' or 'backward'."

    command = UserCommand.drive(distance_m=distance_val, direction=direction_norm)
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    return (
        "Queued drive command"
        f" {command.command_id} for {distance_val:.2f} m {direction_norm}."
    )


@tool("queue_rotate_command")
def queue_rotate_command(angle_deg: Any, direction: str = "left") -> str:
    """Queue a rotate command for the behaviour tree."""

    try:
        angle_val = float(angle_deg)
    except (TypeError, ValueError):
        return "Failed to queue rotate command: angle_deg must be a number."

    if angle_val <= 0.0 or angle_val > 360.0:
        return "Failed to queue rotate command: angle_deg must be in (0, 360]."

    direction_norm = str(direction).lower()
    if direction_norm not in {"left", "right"}:
        return "Failed to queue rotate command: direction must be 'left' or 'right'."

    command = UserCommand.rotate(angle_deg=angle_val, direction=direction_norm)
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    return (
        "Queued rotate command"
        f" {command.command_id} for {angle_val:.1f}° {direction_norm}."
    )


@tool("queue_navigate_command")
def queue_navigate_command(x: Any, y: Any, theta: Optional[Any] = None) -> str:
    """Queue a navigate-to-pose command for the behaviour tree."""

    def _coerce_float(value: Any, name: str) -> Optional[float]:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a number.")

    try:
        x_val = _coerce_float(x, "x")
        y_val = _coerce_float(y, "y")
        theta_val = _coerce_float(theta, "theta")
    except ValueError as exc:
        return f"Failed to queue navigate command: {exc}"

    if x_val is None or y_val is None:
        return "Failed to queue navigate command: x and y are required."

    command = UserCommand.navigate(x=x_val, y=y_val, theta=theta_val)
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    pose = command.parameters.get("pose", {})
    theta_text = f", theta={pose.get('theta'):.2f}" if "theta" in pose else ""
    return (
        "Queued navigate command"
        f" {command.command_id} to ({pose.get('x'):.2f}, {pose.get('y'):.2f}{theta_text})."
    )


@tool("queue_find_object_command")
def queue_find_object_command(object_class: Any) -> str:
    """Queue a find-object command for the behaviour tree."""

    label = str(object_class).strip()
    if not label:
        return "Failed to queue find-object command: object_class must be provided."

    command = UserCommand.find_object(object_class=label)
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    return (
        "Queued find-object command"
        f" {command.command_id} for class '{label}'."
    )


@tool("queue_dock_command")
def queue_dock_command() -> str:
    """Queue a dock command for the behaviour tree."""

    command = UserCommand.dock()
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    return f"Queued dock command {command.command_id}."


@tool("queue_undock_command")
def queue_undock_command() -> str:
    """Queue an undock command for the behaviour tree."""

    command = UserCommand.undock()
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    return f"Queued undock command {command.command_id}."


@tool("clear_command_queue")
def clear_command_queue(confirm: Any = False) -> str:
    """Clear all pending commands and cancel the active command if present."""

    confirm_flag = str(confirm).lower() in {"true", "1", "yes"}

    if not confirm_flag:
        return "Queue clear aborted; confirmation flag not set."

    pending = _blackboard.get(BlackboardDataKey.COMMAND_QUEUE, []) or []
    count = len(pending)
    _blackboard.clear_commands()
    return f"Cleared {count} queued commands and stopped any active command."


@tool("get_command_overview")
def get_command_overview() -> str:
    """Return summaries of the active command and pending queue."""

    snapshot = _blackboard.snapshot_command_state()
    return json.dumps(snapshot)


@tool("get_navigation_status")
def get_navigation_status() -> str:
    """Return the current navigation goal and status information."""

    snapshot = _blackboard.snapshot_navigation_status()
    return json.dumps(snapshot)


@tool("get_motion_status")
def get_motion_status() -> str:
    """Return drive and rotate progress measurements."""

    snapshot = _blackboard.snapshot_motion_status()
    return json.dumps(snapshot)


@tool("get_robot_map")
def get_robot_map() -> str:
    """Return the persistent tracked objects detected in the environment."""

    snapshot = _blackboard.snapshot_robot_map()
    return json.dumps(snapshot)


@tool("get_robot_pose")
def get_robot_pose() -> str:
    """Return the robot's current position and orientation if available."""

    position = _blackboard.get(BlackboardDataKey.ROBOT_POSITION)
    orientation = _blackboard.get(BlackboardDataKey.ROBOT_ORIENTATION)

    def _serialize_message(msg: Any) -> Dict[str, Any]:
        if msg is None:
            return {}
        return {
            "x": getattr(msg, "x", None),
            "y": getattr(msg, "y", None),
            "z": getattr(msg, "z", None),
            "w": getattr(msg, "w", None),
        }

    payload = {
        "position": _serialize_message(position),
        "orientation": _serialize_message(orientation),
    }
    return json.dumps(payload)


@tool("set_behaviour_tree_pause")
def set_behaviour_tree_pause(paused: Any) -> str:
    """Pause or resume the behaviour tree ticking loop."""

    paused_flag = str(paused).lower() in {"true", "1", "yes"}

    if paused_flag:
        _blackboard.pause_behaviour_tree()
        state = "paused"
    else:
        _blackboard.resume_behaviour_tree()
        state = "resumed"
    return f"Behaviour tree {state}."


LANGCHAIN_TOOLS = [
    get_command_overview,
    get_navigation_status,
    get_motion_status,
    get_robot_map,
    get_robot_pose,
    queue_drive_command,
    queue_rotate_command,
    queue_navigate_command,
    queue_find_object_command,
    queue_dock_command,
    queue_undock_command,
    clear_command_queue,
    set_behaviour_tree_pause,
]
