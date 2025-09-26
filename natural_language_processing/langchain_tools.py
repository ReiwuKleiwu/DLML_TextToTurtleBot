from __future__ import annotations

import json
from typing import Literal, Optional

from langchain.tools import tool
from pydantic import BaseModel, Field, confloat

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from commands.user_command import UserCommand
from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType

_blackboard = Blackboard()
_event_bus = EventBus()


class DriveCommandInput(BaseModel):
    distance_m: confloat(gt=0.0) = Field(
        ...,
        description="Positive distance in meters to drive.",
    )
    direction: Literal["forward", "backward"] = Field(
        "forward",
        description="Direction of travel relative to the robot's frame.",
    )


@tool(name="queue_drive_command", args_schema=DriveCommandInput)
def queue_drive_command(params: DriveCommandInput) -> str:
    """Queue a drive command for execution by the behaviour tree."""

    command = UserCommand.drive(distance_m=params.distance_m, direction=params.direction)
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    return (
        "Queued drive command"
        f" {command.command_id} for {params.distance_m:.2f} m {params.direction}."
    )


class RotateCommandInput(BaseModel):
    angle_deg: confloat(gt=0.0, le=360.0) = Field(
        ...,
        description="Positive rotation magnitude in degrees (0-360].",
    )
    direction: Literal["left", "right"] = Field(
        ...,
        description="Rotation direction around the vertical axis.",
    )


@tool(name="queue_rotate_command", args_schema=RotateCommandInput)
def queue_rotate_command(params: RotateCommandInput) -> str:
    """Queue a rotate command for the behaviour tree."""

    command = UserCommand.rotate(angle_deg=params.angle_deg, direction=params.direction)
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    return (
        "Queued rotate command"
        f" {command.command_id} for {params.angle_deg:.1f}° {params.direction}."
    )


class NavigateCommandInput(BaseModel):
    x: float = Field(..., description="Target x-coordinate in meters.")
    y: float = Field(..., description="Target y-coordinate in meters.")
    theta: Optional[float] = Field(
        None,
        description="Optional heading in radians relative to the map frame.",
    )


@tool(name="queue_navigate_command", args_schema=NavigateCommandInput)
def queue_navigate_command(params: NavigateCommandInput) -> str:
    """Queue a navigate-to-pose command for the behaviour tree."""

    command = UserCommand.navigate(x=params.x, y=params.y, theta=params.theta)
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    pose = command.parameters.get("pose", {})
    theta_text = f", theta={pose.get('theta'):.2f}" if "theta" in pose else ""
    return (
        "Queued navigate command"
        f" {command.command_id} to ({pose.get('x'):.2f}, {pose.get('y'):.2f}{theta_text})."
    )


class FindObjectCommandInput(BaseModel):
    object_class: str = Field(..., min_length=1, description="Object class to search for.")


@tool(name="queue_find_object_command", args_schema=FindObjectCommandInput)
def queue_find_object_command(params: FindObjectCommandInput) -> str:
    """Queue a find-object command for the behaviour tree."""

    command = UserCommand.find_object(object_class=params.object_class)
    _event_bus.publish(DomainEvent(EventType.COMMAND_RECEIVED, command))
    return (
        "Queued find-object command"
        f" {command.command_id} for class '{params.object_class}'."
    )


class ClearCommandQueueInput(BaseModel):
    confirm: bool = Field(..., description="Set to true to clear all pending and active commands.")


@tool(name="clear_command_queue", args_schema=ClearCommandQueueInput)
def clear_command_queue(params: ClearCommandQueueInput) -> str:
    """Clear all pending commands and cancel the active command if present."""

    if not params.confirm:
        return "Queue clear aborted; confirmation flag not set."

    pending = _blackboard.get(BlackboardDataKey.COMMAND_QUEUE, []) or []
    count = len(pending)
    _blackboard.clear_commands()
    return f"Cleared {count} queued commands and stopped any active command."


class CommandOverviewInput(BaseModel):
    """No parameters required."""


@tool(name="get_command_overview", args_schema=CommandOverviewInput)
def get_command_overview(_: CommandOverviewInput) -> str:
    """Return summaries of the active command and pending queue."""

    snapshot = _blackboard.snapshot_command_state()
    return json.dumps(snapshot)


class NavigationStatusInput(BaseModel):
    """No parameters required."""


@tool(name="get_navigation_status", args_schema=NavigationStatusInput)
def get_navigation_status(_: NavigationStatusInput) -> str:
    """Return the current navigation goal and status information."""

    snapshot = _blackboard.snapshot_navigation_status()
    return json.dumps(snapshot)


class MotionStatusInput(BaseModel):
    """No parameters required."""


@tool(name="get_motion_status", args_schema=MotionStatusInput)
def get_motion_status(_: MotionStatusInput) -> str:
    """Return drive and rotate progress measurements."""

    snapshot = _blackboard.snapshot_motion_status()
    return json.dumps(snapshot)


class RobotMapInput(BaseModel):
    """No parameters required."""


@tool(name="get_robot_map", args_schema=RobotMapInput)
def get_robot_map(_: RobotMapInput) -> str:
    """Return the persistent tracked objects detected in the environment."""

    snapshot = _blackboard.snapshot_robot_map()
    return json.dumps(snapshot)


class BehaviourTreePauseInput(BaseModel):
    paused: bool = Field(..., description="True pauses ticking; false resumes execution.")


@tool(name="set_behaviour_tree_pause", args_schema=BehaviourTreePauseInput)
def set_behaviour_tree_pause(params: BehaviourTreePauseInput) -> str:
    """Pause or resume the behaviour tree ticking loop."""

    if params.paused:
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
    queue_drive_command,
    queue_rotate_command,
    queue_navigate_command,
    queue_find_object_command,
    clear_command_queue,
    set_behaviour_tree_pause,
]
