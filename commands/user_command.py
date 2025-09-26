from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict
from uuid import uuid4
import time


class CommandType(Enum):
    NAVIGATE_TO_POSE = auto()
    ROTATE = auto()
    DRIVE = auto()
    FIND_OBJECT = auto()


@dataclass(frozen=True)
class UserCommand:
    """User command parsed from natural language and queued for execution."""

    command_type: CommandType
    parameters: Dict[str, Any] = field(default_factory=dict)
    command_id: str = field(default_factory=lambda: str(uuid4()))
    timestamp: float = field(default_factory=time.time)

    @classmethod
    def navigate(cls, x: float, y: float, theta: float = 0.0):
        return cls(
            command_type=CommandType.NAVIGATE_TO_POSE,
            parameters={"pose": {"x": x, "y": y, "theta": theta}},
        )

    @classmethod
    def drive(cls, distance_m: float, direction: str = "forward"):
        return cls(
            command_type=CommandType.DRIVE,
            parameters={"distance_m": distance_m, "direction": direction},
        )

    @classmethod
    def rotate(cls, angle_deg: float, direction: str):
        return cls(
            command_type=CommandType.ROTATE,
            parameters={"angle_deg": angle_deg, "direction": direction},
        )

    @classmethod
    def find_object(cls, object_class: str):
        return cls(
            command_type=CommandType.FIND_OBJECT,
            parameters={"object_class": object_class},
        )

    def cleanup(self) -> None:
        """Reset any blackboard state associated with this command."""
        from events.event_bus import EventBus
        from events.interfaces.events import DomainEvent, EventType

        bus = EventBus()

        if self.command_type == CommandType.DRIVE:
            bus.publish(DomainEvent(EventType.DRIVE_GOAL_CLEARED, None))
        elif self.command_type == CommandType.ROTATE:
            bus.publish(DomainEvent(EventType.ROTATE_GOAL_CLEARED, None))
        elif self.command_type == CommandType.NAVIGATE_TO_POSE:
            bus.publish(DomainEvent(EventType.NAVIGATION_CANCEL_REQUEST, None))
            bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_CLEARED, None))
