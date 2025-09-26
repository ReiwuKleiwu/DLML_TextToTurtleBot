"""Domain event definitions for the TurtleBot text command system."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import time
from typing import Any, Dict


class EventType(str, Enum):
    """Enumerates the domain events shared across the system."""
    
    LIDAR_OBSTACLE_PRESENT = "lidar-obstacle-present"
    LIDAR_OBSTACLE_ABSENT = "lidar-no-obstacle-absent"

    OBJECTS_DETECTED = "objects-detected"
    OBJECT_WORLD_COORDINATES_UPDATED = "object-world-coordinates-updated"

    TARGET_OBJECT_SELECTED = "target-object-selected"
    TARGET_REACHED = 'target-reached'

    ROBOT_POSITION_UPDATED = "robot-position-updated"
    ROBOT_ORIENTATION_UPDATED = "robot-orientation-updated"
    ROBOT_IS_TURNING_UPDATED = "robot-is-turning-update"

    MAP_UPDATED = "map-updated"

    COMMAND_RECEIVED = "command-received"
    COMMAND_STARTED = "command-started"
    COMMAND_COMPLETED = "command-completed"
    COMMAND_CANCELLED = "command-cancelled"

    DRIVE_GOAL_SET = "drive-goal-set"
    DRIVE_PROGRESS_UPDATED = "drive-progress-updated"
    DRIVE_GOAL_CLEARED = "drive-goal-cleared"

    ROTATE_GOAL_SET = "rotate-goal-set"
    ROTATE_PROGRESS_UPDATED = "rotate-progress-updated"
    ROTATE_GOAL_CLEARED = "rotate-goal-cleared"

    NAVIGATION_GOAL_SENT = "navigation-goal-sent"
    NAVIGATION_GOAL_ACCEPTED = "navigation-goal-accepted"
    NAVIGATION_GOAL_REJECTED = "navigation-goal-rejected"
    NAVIGATION_GOAL_SUCCEEDED = "navigation-goal-succeeded"
    NAVIGATION_GOAL_ABORTED = "navigation-goal-aborted"
    NAVIGATION_GOAL_CANCELLED = "navigation-goal-cancelled"
    NAVIGATION_FEEDBACK = "navigation-feedback"
    NAVIGATION_GOAL_CLEARED = "navigation-goal-cleared"
    NAVIGATION_CANCEL_REQUEST = "navigation-cancel-request"


@dataclass(slots=True)
class DomainEvent:
    """Immutable payload describing a published domain event."""

    event_type: EventType
    data: Any
    timestamp: float = field(default_factory=lambda: time.time())
