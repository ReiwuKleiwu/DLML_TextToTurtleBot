"""Domain event definitions for the TurtleBot text command system."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import time
from typing import Any, Dict


class EventType(str, Enum):
    """Enumerates the domain events shared across the system."""

    # Camera and vision events
    OBJECT_DETECTED = "object_detected"
    DEPTH_DATA_UPDATED = "depth_data_updated"
    WORLD_COORDINATES_CALCULATED = "world_coordinates_calculated"
    TARGET_SELECTED = "target_selected"
    CAMERA_FRAME_PROCESSED = "camera_frame_processed"

    # LiDAR events
    LIDAR_SCAN_PROCESSED = "lidar_scan_processed"
    OBSTACLE_DETECTED = "obstacle_detected"
    OBSTACLE_CLEARED = "obstacle_cleared"

    # IR sensor events
    IR_OBSTACLE_DETECTED = "ir_obstacle_detected"
    IR_OBSTACLE_CLEARED = "ir_obstacle_cleared"

    # State machine events
    STATE_CHANGED = "state_changed"
    STATE_PUSHED = "state_pushed"
    STATE_POPPED = "state_popped"

    # Navigation events
    TARGET_REACHED = "target_reached"
    NAVIGATION_STARTED = "navigation_started"
    NAVIGATION_STOPPED = "navigation_stopped"

    # System events
    SENSOR_DATA_UPDATED = "sensor_data_updated"
    VISUALIZATION_UPDATE = "visualization_update"

    # Robot motion events
    ROBOT_TRANSFORM_UPDATED = "robot_transform_updated"


@dataclass(slots=True)
class DomainEvent:
    """Immutable payload describing a published domain event."""

    event_type: EventType
    source: str
    data: Dict[str, Any]
    timestamp: float = field(default_factory=lambda: time.time())
