"""Domain state definitions for robot behaviour orchestration."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, Optional


class RobotState(Enum):
    """High level states used by the robot controller."""

    IDLE = auto()
    EXPLORE = auto()
    AVOID_OBSTACLE = auto()
    OBJECT_FOUND = auto()
    OBJECT_REACHED = auto()
    NAVIGATE = auto()
    EXIT = auto()


class RobotStateSource(Enum):
    """Captures the originator of a state transition."""

    USER = auto()
    LIDAR = auto()
    IR = auto()
    BUMPER = auto()
    CAMERA = auto()
    NAVIGATION = auto()


@dataclass
class StateSnapshot:
    """Data stored alongside a state on the state stack."""

    value: RobotState
    source: RobotStateSource
    data: Dict[str, object] = field(default_factory=dict)

    def set_data(self, payload: Optional[Dict[str, object]]) -> None:
        self.data = payload or {}
