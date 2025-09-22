"""Service that determines when the robot has reached its target object."""
from __future__ import annotations

import math
import time
from typing import Optional

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus
from code.core.state import RobotState, RobotStateSource
from code.core.state_machine import RobotStateMachine


class TargetReachedService:
    """Evaluates whether the robot is close enough to the selected target."""

    def __init__(
        self,
        event_bus: EventBus,
        state_machine: RobotStateMachine,
        reach_distance_threshold: float = 1.5,
    ) -> None:
        self._bus = event_bus
        self._state_machine = state_machine
        self._reach_distance_threshold = reach_distance_threshold

        self._robot_position: Optional[tuple[float, float, float]] = None
        self._current_target_object: Optional[str] = None
        self._target_reached = False
        self._persistent_objects_map = {}

        self._bus.subscribe(EventType.ROBOT_TRANSFORM_UPDATED, self._on_robot_transform_updated)
        self._bus.subscribe(EventType.TARGET_SELECTED, self._on_target_selected)
        self._bus.subscribe(EventType.SENSOR_DATA_UPDATED, self._on_map_updated)

    def _on_robot_transform_updated(self, event: DomainEvent) -> None:
        self._robot_position = event.data.get('position')
        self._check_target_proximity()

    def _on_target_selected(self, event: DomainEvent) -> None:
        if event.source != "TargetSelector":
            return

        new_target = event.data.get('target_object_class')
        if new_target != self._current_target_object:
            self._current_target_object = new_target
            self._target_reached = False

    def _on_map_updated(self, event: DomainEvent) -> None:
        if event.source != "MapService":
            return
        self._persistent_objects_map = event.data.get('persistent_objects_map', {})
        self._check_target_proximity()

    def _check_target_proximity(self) -> None:
        if not all([self._robot_position, self._current_target_object, self._persistent_objects_map]):
            return

        if self._target_reached:
            return

        target_objects = self._persistent_objects_map.get(self._current_target_object)
        if not target_objects:
            return

        selected_target = None
        selected_distance = float('inf')

        for target_obj in target_objects:
            if target_obj.get('is_selected_target') and target_obj.get('world_coords'):
                distance = self._calculate_distance(self._robot_position, target_obj['world_coords'])
                selected_target = target_obj
                selected_distance = distance
                break

        if selected_target and selected_distance <= self._reach_distance_threshold:
            self._target_reached = True
            self._state_machine.pop_state(RobotState.OBJECT_FOUND, RobotStateSource.CAMERA)
            self._state_machine.pop_state(RobotState.EXPLORE, RobotStateSource.USER)

            self._bus.publish_event(
                EventType.TARGET_REACHED,
                source="TargetReachedService",
                data={
                    'target_object_class': self._current_target_object,
                    'target_world_coords': selected_target['world_coords'],
                    'robot_position': self._robot_position,
                    'distance': selected_distance,
                    'reach_threshold': self._reach_distance_threshold,
                    'timestamp': time.time(),
                }
            )
            self._current_target_object = None
            self._target_reached = False

    def _calculate_distance(self, pos1, pos2) -> float:
        if not pos1 or not pos2:
            return float('inf')

        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        dz = pos1[2] - pos2[2]

        return math.sqrt(dx * dx + dy * dy + dz * dz)
