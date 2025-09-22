"""Application controller that reacts to obstacle related events."""
from __future__ import annotations

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus
from code.core.state import RobotState, RobotStateSource
from code.core.state_machine import RobotStateMachine


class ObstacleController:
    """Translate sensor obstacle events into state transitions."""

    def __init__(self, event_bus: EventBus, state_machine: RobotStateMachine) -> None:
        self._bus = event_bus
        self._state_machine = state_machine

        self._bus.subscribe(EventType.OBSTACLE_DETECTED, self._on_lidar_detected)
        self._bus.subscribe(EventType.OBSTACLE_CLEARED, self._on_lidar_cleared)
        self._bus.subscribe(EventType.IR_OBSTACLE_DETECTED, self._on_ir_detected)
        self._bus.subscribe(EventType.IR_OBSTACLE_CLEARED, self._on_ir_cleared)

    def _on_lidar_detected(self, event: DomainEvent) -> None:
        direction = event.data.get('direction', 1)
        self._state_machine.push_state(
            RobotState.AVOID_OBSTACLE,
            RobotStateSource.LIDAR,
            data={"direction": direction},
        )

    def _on_lidar_cleared(self, event: DomainEvent) -> None:
        self._state_machine.pop_state(RobotState.AVOID_OBSTACLE, RobotStateSource.LIDAR)

    def _on_ir_detected(self, event: DomainEvent) -> None:
        self._state_machine.push_state(
            RobotState.AVOID_OBSTACLE,
            RobotStateSource.IR,
            data={"intensity": event.data.get('max_intensity')},
        )

    def _on_ir_cleared(self, event: DomainEvent) -> None:
        self._state_machine.pop_state(RobotState.AVOID_OBSTACLE, RobotStateSource.IR)
