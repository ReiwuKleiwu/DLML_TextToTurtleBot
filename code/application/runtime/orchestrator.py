"""High level orchestration for the TurtleBot behaviours."""
from __future__ import annotations

from typing import Any, Dict

from code.core.interfaces.event_bus import EventBus
from code.core.state import RobotState, RobotStateSource
from code.core.state_machine import RobotStateMachine


class TurtleBotOrchestrator:
    """Coordinates behaviours and exposes a simple API to the outside."""

    def __init__(
        self,
        event_bus: EventBus,
        state_machine: RobotStateMachine,
        behaviours: Dict[RobotState, Any],
    ) -> None:
        self._event_bus = event_bus
        self._state_machine = state_machine
        self._behaviours = behaviours
        self._paused = False

    def request_target(self, target: str) -> None:
        self._state_machine.push_state(
            RobotState.EXPLORE,
            RobotStateSource.USER,
            {"target_object": target},
        )

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    def is_paused(self) -> bool:
        return self._paused

    def perform_control_step(self) -> None:
        if self._paused:
            return
        current = self._state_machine.current_state
        if current is None:
            raise RuntimeError("State machine has no active state")

        behaviour = self._behaviours.get(current.value)
        if behaviour is None:
            return

        execute = getattr(behaviour, "execute", None)
        if callable(execute):
            execute()
