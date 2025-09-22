"""Stack based state machine that emits events on transitions."""
from __future__ import annotations

from typing import List, Optional

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus
from code.core.state import RobotState, RobotStateSource, StateSnapshot


class RobotStateMachine:
    """Maintains a stack of high-level robot states."""

    def __init__(self, event_bus: EventBus) -> None:
        self._event_bus = event_bus
        self._states: List[StateSnapshot] = [
            StateSnapshot(RobotState.IDLE, RobotStateSource.USER, {})
        ]
        self._log_state_stack()

    def push_state(
        self,
        new_state: RobotState,
        source: RobotStateSource,
        data: Optional[dict] = None,
    ) -> None:
        current = self.current_state
        if current and current.value == new_state and current.source != RobotStateSource.USER:
            return

        snapshot = StateSnapshot(new_state, source, data or {})
        self._states.append(snapshot)

        self._publish(EventType.STATE_PUSHED, {
            'new_state': new_state.name,
            'new_state_source': source.name,
            'old_state': current.value.name if current else None,
            'old_state_source': current.source.name if current else None,
            'state_data': snapshot.data,
        })

        self._publish(EventType.STATE_CHANGED, {
            'current_state': new_state.name,
            'current_state_source': source.name,
            'previous_state': current.value.name if current else None,
            'state_data': snapshot.data,
        })
        self._log_state_stack()

    def pop_state(self, state: RobotState, source: RobotStateSource) -> None:
        current = self.current_state
        if not current or current.value != state or current.source != source:
            return

        popped = self._states.pop()
        new_current = self.current_state

        self._publish(EventType.STATE_POPPED, {
            'popped_state': popped.value.name,
            'popped_state_source': popped.source.name,
            'new_current_state': new_current.value.name if new_current else None,
            'new_current_state_source': new_current.source.name if new_current else None,
        })

        self._publish(EventType.STATE_CHANGED, {
            'current_state': new_current.value.name if new_current else None,
            'current_state_source': new_current.source.name if new_current else None,
            'previous_state': popped.value.name,
            'state_data': new_current.data if new_current else None,
        })
        self._log_state_stack()

    @property
    def current_state(self) -> Optional[StateSnapshot]:
        if not self._states:
            return None
        return self._states[-1]

    def find_state(self, value: RobotState) -> Optional[StateSnapshot]:
        for snapshot in reversed(self._states):
            if snapshot.value == value:
                return snapshot
        return None

    def get_state_stack(self) -> list[StateSnapshot]:
        return list(self._states)

    def reset(self) -> None:
        self._states = [StateSnapshot(RobotState.IDLE, RobotStateSource.USER, {})]
        self._log_state_stack()

    def _publish(self, event_type: EventType, payload: dict) -> None:
        self._event_bus.publish_event(event_type, source="StateMachine", data=payload)

    def _log_state_stack(self) -> None:
        header = "[STATE STACK]"
        cols = ("Idx", "State", "Source", "Data")
        rows = [
            (
                str(index),
                snapshot.value.name,
                snapshot.source.name,
                repr(snapshot.data),
            )
            for index, snapshot in enumerate(self._states)
        ]
        widths = [
            max(len(col), *(len(row[i]) for row in rows)) if rows else len(col)
            for i, col in enumerate(cols)
        ]

        separator = "+".join("-" * width for width in widths)

        def fmt(row: tuple[str, str, str, str]) -> str:
            return " | ".join(row[i].ljust(widths[i]) for i in range(4))

        print(header)
        print(separator)
        print(fmt(cols))
        print(separator)
        for row in rows:
            print(fmt(row))
        print(separator)
