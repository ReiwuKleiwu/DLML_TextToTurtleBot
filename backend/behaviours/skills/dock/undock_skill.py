from __future__ import annotations

from events.interfaces.events import EventType
from navigation.docking_client import DockingClient

from behaviours.skills.dock.docking_base import _BaseDockingSkill


class UndockSkill(_BaseDockingSkill):
    """Behaviour that issues an undock action and waits for completion."""

    def __init__(self, name: str, client: DockingClient) -> None:
        super().__init__(name, client)
        self._event_bus.subscribe(EventType.UNDOCK_GOAL_SUCCEEDED, self._on_goal_succeeded)
        self._event_bus.subscribe(EventType.UNDOCK_GOAL_ABORTED, self._on_goal_failed)
        self._event_bus.subscribe(EventType.UNDOCK_GOAL_CANCELLED, self._on_goal_failed)
        self._event_bus.subscribe(EventType.UNDOCK_GOAL_REJECTED, self._on_goal_rejected)

    def initialise(self) -> None:
        super().initialise()
        if not self._client.send_undock_goal():
            self._status = "failed"
            self._goal_active = False

