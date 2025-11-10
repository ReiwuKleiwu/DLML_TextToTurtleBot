from __future__ import annotations

from typing import Optional

import py_trees
from py_trees.common import Status

from events.event_bus import EventBus
from events.interfaces.events import DomainEvent
from navigation.docking_client import DockingClient


class _BaseDockingSkill(py_trees.behaviour.Behaviour):
    """Shared logic for dock and undock skills driven by the docking action."""

    def __init__(self, name: str, client: DockingClient) -> None:
        super().__init__(name)
        self._client = client
        self._event_bus = EventBus()
        self._status: Optional[str] = None
        self._goal_active = False

    def initialise(self) -> None:
        self._status = None
        self._goal_active = True

    def update(self) -> Status:
        if self._status == "failed":
            self._goal_active = False
            return Status.FAILURE
        if self._status == "succeeded":
            self._goal_active = False
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if new_status == Status.INVALID and self._goal_active:
            self._client.cancel_goal()
        self._goal_active = False

    def _on_goal_succeeded(self, _: DomainEvent) -> None:
        if self._goal_active:
            self._status = "succeeded"

    def _on_goal_failed(self, _: DomainEvent) -> None:
        if self._goal_active:
            self._status = "failed"

    def _on_goal_rejected(self, _: DomainEvent) -> None:
        if self._goal_active:
            self._status = "failed"

