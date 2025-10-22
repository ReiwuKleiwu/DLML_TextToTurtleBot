import py_trees
from py_trees.common import Status

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType


class WaitForNavOrTarget(py_trees.behaviour.Behaviour):
    """Waits until navigation succeeds or the target reached detector fires."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._blackboard = Blackboard()
        self._event_bus = EventBus()
        self._target_reached = False
        self._cancel_requested = False
        self._event_bus.subscribe(EventType.TARGET_REACHED, self._on_target_reached)

    def initialise(self) -> None:
        self._target_reached = False
        self._cancel_requested = False

    def update(self) -> Status:
        nav_status = self._blackboard.get(BlackboardDataKey.NAVIGATION_STATUS)
        if self._target_reached:
            if nav_status not in {None, "succeeded", "cancelled"} and not self._cancel_requested:
                self._event_bus.publish(DomainEvent(EventType.NAVIGATION_CANCEL_REQUEST, None))
                self._cancel_requested = True
            return Status.SUCCESS

        if nav_status == "succeeded":
            return Status.SUCCESS
        if nav_status == "cancelled":
            return Status.SUCCESS if self._cancel_requested else Status.FAILURE
        if nav_status in {"aborted", "rejected"}:
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self._target_reached = False
        self._cancel_requested = False

    def _on_target_reached(self, event: DomainEvent) -> None:      
        self._target_reached = True
