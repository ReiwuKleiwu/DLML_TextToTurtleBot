import py_trees
from py_trees.common import Status

from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType
from core.perception.detection.object_detector import DetectedObject


class WaitForTargetReached(py_trees.behaviour.Behaviour):
    """Waits for the target reached event while ensuring the target stays selected."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._blackboard = Blackboard()
        self._event_bus = EventBus()
        self._target_reached = False
        self._event_bus.subscribe(EventType.TARGET_REACHED, self._on_target_reached)

    def initialise(self) -> None:
        self._target_reached = False

    def update(self) -> Status:
        if self._target_reached:
            return Status.SUCCESS

        target_object_class = self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS)
        selected: DetectedObject = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)
        if not selected or selected.name.lower() != target_object_class:
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self._target_reached = False

    def _on_target_reached(self, event: DomainEvent) -> None:
        if self.status == Status.INVALID:
            return
       
        self._target_reached = True
