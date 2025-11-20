import py_trees
from py_trees.common import Status

from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType


class SetTargetClass(py_trees.behaviour.Behaviour):
    """Publishes the desired target object class via the event bus."""

    def __init__(self, name: str, target_class: str) -> None:
        super().__init__(name)
        self._target_class = target_class.lower()
        self._event_bus = EventBus()

    def update(self) -> Status:
        if not self._target_class:
            self.logger.error("No target object class provided")
            return Status.FAILURE

        self._event_bus.publish(DomainEvent(EventType.TARGET_OBJECT_CLASS_SET, self._target_class))
        return Status.SUCCESS
