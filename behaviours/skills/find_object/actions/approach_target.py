import py_trees
from py_trees.common import Status

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType
from perception.detection.object_detector import DetectedObject
from utils.twist_wrapper import TwistWrapper


class ApproachTarget(py_trees.behaviour.Behaviour):
    """Drives forward while waiting for the target reached detector to trigger."""

    def __init__(self, name: str, twist: TwistWrapper, publisher) -> None:
        super().__init__(name)
        self._twist = twist
        self._publisher = publisher
        self._blackboard = Blackboard()
        self._event_bus = EventBus()
        self._target_reached = False
        self._event_bus.subscribe(EventType.TARGET_REACHED, self._on_target_reached)

    def initialise(self) -> None:
        self._target_reached = False

    def update(self) -> Status:
        print("Approaching")
        if self._target_reached:
            self._stop()
            return Status.SUCCESS
        
        target_object_class = self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS)
        selected_object: DetectedObject = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)

        if not selected_object or selected_object.name.lower() != target_object_class:
            self._stop()
            return Status.FAILURE

        self._twist.reset()
        self._twist.linear.x = 1.0
        self._publisher.publish(self._twist.get_message())
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self._target_reached = False
        self._stop()

    def _on_target_reached(self, event: DomainEvent) -> None:     
        self._target_reached = True

    def _stop(self) -> None:
        self._twist.reset()
        self._publisher.publish(self._twist.get_message())
