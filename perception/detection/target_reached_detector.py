from perception.detection.object_detector import DetectedObject
from rclpy.node import Node

from events.event_bus import EventBus
from events.interfaces.events import EventType, DomainEvent
from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey

class TargetReachedDetector:
    def __init__(self, node: Node, target_reached_threshold: float = 1.5) -> None:
        self._event_bus = EventBus()
        self._blackboard = Blackboard()

        # Distance from the target object where it counts as "reached"
        self._target_object_reached_threshold = target_reached_threshold # meters
        self._timer = node.create_timer(0.1, self._check_target_reached)

    def _check_target_reached(self) -> None:
        target_object_class: str = self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS)
        selected_target_object: DetectedObject = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)
        
        if not selected_target_object or not target_object_class:
            return
        
        robot_position = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)

        if not robot_position:
            return
        
        target_object_distance_from_robot = selected_target_object.distance_to_world_coordinates(robot_position.x, robot_position.y, robot_position.z)
        
        if target_object_distance_from_robot <= self._target_object_reached_threshold:
            self._event_bus.publish(DomainEvent(EventType.TARGET_REACHED, None))
        
