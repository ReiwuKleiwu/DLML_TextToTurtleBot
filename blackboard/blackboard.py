# Singleton Blackboard for sharing data across the behavior tree
from utils.singleton_meta import SingletonMeta
from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey

# The blackboard acts as a central hub for all the information the robot perceives
class Blackboard(metaclass=SingletonMeta):
    def __init__(self):
        self._event_bus = EventBus()
        self.data = {}

        self._event_bus.subscribe(EventType.LIDAR_OBSTACLE_PRESENT, self._on_lidar_obstacle_present)
        self._event_bus.subscribe(EventType.LIDAR_OBSTACLE_ABSENT, self._on_lidar_obstacle_absent)
        self._event_bus.subscribe(EventType.TARGET_OBJECT_SELECTED, self._on_target_object_selected)
        self._event_bus.subscribe(EventType.OBJECTS_DETECTED, self._on_objects_detected)
        self._event_bus.subscribe(EventType.OBJECT_WORLD_COORDINATES_UPDATED, self._on_object_world_coordinates_updated)
        self._event_bus.subscribe(EventType.ROBOT_POSITION_UPDATED, self._on_robot_position_updated)
        self._event_bus.subscribe(EventType.ROBOT_ORIENTATION_UPDATED, self._on_robot_orientation_updated)
        self._event_bus.subscribe(EventType.MAP_UPDATED, self._on_map_updated)
        self._event_bus.subscribe(EventType.TARGET_REACHED, self._on_target_reached)

        self._set(BlackboardDataKey.TARGET_OBJECT_CLASS, 'chair')

    # The blackboard is readonly from the outside! 
    def _set(self, key, value):
        self.data[key] = value

    def get(self, key, default=None):
        return self.data.get(key, default)
    
    ### Event Handlers ###
    
    def _on_lidar_obstacle_present(self, event: DomainEvent):
        self._set(BlackboardDataKey.LIDAR_OBSTACLE_PRESENT, True)

    def _on_lidar_obstacle_absent(self, event: DomainEvent):
        self._set(BlackboardDataKey.LIDAR_OBSTACLE_PRESENT, False)

    def _on_target_object_selected(self, event: DomainEvent):
        # data is of type DetectedObject in this case
        self._set(BlackboardDataKey.SELECTED_TARGET_OBJECT, event.data)

    def _on_objects_detected(self, event: DomainEvent):
        # data is of type Dict[str, List[DetectedObject]] in this case
        self._set(BlackboardDataKey.DETECTED_OBJECTS, event.data)

    def _on_object_world_coordinates_updated(self, event: DomainEvent):
        # data is of type Dict[str, List[DetectedObject]] in this case
        self._set(BlackboardDataKey.DETECTED_OBJECTS_WITH_COORDINATES, event.data)

    def _on_robot_position_updated(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.Vector3
        self._set(BlackboardDataKey.ROBOT_POSITION, event.data)

    def _on_robot_orientation_updated(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.Quaternion
        self._set(BlackboardDataKey.ROBOT_ORIENTATION, event.data)

    def _on_robot_is_turning_updated(self, event: DomainEvent):
        # data is of type bool
        self._set(BlackboardDataKey.ROBOT_IS_TURNING, event.data)

    def _on_map_updated(self, event: DomainEvent):
        # data is of type List[PersistentTrackedObject]
        self._set(BlackboardDataKey.ROBOT_MAP, event.data)

    def _on_target_reached(self, event: DomainEvent):
        # data is None
        self._set(BlackboardDataKey.SELECTED_TARGET_OBJECT, None)
        self._set(BlackboardDataKey.TARGET_OBJECT_CLASS, None)
