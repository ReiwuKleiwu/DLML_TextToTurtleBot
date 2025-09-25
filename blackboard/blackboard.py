# Singleton Blackboard for sharing data across the behavior tree
from collections import deque
from typing import Deque, Optional

from utils.singleton_meta import SingletonMeta
from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from commands.user_command import UserCommand


# The blackboard acts as a central hub for all the information the robot perceives
class Blackboard(metaclass=SingletonMeta):
    def __init__(self):
        self._event_bus = EventBus()
        self.data = {}

        # Internal command queue storage (FIFO)
        self._command_queue: Deque[UserCommand] = deque()

        self._event_bus.subscribe(EventType.LIDAR_OBSTACLE_PRESENT, self._on_lidar_obstacle_present)
        self._event_bus.subscribe(EventType.LIDAR_OBSTACLE_ABSENT, self._on_lidar_obstacle_absent)
        self._event_bus.subscribe(EventType.TARGET_OBJECT_SELECTED, self._on_target_object_selected)
        self._event_bus.subscribe(EventType.OBJECTS_DETECTED, self._on_objects_detected)
        self._event_bus.subscribe(EventType.OBJECT_WORLD_COORDINATES_UPDATED, self._on_object_world_coordinates_updated)
        self._event_bus.subscribe(EventType.ROBOT_POSITION_UPDATED, self._on_robot_position_updated)
        self._event_bus.subscribe(EventType.ROBOT_ORIENTATION_UPDATED, self._on_robot_orientation_updated)
        self._event_bus.subscribe(EventType.MAP_UPDATED, self._on_map_updated)
        self._event_bus.subscribe(EventType.TARGET_REACHED, self._on_target_reached)
        self._event_bus.subscribe(EventType.COMMAND_RECEIVED, self._on_command_received)
        self._event_bus.subscribe(EventType.COMMAND_STARTED, self._on_command_started)
        self._event_bus.subscribe(EventType.COMMAND_COMPLETED, self._on_command_completed)
        self._event_bus.subscribe(EventType.COMMAND_CANCELLED, self._on_command_cancelled)

        self._set(BlackboardDataKey.TARGET_OBJECT_CLASS, 'chair')
        self._set(BlackboardDataKey.COMMAND_QUEUE, [])
        self._set(BlackboardDataKey.ACTIVE_COMMAND_ID, None)

    # The blackboard is readonly from the outside!
    def _set(self, key, value):
        self.data[key] = value

    def get(self, key, default=None):
        return self.data.get(key, default)
    
    # Command handling

    def enqueue_command(self, command: UserCommand) -> None:
        """Push a user command to the queue and mirror the state on the blackboard."""
        self._command_queue.append(command)
        self._sync_command_queue()

    def pop_command(self) -> Optional[UserCommand]:
        """Remove and return the next command, updating the published queue state."""
        if not self._command_queue:
            return None
        command = self._command_queue.popleft()
        self._sync_command_queue()
        return command

    def peek_command(self) -> Optional[UserCommand]:
        """Return, without removing, the next command in the queue."""
        return self._command_queue[0] if self._command_queue else None

    def clear_commands(self) -> None:
        """Remove all pending commands."""
        self._command_queue.clear()
        self._sync_command_queue()

    def set_active_command(self, command_id: Optional[str]) -> None:
        """Update the currently executing command identifier."""
        self._set(BlackboardDataKey.ACTIVE_COMMAND_ID, command_id)

    def _sync_command_queue(self) -> None:
        """Expose a snapshot of the command queue via the blackboard data store."""
        self._set(BlackboardDataKey.COMMAND_QUEUE, list(self._command_queue))

    def _remove_command_by_id(self, command_id: str) -> bool:
        """Remove a queued command by identifier; returns True if found."""
        original_length = len(self._command_queue)
        if original_length == 0:
            return False

        self._command_queue = deque(
            command for command in self._command_queue if command.command_id != command_id
        )
        removed = len(self._command_queue) != original_length
        if removed:
            self._sync_command_queue()
        return removed

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

    def _on_command_received(self, event: DomainEvent):
        command = event.data
        if not isinstance(command, UserCommand):
            return
        self.enqueue_command(command)

    def _on_command_started(self, event: DomainEvent):
        command_id = event.data
        if isinstance(command_id, str):
            # Ensure the command is no longer in the pending queue
            self._remove_command_by_id(command_id)
            self.set_active_command(command_id)

    def _on_command_completed(self, event: DomainEvent):
        command_id = event.data
        if isinstance(command_id, str) and self.get(BlackboardDataKey.ACTIVE_COMMAND_ID) == command_id:
            self.set_active_command(None)

    def _on_command_cancelled(self, event: DomainEvent):
        command_id = event.data
        if not isinstance(command_id, str):
            return

        removed = self._remove_command_by_id(command_id)
        if removed and self.get(BlackboardDataKey.ACTIVE_COMMAND_ID) == command_id:
            self.set_active_command(None)
