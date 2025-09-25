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
        self._event_bus.subscribe(EventType.DRIVE_GOAL_SET, self._on_drive_goal_set)
        self._event_bus.subscribe(EventType.DRIVE_PROGRESS_UPDATED, self._on_drive_progress_updated)
        self._event_bus.subscribe(EventType.DRIVE_GOAL_CLEARED, self._on_drive_goal_cleared)
        self._event_bus.subscribe(EventType.ROTATE_GOAL_SET, self._on_rotate_goal_set)
        self._event_bus.subscribe(EventType.ROTATE_PROGRESS_UPDATED, self._on_rotate_progress_updated)
        self._event_bus.subscribe(EventType.ROTATE_GOAL_CLEARED, self._on_rotate_goal_cleared)

        self._set(BlackboardDataKey.TARGET_OBJECT_CLASS, 'chair')
        self._set(BlackboardDataKey.COMMAND_QUEUE, [])
        self._set(BlackboardDataKey.ACTIVE_COMMAND, None)
        self._set(BlackboardDataKey.DRIVE_TARGET_DISTANCE, None)
        self._set(BlackboardDataKey.DRIVE_START_POSE, None)
        self._set(BlackboardDataKey.DRIVE_DISTANCE_TRAVELLED, 0.0)
        self._set(BlackboardDataKey.DRIVE_DIRECTION_SIGN, None)
        self._set(BlackboardDataKey.ROTATE_TARGET_ANGLE, None)
        self._set(BlackboardDataKey.ROTATE_START_YAW, None)
        self._set(BlackboardDataKey.ROTATE_ANGLE_TRAVELLED, 0.0)
        self._set(BlackboardDataKey.ROTATE_DIRECTION_SIGN, None)

        self._seed_rectangle_commands()

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

    def set_active_command(self, command: Optional[UserCommand]) -> None:
        """Update the currently executing command."""
        self._set(BlackboardDataKey.ACTIVE_COMMAND, command)

    def set_drive_goal(self, *, target_distance: float, start_pose: dict, direction_sign: int) -> None:
        self._set(BlackboardDataKey.DRIVE_TARGET_DISTANCE, target_distance)
        self._set(BlackboardDataKey.DRIVE_START_POSE, start_pose)
        self._set(BlackboardDataKey.DRIVE_DIRECTION_SIGN, direction_sign)
        self._set(BlackboardDataKey.DRIVE_DISTANCE_TRAVELLED, 0.0)

    def update_drive_distance(self, distance: float) -> None:
        self._set(BlackboardDataKey.DRIVE_DISTANCE_TRAVELLED, distance)

    def clear_drive_goal(self) -> None:
        self._set(BlackboardDataKey.DRIVE_TARGET_DISTANCE, None)
        self._set(BlackboardDataKey.DRIVE_START_POSE, None)
        self._set(BlackboardDataKey.DRIVE_DIRECTION_SIGN, None)
        self._set(BlackboardDataKey.DRIVE_DISTANCE_TRAVELLED, 0.0)

    def set_rotate_goal(self, *, target_angle: float, start_yaw: float, direction_sign: int) -> None:
        self._set(BlackboardDataKey.ROTATE_TARGET_ANGLE, target_angle)
        self._set(BlackboardDataKey.ROTATE_START_YAW, start_yaw)
        self._set(BlackboardDataKey.ROTATE_DIRECTION_SIGN, direction_sign)
        self._set(BlackboardDataKey.ROTATE_ANGLE_TRAVELLED, 0.0)

    def update_rotate_angle(self, angle: float) -> None:
        self._set(BlackboardDataKey.ROTATE_ANGLE_TRAVELLED, angle)

    def clear_rotate_goal(self) -> None:
        self._set(BlackboardDataKey.ROTATE_TARGET_ANGLE, None)
        self._set(BlackboardDataKey.ROTATE_START_YAW, None)
        self._set(BlackboardDataKey.ROTATE_DIRECTION_SIGN, None)
        self._set(BlackboardDataKey.ROTATE_ANGLE_TRAVELLED, 0.0)

    def _sync_command_queue(self) -> None:
        """Expose a snapshot of the command queue via the blackboard data store."""
        self._set(BlackboardDataKey.COMMAND_QUEUE, list(self._command_queue))

    def _remove_command_by_id(self, command_id: str) -> Optional[UserCommand]:
        """Remove a queued command by identifier and return it if present."""
        if not self._command_queue:
            return None

        for command in list(self._command_queue):
            if command.command_id == command_id:
                self._command_queue.remove(command)
                self._sync_command_queue()
                return command
        return None

    def _seed_rectangle_commands(self) -> None:
        """Seed the queue with commands to drive a 1m x 1m rectangle."""
        sequence = [
            UserCommand.drive(distance_m=1.0, direction="forward"),
            UserCommand.rotate(angle_deg=90.0, direction="left"),
            UserCommand.drive(distance_m=1.0, direction="forward"),
            UserCommand.rotate(angle_deg=90.0, direction="left"),
            UserCommand.drive(distance_m=1.0, direction="forward"),
            UserCommand.rotate(angle_deg=90.0, direction="left"),
            UserCommand.drive(distance_m=1.0, direction="forward"),
            UserCommand.rotate(angle_deg=90.0, direction="left"),
        ]

        for command in sequence:
            self.enqueue_command(command)
    
    ### Event Handlers ###

    def _on_command_started(self, event: DomainEvent):
        # data is of type UserCommand
        command = event.data
        if not isinstance(command, UserCommand):
            return

        # Remove from pending queue if it was still there and mark active
        self._remove_command_by_id(command.command_id)
        self.set_active_command(command)

    def _on_command_completed(self, event: DomainEvent):
        # data is of type UserCommand
        active_command: Optional[UserCommand] = self.get(BlackboardDataKey.ACTIVE_COMMAND)
        command = event.data

        if not isinstance(command, UserCommand):
            return

        if active_command and active_command.command_id == command.command_id:
            command.cleanup()
            self.set_active_command(None)

    def _on_command_cancelled(self, event: DomainEvent):
        # data is of type UserCommand
        active_command: Optional[UserCommand] = self.get(BlackboardDataKey.ACTIVE_COMMAND)
        command = event.data

        if not isinstance(command, UserCommand):
            return

        self._remove_command_by_id(command.command_id)
        if active_command and active_command.command_id == command.command_id:
            self.set_active_command(None)
        command.cleanup()

    def _on_command_received(self, event: DomainEvent):
        command = event.data
        if not isinstance(command, UserCommand):
            return
        self.enqueue_command(command)

    def _on_drive_goal_set(self, event: DomainEvent) -> None:
        # data is of type Dict[str, Any] with keys target_distance, start_pose, direction_sign
        data = event.data or {}
        target_distance = data.get("target_distance")
        start_pose = data.get("start_pose")
        direction_sign = data.get("direction_sign")

        if target_distance is None or start_pose is None or direction_sign is None:
            return

        self.set_drive_goal(
            target_distance=float(target_distance),
            start_pose=start_pose,
            direction_sign=int(direction_sign),
        )

    def _on_drive_progress_updated(self, event: DomainEvent) -> None:
        # data is of type float (distance in meters)
        distance = event.data
        if distance is None:
            return
        self.update_drive_distance(float(distance))

    def _on_drive_goal_cleared(self, event: DomainEvent) -> None:
        # data is None
        self.clear_drive_goal()

    def _on_rotate_goal_set(self, event: DomainEvent) -> None:
        # data is of type Dict[str, Any] with keys target_angle, start_yaw, direction_sign
        data = event.data or {}
        target_angle = data.get("target_angle")
        start_yaw = data.get("start_yaw")
        direction_sign = data.get("direction_sign")

        if target_angle is None or start_yaw is None or direction_sign is None:
            return

        self.set_rotate_goal(
            target_angle=float(target_angle),
            start_yaw=float(start_yaw),
            direction_sign=int(direction_sign),
        )

    def _on_rotate_progress_updated(self, event: DomainEvent) -> None:
        # data is of type float (angle in radians)
        angle = event.data
        if angle is None:
            return
        self.update_rotate_angle(float(angle))

    def _on_rotate_goal_cleared(self, event: DomainEvent) -> None:
        # data is None
        self.clear_rotate_goal()

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
