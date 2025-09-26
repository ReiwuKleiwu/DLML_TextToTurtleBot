# Singleton Blackboard for sharing data across the behavior tree
from collections import deque
from typing import Deque, Optional
from threading import RLock, Timer

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
        self._command_lock = RLock()

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
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_SENT, self._on_navigation_goal_sent)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_ACCEPTED, self._on_navigation_goal_accepted)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_REJECTED, self._on_navigation_goal_rejected)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_SUCCEEDED, self._on_navigation_goal_succeeded)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_ABORTED, self._on_navigation_goal_aborted)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_CANCELLED, self._on_navigation_goal_cancelled)
        self._event_bus.subscribe(EventType.NAVIGATION_FEEDBACK, self._on_navigation_feedback)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_CLEARED, self._on_navigation_goal_cleared)
        self._event_bus.subscribe(EventType.TARGET_OBJECT_CLASS_SET, self._on_target_object_class_set)
        self._event_bus.subscribe(EventType.CAMERA_RESOLUTION_SET, self._on_camera_resolution_set)

        self._set(BlackboardDataKey.TARGET_OBJECT_CLASS, None)
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
        self._set(BlackboardDataKey.NAVIGATION_CURRENT_GOAL, None)
        self._set(BlackboardDataKey.NAVIGATION_STATUS, None)
        self._set(BlackboardDataKey.NAVIGATION_FEEDBACK, None)

        self._seed_test_commands()

    # The blackboard is readonly from the outside!
    def _set(self, key, value):
        self.data[key] = value

    def get(self, key, default=None):
        return self.data.get(key, default)
    
    # Command handling

    def _seed_test_commands(self) -> None:

        # nav_command = UserCommand.navigate(1.0, 1.0)
        # rotate_command = UserCommand.rotate(90, 'left')
        # drive_command = UserCommand.drive(1.0, 'forward')
        # nav_command_2 = UserCommand.navigate(10.0, -4.0)
        find_chair = UserCommand.find_object('chair')
        # navigate_to_point = UserCommand.navigate(1.0, 1.0)
        # find_person = UserCommand.find_object('person')

        nav_1 = UserCommand.navigate(1.0, 1.0)
        nav_2 = UserCommand.navigate(1.5, 1.0)
        nav_3 = UserCommand.navigate(2.0, 1.0)
        nav_4 = UserCommand.navigate(2.5, 1.0)
        nav_5 = UserCommand.navigate(3.0, 1.0)
        nav_6 = UserCommand.navigate(3.5, 1.0)

        commands = [nav_1, nav_2, nav_3, nav_4, nav_5, nav_6, find_chair]

        for command in commands:
            self.enqueue_command(command)

        # def cancel_pending_find() -> None:
        #    self.cancel_active_command()

        # Timer(20.0, cancel_pending_find).start()


    def enqueue_command(self, command: UserCommand) -> None:
        """Push a user command to the queue and mirror the state on the blackboard."""
        with self._command_lock:
            self._command_queue.append(command)
            self._sync_command_queue()

    def pop_command(self) -> Optional[UserCommand]:
        """Remove and return the next command, updating the published queue state."""
        with self._command_lock:
            if not self._command_queue:
                return None
            command = self._command_queue.popleft()
            self._sync_command_queue()
            return command

    def peek_command(self) -> Optional[UserCommand]:
        """Return, without removing, the next command in the queue."""
        with self._command_lock:
            return self._command_queue[0] if self._command_queue else None

    def clear_commands(self) -> None:
        """Remove all pending commands and cancel any active one."""
        # Drop any queued commands and mirror the empty queue on the blackboard.
        with self._command_lock:
            self._command_queue.clear()
            self._sync_command_queue()

        # Cancel the active command (if any) to release related state.
        self.cancel_active_command()

    def cancel_active_command(self) -> Optional[UserCommand]:
        """Cancel the currently active command, removing it from the queue if present."""
        with self._command_lock:
            active_command: Optional[UserCommand] = self.get(BlackboardDataKey.ACTIVE_COMMAND)

            if not isinstance(active_command, UserCommand):
                return None

            print(f"Cancelling active command: {active_command.command_id}")

            # This should theoretically never happen, but just in case
            self._remove_command_by_id(active_command.command_id)

            self.set_active_command(None)

        active_command.cleanup()
        return active_command

    def set_active_command(self, command: Optional[UserCommand]) -> None:
        """Update the currently executing command."""
        with self._command_lock:
            self._set(BlackboardDataKey.ACTIVE_COMMAND, command)

    def _sync_command_queue(self) -> None:
        """Expose a snapshot of the command queue via the blackboard data store."""
        with self._command_lock:
            self._set(BlackboardDataKey.COMMAND_QUEUE, list(self._command_queue))

    def _remove_command_by_id(self, command_id: str) -> Optional[UserCommand]:
        """Remove a queued command by identifier and return it if present."""
        with self._command_lock:
            if not self._command_queue:
                return None

            for command in list(self._command_queue):
                if command.command_id == command_id:
                    self._command_queue.remove(command)
                    self._sync_command_queue()
                    return command
        return None
    
    ### Event Handlers ###

    def _on_command_started(self, event: DomainEvent):
        # data is of type UserCommand
        command = event.data
        if not isinstance(command, UserCommand):
            return

        print(f"Command started: {command.command_id}")

        # Remove from pending queue if it was still there and mark active
        self._remove_command_by_id(command.command_id)
        self.set_active_command(command)

    def _on_command_completed(self, event: DomainEvent):
        # data is of type UserCommand
        active_command: Optional[UserCommand] = self.get(BlackboardDataKey.ACTIVE_COMMAND)
        command = event.data

        if not isinstance(command, UserCommand):
            return
        
        print(f"Command completed: {command.command_id}")

        if active_command and active_command.command_id == command.command_id:
            command.cleanup()
            self.set_active_command(None)

    def _on_command_cancelled(self, event: DomainEvent):
        # data is of type UserCommand
        active_command: Optional[UserCommand] = self.get(BlackboardDataKey.ACTIVE_COMMAND)
        command = event.data

        if not isinstance(command, UserCommand):
            return

        # This should theoretically never happen, but just in case
        if active_command and active_command.command_id == command.command_id:
            self.cancel_active_command()
        else:
            self._remove_command_by_id(command.command_id)
            command.cleanup()

    def _on_command_received(self, event: DomainEvent):
        # data is of type UserCommand
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

        self._set(BlackboardDataKey.DRIVE_TARGET_DISTANCE, target_distance)
        self._set(BlackboardDataKey.DRIVE_START_POSE, start_pose)
        self._set(BlackboardDataKey.DRIVE_DIRECTION_SIGN, direction_sign)
        self._set(BlackboardDataKey.DRIVE_DISTANCE_TRAVELLED, 0.0)

    def _on_drive_progress_updated(self, event: DomainEvent) -> None:
        # data is of type float (distance in meters)
        distance = event.data
        if distance is None:
            return
        self._set(BlackboardDataKey.DRIVE_DISTANCE_TRAVELLED, distance)

    def _on_drive_goal_cleared(self, event: DomainEvent) -> None:
        # data is None
        self._set(BlackboardDataKey.DRIVE_TARGET_DISTANCE, None)
        self._set(BlackboardDataKey.DRIVE_START_POSE, None)
        self._set(BlackboardDataKey.DRIVE_DIRECTION_SIGN, None)
        self._set(BlackboardDataKey.DRIVE_DISTANCE_TRAVELLED, 0.0)

    def _on_rotate_goal_set(self, event: DomainEvent) -> None:
        # data is of type Dict[str, Any] with keys target_angle, start_yaw, direction_sign
        data = event.data or {}
        target_angle = data.get("target_angle")
        start_yaw = data.get("start_yaw")
        direction_sign = data.get("direction_sign")

        if target_angle is None or start_yaw is None or direction_sign is None:
            return

        self._set(BlackboardDataKey.ROTATE_TARGET_ANGLE, target_angle)
        self._set(BlackboardDataKey.ROTATE_START_YAW, start_yaw)
        self._set(BlackboardDataKey.ROTATE_DIRECTION_SIGN, direction_sign)
        self._set(BlackboardDataKey.ROTATE_ANGLE_TRAVELLED, 0.0)

    def _on_rotate_progress_updated(self, event: DomainEvent) -> None:
        # data is of type float (angle in radians)
        angle = event.data
        if angle is None:
            return
        self._set(BlackboardDataKey.ROTATE_ANGLE_TRAVELLED, angle)

    def _on_rotate_goal_cleared(self, event: DomainEvent) -> None:
        # data is None
        self._set(BlackboardDataKey.ROTATE_TARGET_ANGLE, None)
        self._set(BlackboardDataKey.ROTATE_START_YAW, None)
        self._set(BlackboardDataKey.ROTATE_DIRECTION_SIGN, None)
        self._set(BlackboardDataKey.ROTATE_ANGLE_TRAVELLED, 0.0)

    def _on_navigation_goal_sent(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.PoseStamped
        goal = event.data
        self._set(BlackboardDataKey.NAVIGATION_CURRENT_GOAL, goal)
        self._set(BlackboardDataKey.NAVIGATION_STATUS, "sent")

    def _on_navigation_goal_accepted(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.PoseStamped
        goal = event.data
        self._set(BlackboardDataKey.NAVIGATION_CURRENT_GOAL, goal)
        self._set(BlackboardDataKey.NAVIGATION_STATUS, "accepted")

    def _on_navigation_goal_rejected(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.PoseStamped or None
        self._set(BlackboardDataKey.NAVIGATION_STATUS, "rejected")

    def _on_navigation_goal_succeeded(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.PoseStamped or None
        goal = event.data
        if goal is not None:
            self._set(BlackboardDataKey.NAVIGATION_CURRENT_GOAL, goal)
        self._set(BlackboardDataKey.NAVIGATION_STATUS, "succeeded")

    def _on_navigation_goal_aborted(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.PoseStamped or None
        self._set(BlackboardDataKey.NAVIGATION_STATUS, "aborted")

    def _on_navigation_goal_cancelled(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.PoseStamped or None
        self._set(BlackboardDataKey.NAVIGATION_STATUS, "cancelled")

    def _on_navigation_feedback(self, event: DomainEvent):
        # data is of type nav2_msgs.action.NavigateToPose.Feedback
        self._set(BlackboardDataKey.NAVIGATION_FEEDBACK, event.data)

    def _on_navigation_goal_cleared(self, event: DomainEvent):
        # data is None
        self._set(BlackboardDataKey.NAVIGATION_CURRENT_GOAL, None)
        self._set(BlackboardDataKey.NAVIGATION_STATUS, None)
        self._set(BlackboardDataKey.NAVIGATION_FEEDBACK, None)

    def _on_target_object_class_set(self, event: DomainEvent):
        # data is of type str or None
        self._set(BlackboardDataKey.TARGET_OBJECT_CLASS, event.data)

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
        self._event_bus.publish(DomainEvent(EventType.NAVIGATION_CANCEL_REQUEST, None))

    def _on_camera_resolution_set(self, event: DomainEvent):
        # data is of type Dict with keys 'width' and 'height'
        self._set(BlackboardDataKey.CAMERA_RESOLUTION, event.data)
