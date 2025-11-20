# Singleton Blackboard for sharing data across the behavior tree
from collections import deque
from typing import Any, Deque, Dict, List, Optional
from threading import RLock
import math
import time

from shared.utils.singleton_meta import SingletonMeta
from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from backend.commands.user_command import UserCommand
from std_msgs.msg import String
import pickle
import base64

# The blackboard acts as a central hub for all the information the robot perceives
class Blackboard(metaclass=SingletonMeta):
    def __init__(self):
        self._event_bus = EventBus()
        self.data = {}

        # Internal command queue storage (FIFO)
        self._command_queue: Deque[UserCommand] = deque()
        self._lock = RLock()

        self._event_bus.subscribe(EventType.LIDAR_OBSTACLE_PRESENT, self._on_lidar_obstacle_present)
        self._event_bus.subscribe(EventType.LIDAR_OBSTACLE_ABSENT, self._on_lidar_obstacle_absent)
        self._event_bus.subscribe(EventType.LIDAR_POINTS_UPDATED, self._on_lidar_points_updated)
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
        self._set(BlackboardDataKey.BEHAVIOUR_TREE_PAUSED, False)
        self._set(BlackboardDataKey.ROBOT_TRAIL, [])
        self._set(BlackboardDataKey.LLM_CHAT_LOG, [])
        self._set(BlackboardDataKey.LIDAR_POINTS, {"points": []})
        self._set(BlackboardDataKey.LLM_CAPABILITIES, {})
        self._set(BlackboardDataKey.LATEST_CAMERA_FRAME, None)

        self._robot_trail_max_length = 200
        self._robot_trail_min_distance = 0.05
        self._chat_log_max_entries = 200

        # self._seed_test_commands()

    # The blackboard is readonly from the outside!
    def _set(self, key, value):
        self.data[key] = value

    def get(self, key, default=None):
        return self.data.get(key, default)

    def set_llm_capabilities(self, capabilities: Dict[str, Any]) -> None:
        """Expose the active LLM capability metadata to other subsystems."""
        snapshot = dict(capabilities or {})
        with self._lock:
            self._set(BlackboardDataKey.LLM_CAPABILITIES, snapshot)

    def store_camera_frame(self, frame: Any, *, timestamp: Optional[float] = None) -> None:
        """Cache the most recent RGB camera frame for downstream consumers."""
        if frame is None:
            return

        payload = {
            "image": frame,
            "timestamp": timestamp if timestamp is not None else time.time(),
        }

        with self._lock:
            self._set(BlackboardDataKey.LATEST_CAMERA_FRAME, payload)

    def get_latest_camera_frame(self) -> Optional[Dict[str, Any]]:
        """Return the most recent cached RGB frame metadata."""
        with self._lock:
            frame = self.data.get(BlackboardDataKey.LATEST_CAMERA_FRAME)

        if frame is None or not isinstance(frame, dict):
            return None

        return frame.copy()

    # Behaviour tree control

    def pause_behaviour_tree(self) -> None:
        """Flag the behaviour tree loop as paused."""
        with self._lock:
            self._set(BlackboardDataKey.BEHAVIOUR_TREE_PAUSED, True)

    def resume_behaviour_tree(self) -> None:
        """Flag the behaviour tree loop as running."""
        with self._lock:
            self._set(BlackboardDataKey.BEHAVIOUR_TREE_PAUSED, False)

    def is_behaviour_tree_paused(self) -> bool:
        """Return whether the behaviour tree should halt ticking."""
        with self._lock:
            return bool(self.get(BlackboardDataKey.BEHAVIOUR_TREE_PAUSED, False))

    # Command handling

    def _seed_test_commands(self) -> None:
        pass


    def enqueue_command(self, command: UserCommand) -> None:
        """Push a user command to the queue and mirror the state on the blackboard."""
        with self._lock:
            self._command_queue.append(command)
            self._sync_command_queue()

    def pop_command(self) -> Optional[UserCommand]:
        """Remove and return the next command, updating the published queue state."""
        with self._lock:
            if not self._command_queue:
                return None
            command = self._command_queue.popleft()
            self._sync_command_queue()
            return command

    def peek_command(self) -> Optional[UserCommand]:
        """Return, without removing, the next command in the queue."""
        with self._lock:
            return self._command_queue[0] if self._command_queue else None

    def clear_commands(self) -> None:
        """Remove all pending commands and cancel any active one."""
        # Drop any queued commands and mirror the empty queue on the blackboard.
        with self._lock:
            self._command_queue.clear()
            self._sync_command_queue()

        # Cancel the active command (if any) to release related state.
        self.cancel_active_command()

    def cancel_active_command(self) -> Optional[UserCommand]:
        """Cancel the currently active command, removing it from the queue if present."""
        with self._lock:
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
        with self._lock:
            self._set(BlackboardDataKey.ACTIVE_COMMAND, command)

    def _sync_command_queue(self) -> None:
        """Expose a snapshot of the command queue via the blackboard data store."""
        with self._lock:
            self._set(BlackboardDataKey.COMMAND_QUEUE, list(self._command_queue))

    def _remove_command_by_id(self, command_id: str) -> Optional[UserCommand]:
        """Remove a queued command by identifier and return it if present."""
        with self._lock:
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

    def _on_lidar_points_updated(self, event: DomainEvent) -> None:
        data = event.data or {}
        raw_points = data.get("points")

        sanitized_points: List[Dict[str, float]] = []
        if isinstance(raw_points, list):
            for point in raw_points:
                if not isinstance(point, dict):
                    continue

                x = point.get("x")
                y = point.get("y")
                if x is None or y is None:
                    continue

                try:
                    x_value = float(x)
                    y_value = float(y)
                except (TypeError, ValueError):
                    continue

                sanitized: Dict[str, float] = {"x": x_value, "y": y_value}

                distance = point.get("distance")
                if distance is not None:
                    try:
                        distance_value = float(distance)
                    except (TypeError, ValueError):
                        distance_value = None
                    if distance_value is not None and math.isfinite(distance_value):
                        sanitized["distance"] = distance_value

                sanitized_points.append(sanitized)

        timestamp = data.get("timestamp")
        timestamp_value = None
        if isinstance(timestamp, (int, float)) and math.isfinite(float(timestamp)):
            timestamp_value = float(timestamp)

        payload: Dict[str, Any] = {
            "points": sanitized_points,
            "frame_id": data.get("frame_id"),
        }
        if timestamp_value is not None:
            payload["timestamp"] = timestamp_value

        self._set(BlackboardDataKey.LIDAR_POINTS, payload)

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
        translation = event.data
        with self._lock:
            self._set(BlackboardDataKey.ROBOT_POSITION, translation)
            self._update_robot_trail(translation)

    def _on_robot_orientation_updated(self, event: DomainEvent):
        # data is of type geometry_msgs.msg.Quaternion
        self._set(BlackboardDataKey.ROBOT_ORIENTATION, event.data)

    def _update_robot_trail(self, translation: Any) -> None:
        """Append the latest robot position to the bounded trail."""
        if translation is None:
            return

        try:
            point = {
                "x": float(getattr(translation, "x", 0.0)),
                "y": float(getattr(translation, "y", 0.0)),
                "z": float(getattr(translation, "z", 0.0)),
                "timestamp": time.time(),
            }
        except (TypeError, ValueError):
            return

        trail = list(self.data.get(BlackboardDataKey.ROBOT_TRAIL, []))
        if trail:
            last = trail[-1]
            dx = point["x"] - last.get("x", 0.0)
            dy = point["y"] - last.get("y", 0.0)
            dz = point["z"] - last.get("z", 0.0)
            if (dx * dx + dy * dy + dz * dz) < (self._robot_trail_min_distance ** 2):
                trail[-1] = point
            else:
                trail.append(point)
        else:
            trail.append(point)

        if len(trail) > self._robot_trail_max_length:
            trail = trail[-self._robot_trail_max_length:]

        self._set(BlackboardDataKey.ROBOT_TRAIL, trail)

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

    # Snapshot helpers for LLM tools

    def append_chat_message(self, role: str, text: str, metadata: Optional[Dict[str, Any]] = None) -> None:
        """Record a conversational exchange for the mission board chat window."""
        if not text:
            return

        entry = {
            "role": role or "unknown",
            "text": str(text),
            "metadata": metadata or {},
            "timestamp": time.time(),
        }

        with self._lock:
            log = list(self.data.get(BlackboardDataKey.LLM_CHAT_LOG, []))
            log.append(entry)
            if len(log) > self._chat_log_max_entries:
                log = log[-self._chat_log_max_entries:]
            self._set(BlackboardDataKey.LLM_CHAT_LOG, log)

    def snapshot_chat_log(self) -> List[Dict[str, Any]]:
        """Return a shallow copy of the recorded LLM conversation."""
        with self._lock:
            log = list(self.data.get(BlackboardDataKey.LLM_CHAT_LOG, []))

        snapshot: List[Dict[str, Any]] = []
        for entry in log:
            snapshot.append(
                {
                    "role": entry.get("role"),
                    "text": entry.get("text"),
                    "metadata": self._serialize_value(entry.get("metadata")),
                    "timestamp": entry.get("timestamp"),
                }
            )
        return snapshot

    def snapshot_command_state(self) -> Dict[str, Any]:
        """Return summaries of active and pending commands."""
        with self._lock:
            pending = [self._summarize_command(cmd) for cmd in list(self._command_queue)]
            active = self.data.get(BlackboardDataKey.ACTIVE_COMMAND)
            active_summary = (
                self._summarize_command(active)
                if isinstance(active, UserCommand)
                else None
            )

        return {
            "pending_count": len(pending),
            "pending_commands": pending,
            "active_command": active_summary,
        }

    def snapshot_navigation_status(self) -> Dict[str, Any]:
        """Return the latest navigation goal and status information."""
        with self._lock:
            status = self.data.get(BlackboardDataKey.NAVIGATION_STATUS)
            goal = self.data.get(BlackboardDataKey.NAVIGATION_CURRENT_GOAL)
            feedback = self.data.get(BlackboardDataKey.NAVIGATION_FEEDBACK)

        return {
            "status": status,
            "goal": self._format_pose(goal),
            "feedback": self._serialize_value(feedback),
        }

    def snapshot_motion_status(self) -> Dict[str, Any]:
        """Return drive and rotate progress snapshots."""
        with self._lock:
            drive = {
                "target_distance_m": self.data.get(BlackboardDataKey.DRIVE_TARGET_DISTANCE),
                "distance_travelled_m": self.data.get(BlackboardDataKey.DRIVE_DISTANCE_TRAVELLED),
                "direction_sign": self.data.get(BlackboardDataKey.DRIVE_DIRECTION_SIGN),
                "start_pose": self.data.get(BlackboardDataKey.DRIVE_START_POSE),
            }
            rotate = {
                "target_angle_rad": self.data.get(BlackboardDataKey.ROTATE_TARGET_ANGLE),
                "angle_travelled_rad": self.data.get(BlackboardDataKey.ROTATE_ANGLE_TRAVELLED),
                "direction_sign": self.data.get(BlackboardDataKey.ROTATE_DIRECTION_SIGN),
                "start_yaw": self.data.get(BlackboardDataKey.ROTATE_START_YAW),
            }

        return {
            "drive": self._serialize_mapping(drive),
            "rotate": self._serialize_mapping(rotate),
        }

    def snapshot_robot_map(self) -> Dict[str, Any]:
        """Return a summary of persistent tracked objects."""
        with self._lock:
            persistent_objects: List[Any] = (
                self.data.get(BlackboardDataKey.ROBOT_MAP) or []
            )

        return {
            "persistent_object_count": len(persistent_objects),
            "persistent_objects": [
                self._summarize_tracked_object(obj) for obj in persistent_objects
            ],
        }

    @staticmethod
    def _summarize_command(command: UserCommand) -> Dict[str, Any]:
        return {
            "command_id": command.command_id,
            "command_type": command.command_type.name.lower(),
            "parameters": command.parameters,
            "timestamp": command.timestamp,
        }

    @classmethod
    def _serialize_mapping(cls, mapping: Dict[str, Any]) -> Dict[str, Any]:
        return {key: cls._serialize_value(value) for key, value in mapping.items()}

    @staticmethod
    def _serialize_value(value: Any) -> Any:
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, dict):
            return {key: Blackboard._serialize_value(val) for key, val in value.items()}
        if isinstance(value, (list, tuple)):
            return [Blackboard._serialize_value(item) for item in value]
        return str(value)

    @classmethod
    def _summarize_tracked_object(cls, tracked_object: Any) -> Dict[str, Any]:
        detected = getattr(tracked_object, "detected_object", None)
        detected_summary = None
        if detected is not None:
            detected_summary = {
                "class_name": getattr(detected, "name", None),
                "confidence": getattr(detected, "confidence", None),
                "world_coordinates": {
                    "x": getattr(detected, "world_x", None),
                    "y": getattr(detected, "world_y", None),
                    "z": getattr(detected, "world_z", None),
                },
            }

        return {
            "detected_object": detected_summary or cls._serialize_value(detected),
            "total_detections": getattr(tracked_object, "total_detections", None),
            "first_seen_timestamp": getattr(tracked_object, "first_seen_timestamp", None),
            "last_seen_timestamp": getattr(tracked_object, "last_seen_timestamp", None),
        }

    @staticmethod
    def _format_pose(goal: Any) -> Any:
        if goal is None:
            return None
        try:
            pose = goal.pose
            position = pose.position
            orientation = pose.orientation
            return {
                "position": {
                    "x": getattr(position, "x", None),
                    "y": getattr(position, "y", None),
                    "z": getattr(position, "z", None),
                },
                "orientation": {
                    "x": getattr(orientation, "x", None),
                    "y": getattr(orientation, "y", None),
                    "z": getattr(orientation, "z", None),
                    "w": getattr(orientation, "w", None),
                },
                "frame_id": getattr(goal.header, "frame_id", None),
            }
        except AttributeError:
            return Blackboard._serialize_value(goal)

    def publish(self, publisher):
        # print(self.data.keys())
        pickled = pickle.dumps(self.data)
        encoded = base64.b64encode(pickled).decode('utf-8')

        msg = String()
        msg.data = encoded

        publisher.publish(msg)

    def load_dict(self, dictionary):
        self.data = dictionary
        self._command_queue = self.get(BlackboardDataKey.COMMAND_QUEUE)

