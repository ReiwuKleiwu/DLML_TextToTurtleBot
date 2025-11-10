"""Helpers to generate serialized state for the mission board frontend."""
from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey


class MissionBoardStateBuilder:
    """Collects and serializes the current blackboard state for the mission board UI."""

    def __init__(self) -> None:
        self._blackboard = Blackboard()

    def build_state(self) -> Dict[str, Any]:
        robot_position = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)
        robot_orientation = self._blackboard.get(BlackboardDataKey.ROBOT_ORIENTATION)
        robot_trail = list(self._blackboard.get(BlackboardDataKey.ROBOT_TRAIL, []))

        chat_log = self._blackboard.snapshot_chat_log()

        navigation_snapshot = self._blackboard.snapshot_navigation_status()
        motion_snapshot = self._blackboard.snapshot_motion_status()
        command_snapshot = self._blackboard.snapshot_command_state()
        map_snapshot = self._blackboard.snapshot_robot_map()
        lidar_snapshot = self._serialize_lidar_points(
            self._blackboard.get(BlackboardDataKey.LIDAR_POINTS)
        )

        target_object = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)
        target_summary = self._serialize_detected_object(target_object)

        robot_position_dict = self._serialize_vector3(robot_position)
        robot_orientation_dict = self._serialize_quaternion(robot_orientation)
        robot_yaw = self._compute_yaw(robot_orientation)

        target_world = target_summary.get("world") if target_summary else None
        target_distance = None
        if robot_position_dict and target_world:
            target_distance = self._euclidean_distance(robot_position_dict, target_world)
            target_summary["distance_to_robot"] = target_distance

        persistent_objects = self._serialize_persistent_objects(map_snapshot, target_world)

        navigation_goal = self._serialize_navigation_goal(navigation_snapshot.get("goal"))
        navigation_status = navigation_snapshot.get("status")
        navigation_feedback = navigation_snapshot.get("feedback")

        llm_capabilities = self._blackboard.get(BlackboardDataKey.LLM_CAPABILITIES, {}) or {}
        latest_frame_entry = self._blackboard.get_latest_camera_frame()
        latest_camera_metadata: Optional[Dict[str, Any]] = None
        if isinstance(latest_frame_entry, dict):
            timestamp = latest_frame_entry.get("timestamp")
            if isinstance(timestamp, (int, float)):
                latest_camera_metadata = {"timestamp": timestamp}
            else:
                latest_camera_metadata = {"timestamp": None}

        return {
            "timestamp": time.time(),
            "behaviour_tree_paused": self._blackboard.is_behaviour_tree_paused(),
            "robot": {
                "position": robot_position_dict,
                "orientation": robot_orientation_dict,
                "yaw": robot_yaw,
                "trail": robot_trail,
            },
            "target": target_summary,
            "navigation": {
                "status": navigation_status,
                "goal": navigation_goal,
                "feedback": navigation_feedback,
            },
            "motion": motion_snapshot,
            "commands": command_snapshot,
            "persistent_map": {
                "count": map_snapshot.get("persistent_object_count", 0),
                "objects": persistent_objects,
            },
            "lidar": lidar_snapshot,
            "target_class": self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS),
            "chat": chat_log,
            "llm": {
                "capabilities": llm_capabilities,
                "latest_camera_frame": latest_camera_metadata,
            },
        }

    def _serialize_vector3(self, vector: Any) -> Optional[Dict[str, float]]:
        if vector is None:
            return None
        if isinstance(vector, dict):
            x = self._coerce_float(vector.get("x"))
            y = self._coerce_float(vector.get("y"))
            z = self._coerce_float(vector.get("z"))
        else:
            x = self._coerce_float(getattr(vector, "x", None))
            y = self._coerce_float(getattr(vector, "y", None))
            z = self._coerce_float(getattr(vector, "z", None))
        if x is None or y is None:
            return None
        return {
            "x": x,
            "y": y,
            "z": z if z is not None else 0.0,
        }

    def _serialize_quaternion(self, quaternion: Any) -> Optional[Dict[str, float]]:
        components = self._get_quaternion_components(quaternion)
        if components is None:
            return None
        x, y, z, w = components
        return {"x": x, "y": y, "z": z, "w": w}

    def _serialize_navigation_goal(self, goal: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(goal, dict):
            return None
        position = self._serialize_vector3(goal.get("position"))
        orientation = self._serialize_quaternion(goal.get("orientation"))
        return {
            "frame_id": goal.get("frame_id"),
            "position": position,
            "orientation": orientation,
            "yaw": self._compute_yaw(goal.get("orientation")),
        }

    def _serialize_detected_object(self, detected: Any) -> Optional[Dict[str, Any]]:
        if detected is None:
            return None

        name = getattr(detected, "name", None)
        confidence = self._coerce_float(getattr(detected, "confidence", None))
        world_x = self._coerce_float(getattr(detected, "world_x", None))
        world_y = self._coerce_float(getattr(detected, "world_y", None))
        world_z = self._coerce_float(getattr(detected, "world_z", None))

        world = None
        if world_x is not None and world_y is not None:
            world = {
                "x": world_x,
                "y": world_y,
                "z": world_z if world_z is not None else 0.0,
            }

        return {
            "class_name": name,
            "confidence": confidence,
            "world": world,
        }

    def _serialize_persistent_objects(
        self,
        snapshot: Dict[str, Any],
        target_world: Optional[Dict[str, float]],
    ) -> List[Dict[str, Any]]:
        results: List[Dict[str, Any]] = []
        for entry in snapshot.get("persistent_objects", []) or []:
            detected = entry.get("detected_object") or {}
            world = self._serialize_vector3(detected.get("world_coordinates"))
            item = {
                "class_name": detected.get("class_name"),
                "confidence": self._coerce_float(detected.get("confidence")),
                "world": world,
                "total_detections": entry.get("total_detections"),
                "first_seen_timestamp": entry.get("first_seen_timestamp"),
                "last_seen_timestamp": entry.get("last_seen_timestamp"),
            }
            if world and target_world:
                item["is_target"] = self._points_close(world, target_world, tolerance=0.15)
            else:
                item["is_target"] = False
            results.append(item)
        return results

    def _serialize_lidar_points(self, value: Any) -> Dict[str, Any]:
        if not isinstance(value, dict):
            return {"points": []}

        raw_points = value.get("points")
        points: List[Dict[str, float]] = []

        if isinstance(raw_points, list):
            for entry in raw_points:
                if not isinstance(entry, dict):
                    continue

                x = self._coerce_float(entry.get("x"))
                y = self._coerce_float(entry.get("y"))
                if x is None or y is None:
                    continue

                point: Dict[str, float] = {"x": x, "y": y}

                distance = self._coerce_float(entry.get("distance"))
                if distance is not None:
                    point["distance"] = distance

                points.append(point)

        payload: Dict[str, Any] = {"points": points}

        frame_id = value.get("frame_id")
        if isinstance(frame_id, str) and frame_id:
            payload["frame_id"] = frame_id

        timestamp = self._coerce_float(value.get("timestamp"))
        if timestamp is not None:
            payload["timestamp"] = timestamp

        return payload

    def _get_quaternion_components(self, value: Any) -> Optional[Tuple[float, float, float, float]]:
        if value is None:
            return None
        if isinstance(value, dict):
            x = self._coerce_float(value.get("x"))
            y = self._coerce_float(value.get("y"))
            z = self._coerce_float(value.get("z"))
            w = self._coerce_float(value.get("w"))
        else:
            x = self._coerce_float(getattr(value, "x", None))
            y = self._coerce_float(getattr(value, "y", None))
            z = self._coerce_float(getattr(value, "z", None))
            w = self._coerce_float(getattr(value, "w", None))
        if None in (x, y, z, w):
            return None
        return x, y, z, w

    def _compute_yaw(self, quaternion: Any) -> Optional[float]:
        components = self._get_quaternion_components(quaternion)
        if components is None:
            return None
        x, y, z, w = components
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _coerce_float(self, value: Any) -> Optional[float]:
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if math.isnan(number) or math.isinf(number):
            return None
        return number

    def _points_close(
        self,
        point_a: Dict[str, float],
        point_b: Dict[str, float],
        *,
        tolerance: float,
    ) -> bool:
        dx = (point_a.get("x", 0.0) - point_b.get("x", 0.0))
        dy = (point_a.get("y", 0.0) - point_b.get("y", 0.0))
        dz = (point_a.get("z", 0.0) - point_b.get("z", 0.0))
        return (dx * dx + dy * dy + dz * dz) <= tolerance * tolerance

    def _euclidean_distance(
        self,
        a: Dict[str, float],
        b: Dict[str, float],
    ) -> float:
        dx = a["x"] - b["x"]
        dy = a["y"] - b["y"]
        dz = a.get("z", 0.0) - b.get("z", 0.0)
        return math.sqrt(dx * dx + dy * dy + dz * dz)
