"""Service responsible for building a persistent world map from sensor events."""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus


class TemporaryTrackedObject:
    """Represents an object being tracked for consecutive frames before persistence."""

    def __init__(self, object_class, world_coords, detection_info):
        self.object_class = object_class
        self.world_coords = world_coords
        self.detection_info = detection_info
        self.consecutive_frames = 1
        self.first_seen = time.time()
        self.last_seen = time.time()
        self.frames_not_seen = 0

    def update(self, world_coords, detection_info):
        self.world_coords = world_coords
        self.detection_info = detection_info
        self.consecutive_frames += 1
        self.last_seen = time.time()
        self.frames_not_seen = 0

    def distance_to(self, world_coords):
        if not self.world_coords or not world_coords:
            return float('inf')

        dx = self.world_coords[0] - world_coords[0]
        dy = self.world_coords[1] - world_coords[1]
        dz = self.world_coords[2] - world_coords[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)


class PersistentObject:
    """Represents a confirmed persistent object in the world map."""

    def __init__(self, object_class, world_coords, detection_info):
        self.object_class = object_class
        self.world_coords = world_coords
        self.detection_info = detection_info
        self.first_seen = time.time()
        self.last_seen = time.time()
        self.total_detections = 1
        self.is_selected_target = False

    def update_position(self, world_coords, detection_info):
        self.world_coords = world_coords
        self.detection_info = detection_info
        self.last_seen = time.time()
        self.total_detections += 1

    def distance_to(self, world_coords):
        if not self.world_coords or not world_coords:
            return float('inf')

        dx = self.world_coords[0] - world_coords[0]
        dy = self.world_coords[1] - world_coords[1]
        dz = self.world_coords[2] - world_coords[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)


class MapTrackerService:
    """Event-driven map service with temporary tracking and persistent object storage."""

    def __init__(self, event_bus: EventBus):
        self._bus = event_bus

        self.consecutive_frames_threshold = 30
        self.temporary_match_distance = 0.5
        self.persistent_match_distance = 1.5
        self.temporary_object_timeout = 5.0

        self.max_object_processing_distance = 7.5 # Objects that are further away (meters) will no be processed.

        self.temporary_objects: List[TemporaryTrackedObject] = []
        self.persistent_objects: List[PersistentObject] = []

        self.ignored_classes = {'wall', 'ceiling', 'floor'}

        self.robot_is_turning = False
        self.last_robot_orientation = None
        self.last_robot_position = None
        self.orientation_change_threshold = 0.1

        self._bus.subscribe(EventType.WORLD_COORDINATES_CALCULATED, self._on_world_coordinates_received)
        self._bus.subscribe(EventType.ROBOT_TRANSFORM_UPDATED, self._on_robot_transform_updated)

    def _on_world_coordinates_received(self, event: DomainEvent) -> None:
        if self.robot_is_turning or not self.last_robot_position:
            return

        world_coordinates = event.data.get('world_coordinates', {})

        for temp_obj in self.temporary_objects:
            temp_obj.frames_not_seen += 1

        for object_class, detected_objects in world_coordinates.items():
            for obj_info in detected_objects:
                world_coords = obj_info.get('world_coords')
                if world_coords and not any(coord is None for coord in world_coords):
                    self._process_detected_object(object_class, world_coords, obj_info)

        self._cleanup_and_promote_objects()
        self._publish_map_update()

    def _on_robot_transform_updated(self, event: DomainEvent) -> None:
        position = event.data.get('position')
        orientation = event.data.get('orientation')

        if position is not None and orientation is not None:
            self.update_robot_motion_state(position, orientation)

    def _process_detected_object(self, object_class, world_coords, detection_info) -> None:
        if object_class.lower() in self.ignored_classes:
            return

        distance_to_detected_obj = self._distance(self.last_robot_position, world_coords)
        if distance_to_detected_obj > self.max_object_processing_distance:
            return

        matching_temp_obj = self._find_matching_temporary_object(object_class, world_coords)

        if matching_temp_obj:
            matching_temp_obj.update(world_coords, detection_info)
        else:
            new_temp_obj = TemporaryTrackedObject(object_class, world_coords, detection_info)
            self.temporary_objects.append(new_temp_obj)

    def _find_matching_temporary_object(self, object_class, world_coords):
        best_match = None
        best_distance = float('inf')

        for temp_obj in self.temporary_objects:
            if temp_obj.object_class == object_class:
                distance = temp_obj.distance_to(world_coords)
                if distance <= self.temporary_match_distance and distance < best_distance:
                    best_distance = distance
                    best_match = temp_obj

        return best_match

    def _cleanup_and_promote_objects(self) -> None:
        current_time = time.time()
        objects_to_promote = []
        objects_to_keep = []

        for temp_obj in self.temporary_objects:
            if (current_time - temp_obj.last_seen) > self.temporary_object_timeout:
                continue

            if temp_obj.consecutive_frames >= self.consecutive_frames_threshold:
                objects_to_promote.append(temp_obj)
            else:
                objects_to_keep.append(temp_obj)

        self.temporary_objects = objects_to_keep

        for temp_obj in objects_to_promote:
            self._promote_to_persistent(temp_obj)

    def _promote_to_persistent(self, temp_obj: TemporaryTrackedObject) -> None:
        matching_persistent_obj = self._find_matching_persistent_object(
            temp_obj.object_class, temp_obj.world_coords)

        if matching_persistent_obj:
            matching_persistent_obj.update_position(temp_obj.world_coords, temp_obj.detection_info)
        else:
            new_persistent_obj = PersistentObject(
                temp_obj.object_class,
                temp_obj.world_coords,
                temp_obj.detection_info
            )
            self.persistent_objects.append(new_persistent_obj)

    def _find_matching_persistent_object(self, object_class, world_coords):
        best_match = None
        best_distance = float('inf')

        for persistent_obj in self.persistent_objects:
            if persistent_obj.object_class == object_class:
                distance = persistent_obj.distance_to(world_coords)
                if distance <= self.persistent_match_distance and distance < best_distance:
                    best_distance = distance
                    best_match = persistent_obj

        return best_match

    def update_robot_motion_state(self, position, orientation) -> None:
        if self.last_robot_orientation is None:
            self.last_robot_orientation = orientation
            self.robot_is_turning = False
            return

        yaw_difference = self._calculate_orientation_difference(
            self.last_robot_orientation,
            orientation
        )

        self.robot_is_turning = abs(yaw_difference) > self.orientation_change_threshold
        self.last_robot_orientation = orientation
        self.last_robot_position = position

    def _calculate_orientation_difference(self, q1, q2) -> float:
        import math

        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        yaw1 = math.atan2(2.0 * (w1 * z1 + x1 * y1), 1.0 - 2.0 * (y1 * y1 + z1 * z1))
        yaw2 = math.atan2(2.0 * (w2 * z2 + x2 * y2), 1.0 - 2.0 * (y2 * y2 + z2 * z2))

        diff = yaw2 - yaw1
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return diff

    def _publish_map_update(self) -> None:
        persistent_objects_map: Dict[str, List[Dict[str, object]]] = {}

        for obj in self.persistent_objects:
            persistent_objects_map.setdefault(obj.object_class, []).append({
                'world_coords': obj.world_coords,
                'detection_count': obj.total_detections,
                'first_seen': obj.first_seen,
                'last_seen': obj.last_seen,
                'is_selected_target': obj.is_selected_target,
            })

        self._bus.publish_event(
            EventType.SENSOR_DATA_UPDATED,
            source="MapService",
            data={'persistent_objects_map': persistent_objects_map}
        )

    def get_persistent_objects_snapshot(self) -> Dict[str, List[Dict[str, object]]]:
        """Return a snapshot of persistent objects keyed by object class."""
        snapshot: Dict[str, List[Dict[str, object]]] = {}
        for obj in self.persistent_objects:
            snapshot.setdefault(obj.object_class, []).append({
                'world_coords': obj.world_coords,
                'detection_count': obj.total_detections,
                'first_seen': obj.first_seen,
                'last_seen': obj.last_seen,
                'is_selected_target': obj.is_selected_target,
            })
        return snapshot

    def mark_selected_target(self, target_class: Optional[str], tracking_id: Optional[int]) -> None:
        for obj in self.persistent_objects:
            obj.is_selected_target = False

        if target_class is None or tracking_id is None:
            self._publish_map_update()
            return

        for obj in self.persistent_objects:
            detection_info = obj.detection_info or {}
            if (
                detection_info.get('tracking_id') == tracking_id and
                obj.object_class == target_class
            ):
                obj.is_selected_target = True
                break

        self._publish_map_update()

    def _distance(self, source, target):
        if not source or not target:
            return float('inf')

        dx = source[0] - target[0]
        dy = source[1] - target[1]
        dz = source[2] - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def clear_map(self) -> None:
        self.temporary_objects.clear()
        self.persistent_objects.clear()
        self._publish_map_update()
