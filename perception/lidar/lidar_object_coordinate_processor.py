import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import pyrealsense2 as rs2
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401 - needed for TF conversions

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType
from perception.detection.object_detector import DetectedObject


class LidarObjectCoordinateProcessor:
    """
    Estimate world coordinates for camera detections by intersecting their viewing rays
    with the latest LIDAR point cloud in the world frame.
    """

    def __init__(
        self,
        tf_buffer,
        camera_frame: str = "oakd_rgb_camera_optical_frame",
        world_frame: str = "map",
        lateral_tolerance: float = 0.4,
        max_distance: float = 15.0,
        horizontal_tolerance_ratio: float = 0.2,
        vertical_sample_count: int = 3,
        vertical_sample_stride_ratio: float = 0.15,
    ) -> None:
        self._tf_buffer = tf_buffer
        self._event_bus = EventBus()
        self._blackboard = Blackboard()

        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.lateral_tolerance = max(0.05, float(lateral_tolerance))
        self.max_distance = max(0.0, float(max_distance))
        self.horizontal_tolerance_ratio = max(0.0, float(horizontal_tolerance_ratio))
        self.vertical_sample_count = max(1, int(vertical_sample_count))
        self.vertical_sample_stride_ratio = max(0.01, float(vertical_sample_stride_ratio))

        self.intrinsics: Optional[rs2.intrinsics] = None
        self._latest_detections: Dict[str, List[DetectedObject]] = {}
        self._latest_lidar_points: List[Dict[str, float]] = []
        self._latest_lidar_stamp: Optional[Time] = None

        self._event_bus.subscribe(EventType.OBJECTS_DETECTED, self._on_objects_detected)
        self._event_bus.subscribe(EventType.LIDAR_POINTS_UPDATED, self._on_lidar_points_updated)

    def shutdown(self) -> None:
        self._event_bus.unsubscribe(EventType.OBJECTS_DETECTED, self._on_objects_detected)
        self._event_bus.unsubscribe(EventType.LIDAR_POINTS_UPDATED, self._on_lidar_points_updated)

    def set_camera_intrinsics(self, camera_info) -> None:
        if self.intrinsics:
            return

        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.k[2]
        self.intrinsics.ppy = camera_info.k[5]
        self.intrinsics.fx = camera_info.k[0]
        self.intrinsics.fy = camera_info.k[4]

        if camera_info.distortion_model == "plumb_bob":
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif camera_info.distortion_model == "equidistant":
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [value for value in camera_info.d[:5]]

    def _on_objects_detected(self, event: DomainEvent) -> None:
        data = event.data or {}
        if not isinstance(data, dict):
            self._latest_detections = {}
        else:
            self._latest_detections = data
        self._calculate_world_coordinates()

    def _on_lidar_points_updated(self, event: DomainEvent) -> None:
        data = event.data or {}
        raw_points = data.get("points") if isinstance(data, dict) else None
        if isinstance(raw_points, list):
            self._latest_lidar_points = [point for point in raw_points if self._is_valid_point(point)]
        else:
            self._latest_lidar_points = []

        timestamp = data.get("timestamp") if isinstance(data, dict) else None
        self._latest_lidar_stamp = self._float_timestamp_to_time(timestamp)
        self._calculate_world_coordinates()

    def _calculate_world_coordinates(self) -> None:
        if not self.intrinsics:
            return
        if not self._latest_detections:
            return
        if not self._latest_lidar_points:
            return

        detected_objects_with_coordinates: Dict[str, List[DetectedObject]] = {}
        for detected_class, detected_objects in self._latest_detections.items():
            if not isinstance(detected_objects, list):
                continue
            matched_objects: List[DetectedObject] = []

            for detected_object in detected_objects:
                match = self._match_object_to_lidar_point(detected_object)
                if match is None:
                    continue

                detected_object.world_x = match[0]
                detected_object.world_y = match[1]
                detected_object.world_z = match[2]
                matched_objects.append(detected_object)

            if matched_objects:
                detected_objects_with_coordinates[detected_class] = matched_objects

        self._event_bus.publish(
            DomainEvent(EventType.OBJECT_WORLD_COORDINATES_UPDATED, detected_objects_with_coordinates)
        )

    def _match_object_to_lidar_point(self, detected_object: DetectedObject) -> Optional[Tuple[float, float, float]]:
        center_x = int((detected_object.x1 + detected_object.x2) / 2)
        principal_x = float(self.intrinsics.ppx)
        image_width = float(self.intrinsics.width)

        horizontal_offset_ratio = abs(center_x - principal_x) / max(image_width, 1.0)
        if horizontal_offset_ratio > self.horizontal_tolerance_ratio:
            return None

        bottom_y = min(max(int(detected_object.y2), 0), self.intrinsics.height - 1)
        top_y = min(max(int(detected_object.y1), 0), self.intrinsics.height - 1)
        if bottom_y < 0 or top_y < 0 or bottom_y >= self.intrinsics.height:
            return None

        vertical_span = max(1, bottom_y - top_y)
        stride_pixels = max(1.0, vertical_span * self.vertical_sample_stride_ratio)

        candidate_points: List[Tuple[int, int]] = []
        current_y = float(bottom_y)
        for _ in range(self.vertical_sample_count):
            candidate_points.append((center_x, int(round(current_y))))
            current_y -= stride_pixels
            if current_y < top_y:
                break

        for u, v in candidate_points:
            match = self._match_pixel_ray(u, v)
            if match is not None:
                return match
        return None

    def _match_pixel_ray(self, u: int, v: int) -> Optional[Tuple[float, float, float]]:
        if not (0 <= u < self.intrinsics.width and 0 <= v < self.intrinsics.height):
            return None

        ray_direction_camera = self._pixel_to_unit_ray(u, v)
        if ray_direction_camera is None:
            return None

        origin_world = self._transform_to_world_coordinates(0.0, 0.0, 0.0, self._latest_lidar_stamp)
        direction_world_point = self._transform_to_world_coordinates(
            ray_direction_camera[0],
            ray_direction_camera[1],
            ray_direction_camera[2],
            self._latest_lidar_stamp,
        )

        if origin_world is None or direction_world_point is None:
            return None

        direction_vector = np.array(direction_world_point) - np.array(origin_world)
        direction_xy = direction_vector[:2]
        direction_xy_norm = np.linalg.norm(direction_xy)
        if direction_xy_norm < 1e-6:
            return None
        direction_xy_unit = direction_xy / direction_xy_norm

        origin_xy = np.array(origin_world[:2])

        best_point: Optional[Tuple[float, float, float]] = None
        best_lateral_offset: float = float("inf")

        for point in self._latest_lidar_points:
            point_xy = np.array([point["x"], point["y"]])
            vec_to_point = point_xy - origin_xy
            longitudinal = float(np.dot(vec_to_point, direction_xy_unit))
            if longitudinal <= 0.0:
                continue
            if self.max_distance and longitudinal > self.max_distance:
                continue

            projected_xy = origin_xy + direction_xy_unit * longitudinal
            lateral_offset = float(np.linalg.norm(point_xy - projected_xy))
            if lateral_offset > self.lateral_tolerance:
                continue

            if lateral_offset > best_lateral_offset:
                continue

            best_point = (
                point["x"],
                point["y"],
                self._estimate_world_z(origin_world, direction_vector, direction_xy_norm, longitudinal),
            )
            best_lateral_offset = lateral_offset

        if best_point is None:
            return None

        robot_position = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)
        if robot_position is not None and getattr(robot_position, "z", None) is not None:
            world_z = float(robot_position.z)
            best_point = (best_point[0], best_point[1], world_z)
        return best_point

    def _pixel_to_unit_ray(self, u: int, v: int) -> Optional[np.ndarray]:
        try:
            point = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], 1.0)
        except Exception:
            return None

        ray = np.array(point, dtype=float)
        norm = np.linalg.norm(ray)
        if norm < 1e-6:
            return None
        return ray / norm

    def _estimate_world_z(
        self,
        origin_world: Tuple[float, float, float],
        direction_vector: np.ndarray,
        direction_xy_norm: float,
        longitudinal: float,
    ) -> float:
        if direction_xy_norm < 1e-6:
            return origin_world[2]
        scale = longitudinal / direction_xy_norm
        return float(origin_world[2] + direction_vector[2] * scale)

    def _transform_to_world_coordinates(
        self,
        x: float,
        y: float,
        z: float,
        timestamp: Optional[Time],
    ) -> Optional[Tuple[float, float, float]]:
        if self._tf_buffer is None:
            return None

        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.camera_frame
        if timestamp is not None:
            point_stamped.header.stamp = timestamp
        else:
            try:
                point_stamped.header.stamp = self._tf_buffer.get_latest_common_time(
                    self.camera_frame,
                    self.world_frame,
                )
            except Exception:
                point_stamped.header.stamp = Time()

        point_stamped.point.x = float(x)
        point_stamped.point.y = float(y)
        point_stamped.point.z = float(z)

        try:
            world_point = self._tf_buffer.transform(point_stamped, self.world_frame)
            return (
                float(world_point.point.x),
                float(world_point.point.y),
                float(world_point.point.z),
            )
        except Exception:
            return None

    def _is_valid_point(self, point: Dict[str, float]) -> bool:
        if not isinstance(point, dict):
            return False
        x = point.get("x")
        y = point.get("y")
        if x is None or y is None:
            return False
        try:
            float(x)
            float(y)
        except (TypeError, ValueError):
            return False
        return True

    def _float_timestamp_to_time(self, timestamp: Optional[float]) -> Optional[Time]:
        if timestamp is None:
            return None
        try:
            ts = float(timestamp)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(ts) or ts < 0.0:
            return None
        seconds = int(ts)
        nanoseconds = int((ts - seconds) * 1e9)
        return Time(sec=seconds, nanosec=nanoseconds)
