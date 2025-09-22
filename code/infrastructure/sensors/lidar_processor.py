"""LiDAR processing that produces obstacle events and visualization data."""
from __future__ import annotations

import math
from typing import List, Tuple

from geometry_msgs.msg import PointStamped

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus


class LidarProcessor:
    """Convert ROS laser scans into events understood by the application."""

    def __init__(self, event_bus: EventBus) -> None:
        self._bus = event_bus

        self.latest_scan_points: List[Tuple[float, float]] = []
        self.latest_obstacle_points: List[Tuple[float, float]] = []

        self.tf_buffer = None
        self.world_frame = "map"

        self.min_range = 0.1
        self.max_range = 15.0

        self._obstacle_detected = False

    def set_tf_buffer(self, tf_buffer) -> None:
        self.tf_buffer = tf_buffer

    def handle(self, msg) -> None:
        self._process_full_scan(msg)
        self._evaluate_obstacle(msg)

    def _process_full_scan(self, msg) -> None:
        scan_points = []
        obstacle_points = []

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        timestamp = msg.header.stamp
        frame_id = msg.header.frame_id

        total_valid_ranges = 0
        successful_transforms = 0

        for index, range_val in enumerate(msg.ranges):
            if (
                range_val < self.min_range
                or range_val > self.max_range
                or math.isinf(range_val)
                or math.isnan(range_val)
            ):
                continue

            total_valid_ranges += 1
            angle = angle_min + index * angle_increment

            local_x = range_val * math.cos(angle)
            local_y = range_val * math.sin(angle)

            world_coords = self._transform_to_world_coordinates(local_x, local_y, timestamp, frame_id)
            if not world_coords:
                continue

            successful_transforms += 1
            scan_points.append(world_coords)
            if range_val < 4.0:
                obstacle_points.append(world_coords)

        self.latest_scan_points = scan_points
        self.latest_obstacle_points = obstacle_points

        self._bus.publish_event(
            EventType.LIDAR_SCAN_PROCESSED,
            source="LidarProcessor",
            data={
                'scan_points': scan_points,
                'obstacle_points': obstacle_points,
                'timestamp': timestamp,
                'total_points': total_valid_ranges,
                'successful_transforms': successful_transforms,
            },
        )

    def _evaluate_obstacle(self, msg) -> None:
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        index_front = round((-1.5708 - angle_min) / angle_increment)
        index_start = max(0, index_front - 50)
        index_end = min(len(msg.ranges) - 1, index_front + 50)
        front_ranges = msg.ranges[index_start:index_end + 1]

        valid_front_ranges = [value for value in front_ranges if not (math.isinf(value) or math.isnan(value))]
        if not valid_front_ranges:
            return

        front_distance = min(valid_front_ranges)

        if front_distance < 0.75:
            if not self._obstacle_detected:
                direction = 1
                self._bus.publish_event(
                    EventType.OBSTACLE_DETECTED,
                    source="LidarProcessor",
                    data={
                        'distance': front_distance,
                        'direction': direction,
                        'sensor_type': 'lidar',
                    },
                )
                self._obstacle_detected = True
        else:
            if self._obstacle_detected:
                self._bus.publish_event(
                    EventType.OBSTACLE_CLEARED,
                    source="LidarProcessor",
                    data={'sensor_type': 'lidar'},
                )
                self._obstacle_detected = False

    def _transform_to_world_coordinates(self, local_x, local_y, timestamp, frame_id):
        if self.tf_buffer is None:
            return None

        point_stamped = PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = timestamp
        point_stamped.point.x = local_x
        point_stamped.point.y = local_y
        point_stamped.point.z = 0.0

        try:
            world_point = self.tf_buffer.transform(point_stamped, self.world_frame)
            return (world_point.point.x, world_point.point.y)
        except Exception:  # noqa: BLE001
            try:
                point_stamped.header.stamp = self.tf_buffer.get_latest_common_time(frame_id, self.world_frame)
                world_point = self.tf_buffer.transform(point_stamped, self.world_frame)
                return (world_point.point.x, world_point.point.y)
            except Exception:  # noqa: BLE001
                return None
