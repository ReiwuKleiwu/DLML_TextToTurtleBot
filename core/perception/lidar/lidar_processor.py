import math
from typing import Any, Dict, List, Optional, Tuple

from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType
from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey


class LidarProcessor:
    def __init__(
        self,
        max_distance_threshold: float = 0.2,
        front_angle_dec: float = 30.0,
        pointcloud_distance_limit: float = 15.0,
        point_stride: int = 1,
        scan_heading_offset_rad: float = math.pi / 2,
    ) -> None:
        self._event_bus = EventBus()
        self._blackboard = Blackboard()

        self.max_distance_threshold = max_distance_threshold
        self.front_angle_deg = front_angle_dec
        self.pointcloud_distance_limit = max(0.0, float(pointcloud_distance_limit))
        self.point_stride = max(1, int(point_stride))
        self.scan_heading_offset = float(scan_heading_offset_rad)

    def handle(self, msg) -> None:
        self._check_for_obstacles(msg)
        self._publish_pointcloud(msg)

    def _check_for_obstacles(self, msg) -> None:
        if msg is None:
            return

        # -1.5707 = -90°
        center_angle = -1.5708
        half_width = math.radians(self.front_angle_deg / 2.0)

        # Compute index range, clamped to valid array indices with wrap handling
        def angle_to_index(theta: float) -> int:
            return int(round((theta - msg.angle_min) / msg.angle_increment))

        start_theta = center_angle - half_width
        end_theta = center_angle + half_width

        # If scan wraps (e.g., angle_min near -pi and angle_max near +pi), we may need two slices
        start_idx = angle_to_index(start_theta)
        end_idx = angle_to_index(end_theta)

        n = len(msg.ranges)
        # Normalize indices to [0, n-1]
        start_idx = max(0, min(n - 1, start_idx))
        end_idx = max(0, min(n - 1, end_idx))

        if start_idx <= end_idx:
            sector = msg.ranges[start_idx:end_idx + 1]
        else:
            # Wrapped around: take [start..end_of_array] + [0..end]
            sector = msg.ranges[start_idx:n] + msg.ranges[0:end_idx + 1]

        # Filter out NaNs/inf and obviously bogus negatives
        cleaned = [r for r in sector if (not math.isinf(r)) and (not math.isnan(r)) and (r > 0.0)]

        nearest = min(cleaned) if cleaned else float("inf")

        if nearest >= self.max_distance_threshold:
            self._event_bus.publish(DomainEvent(event_type=EventType.LIDAR_OBSTACLE_ABSENT, data={}))
            return

        # Obstacle detected!
        self._event_bus.publish(DomainEvent(event_type=EventType.LIDAR_OBSTACLE_PRESENT, data={}))

    def _publish_pointcloud(self, msg) -> None:
        if msg is None:
            return

        ranges = getattr(msg, "ranges", None)
        if not ranges:
            return

        robot_pose = self._extract_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y, yaw = robot_pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        angle = getattr(msg, "angle_min", 0.0)
        increment = getattr(msg, "angle_increment", 0.0) or 0.0

        points: List[Dict[str, float]] = []
        stride = max(1, self.point_stride)

        for idx in range(0, len(ranges), stride):
            distance = ranges[idx]
            if distance is None or math.isnan(distance) or math.isinf(distance) or distance <= 0.0:
                angle += increment * stride
                continue

            if self.pointcloud_distance_limit and distance > self.pointcloud_distance_limit:
                angle += increment * stride
                continue

            beam_angle = angle + self.scan_heading_offset

            local_x = distance * math.cos(beam_angle)
            local_y = distance * math.sin(beam_angle)

            world_x = robot_x + (local_x * cos_yaw - local_y * sin_yaw)
            world_y = robot_y + (local_x * sin_yaw + local_y * cos_yaw)

            points.append({
                "x": world_x,
                "y": world_y,
                "distance": distance,
            })

            angle += increment * stride

        data: Dict[str, Any] = {"points": points}

        header = getattr(msg, "header", None)
        if header is not None:
            data["frame_id"] = getattr(header, "frame_id", None)
            stamp = getattr(header, "stamp", None)
            timestamp = self._extract_timestamp(stamp)
            if timestamp is not None:
                data["timestamp"] = timestamp

        self._event_bus.publish(
            DomainEvent(event_type=EventType.LIDAR_POINTS_UPDATED, data=data)
        )

    def _extract_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        position = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)
        orientation = self._blackboard.get(BlackboardDataKey.ROBOT_ORIENTATION)

        if position is None or orientation is None:
            return None

        try:
            pos_x = float(getattr(position, "x", 0.0))
            pos_y = float(getattr(position, "y", 0.0))
        except (TypeError, ValueError):
            return None

        yaw = self._compute_yaw(orientation)
        if yaw is None:
            return None

        return pos_x, pos_y, yaw

    def _compute_yaw(self, quaternion: Any) -> Optional[float]:
        if quaternion is None:
            return None

        try:
            x = float(getattr(quaternion, "x"))
            y = float(getattr(quaternion, "y"))
            z = float(getattr(quaternion, "z"))
            w = float(getattr(quaternion, "w"))
        except (TypeError, ValueError, AttributeError):
            return None

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _extract_timestamp(self, stamp: Any) -> Optional[float]:
        if stamp is None:
            return None

        try:
            seconds = float(getattr(stamp, "sec", 0))
            nanoseconds = float(getattr(stamp, "nanosec", 0))
        except (TypeError, ValueError):
            return None

        return seconds + nanoseconds * 1e-9
