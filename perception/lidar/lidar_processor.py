import math

from events.event_bus import EventBus
from events.interfaces.events import EventType, DomainEvent

class LidarProcessor:
    def __init__(self, max_distance_threshold = 1.0, front_angle_dec = 30.0) -> None:
        self._event_bus = EventBus()
        self.max_distance_threshold = max_distance_threshold
        self.front_angle_deg = front_angle_dec
        return
    
    def handle(self, msg) -> None:
        self._check_for_obstacles(msg)
    
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

        nearest = min(cleaned) if cleaned else float('inf')

        if nearest >= self.max_distance_threshold:
            self._event_bus.publish(DomainEvent(event_type=EventType.LIDAR_OBSTACLE_ABSENT, data={}))
            return
        
        # Obstacle detected!
        self._event_bus.publish(DomainEvent(event_type=EventType.LIDAR_OBSTACLE_PRESENT, data={}))



