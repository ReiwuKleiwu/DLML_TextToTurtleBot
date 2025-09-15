import random
import math
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.events import EventQueue, EventType, Event

class LIDARHandler:
    def __init__(self, state_machine: StateMachine, visualization_service=None):
        self.state_machine = state_machine
        self.visualization_service = visualization_service

        # Store latest LIDAR data for visualization
        self.latest_scan_points = []
        self.latest_obstacle_points = []

        # Event queue for decoupled communication
        self.event_queue = EventQueue()

        # TF buffer for coordinate transformation
        self.tf_buffer = None
        self.world_frame = "map"

        # Filtering parameters
        self.min_range = 0.1  # Minimum valid range (m)
        self.max_range = 15  # Maximum range to consider (m)

        # Track obstacle state for event publishing
        self._obstacle_detected = False

    def set_tf_buffer(self, tf_buffer):
        """Set TF buffer for coordinate transformations"""
        self.tf_buffer = tf_buffer


    def transform_to_world_coordinates(self, local_x, local_y, timestamp, frame_id):
        """Transform LIDAR coordinates to world coordinates using TF"""
        if self.tf_buffer is None:
            return None

        try:
            # Create point in LIDAR frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = frame_id
            point_stamped.header.stamp = timestamp
            point_stamped.point.x = local_x
            point_stamped.point.y = local_y
            point_stamped.point.z = 0.0

            # Transform to world frame - try with exact timestamp first
            world_point = self.tf_buffer.transform(point_stamped, self.world_frame)
            return (world_point.point.x, world_point.point.y)

        except Exception as e:
            # If exact timestamp fails, try with latest available transform
            try:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = frame_id
                point_stamped.header.stamp = self.tf_buffer.get_latest_common_time(frame_id, self.world_frame)
                point_stamped.point.x = local_x
                point_stamped.point.y = local_y
                point_stamped.point.z = 0.0

                world_point = self.tf_buffer.transform(point_stamped, self.world_frame)
                return (world_point.point.x, world_point.point.y)

            except Exception as e2:
                return None

    def process_full_scan(self, msg):
        """Process full LIDAR scan and convert to world coordinates"""
        scan_points = []
        obstacle_points = []

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        timestamp = msg.header.stamp
        frame_id = msg.header.frame_id

        # Process ALL range measurements for full 360° coverage
        total_valid_ranges = 0
        successful_transforms = 0

        for i in range(len(msg.ranges)):
            range_val = msg.ranges[i]

            # Skip invalid measurements
            if (range_val < self.min_range or
                range_val > self.max_range or
                math.isinf(range_val) or
                math.isnan(range_val)):
                continue

            total_valid_ranges += 1

            # Calculate angle for this measurement
            angle = angle_min + i * angle_increment

            # Convert to Cartesian coordinates in LIDAR frame
            # Standard ROS LIDAR convention: X forward, Y left, angle 0 = forward (X-axis)
            local_x = range_val * math.cos(angle)
            local_y = range_val * math.sin(angle)

            # Transform to world coordinates using the actual frame from message
            world_coords = self.transform_to_world_coordinates(local_x, local_y, timestamp, frame_id)

            if world_coords:
                successful_transforms += 1
                scan_points.append(world_coords)

                # Points closer than threshold are considered obstacles for visualization
                if range_val < 4.0:  # Within 4m are considered obstacles for visualization
                    obstacle_points.append(world_coords)

        # Transform success tracking (silent)

        # Store the processed data
        self.latest_scan_points = scan_points
        self.latest_obstacle_points = obstacle_points

        # Update visualization service if available
        if self.visualization_service:
            scan_data = {
                'scan_points': scan_points,
                'obstacle_points': obstacle_points,
                'timestamp': timestamp
            }
            self.visualization_service.update_lidar_data(scan_data)

        # Publish LIDAR scan processed event
        self.event_queue.publish_event(
            EventType.LIDAR_SCAN_PROCESSED,
            source="LidarHandler",
            data={
                'scan_points': scan_points,
                'obstacle_points': obstacle_points,
                'timestamp': timestamp,
                'total_points': total_valid_ranges,
                'successful_transforms': successful_transforms
            }
        )

    def get_latest_scan_data(self):
        """Get the latest processed scan data"""
        return {
            'scan_points': self.latest_scan_points,
            'obstacle_points': self.latest_obstacle_points
        }

    def handle(self, msg):
        # Process full scan for visualization
        self.process_full_scan(msg)

        # Original obstacle avoidance logic
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        index_front = round((-1.5708 - angle_min) / angle_increment)

        index_start = max(0, index_front - 50)
        index_end = min(len(msg.ranges) - 1, index_front + 50)
        front_ranges = msg.ranges[index_start:index_end + 1]

        # Filter out invalid values
        valid_front_ranges = [x for x in front_ranges if not (math.isinf(x) or math.isnan(x))]

        if valid_front_ranges:
            front_distance = min(valid_front_ranges)

            if front_distance < 0.5:
                if not self._obstacle_detected:
                    print(f"[LIDAR]: Detected obstacle at {front_distance:.2f}m")
                    direction = 1
                    self.state_machine.push_state(TurtleBotState.AVOID_OBSTACLE, TurtleBotStateSource.LIDAR, data={"direction": direction})

                    # Publish obstacle detected event
                    self.event_queue.publish_event(
                        EventType.OBSTACLE_DETECTED,
                        source="LidarHandler",
                        data={
                            'distance': front_distance,
                            'direction': direction,
                            'sensor_type': 'lidar'
                        }
                    )
                    self._obstacle_detected = True
            else:
                if self._obstacle_detected:
                    self.state_machine.pop_state(TurtleBotState.AVOID_OBSTACLE, TurtleBotStateSource.LIDAR)

                    # Publish obstacle cleared event
                    self.event_queue.publish_event(
                        EventType.OBSTACLE_CLEARED,
                        source="LidarHandler",
                        data={
                            'sensor_type': 'lidar'
                        }
                    )
                    self._obstacle_detected = False