import time
from classes.events import EventQueue, EventType, Event
from classes.topics.TFSubscriber import TFSubscriber


class TFService:
    """Service that manages robot transform data and publishes transform events"""

    def __init__(self, tf_subscriber: TFSubscriber):
        """
        Initialize the TF service.

        Args:
            tf_subscriber: TFSubscriber instance to get transform data from
        """
        self.tf_subscriber = tf_subscriber

        # Robot state
        self.robot_position = None  # (x, y, z) in world coordinates
        self.robot_orientation = None  # (roll, pitch, yaw) in radians
        self.last_update_time = 0

        # Event system
        self.event_queue = EventQueue()

    def update_robot_transforms(self):
        """Update robot position and orientation and publish transform event"""
        # Get current transform data
        new_position = self.tf_subscriber.get_position()
        new_orientation = self.tf_subscriber.get_orientation()
        current_time = time.time()

        # Update stored values
        self.robot_position = new_position
        self.robot_orientation = new_orientation
        self.last_update_time = current_time

        # Only publish if we have valid data
        if self.robot_position is not None and self.robot_orientation is not None:
            self._publish_transform_update()

    def _publish_transform_update(self):
        """Publish robot transform update event"""
        self.event_queue.publish_event(
            EventType.ROBOT_TRANSFORM_UPDATED,
            source="TFService",
            data={
                'position': self.robot_position,
                'orientation': self.robot_orientation,
                'timestamp': time.time()
            }
        )

    def get_robot_position(self):
        """Get current robot position"""
        return self.robot_position

    def get_robot_orientation(self):
        """Get current robot orientation"""
        return self.robot_orientation

    def get_robot_pose(self):
        """Get current robot position and orientation as tuple"""
        return self.robot_position, self.robot_orientation

    def is_transform_available(self):
        """Check if valid transform data is available"""
        return (self.robot_position is not None and
                self.robot_orientation is not None)

    def get_status(self):
        """Get current service status"""
        return {
            'position': self.robot_position,
            'orientation': self.robot_orientation,
            'transform_available': self.is_transform_available(),
            'last_update_time': self.last_update_time
        }