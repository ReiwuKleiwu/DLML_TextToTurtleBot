import math
import tf2_ros
from rclpy.node import Node
import rclpy
from events.event_bus import EventBus
from events.interfaces.events import EventType, DomainEvent


class TFSubscriber:
    def __init__(self, node: Node, base_link_frame: str = "base_link", world_frame: str = "map"):
        """
        Initialize the TF subscriber to listen to base_link position.
        
        Args:
            node: The ROS2 node instance
            base_link_frame: The frame to track (default: "base_link")
            world_frame: The world/reference frame to get absolute position from (default: "map")
        """
        self.node = node
        self.base_link_frame = base_link_frame
        self.world_frame = world_frame

        self._event_bus = EventBus()
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        
        # Store the previous and latest transform
        self.previous_transform = None
        self.latest_transform = None
        
        # Create a timer to periodically get transforms
        self.timer = self.node.create_timer(0.1, self.update_transform)

        self.turning_threshold_rad_s = 0.1
        self.turning_state = False
        self.previous_turning_state = False
    
    
    def update_transform(self):
        """
        Periodically update the transform from world frame to base_link.
        """
        try:
            # Get the latest transform
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_link_frame,
                rclpy.time.Time()
            )
            #print(transform)

            self.previous_transform = self.latest_transform
            self.latest_transform = transform

            self._get_position()
            self._get_orientation()

            self.previous_turning_state = self.turning_state
            self.turning_state = self._check_robot_is_turning()

            if self.turning_state != self.previous_turning_state:
                self._event_bus.publish(DomainEvent(EventType.ROBOT_IS_TURNING_UPDATED, self.turning_state))
        
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Transform not available yet, this is normal during startup
            pass
    
    def _get_position(self):
        """
        Get the current position of base_link in the world frame.
        
        Returns:
            tuple: (x, y, z) position or None if no transform available
        """
        if self.latest_transform is None:
            return None
            
        translation = self.latest_transform.transform.translation
        self._event_bus.publish(DomainEvent(EventType.ROBOT_POSITION_UPDATED, translation))
        return (translation.x, translation.y, translation.z)

    def _get_orientation(self):
        """
        Get the current orientation of base_link in the world frame.
        
        Returns:
            tuple: (x, y, z, w) quaternion or None if no transform available
        """
        if self.latest_transform is None:
            return None
            
        rotation = self.latest_transform.transform.rotation
        self._event_bus.publish(DomainEvent(EventType.ROBOT_ORIENTATION_UPDATED, rotation))
        return (rotation.x, rotation.y, rotation.z, rotation.w)
    
    def _check_robot_is_turning(self) -> bool:
        if self.latest_transform is None or self.previous_transform is None:
            return False

         # Extract timestamps (builtin_interfaces/Time)
        t_curr = self.latest_transform.header.stamp
        t_prev = self.previous_transform.header.stamp
        dt = (t_curr.sec - t_prev.sec) + 1e-9 * (t_curr.nanosec - t_prev.nanosec)

        if dt <= 0:
            return False

        # Yaw at previous and current times
        yaw_prev = self._yaw_from_quaternion(self.previous_transform.transform.rotation)
        yaw_curr = self._yaw_from_quaternion(self.latest_transform.transform.rotation)

        # Shortest angle delta and angular speed
        dyaw = self._angle_diff(yaw_curr, yaw_prev)

        yaw_rate = abs(dyaw) / dt

        is_turning = yaw_rate >= self.turning_threshold_rad_s

        return is_turning
        

    def _yaw_from_quaternion(self, q) -> float:
        """
        Extract yaw (rotation about Z) from a geometry_msgs.msg.Quaternion-like object.
        Returns yaw in radians in [-pi, pi].
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _angle_diff(self, a: float, b: float) -> float:
        """
        Smallest signed difference a - b wrapped to [-pi, pi].
        """
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d
        
        

    