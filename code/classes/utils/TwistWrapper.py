from geometry_msgs.msg import Twist, TwistStamped
from builtin_interfaces.msg import Time
import rclpy


class TwistWrapper:
    """
    A wrapper class that provides a unified interface for both Twist and TwistStamped messages.
    This allows behavior classes to use the same API regardless of whether they're working with
    a physical TurtleBot (Twist) or the TurtleBot simulator (TwistStamped).
    """
    
    def __init__(self, use_stamped: bool = False):
        """
        Initialize the TwistWrapper.
        
        Args:
            use_stamped: If True, use TwistStamped (for simulator), otherwise use Twist (for physical robot)
        """
        self.use_stamped = use_stamped
        
        if use_stamped:
            self._msg = TwistStamped()
            # Set the header timestamp for TwistStamped
            self._msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self._msg.header.frame_id = "base_link"
        else:
            self._msg = Twist()
    
    @property
    def linear(self):
        """
        Get the linear velocity component.
        Returns the appropriate linear object based on message type.
        """
        if self.use_stamped:
            return self._msg.twist.linear
        else:
            return self._msg.linear
    
    @property
    def angular(self):
        """
        Get the angular velocity component.
        Returns the appropriate angular object based on message type.
        """
        if self.use_stamped:
            return self._msg.twist.angular
        else:
            return self._msg.angular
    
    def get_message(self):
        """
        Get the underlying ROS message for publishing.
        Updates timestamp for TwistStamped messages.
        
        Returns:
            The underlying Twist or TwistStamped message ready for publishing
        """
        if self.use_stamped:
            # Update timestamp for each publish
            self._msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        
        return self._msg
    
    def reset(self):
        """
        Reset all velocity components to zero.
        """
        self.linear.x = 0.0
        self.linear.y = 0.0
        self.linear.z = 0.0
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = 0.0