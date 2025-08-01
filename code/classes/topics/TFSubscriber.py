import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
import rclpy


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
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        
        # Store the latest transform
        self.latest_transform = None
        
        # Create a timer to periodically get transforms
        self.timer = self.node.create_timer(0.1, self.update_transform)
    
    
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
            self.latest_transform = transform
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Transform not available yet, this is normal during startup
            pass
    
    def get_position(self):
        """
        Get the current position of base_link in the world frame.
        
        Returns:
            tuple: (x, y, z) position or None if no transform available
        """
        if self.latest_transform is None:
            return None
            
        translation = self.latest_transform.transform.translation
        return (translation.x, translation.y, translation.z)
    
    def get_orientation(self):
        """
        Get the current orientation of base_link in the world frame.
        
        Returns:
            tuple: (x, y, z, w) quaternion or None if no transform available
        """
        if self.latest_transform is None:
            return None
            
        rotation = self.latest_transform.transform.rotation
        return (rotation.x, rotation.y, rotation.z, rotation.w)
    
    # def get_transform(self):
    #     """
    #     Get the latest transform.
        
    #     Returns:
    #         TransformStamped: The latest transform or None if not available
    #     """
    #     return self.latest_transform
    
    # def get_reference_frame(self):
    #     """
    #     Get the reference frame (world frame) for the current absolute position.
        
    #     Returns:
    #         str: The reference frame ID or None if no transform available
    #     """
    #     if self.latest_transform is None:
    #         return None
    #     return self.latest_transform.header.frame_id
    
    # def is_transform_available(self):
    #     """
    #     Check if a transform is currently available.
        
    #     Returns:
    #         bool: True if transform is available, False otherwise
    #     """
    #     return self.latest_transform is not None

