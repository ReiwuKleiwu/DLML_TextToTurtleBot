import math
from typing import Tuple, Optional, Dict, Any
from sensor_msgs.msg import LaserScan
from classes.topics.TFSubscriber import TFSubscriber


class ObjectPositionEstimator:
    def __init__(self, tf_subscriber: TFSubscriber, camera_frame: str = "oakd_rgb_camera_optical_frame"):
        """
        Initialize the 2D position estimator.
        
        Args:
            tf_subscriber: TF subscriber to get robot and camera poses
            camera_frame: Frame ID of the camera for precise positioning
        """
        self.tf_subscriber = tf_subscriber
        self.camera_frame = camera_frame
        
        # Camera parameters (these should ideally be read from camera_info topic)
        # For OAK-D Pro camera, typical values:
        self.camera_fov_horizontal = math.radians(69.4)  # Horizontal field of view in radians
        
        # Camera dimensions will be updated dynamically from actual camera data
        self.image_width = None
        self.image_height = None
        
        # Cache for latest LIDAR data
        self.latest_lidar_data = None
        
    def update_camera_params(self, width: int, height: int, fov_h: float = None):
        """
        Update camera parameters with actual values from camera stream.
        
        Args:
            width: Image width in pixels
            height: Image height in pixels
            fov_h: Horizontal field of view in radians (optional)
        """
        self.image_width = width
        self.image_height = height
        if fov_h is not None:
            self.camera_fov_horizontal = fov_h
    
    def update_lidar_data(self, lidar_msg: LaserScan):
        """
        Update the cached LIDAR data.
        
        Args:
            lidar_msg: Latest LIDAR scan message
        """
        self.latest_lidar_data = lidar_msg
    
    def pixel_to_angle(self, pixel_x: float, pixel_y: float) -> float:
        """
        Convert pixel coordinates to horizontal angular coordinate relative to camera center.
        
        Args:
            pixel_x: X coordinate in pixels
            pixel_y: Y coordinate in pixels (not used in 2D estimation)
            
        Returns:
            Horizontal angle in radians
            Positive angle = right, negative = left
        """
        if self.image_width is None:
            raise ValueError("Camera dimensions not set. Call update_camera_params() first.")
        
        # Convert pixel coordinates to normalized coordinates [-1, 1]
        norm_x = (pixel_x - self.image_width / 2) / (self.image_width / 2)
        
        # Convert to angular coordinates
        horizontal_angle = norm_x * (self.camera_fov_horizontal / 2)
        
        return horizontal_angle
    
    def get_object_center_pixel(self, bounding_box: Dict[str, int]) -> Tuple[float, float]:
        """
        Calculate the center pixel coordinates from a bounding box.
        
        Args:
            bounding_box: Dictionary with keys 'x1', 'y1', 'x2', 'y2'
            
        Returns:
            Tuple of (center_x, center_y) in pixels
        """
        center_x = (bounding_box['x1'] + bounding_box['x2']) / 2
        center_y = (bounding_box['y1'] + bounding_box['y2']) / 2
        return center_x, center_y
    
    def get_lidar_distance_at_angle(self, target_angle: float) -> Optional[float]:
        """
        Get LIDAR distance measurement at a specific angle.
        
        Args:
            target_angle: Angle in radians (0 = forward, positive = counter-clockwise)
            
        Returns:
            Distance in meters, or None if no valid measurement
        """
        if self.latest_lidar_data is None:
            return None
            
        # Calculate the index in the LIDAR array for the target angle
        angle_min = self.latest_lidar_data.angle_min
        angle_increment = self.latest_lidar_data.angle_increment
        
        # Find the closest index to our target angle
        target_index = round((target_angle - angle_min) / angle_increment)
        
        # Ensure index is within bounds
        if 0 <= target_index < len(self.latest_lidar_data.ranges):
            distance = self.latest_lidar_data.ranges[target_index]
            # Filter out invalid readings
            if not math.isinf(distance) and not math.isnan(distance) and distance > 0:
                return distance
        
        return None
    
    def estimate_object_position(self, bounding_box: Dict[str, int], robot_position: Tuple[float, float],
                               robot_orientation: Tuple[float, float, float, float]) -> Optional[Tuple[float, float]]:
        """
        Estimate the 2D world position of an object using camera and LIDAR data.
        
        Args:
            bounding_box: Object bounding box with keys 'x1', 'y1', 'x2', 'y2'
            robot_position: Robot position (x, y) in world frame
            robot_orientation: Robot orientation quaternion (x, y, z, w) in world frame
            
        Returns:
            Estimated object position (x, y) in world frame, or None if estimation failed
        """
        # Get object center in pixel coordinates
        center_x, center_y = self.get_object_center_pixel(bounding_box)
        
        # Convert to horizontal angular coordinate relative to camera
        horizontal_angle = self.pixel_to_angle(center_x, center_y)
        
        # Convert robot orientation quaternion to yaw angle
        robot_yaw = self.quaternion_to_yaw(robot_orientation)
        
        # Calculate the absolute angle of the object in world frame
        # Note: We assume camera is aligned with robot's front (same orientation)
        object_angle_world = robot_yaw + horizontal_angle
        
        # Get distance from LIDAR at this angle
        distance = self.get_lidar_distance_at_angle(horizontal_angle)
        
        if distance is None:
            return None
        
        # Calculate object position in 2D world coordinates
        object_x = robot_position[0] + distance * math.cos(object_angle_world)
        object_y = robot_position[1] + distance * math.sin(object_angle_world)
        
        return (object_x, object_y)
    
    def estimate_object_position_with_camera_tf(self, bounding_box: Dict[str, int]) -> Optional[Tuple[float, float]]:
        """
        Estimate 2D object position using camera's TF frame for higher precision.
        
        Args:
            bounding_box: Object bounding box with keys 'x1', 'y1', 'x2', 'y2'
            
        Returns:
            Estimated object position (x, y) in world frame, or None if estimation failed
        """
        # Try to get camera frame transform for more precise positioning
        try:
            camera_transform = self.tf_subscriber.tf_buffer.lookup_transform(
                self.tf_subscriber.world_frame,
                self.camera_frame,
                self.tf_subscriber.node.get_clock().now()
            )
            
            camera_position = (
                camera_transform.transform.translation.x,
                camera_transform.transform.translation.y
            )
            camera_orientation = (
                camera_transform.transform.rotation.x,
                camera_transform.transform.rotation.y,
                camera_transform.transform.rotation.z,
                camera_transform.transform.rotation.w
            )
            
            return self.estimate_object_position(bounding_box, camera_position, camera_orientation)
            
        except Exception:
            # Fallback to robot position if camera frame is not available
            robot_position = self.tf_subscriber.get_position()
            robot_orientation = self.tf_subscriber.get_orientation()
            
            if robot_position is None or robot_orientation is None:
                return None
            
            # Extract only x, y from robot position
            robot_position_2d = (robot_position[0], robot_position[1])
            return self.estimate_object_position(bounding_box, robot_position_2d, robot_orientation)
    
    @staticmethod
    def quaternion_to_yaw(quaternion: Tuple[float, float, float, float]) -> float:
        """
        Convert quaternion to yaw angle.
        
        Args:
            quaternion: Quaternion (x, y, z, w)
            
        Returns:
            Yaw angle in radians
        """
        x, y, z, w = quaternion
        
        # Yaw calculation from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def get_angular_precision_info(self) -> Dict[str, float]:
        """
        Get information about the angular precision of the system.
        
        Returns:
            Dictionary with precision metrics
        """
        if self.image_width is None:
            return {
                'camera_horizontal_resolution_rad_per_pixel': None,
                'camera_horizontal_resolution_deg_per_pixel': None,
                'lidar_angular_resolution_rad': None,
                'lidar_angular_resolution_deg': None,
                'error': 'Camera dimensions not set'
            }
        
        # Calculate angular resolution per pixel
        horizontal_res = self.camera_fov_horizontal / self.image_width
        
        # LIDAR angular resolution
        lidar_res = None
        if self.latest_lidar_data is not None:
            lidar_res = self.latest_lidar_data.angle_increment
        
        return {
            'camera_horizontal_resolution_rad_per_pixel': horizontal_res,
            'camera_horizontal_resolution_deg_per_pixel': math.degrees(horizontal_res),
            'lidar_angular_resolution_rad': lidar_res,
            'lidar_angular_resolution_deg': math.degrees(lidar_res) if lidar_res else None
        }
