import cv2
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import rclpy

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource

class DepthCameraHandler:
    def __init__(self, bridge: CvBridge, state_machine: StateMachine):
        self.bridge = bridge
        self.state_machine = state_machine

        self.target_close_counter = 0
        self.required_frames = 5
        self.target_area_threshold = 0.30

        # Camera intrinsics for 3D coordinate calculation
        self.intrinsics = None

        # Shared detection data from CameraHandler
        self.shared_detections = None

        # For error logging (will be set if available)
        self.logger = None

        # TF buffer for coordinate transformations
        self.tf_buffer = None
        self.camera_frame = "oakd_rgb_camera_optical_frame"  # Default OAK-D camera frame
        self.world_frame = "map"  # Target world frame
    

    def handle(self, msg):
        # Store message timestamp for TF transformations
        self.current_msg_timestamp = msg.header.stamp

        # Convert ROS depth image to OpenCV format
        # For 32FC1 encoding, values are in meters (floating point)
        # For 16UC1 encoding, values are typically in millimeters (uint16)
        try:
            if msg.encoding == "32FC1":
                depth_image_raw = self.bridge.imgmsg_to_cv2(msg, "32FC1")
                # Depth values are in meters, convert to millimeters for consistency
                depth_image_raw = depth_image_raw * 1000.0
            else:
                # Try to use the original encoding
                depth_image_raw = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
                # Assume meters and convert to mm if values seem to be in meter range
                if np.nanmax(depth_image_raw) < 50:  # Likely in meters if max < 50
                    depth_image_raw = depth_image_raw * 1000.0

        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to convert depth image: {e}")
            else:
                print(f"[ERROR] Failed to convert depth image: {e}")
            return

        # Create display version - handle invalid values and normalize for visualization
        depth_image_display = np.nan_to_num(depth_image_raw, nan=0.0, posinf=0.0, neginf=0.0)
        depth_clipped = np.clip(depth_image_display, 0, 25000)  # 25 meters in mm
        depth_normalized = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX)
        depth_grayscale = depth_normalized.astype(np.uint8)
        depth_display = cv2.cvtColor(depth_grayscale, cv2.COLOR_GRAY2BGR)

        # Draw bounding boxes and calculate distances using RAW depth data
        if self.shared_detections:
            depth_display = self._draw_detections_with_depth(depth_display, depth_image_raw)

        # Resize for display
        depth_resized = cv2.resize(depth_display, (1280, 720), interpolation=cv2.INTER_NEAREST)

        # Display the depth image
        cv2.imshow("TextToTurtlebot Camera Depth", depth_resized)
        cv2.waitKey(1)

    def _analyze_depth(self, depth_image):
        """Analyze depth image to get center depth and closest object info"""
        height, width = depth_image.shape
        center_x, center_y = width // 2, height // 2
        
        # Get depth at center pixel
        center_depth = depth_image[center_y, center_x]
        if center_depth > 0:
            print(f"Center depth: {center_depth:.1f}mm")
            
            # Convert to 3D coordinates if intrinsics available
            if self.intrinsics:
                result = rs2.rs2_deproject_pixel_to_point(
                    self.intrinsics, [center_x, center_y], center_depth
                )
                print(f"Center 3D coord: {result[0]:.2f}, {result[1]:.2f}, {result[2]:.2f}")
        
        # Find closest object
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            min_depth = valid_depths.min()
            indices = np.array(np.where(depth_image == min_depth))[:, 0]
            closest_pixel = (indices[1], indices[0])
            
            print(f"Closest object at pixel({closest_pixel[0]}, {closest_pixel[1]}): {min_depth:.1f}mm")
            
            if self.intrinsics:
                result = rs2.rs2_deproject_pixel_to_point(
                    self.intrinsics, [closest_pixel[0], closest_pixel[1]], min_depth
                )
                print(f"Closest 3D coord: {result[0]:.2f}, {result[1]:.2f}, {result[2]:.2f}")

    def set_camera_intrinsics(self, camera_info):
        """Set camera intrinsics from CameraInfo message"""
        if self.intrinsics:
            return
            
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.k[2]
        self.intrinsics.ppy = camera_info.k[5]
        self.intrinsics.fx = camera_info.k[0]
        self.intrinsics.fy = camera_info.k[4]
        
        if camera_info.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif camera_info.distortion_model == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in camera_info.d]

    def set_shared_detections(self, detections_data):
        """Set detection data from CameraHandler"""
        self.shared_detections = detections_data

    def set_logger(self, logger):
        """Set logger instance for error reporting"""
        self.logger = logger

    def set_tf_buffer(self, tf_buffer):
        """Set TF buffer for coordinate transformations"""
        self.tf_buffer = tf_buffer

    def transform_to_world_coordinates(self, camera_x, camera_y, camera_z, timestamp):
        """
        Transform camera coordinates to world coordinates using TF.

        Args:
            camera_x, camera_y, camera_z: Coordinates in camera frame (meters)
            timestamp: ROS timestamp from the image message

        Returns:
            tuple: (world_x, world_y, world_z) in world frame or None if transform failed
        """
        if self.tf_buffer is None:
            return None

        try:
            # Create a point in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = timestamp
            point_stamped.point.x = camera_x
            point_stamped.point.y = camera_y
            point_stamped.point.z = camera_z

            # Transform to world frame
            world_point = self.tf_buffer.transform(point_stamped, self.world_frame)

            return (world_point.point.x, world_point.point.y, world_point.point.z)

        except Exception as e:
            if self.logger:
                self.logger.debug(f"TF transform failed: {e}")
            return None

    def _draw_detections_with_depth(self, depth_display, depth_image):
        """Draw bounding boxes on depth image and calculate 3D coordinates"""
        all_detections, target_object, selected_target_info = self.shared_detections
        
        for detected_object_class in all_detections:
            if detected_object_class in all_detections:
                for detection in all_detections[detected_object_class]:
                    # Calculate center of bounding box
                    center_x = (detection['x1'] + detection['x2']) // 2
                    center_y = (detection['y1'] + detection['y2']) // 2
                    
                    # Get depth at center pixel from RAW depth data
                    if (0 <= center_x < depth_image.shape[1] and
                        0 <= center_y < depth_image.shape[0]):
                        # Sample a small region around center for more robust depth reading
                        region_size = 5
                        y_start = max(0, center_y - region_size//2)
                        y_end = min(depth_image.shape[0], center_y + region_size//2 + 1)
                        x_start = max(0, center_x - region_size//2)
                        x_end = min(depth_image.shape[1], center_x + region_size//2 + 1)

                        depth_region = depth_image[y_start:y_end, x_start:x_end]
                        valid_depths = depth_region[depth_region > 0]

                        if len(valid_depths) > 0:
                            center_depth = np.median(valid_depths)  # Use median for robustness
                        else:
                            center_depth = depth_image[center_y, center_x]
                        
                        # Determine color based on object type
                        if (detected_object_class == target_object and 
                            selected_target_info and 
                            detection == selected_target_info):
                            color = (0, 0, 255)  # Red for selected target
                        elif detected_object_class == target_object:
                            color = (0, 165, 255)  # Orange for other targets of same class
                        else:
                            color = (0, 255, 0)  # Green for other objects
                
                        # Draw bounding box
                        cv2.rectangle(depth_display, (detection['x1'], detection['y1']), 
                                    (detection['x2'], detection['y2']), color, 1)
                        
                        # Create label with distance
                        if center_depth > 0:
                            distance_text = f"{detected_object_class}: {center_depth:.1f}mm"

                            if self.intrinsics:
                                # Calculate 3D coordinates (depth in mm, convert to meters for RealSense)
                                depth_meters = center_depth / 1000.0
                                try:
                                    # Get camera coordinates
                                    camera_coords = rs2.rs2_deproject_pixel_to_point(
                                        self.intrinsics, [center_x, center_y], depth_meters
                                    )

                                    # Transform to world coordinates
                                    world_coords = self.transform_to_world_coordinates(
                                        camera_coords[0], camera_coords[1], camera_coords[2],
                                        self.current_msg_timestamp
                                    )

                                    if world_coords:
                                        # Display world coordinates
                                        distance_text += f" W:({world_coords[0]:.2f}, {world_coords[1]:.2f}, {world_coords[2]:.2f}m)"
                                    else:
                                        # Fallback to camera coordinates
                                        distance_text += f" C:({camera_coords[0]:.2f}, {camera_coords[1]:.2f}, {camera_coords[2]:.2f}m)"

                                except Exception as e:
                                    distance_text += " (3D calc failed)"
                        else:
                            distance_text = f"{detected_object_class}: No depth"
                        
                        # Draw label
                        cv2.putText(depth_display, distance_text, 
                                  (detection['x1'], detection['y1'] - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                        
                        # Draw center point
                        cv2.circle(depth_display, (center_x, center_y), 3, color, -1)
        
        return depth_display
