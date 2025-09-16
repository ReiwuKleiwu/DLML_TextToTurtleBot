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
from classes.events import EventQueue, EventType, Event

class DepthCameraHandler:
    def __init__(self, bridge: CvBridge, state_machine: StateMachine):
        self.bridge = bridge
        self.state_machine = state_machine

        self.target_close_counter = 0
        self.required_frames = 5
        self.target_area_threshold = 0.30

        # Camera intrinsics for 3D coordinate calculation
        self.intrinsics = None

        # Shared detection data from CameraHandler (via events)
        self.shared_detections = None

        # For error logging (will be set if available)
        self.logger = None

        # TF buffer for coordinate transformations
        self.tf_buffer = None
        self.camera_frame = "oakd_rgb_camera_optical_frame"  # Default OAK-D camera frame
        self.world_frame = "map"  # Target world frame

        # Store latest calculated coordinates
        self.latest_object_coordinates = {}

        # Simple tracking system for all objects
        self.object_tracking_map = {}  # Maps (object_class, bbox_hash) -> tracking_id
        self.world_coord_tracking = {}  # Maps tracking_id -> last_known_world_coords
        self.next_tracking_id = 1000  # Start from 1000 to distinguish from TargetSelector IDs

        # Event queue for decoupled communication
        self.event_queue = EventQueue()

        # Subscribe to object detection events from CameraHandler
        self.event_queue.subscribe(EventType.OBJECT_DETECTED, self._on_object_detected)

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
            elif msg.encoding == "16UC1":
                depth_image_raw = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                # Depth values are already in millimeters
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

        # Calculate world coordinates for detected objects
        if self.shared_detections:
            self._calculate_world_coordinates(depth_image_raw)

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

    def _on_object_detected(self, event: Event):
        """Handle object detection events from CameraHandler"""
        data = event.data
        self.shared_detections = (
            data.get('all_detections'),
            data.get('target_object'),
            data.get('selected_target_info'),
            data.get('selected_target_tracking_id')
        )

    def set_logger(self, logger):
        """Set logger instance for error reporting"""
        self.logger = logger

    def set_tf_buffer(self, tf_buffer):
        """Set TF buffer for coordinate transformations"""
        self.tf_buffer = tf_buffer


    def get_latest_coordinates(self):
        """Get the latest calculated object coordinates"""
        return self.latest_object_coordinates


    def _get_or_assign_tracking_id(self, object_class, bbox_hash, world_coords):
        """Get existing tracking ID or assign new one for non-target objects"""

        # First, try exact bbox_hash match
        bbox_key = (object_class, bbox_hash)
        if bbox_key in self.object_tracking_map:
            tracking_id = self.object_tracking_map[bbox_key]
            # Update world coordinates for this tracking ID
            if world_coords:
                self.world_coord_tracking[tracking_id] = world_coords
            return tracking_id

        # If no exact match and we have world coordinates, try to match by proximity
        if world_coords:
            for existing_tracking_id, last_coords in self.world_coord_tracking.items():
                if last_coords:
                    # Calculate 3D distance
                    dx = world_coords[0] - last_coords[0]
                    dy = world_coords[1] - last_coords[1]
                    dz = world_coords[2] - last_coords[2]
                    distance = (dx*dx + dy*dy + dz*dz) ** 0.5

                    # If within 40cm, consider it the same object
                    if distance < 0.4:
                        # Update tracking map with new bbox_hash
                        self.object_tracking_map[bbox_key] = existing_tracking_id
                        self.world_coord_tracking[existing_tracking_id] = world_coords
                        return existing_tracking_id

        # No match found, assign new tracking ID
        new_tracking_id = self.next_tracking_id
        self.next_tracking_id += 1

        self.object_tracking_map[bbox_key] = new_tracking_id
        if world_coords:
            self.world_coord_tracking[new_tracking_id] = world_coords

        return new_tracking_id

    def _cleanup_old_tracking_data(self):
        """Remove old tracking data to prevent memory leaks"""
        # Keep only the most recent 100 tracking IDs to prevent unbounded growth
        if len(self.world_coord_tracking) > 100:
            # Sort by tracking_id (newer IDs are higher) and keep the latest 50
            sorted_ids = sorted(self.world_coord_tracking.keys())
            ids_to_remove = sorted_ids[:-50]

            for tracking_id in ids_to_remove:
                del self.world_coord_tracking[tracking_id]

            # Also clean up bbox tracking map
            bbox_keys_to_remove = []
            for bbox_key, tracking_id in self.object_tracking_map.items():
                if tracking_id in ids_to_remove:
                    bbox_keys_to_remove.append(bbox_key)

            for bbox_key in bbox_keys_to_remove:
                del self.object_tracking_map[bbox_key]

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

            # Transform to world frame - try with exact timestamp first
            world_point = self.tf_buffer.transform(point_stamped, self.world_frame)
            return (world_point.point.x, world_point.point.y, world_point.point.z)

        except Exception as e:
            # If exact timestamp fails, try with latest available transform
            try:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.camera_frame
                point_stamped.header.stamp = self.tf_buffer.get_latest_common_time(self.camera_frame, self.world_frame)
                point_stamped.point.x = camera_x
                point_stamped.point.y = camera_y
                point_stamped.point.z = camera_z

                world_point = self.tf_buffer.transform(point_stamped, self.world_frame)
                return (world_point.point.x, world_point.point.y, world_point.point.z)

            except Exception as e2:
                # Both attempts failed
                if self.logger:
                    self.logger.debug(f"TF transform failed (exact timestamp): {e}")
                    self.logger.debug(f"TF transform failed (latest time): {e2}")
                return None

    def _calculate_world_coordinates(self, depth_image):
        """Calculate world coordinates for all detected objects"""
        all_detections, target_object, selected_target_info, selected_target_tracking_id = self.shared_detections

        # Create object coordinate map
        object_coordinates = {}

        for detected_object_class in all_detections:
            object_coordinates[detected_object_class] = []

            for i, detection in enumerate(all_detections[detected_object_class]):
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

                    # Calculate world coordinates if depth is valid
                    world_coords = None
                    camera_coords = None

                    if center_depth > 0 and self.intrinsics:
                        try:
                            # Calculate 3D coordinates (depth in mm, convert to meters for RealSense)
                            depth_meters = center_depth / 1000.0

                            # Get camera coordinates
                            camera_coords = rs2.rs2_deproject_pixel_to_point(
                                self.intrinsics, [center_x, center_y], depth_meters
                            )

                            # Transform to world coordinates
                            world_coords = self.transform_to_world_coordinates(
                                camera_coords[0], camera_coords[1], camera_coords[2],
                                self.current_msg_timestamp
                            )

                        except Exception as e:
                            if self.logger:
                                self.logger.debug(f"3D coordinate calculation failed: {e}")

                    # Determine if this is the selected target
                    is_selected = (detected_object_class == target_object and
                                   selected_target_info and
                                   detection == selected_target_info)

                    # Calculate bbox hash for tracking
                    bbox_hash = hash((detection['x1'], detection['y1'], detection['x2'], detection['y2']))

                    # Assign tracking ID: Use TargetSelector ID for selected target, otherwise use our tracking system
                    if is_selected and selected_target_tracking_id:
                        tracking_id = selected_target_tracking_id
                    else:
                        # Use our tracking system for all non-target objects
                        tracking_id = self._get_or_assign_tracking_id(detected_object_class, bbox_hash, world_coords)

                    # Create object info with coordinates
                    object_info = {
                        'detection': detection,
                        'distance_mm': center_depth if center_depth > 0 else None,
                        'camera_coords': camera_coords,
                        'world_coords': world_coords,
                        'is_selected_target': is_selected,
                        'tracking_id': tracking_id,  # Now ALL objects have tracking IDs
                        'object_id': i,  # Unique ID for this specific object instance
                        'bbox_hash': bbox_hash  # Hash for matching
                    }

                    object_coordinates[detected_object_class].append(object_info)

        # Store coordinates in the handler
        self.latest_object_coordinates = object_coordinates

        # Cleanup old tracking data periodically
        self._cleanup_old_tracking_data()

        # Publish world coordinates via event queue
        self.event_queue.publish_event(
            EventType.WORLD_COORDINATES_CALCULATED,
            source="DepthCameraHandler",
            data={
                'world_coordinates': object_coordinates
            }
        )