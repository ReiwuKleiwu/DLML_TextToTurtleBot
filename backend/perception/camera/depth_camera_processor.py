from typing import List, Dict

import numpy as np
from numpy import ndarray
from shared.blackboard.blackboard import Blackboard
from cv_bridge import CvBridge
from shared.events.event_bus import EventBus
from shared.events.interfaces.events import EventType, DomainEvent
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
import pyrealsense2 as rs2
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401 - registers geometry conversions for tf transforms

from backend.perception.detection.object_detector import DetectedObject

class DepthCameraProcessor():
    def __init__(self, bridge: CvBridge, tf_buffer):
        self._event_bus = EventBus()
        self._blackboard = Blackboard()
        self._bridge = bridge
        self.intrinsics = None
        self.tf_buffer = tf_buffer
        self.current_msg_timestamp = None
        self.camera_frame = "oakd_rgb_camera_optical_frame"
        self.world_frame = "map"

    def handle(self, msg) -> None: 
        self.current_msg_timestamp = msg.header.stamp

        if msg.encoding == '32FC1':
            depth_image_raw = self._bridge.imgmsg_to_cv2(msg, '32FC1') * 1000.0
        elif msg.encoding == '16UC1':
            depth_image_raw = self._bridge.imgmsg_to_cv2(msg, '16UC1')
        else: 
            depth_image_raw = self._bridge.imgmsg_to_cv2(msg, msg.encoding)
            if np.nanmax(depth_image_raw) < 50:
                depth_image_raw = depth_image_raw * 1000.0

        detected_objects = self._blackboard.get(BlackboardDataKey.DETECTED_OBJECTS, {})

        if detected_objects is None:
            return
        
        self._calculate_world_coordinates(depth_image_raw)
        
    def set_camera_intrinsics(self, camera_info) -> None:
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
        self.intrinsics.coeffs = [value for value in camera_info.d[:5]]
        
    def _calculate_world_coordinates(self, depth_image: ndarray) -> None:     
        detected_objects: Dict[str, List[DetectedObject]] = self._blackboard.get(BlackboardDataKey.DETECTED_OBJECTS, {})
        detected_object_classes = set(detected_objects.keys())
        detected_objects_with_coordinates = {}

        if not self.intrinsics:
            return

        for detected_object_class in detected_object_classes:
            for i, detected_object in enumerate(detected_objects[detected_object_class]):
                # Bounding boxes are stored as top-left/bottom-right pixel coordinates
                center_x = (detected_object.x1 + detected_object.x2) // 2
                center_y = (detected_object.y1 + detected_object.y2) // 2

                if not (0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]):
                    continue
                
                # I'll make this a class parameter later, promised ;)
                region_size = 5
                y_start = max(0, center_y - region_size // 2)
                y_end = min(depth_image.shape[0], center_y + region_size // 2 + 1)
                x_start = max(0, center_x - region_size // 2)
                x_end = min(depth_image.shape[1], center_x + region_size // 2 + 1)

                depth_region = depth_image[y_start:y_end, x_start:x_end]
                valid_depths = depth_region[depth_region > 0]

                if len(valid_depths) > 0:
                    center_depth = np.median(valid_depths)
                else:
                    center_depth = depth_image[center_y, center_x]

                if center_depth == 0:
                    continue
                
                depth_meters = center_depth / 1000.0
                camera_coords = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center_x, center_y], depth_meters)
                world_coords = self._transform_to_world_coordinates(
                    camera_coords[0],
                    camera_coords[1],
                    camera_coords[2],
                    self.current_msg_timestamp,
                )

                if world_coords is None:
                    continue

                detected_object.world_x = world_coords[0]
                detected_object.world_y = world_coords[1]
                detected_object.world_z = world_coords[2]

                if detected_object_class not in detected_objects_with_coordinates:
                    detected_objects_with_coordinates[detected_object_class] = []

                detected_objects_with_coordinates[detected_object_class].append(detected_object)

        self._event_bus.publish(DomainEvent(EventType.OBJECT_WORLD_COORDINATES_UPDATED, detected_objects_with_coordinates))
    
    def _transform_to_world_coordinates(self, camera_x: float, camera_y: float, camera_z: float, timestamp):
        if self.tf_buffer is None:
            return None
        
        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.header.stamp = timestamp
        point_stamped.point.x = camera_x
        point_stamped.point.y = camera_y
        point_stamped.point.z = camera_z
    
        try:
            world_point = self.tf_buffer.transform(point_stamped, self.world_frame)
            return (world_point.point.x, world_point.point.y, world_point.point.z)
        except Exception as exc:  # noqa: BLE001
            try:
                point_stamped.header.stamp = self.tf_buffer.get_latest_common_time(
                    self.camera_frame,
                    self.world_frame,
                )
                world_point = self.tf_buffer.transform(point_stamped, self.world_frame)
                return (world_point.point.x, world_point.point.y, world_point.point.z)
            except Exception as exc2:  # noqa: BLE001
                return None
        



  
