"""Depth camera processing that enriches detections with world coordinates."""
from __future__ import annotations

import numpy as np
import pyrealsense2 as rs2
if not hasattr(rs2, 'intrinsics'):  # pragma: no cover - platform guard
    import pyrealsense2.pyrealsense2 as rs2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401 - required to register geometry conversions

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus


class DepthProcessor:
    """Consumes depth images and emits WORLD_COORDINATES_CALCULATED events."""

    def __init__(self, event_bus: EventBus, bridge: CvBridge) -> None:
        self._bus = event_bus
        self._bridge = bridge

        self.intrinsics = None
        self.shared_detections = None
        self.logger = None
        self.tf_buffer = None
        self.camera_frame = "oakd_rgb_camera_optical_frame"
        self.world_frame = "map"
        self.latest_object_coordinates = {}
        self.object_tracking_map = {}
        self.world_coord_tracking = {}
        self.next_tracking_id = 1000
        self.current_msg_timestamp = None

        self._bus.subscribe(EventType.OBJECT_DETECTED, self._on_object_detected)

    def handle(self, msg) -> None:
        self.current_msg_timestamp = msg.header.stamp

        try:
            if msg.encoding == "32FC1":
                depth_image_raw = self._bridge.imgmsg_to_cv2(msg, "32FC1") * 1000.0
            elif msg.encoding == "16UC1":
                depth_image_raw = self._bridge.imgmsg_to_cv2(msg, "16UC1")
            else:
                depth_image_raw = self._bridge.imgmsg_to_cv2(msg, msg.encoding)
                if np.nanmax(depth_image_raw) < 50:
                    depth_image_raw = depth_image_raw * 1000.0
        except Exception as exc:  # noqa: BLE001
            if self.logger:
                self.logger.error(f"Failed to convert depth image: {exc}")
            else:
                print(f"[ERROR] Failed to convert depth image: {exc}")
            return

        if self.shared_detections:
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
        self.intrinsics.coeffs = [value for value in camera_info.d]

    def _on_object_detected(self, event: DomainEvent) -> None:
        data = event.data
        self.shared_detections = (
            data.get('all_detections'),
            data.get('target_object'),
            data.get('selected_target_info'),
            data.get('selected_target_tracking_id'),
        )

    def set_logger(self, logger) -> None:
        self.logger = logger

    def set_tf_buffer(self, tf_buffer) -> None:
        self.tf_buffer = tf_buffer

    def get_latest_coordinates(self):
        return self.latest_object_coordinates

    def _get_or_assign_tracking_id(self, object_class, bbox_hash, world_coords):
        bbox_key = (object_class, bbox_hash)
        if bbox_key in self.object_tracking_map:
            tracking_id = self.object_tracking_map[bbox_key]
            if world_coords:
                self.world_coord_tracking[tracking_id] = world_coords
            return tracking_id

        if world_coords:
            for existing_tracking_id, last_coords in self.world_coord_tracking.items():
                if last_coords:
                    dx = world_coords[0] - last_coords[0]
                    dy = world_coords[1] - last_coords[1]
                    dz = world_coords[2] - last_coords[2]
                    distance = (dx * dx + dy * dy + dz * dz) ** 0.5
                    if distance < 0.4:
                        self.object_tracking_map[bbox_key] = existing_tracking_id
                        self.world_coord_tracking[existing_tracking_id] = world_coords
                        return existing_tracking_id

        new_tracking_id = self.next_tracking_id
        self.next_tracking_id += 1
        self.object_tracking_map[bbox_key] = new_tracking_id
        if world_coords:
            self.world_coord_tracking[new_tracking_id] = world_coords
        return new_tracking_id

    def _cleanup_old_tracking_data(self) -> None:
        if len(self.world_coord_tracking) <= 100:
            return
        sorted_ids = sorted(self.world_coord_tracking.keys())
        ids_to_remove = sorted_ids[:-50]
        for tracking_id in ids_to_remove:
            self.world_coord_tracking.pop(tracking_id, None)

        bbox_keys_to_remove = [
            bbox_key
            for bbox_key, tracking_id in self.object_tracking_map.items()
            if tracking_id in ids_to_remove
        ]
        for bbox_key in bbox_keys_to_remove:
            self.object_tracking_map.pop(bbox_key, None)

    def transform_to_world_coordinates(self, camera_x, camera_y, camera_z, timestamp):
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
                if self.logger:
                    self.logger.debug(f"TF transform failed (exact timestamp): {exc}")
                    self.logger.debug(f"TF transform failed (latest time): {exc2}")
                return None

    def _calculate_world_coordinates(self, depth_image) -> None:
        all_detections, target_object, selected_target_info, selected_target_tracking_id = self.shared_detections
        object_coordinates = {}

        for detected_object_class in all_detections:
            object_coordinates[detected_object_class] = []
            for index, detection in enumerate(all_detections[detected_object_class]):
                center_x = (detection['x1'] + detection['x2']) // 2
                center_y = (detection['y1'] + detection['y2']) // 2

                if not (0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]):
                    continue

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

                world_coords = None
                camera_coords = None

                if center_depth > 0 and self.intrinsics:
                    try:
                        depth_meters = center_depth / 1000.0
                        camera_coords = rs2.rs2_deproject_pixel_to_point(
                            self.intrinsics,
                            [center_x, center_y],
                            depth_meters,
                        )
                        world_coords = self.transform_to_world_coordinates(
                            camera_coords[0],
                            camera_coords[1],
                            camera_coords[2],
                            self.current_msg_timestamp,
                        )
                    except Exception as exc:  # noqa: BLE001
                        if self.logger:
                            self.logger.debug(f"3D coordinate calculation failed: {exc}")

                is_selected = (
                    detected_object_class == target_object
                    and selected_target_info
                    and detection == selected_target_info
                )
                bbox_hash = hash((detection['x1'], detection['y1'], detection['x2'], detection['y2']))

                if is_selected and selected_target_tracking_id:
                    tracking_id = selected_target_tracking_id
                else:
                    tracking_id = self._get_or_assign_tracking_id(
                        detected_object_class,
                        bbox_hash,
                        world_coords,
                    )

                object_info = {
                    'detection': detection,
                    'distance_mm': center_depth if center_depth > 0 else None,
                    'camera_coords': camera_coords,
                    'world_coords': world_coords,
                    'is_selected_target': is_selected,
                    'tracking_id': tracking_id,
                    'object_id': index,
                    'bbox_hash': bbox_hash,
                }
                object_coordinates[detected_object_class].append(object_info)

        self.latest_object_coordinates = object_coordinates
        self._cleanup_old_tracking_data()

        self._bus.publish_event(
            EventType.WORLD_COORDINATES_CALCULATED,
            source="DepthProcessor",
            data={'world_coordinates': object_coordinates},
        )
