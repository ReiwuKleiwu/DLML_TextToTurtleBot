from typing import Any, Dict, List, Optional

import time

from events.event_bus import EventBus
from events.interfaces.events import EventType, DomainEvent

from perception.detection.object_detector import DetectedObject, ObjectDetector
from perception.detection.target_selector import TargetSelector

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey

import cv2
from cv_bridge import CvBridge

class CameraProcessor:
    def __init__(self, bridge: CvBridge, object_detector: ObjectDetector, target_selector: TargetSelector):
        self._event_bus = EventBus()
        self._blackboard = Blackboard()
        self._bridge = bridge
        self._object_detector = object_detector
        self._target_selector = target_selector
        self._snapshot_interval_secs = 0.1
        self._last_snapshot_ts = 0.0
    
    def handle(self, msg) -> None:
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        camera_height, camera_width = cv_image.shape[:2]

        camera_resolution = self._blackboard.get(BlackboardDataKey.CAMERA_RESOLUTION)
        if not camera_resolution: 
            camera_resolution = {
                'width': camera_width,
                'height': camera_height
            }
            self._event_bus.publish(DomainEvent(EventType.CAMERA_RESOLUTION_SET, camera_resolution))
        
        self._object_detector.detect(cv_image)

        detected_objects: Dict[str, List[DetectedObject]] = self._blackboard.get(BlackboardDataKey.DETECTED_OBJECTS, {})
        detected_object_classes = set(detected_objects.keys())

        target_object_class = self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS)

        if target_object_class and target_object_class in detected_object_classes:
            self._target_selector.select_target()

        selected_target = self._blackboard.get(BlackboardDataKey.SELECTED_TARGET_OBJECT)

        detected_objects_with_coordinates: Dict[str, List[DetectedObject]] = self._blackboard.get(BlackboardDataKey.DETECTED_OBJECTS_WITH_COORDINATES, {})

        for detected_object_class in detected_object_classes:
            for i, detected_object in enumerate(detected_objects[detected_object_class]):
                if detected_object_class == target_object_class and selected_target and detected_object == selected_target:
                    color = (0, 0, 255) # Red for selected target
                elif detected_object_class == target_object_class:
                    color = (0, 255, 255) # Yellow for other targets of the same class
                else:
                    color = (0, 255, 0) # Green for other objects

                cv2.rectangle(
                    cv_image,
                    (detected_object.x1, detected_object.y1),
                    (detected_object.x2, detected_object.y2),
                    color,
                    1
                )

                label_text = f"{detected_object.name} | {detected_object.confidence:.2f}"

                # loop over detected objects with coordinates, find matching object and append coordinates to label
                detected_objects_with_coordinates_of_class = detected_objects_with_coordinates.get(detected_object_class, [])
                for obj_with_coords in detected_objects_with_coordinates_of_class:
                    if obj_with_coords == detected_object and obj_with_coords.has_world_coordinates():
                        wx = obj_with_coords.world_x
                        wy = obj_with_coords.world_y
                        label_text += f" | (X: {wx:.2f}, Y: {wy:.2f})"

                        robot_position = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)

                        if not robot_position:
                            break

                        object_distance_from_robot = obj_with_coords.distance_to_world_coordinates(robot_position.x, robot_position.y, robot_position.z)

                        label_text += f" | Dist: {object_distance_from_robot:.2f}m"
                        break

                cv2.putText(cv_image, label_text,
                            (detected_object.x1, detected_object.y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1    
                )

        self._maybe_cache_frame(cv_image)

        cv2.imshow("TextToTurtlebot Camera", cv_image)
        cv2.waitKey(1)

    def _maybe_cache_frame(self, image) -> None:
        """Store a periodic snapshot of the RGB feed on the blackboard."""
        now = time.time()
        if now - self._last_snapshot_ts < self._snapshot_interval_secs:
            return

        self._blackboard.store_camera_frame(image.copy(), timestamp=now)
        self._last_snapshot_ts = now
