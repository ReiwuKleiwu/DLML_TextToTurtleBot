from math import hypot
from typing import Dict, List, Optional, Tuple

from perception.detection.object_detector import DetectedObject

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey

from events.event_bus import EventBus
from events.interfaces.events import EventType, DomainEvent


class TargetSelector:
    """
    Handles target selection and persistence when multiple objects of the same class are detected.
    Implements strategies to prevent oscillation between targets.
    """

    def __init__(self, persistence_frames: int = 10, distance_threshold: float = 50.0):
        """
        Initialize the target selector.

        Args:
            persistence_frames: Number of frames to persist with a target before allowing switching
            distance_threshold: Maximum pixel distance for considering targets as "the same"
        """
        self._event_bus = EventBus()
        self._blackboard = Blackboard()
        self._persistence_frames = persistence_frames
        self._distance_threshold = distance_threshold
        self._current_target: Optional[DetectedObject] = None
        self._frames_with_current_target: int = 0

    def select_target(self) -> DetectedObject:
        """
        Select the best target from multiple detections of the same class.

        Args:
            detections: List of detected objects
        """
        detections_with_coordinates = self._blackboard.get(BlackboardDataKey.DETECTED_OBJECTS_WITH_COORDINATES, {})
        detections = self._blackboard.get(BlackboardDataKey.DETECTED_OBJECTS)

        target_object_class = self._blackboard.get(BlackboardDataKey.TARGET_OBJECT_CLASS)
       
        if not target_object_class:
            self._reset_target()
            self._event_bus.publish(DomainEvent(event_type=EventType.TARGET_OBJECT_SELECTED, data=None))
            return 

        target_detections = detections.get(target_object_class, [])
        if not target_detections:
            self._reset_target()
            self._event_bus.publish(DomainEvent(event_type=EventType.TARGET_OBJECT_SELECTED, data=None))
            return 

        if self._current_target is not None:

            target_detections_with_coordinates = detections_with_coordinates.get(target_object_class, [])

            if target_detections_with_coordinates:
                closest_detection, distance = self._closest_detection(target_detections_with_coordinates, self._current_target)
                if closest_detection is not None and distance <= self._distance_threshold:
                    self._current_target = closest_detection
                    self._frames_with_current_target += 1
                    self._event_bus.publish(DomainEvent(event_type=EventType.TARGET_OBJECT_SELECTED, data=closest_detection))
                    return 

            if self._frames_with_current_target < self._persistence_frames:
                self._frames_with_current_target += 1
                self._event_bus.publish(DomainEvent(event_type=EventType.TARGET_OBJECT_SELECTED, data=self._current_target))
                return 

        new_target = self._select_preferred_detection(target_detections)
        self._current_target = new_target
        self._frames_with_current_target = 0
        self._event_bus.publish(DomainEvent(event_type=EventType.TARGET_OBJECT_SELECTED, data=new_target))
        return 

    def _reset_target(self) -> None:
        self._current_target = None
        self._frames_with_current_target = 0

    def _closest_detection(
        self,
        detections: List[DetectedObject],
        reference: DetectedObject,
    ) -> Tuple[Optional[DetectedObject], float]:
        ref_cx, ref_cy = self._detection_center(reference)
        closest_detection: Optional[DetectedObject] = None
        closest_distance = float("inf")

        for detection in detections:
            cx, cy = self._detection_center(detection)
            distance = hypot(cx - ref_cx, cy - ref_cy)
            if distance < closest_distance:
                closest_detection = detection
                closest_distance = distance

        return closest_detection, closest_distance

    def _select_preferred_detection(self, detections: List[DetectedObject]) -> DetectedObject:
        return max(detections, key=self._detection_confidence)

    @staticmethod
    def _detection_center(detection: DetectedObject) -> Tuple[float, float]:
        cx = (detection.x1 + detection.x2) / 2.0
        cy = (detection.y1 + detection.y2) / 2.0
        return cx, cy

    @staticmethod
    def _detection_area(detection: DetectedObject) -> float:
        width = detection.x2 - detection.x1
        height = detection.y2 - detection.y1
        return max(width, 0) * max(height, 0)

    @staticmethod
    def _detection_confidence(detection: DetectedObject) -> float:
        return detection.confidence
