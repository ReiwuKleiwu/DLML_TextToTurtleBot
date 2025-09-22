"""ROS camera subscriber adapter that raises application events."""
from __future__ import annotations

import threading
from typing import Any, Dict, List, Optional

import cv2
from cv_bridge import CvBridge

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus
from code.infrastructure.perception.object_detector import ObjectDetector
from code.infrastructure.perception.target_selector import TargetSelector


class CameraProcessor:
    """Wraps the RGB camera stream, detector, and target selection logic."""

    def __init__(
        self,
        event_bus: EventBus,
        bridge: CvBridge,
        model_path: str,
        persistence_frames: int = 15,
        distance_threshold: float = 75.0,
    ) -> None:
        self._bus = event_bus
        self._bridge = bridge
        self._detector = ObjectDetector(model_path=model_path)
        self._target_selector = TargetSelector(
            event_bus=event_bus,
            persistence_frames=persistence_frames,
            distance_threshold=distance_threshold,
        )

        self._target_object: Optional[str] = None
        self._target_stack: List[str] = []
        self._persistent_objects_map: Dict[str, Any] = {}
        self._frame_lock = threading.Lock()
        self._latest_frame_jpeg: Optional[bytes] = None

        self._bus.subscribe(EventType.SENSOR_DATA_UPDATED, self._on_map_updated)
        self._bus.subscribe(EventType.TARGET_REACHED, self._on_target_reached)
        self._bus.subscribe(EventType.STATE_PUSHED, self._on_state_pushed)
        self._bus.subscribe(EventType.STATE_POPPED, self._on_state_popped)

    def set_target_object(self, target: Optional[str]) -> None:
        if target != self._target_object:
            self._target_object = target
            self._target_selector.reset()

    def handle(self, msg) -> None:
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        camera_height, camera_width = cv_image.shape[:2]

        detected_objects, _, all_detections = self._detector.detect(cv_image)

        selected_target_info = None
        if self._target_object and self._target_object in all_detections:
            selected_target_info = self._target_selector.select_target(
                self._target_object,
                all_detections[self._target_object],
            )

        selected_target_tracking_id = None
        if selected_target_info:
            selected_target_tracking_id = self._target_selector.get_current_tracking_id()

        self._bus.publish_event(
            EventType.OBJECT_DETECTED,
            source="CameraProcessor",
            data={
                'all_detections': all_detections,
                'target_object': self._target_object,
                'selected_target_info': selected_target_info,
                'selected_target_tracking_id': selected_target_tracking_id,
                'camera_width': camera_width,
                'camera_height': camera_height,
            }
        )

        self._draw_visualizations(
            cv_image,
            detected_objects,
            all_detections,
            selected_target_info,
        )

        success, buffer = cv2.imencode('.jpg', cv_image)
        if success:
            with self._frame_lock:
                self._latest_frame_jpeg = buffer.tobytes()

    def _draw_visualizations(
        self,
        cv_image,
        detected_objects,
        all_detections,
        selected_target_info,
    ) -> None:
        for detected_object_class in detected_objects:
            if detected_object_class not in all_detections:
                continue
            for detection in all_detections[detected_object_class]:
                color = (0, 255, 0)
                if detected_object_class == self._target_object:
                    if selected_target_info and detection == selected_target_info:
                        color = (0, 0, 255)
                    else:
                        color = (0, 165, 255)

                cv2.rectangle(
                    cv_image,
                    (detection['x1'], detection['y1']),
                    (detection['x2'], detection['y2']),
                    color,
                    1,
                )

                label_text = detected_object_class

                if detected_object_class in self._persistent_objects_map:
                    persistent_objects = self._persistent_objects_map[detected_object_class]
                    if persistent_objects:
                        most_recent_obj = max(
                            persistent_objects,
                            key=lambda obj: obj.get('last_seen', 0),
                        )
                        if 'world_coords' in most_recent_obj and most_recent_obj['world_coords']:
                            wx, wy, wz = most_recent_obj['world_coords']
                            label_text += f" World:({wx:.2f}, {wy:.2f}, {wz:.2f}m)"
                            if 'total_detections' in most_recent_obj:
                                detection_count = most_recent_obj['total_detections']
                                label_text += f" [{detection_count}x]"

                cv2.putText(
                    cv_image,
                    label_text,
                    (detection['x1'], detection['y1'] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    color,
                    1,
                )

                if (
                    detected_object_class == self._target_object
                    and selected_target_info
                    and detection == selected_target_info
                ):
                    center_x = (detection['x1'] + detection['x2']) // 2
                    center_y = (detection['y1'] + detection['y2']) // 2
                    cv2.circle(cv_image, (center_x, center_y), 5, color, -1)

    def _on_map_updated(self, event: DomainEvent) -> None:
        self._persistent_objects_map = event.data.get('persistent_objects_map', {})

    def _on_target_reached(self, event: DomainEvent) -> None:
        self._target_selector.reset()
        if self._target_stack:
            self._target_object = self._target_stack[-1]
        else:
            self._target_object = None

    def reset_targets(self) -> None:
        """Clear any queued targets and reset selector state."""
        self._target_stack.clear()
        self._target_object = None
        self._target_selector.reset()

    def get_latest_frame(self) -> Optional[bytes]:
        """Return the most recent camera frame encoded as JPEG bytes."""
        with self._frame_lock:
            return self._latest_frame_jpeg

    def _on_state_pushed(self, event: DomainEvent) -> None:
        if event.source != "StateMachine":
            return

        if event.data.get('new_state') != 'EXPLORE':
            return

        state_data = event.data.get('state_data') or {}
        target = state_data.get('target_object')
        if not target:
            return

        self._target_stack.append(target)
        self.set_target_object(target)

    def _on_state_popped(self, event: DomainEvent) -> None:
        if event.source != "StateMachine":
            return

        if event.data.get('popped_state') != 'EXPLORE':
            return

        if self._target_stack:
            self._target_stack.pop()

        new_target = self._target_stack[-1] if self._target_stack else None
        self.set_target_object(new_target)
