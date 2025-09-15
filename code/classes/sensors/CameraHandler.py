import cv2
from cv_bridge import CvBridge

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.perception.ObjectDetector import ObjectDetector
from classes.utils.TargetSelector import TargetSelector
from classes.events import EventQueue, EventType, Event

class CameraHandler:
    def __init__(self, bridge: CvBridge, state_machine: StateMachine):
        self.target_object = None
        self.bridge = bridge
        self.state_machine = state_machine
        self.object_detector = ObjectDetector(model_path='models')
        self.target_selector = TargetSelector(persistence_frames=15, distance_threshold=75.0)

        self.target_close_counter = 0
        self.required_frames = 5
        self.target_area_threshold = 0.30

        # Event queue for decoupled communication
        self.event_queue = EventQueue()

        # Store world coordinates from depth handler events
        self.world_coordinates = {}

        # Subscribe to world coordinates events
        self.event_queue.subscribe(EventType.WORLD_COORDINATES_CALCULATED, self._on_world_coordinates_updated)


    def is_target_close(self, target_info, camera_width, camera_height):
        box_width = target_info['x2'] - target_info['x1']
        box_height = target_info['y2'] - target_info['y1']
        box_area = box_width * box_height
        image_area = camera_width * camera_height
        
        # if object takes up enough area of the total camera dimensions
        return box_area / image_area > self.target_area_threshold
    

    def get_target_object(self):
        target_object = self.state_machine.get_current_state().data.get("target_object", None)
        if target_object is not None and target_object != self.target_object:
            # Target changed, reset the selector
            self.target_object = target_object
            self.target_selector.reset()

    def _on_world_coordinates_updated(self, event: Event):
        """Handle world coordinates update events from depth handler"""
        self.world_coordinates = event.data.get('world_coordinates', {})

    def handle(self, msg):
        self.get_target_object()

        # Get camera image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        camera_height, camera_width = cv_image.shape[:2]

        detected_objects, detected_objects_info, all_detections = self.object_detector.detect(cv_image)

        # Get the selected target using the target selector
        selected_target_info = None
        if self.target_object and self.target_object in all_detections:
            selected_target_info = self.target_selector.select_target(
                self.target_object, 
                all_detections[self.target_object]
            )
        
        # Publish detection data via event queue for depth camera handler to consume
        self.event_queue.publish_event(
            EventType.OBJECT_DETECTED,
            source="CameraHandler",
            data={
                'all_detections': all_detections,
                'target_object': self.target_object,
                'selected_target_info': selected_target_info,
                'camera_width': camera_width,
                'camera_height': camera_height
            }
        )

        # Draw all detected objects with world coordinates
        for detected_object_class in detected_objects:
            if detected_object_class in all_detections:
                for i, detection in enumerate(all_detections[detected_object_class]):
                    # Highlight the selected target differently
                    if (detected_object_class == self.target_object and
                        selected_target_info and
                        detection == selected_target_info):
                        color = (0, 0, 255)  # Red for selected target
                    elif detected_object_class == self.target_object:
                        color = (0, 165, 255)  # Orange for other targets of same class
                    else:
                        color = (0, 255, 0)  # Green for other objects

                    cv2.rectangle(cv_image, (detection['x1'], detection['y1']),
                                (detection['x2'], detection['y2']), color, 1)

                    # Create label with class name and world coordinates
                    label_text = detected_object_class

                    # Add world coordinates if available - match by bounding box coordinates
                    if detected_object_class in self.world_coordinates:
                        # Find matching object by comparing bounding box hash
                        matching_obj = None
                        detection_hash = hash((detection['x1'], detection['y1'], detection['x2'], detection['y2']))

                        for obj_info in self.world_coordinates[detected_object_class]:
                            if 'bbox_hash' in obj_info and obj_info['bbox_hash'] == detection_hash:
                                matching_obj = obj_info
                                break
                            # Fallback to coordinate comparison for backwards compatibility
                            elif (obj_info['detection']['x1'] == detection['x1'] and
                                  obj_info['detection']['y1'] == detection['y1'] and
                                  obj_info['detection']['x2'] == detection['x2'] and
                                  obj_info['detection']['y2'] == detection['y2']):
                                matching_obj = obj_info
                                break

                        if matching_obj:
                            if matching_obj['world_coords']:
                                wx, wy, wz = matching_obj['world_coords']
                                label_text += f" W:({wx:.2f}, {wy:.2f}, {wz:.2f}m)"
                            elif matching_obj['distance_mm']:
                                label_text += f" {matching_obj['distance_mm']:.0f}mm"

                            # Add object ID for debugging multiple objects of same class
                            if 'object_id' in matching_obj:
                                label_text += f" #{matching_obj['object_id']}"

                    # Draw label
                    cv2.putText(cv_image, label_text,
                              (detection['x1'], detection['y1'] - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                    # Draw center point for selected target
                    if (detected_object_class == self.target_object and
                        selected_target_info and
                        detection == selected_target_info):
                        center_x = (detection['x1'] + detection['x2']) // 2
                        center_y = (detection['y1'] + detection['y2']) // 2
                        cv2.circle(cv_image, (center_x, center_y), 5, color, -1)

        cv2.imshow('TextToTurtlebot Camera', cv_image)
        cv2.waitKey(1)

        # Check if we have a selected target
        if not selected_target_info:
            if self.state_machine.get_current_state().value == TurtleBotState.OBJECT_FOUND: 
                self.target_close_counter = 0
                self.state_machine.pop_state(TurtleBotState.OBJECT_FOUND, TurtleBotStateSource.CAMERA)
            return

        target_info = selected_target_info

        if self.is_target_close(target_info, camera_width, camera_height):
            self.target_close_counter += 1
        else:
            self.target_close_counter = 0

        target_state_data = {
                "object_found_data": {
                    "camera": {
                        "width": camera_width,
                        "height": camera_height
                    },
                    "class": self.target_object,
                    "bounding_box_coordinates": {
                        "x1": target_info['x1'],
                        "y1": target_info['y1'],
                        "x2": target_info['x2'],
                        "y2": target_info['y2']
                    }   
                }
            }
        
        if self.target_close_counter >= self.required_frames:
            # Objekt wurde für ausreichend Frames als "nah" eingestuft -> OBJECT_REACHED
            self.state_machine.pop_state(
                TurtleBotState.OBJECT_FOUND,
                TurtleBotStateSource.CAMERA
            )
            self.state_machine.pop_state(
                TurtleBotState.EXPLORE,
                TurtleBotStateSource.USER
            )
            self.target_object = None
            self.target_selector.reset()
        elif self.state_machine.get_current_state().value == TurtleBotState.OBJECT_FOUND:
            self.state_machine.get_current_state().set_data(target_state_data)
        else:
            # Objekt wurde erstmals gesehen -> OBJECT_FOUND
            self.state_machine.push_state(
                TurtleBotState.OBJECT_FOUND,
                TurtleBotStateSource.CAMERA,
                data=target_state_data
            )
