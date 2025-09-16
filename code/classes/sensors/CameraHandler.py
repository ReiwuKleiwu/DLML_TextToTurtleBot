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

        # Store persistent object data from map service
        self.persistent_objects_map = {}

        # Subscribe to map service events for persistent object data
        self.event_queue.subscribe(EventType.SENSOR_DATA_UPDATED, self._on_map_updated)

        # Subscribe to target reached events
        self.event_queue.subscribe(EventType.TARGET_REACHED, self._on_target_reached)


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

    def _on_map_updated(self, event: Event):
        """Handle map service update events with persistent object data"""
        self.persistent_objects_map = event.data.get('persistent_objects_map', {})

    def _on_target_reached(self, event: Event):
        """Handle target reached events from TargetReachedService"""
        self.target_object = None
        self.target_selector.reset()

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

        # Get tracking ID for the selected target
        selected_target_tracking_id = None
        if selected_target_info:
            selected_target_tracking_id = self.target_selector.get_current_tracking_id()

        # Publish detection data via event queue for depth camera handler to consume
        self.event_queue.publish_event(
            EventType.OBJECT_DETECTED,
            source="CameraHandler",
            data={
                'all_detections': all_detections,
                'target_object': self.target_object,
                'selected_target_info': selected_target_info,
                'selected_target_tracking_id': selected_target_tracking_id,
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

                    # Add world coordinates and tracking info from map service
                    if detected_object_class in self.persistent_objects_map:
                        persistent_objects = self.persistent_objects_map[detected_object_class]

                        # Find the closest persistent object to current detection center
                        detection_center_x = (detection['x1'] + detection['x2']) // 2
                        detection_center_y = (detection['y1'] + detection['y2']) // 2

                        if persistent_objects:
                            # Use the most recently seen object for better matching
                            most_recent_obj = max(persistent_objects, key=lambda obj: obj.get('last_seen', 0))

                            if 'world_coords' in most_recent_obj and most_recent_obj['world_coords']:
                                wx, wy, wz = most_recent_obj['world_coords']
                                label_text += f" World:({wx:.2f}, {wy:.2f}, {wz:.2f}m)"

                                # Add tracking statistics for debugging
                                if 'total_detections' in most_recent_obj:
                                    detection_count = most_recent_obj['total_detections']
                                    label_text += f" [{detection_count}x]"

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
        
        # COMMENTED OUT: Object reached detection now handled by TargetReachedService based on world coordinates
        # if self.target_close_counter >= self.required_frames:
        #     # Objekt wurde für ausreichend Frames als "nah" eingestuft -> OBJECT_REACHED
        #     self.state_machine.pop_state(
        #         TurtleBotState.OBJECT_FOUND,
        #         TurtleBotStateSource.CAMERA
        #     )
        #     self.state_machine.pop_state(
        #         TurtleBotState.EXPLORE,
        #         TurtleBotStateSource.USER
        #     )
        #     self.target_object = None
        #     self.target_selector.reset()
        # elif self.state_machine.get_current_state().value == TurtleBotState.OBJECT_FOUND:

        if self.state_machine.get_current_state().value == TurtleBotState.OBJECT_FOUND:
            self.state_machine.get_current_state().set_data(target_state_data)
        else:
            # Objekt wurde erstmals gesehen -> OBJECT_FOUND
            self.state_machine.push_state(
                TurtleBotState.OBJECT_FOUND,
                TurtleBotStateSource.CAMERA,
                data=target_state_data
            )
