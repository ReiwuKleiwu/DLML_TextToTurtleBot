import cv2
from cv_bridge import CvBridge

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.perception.ObjectDetector import ObjectDetector

class CameraHandler:
    def __init__(self, bridge: CvBridge, state_machine: StateMachine, target_object: str):
        self.target_object = target_object
        self.bridge = bridge
        self.state_machine = state_machine
        self.object_detector = ObjectDetector(model_path='models')

        self.target_close_counter = 0
        self.required_frames = 5
        self.target_area_threshold = 0.15


    def is_target_close(self, target_info, camera_width, camera_height):
        box_width = target_info['x2'] - target_info['x1']
        box_height = target_info['y2'] - target_info['y1']
        box_area = box_width * box_height
        image_area = camera_width * camera_height
        
        # if object takes up enough area of the total camera dimensions
        return box_area / image_area > self.target_area_threshold

    def handle(self, msg):
        # Get camera image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        camera_height, camera_width = cv_image.shape[:2]

        detected_objects, detected_objects_info = self.object_detector.detect(cv_image)

        for detected_object_class in detected_objects:
            detected_object_info = detected_objects_info[detected_object_class]

            color = (0, 0, 255) if (detected_object_class == self.target_object) else (0, 255, 0)

            cv2.rectangle(cv_image, (detected_object_info['x1'], detected_object_info['y1']), (detected_object_info['x2'], detected_object_info['y2']), color, 1)
            cv2.putText(cv_image, detected_object_class, (detected_object_info['x1'], detected_object_info['y1'] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        cv2.imshow('TextToTurtlebot Camera', cv_image)
        cv2.waitKey(1)

        if self.target_object not in detected_objects and self.state_machine.get_current_state().value == TurtleBotState.OBJECT_FOUND: 
            self.target_close_counter = 0
            self.state_machine.pop_state(TurtleBotState.OBJECT_FOUND, TurtleBotStateSource.CAMERA)
        
        if self.target_object not in detected_objects: return

        target_info = detected_objects_info[self.target_object]

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
            self.state_machine.push_state(
                TurtleBotState.OBJECT_REACHED,
                TurtleBotStateSource.CAMERA,
                data=target_state_data
            )
        elif self.state_machine.get_current_state().value == TurtleBotState.OBJECT_FOUND:
            self.state_machine.get_current_state().set_data(target_state_data)
        else:
            # Objekt wurde erstmals gesehen -> OBJECT_FOUND
            self.state_machine.push_state(
                TurtleBotState.OBJECT_FOUND,
                TurtleBotStateSource.CAMERA,
                data=target_state_data
            )
