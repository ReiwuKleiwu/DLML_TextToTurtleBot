import cv2

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.perception.ObjectDetector import ObjectDetector

class CameraHandler:
    def __init__(self, bridge, state_machine: StateMachine, target_object: str):
        self.target_object = target_object
        self.bridge = bridge
        self.state_machine = state_machine
        self.object_detector = ObjectDetector(model_path='yolov8s-worldv2.pt')

    def handle(self, msg):
        # Get camera image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected_objects, detected_objects_info = self.object_detector.detect(cv_image)

        for detected_object_class in detected_objects:
            detected_object_info = detected_objects_info[detected_object_class]

            color = (0, 0, 255) if (detected_object_class == self.target_object) else (0, 255, 0)

            cv2.rectangle(cv_image, (detected_object_info['x1'], detected_object_info['y1']), (detected_objects_info['x2'], detected_objects_info['y2']), color, 2)
            cv2.putText(cv_image, detected_object_class, (detected_object_info['x1'], detected_object_info['y1'] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow('TextToTurtlebot Camera', cv_image)
        cv2.waitKey(1)

        if self.target_object not in detected_objects: return

        target_info = detected_objects_info[self.target_object]

        self.state_machine.push_state(
            TurtleBotState.OBJECT_FOUND,
            TurtleBotStateSource.CAMERA,
            data= {
                "object": self.target_object,
                "coordinates": {
                    "x1": target_info['x1'],
                    "y1": target_info['y1'],
                    "x2": target_info['x2'],
                    "y2": target_info['y2']
                }
            }
        )
