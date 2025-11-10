import math
from typing import Dict, List
from numpy import ndarray
from ultralytics import YOLO

from events.event_bus import EventBus
from events.interfaces.events import EventType, DomainEvent

class DetectedObject: 
    def __init__(self, x1: int, y1: int, x2: int, y2: int, name: str, confidence: float):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.world_x = None
        self.world_y = None
        self.world_z = None
        self.name = name
        self.confidence = confidence

    def has_world_coordinates(self) -> bool:
        return self.world_x is not None and self.world_y is not None and self.world_z is not None

    # Two detected objects are equal if their bounding boxes are "close" to each other
    def __eq__(self, other):
        if not isinstance(other, DetectedObject):
            return False
        
        max_pixel_threshold = 10

        # Check if the name is the same and if the bounding boxes are within the threshold
        return (self.name == other.name and
                abs(self.x1 - other.x1) <= max_pixel_threshold and
                abs(self.y1 - other.y1) <= max_pixel_threshold and
                abs(self.x2 - other.x2) <= max_pixel_threshold and
                abs(self.y2 - other.y2) <= max_pixel_threshold)
    
    def distance_to_other_object(self, other_object) -> float:
        if not isinstance(other_object, DetectedObject):
            return float('inf')

        if not self.has_world_coordinates() or not other_object.has_world_coordinates():
            return float('inf')
        
        dx = self.world_x - other_object.world_x
        dy = self.world_y - other_object.world_y
        dz = self.world_z - other_object.world_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def distance_to_world_coordinates(self, world_x: float, world_y: float, world_z: float) -> float:
        if not self.has_world_coordinates():
            return float('inf')
        
        dx = self.world_x - world_x
        dy = self.world_y - world_y
        dz = self.world_z - world_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)


class ObjectDetector:
    def __init__(self, model_path: str, confidence_threshold: float = 0.3):
        self._model = self._create_custom_yolo_model(model_path)
        self._event_bus = EventBus()
        self._confidence_threshold = confidence_threshold
    

    def _create_custom_yolo_model(self, model_path: str):
        model = YOLO(f"{model_path}/yolov8s-worldv2.pt")

        classes = ["table", "monitor", "closed door", "open door", "chair", "computer",
             "person", "fridge", "fire extinguisher", "window", "blackboard",
             "kitchen cabinet", "wall", "toilet", "towel", "radiator", "desk",
             "bin", "door"
             ]

        model.set_classes(
            classes
        )
        
        model.save(f"{model_path}/yolo_model_custom.pt")

        return model
    
    def detect(self, image: ndarray) -> None:
        results = self._model.predict(image, max_det=20, verbose=False)
        boxes = results[0].boxes
        names = results[0].names
        all_detections = {}  # Store all detections for each class

        for i, box in enumerate(boxes):
            cls_id = int(boxes.cls[i])
            name = names[cls_id]

            # Skip wall detections
            if name.lower() == 'wall':
                continue

            coords = box.xyxy[0]  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = map(int, coords)
            confidence = float(boxes.conf[i])

            # Skip low-confidence detections
            if confidence < self._confidence_threshold:
                continue

            detected_object = DetectedObject(x1, y1, x2, y2, name, confidence)

            # Store all detections for each class
            if name not in all_detections:
                all_detections[name] = []
            all_detections[name].append(detected_object)

        self._event_bus.publish(DomainEvent(EventType.OBJECTS_DETECTED, all_detections))