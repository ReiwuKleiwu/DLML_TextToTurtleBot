from ultralytics import YOLO
import logging

# Necessary in oder to prevent yolo from spamming the console
logging.getLogger("ultralytics").setLevel(logging.ERROR)

class ObjectDetector:
    def __init__(self, model_path: str):
        self._model = self.create_custom_yolo_model(model_path)

    def create_custom_yolo_model(self, model_path: str):
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

    def detect(self, image):
        results = self._model.predict(image, max_det=20, verbose=False)
        boxes = results[0].boxes
        detected = set()
        info = {}
        all_detections = {}  # Store all detections for each class
        
        for i, box in enumerate(boxes):
            cls_id = int(boxes.cls[i])
            name = results[0].names[cls_id]
            coords = box.xyxy[0]  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = map(int, coords)
            detected.add(name)
            
            # Store all detections for each class
            if name not in all_detections:
                all_detections[name] = []
            all_detections[name].append({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})
            
            # Keep the last detection for backward compatibility (will be updated by CameraHandler)
            info[name] = {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2}

        return detected, info, all_detections