from ultralytics import YOLO

class ObjectDetector:
    def __init__(self, model_path: str):
        self._model = self.create_custom_yolo_model(model_path)

    def create_custom_yolo_model(self, model_path: str):
        model = YOLO(f"{model_path}/yolov8s-worldv2.pt")
        model.set_classes(
            ["table", "monitor", "closed door", "open door", "chair", "computer",
             "person", "fridge", "fire extinguisher", "window", "blackboard",
             "kitchen cabinet", "wall", "toilet", "towel", "radiator", "desk",
             "bin"
             ]
        )
        model.save(f"{model_path}/yolo_model_custom.pt")

        return model

    def detect(self, image):
        results = self._model.predict(image, max_det=20, verbose=False)
        boxes = results[0].boxes
        detected = set()
        info = {}
        for i, box in enumerate(boxes):
            if len(box.cls) <= i:
                continue
             
            cls_id = int(box.cls[i])
            name = results[0].names[cls_id]
            coords = box.xyxy[0]  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = map(int, coords)
            detected.add(name)
            info[name] = {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2}

        return detected, info