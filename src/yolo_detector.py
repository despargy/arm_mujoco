# yolo_detector.py
from ultralytics import YOLO
import cv2

class YOLODetector:
    def __init__(self, model_path="yolo11n.pt"): 
        self.model = YOLO(model_path)

    def detect(self, frame_bgr):
        """
        Detect objects in a given BGR frame.
        Returns a list of (label, confidence, (x1, y1, x2, y2)) tuples.
        """
        results = self.model.predict(source=frame_bgr, stream=True, verbose=False)
        detections = []

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls.item())
                label = result.names[cls_id]
                conf = float(box.conf.item())
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                detections.append((label, conf, (x1, y1, x2, y2)))

        return detections

