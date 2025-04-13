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
        results = self.model.predict(source=frame_bgr, imgsz=640, conf=0.25, verbose=False, stream=True)[0]
        detections = []
        for box in results.boxes:
            label = results.names[int(box.cls)]
            conf = float(box.conf)
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append((label, conf, (x1, y1, x2, y2)))
        return detections
