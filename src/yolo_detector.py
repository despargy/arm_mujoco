from ultralytics import YOLO
import cv2

class YOLODetector:
    def __init__(self, model_path="yolo11n.pt"): 
        self.model = YOLO(model_path)

    def detect(self, frame):
        """
        Detect objects in a given BGR frame.
        Returns: List of (label, confidence, (x1, y1, x2, y2), area)
        """
        results = self.model.predict(source=frame, stream=True, verbose=False, conf=0.25, device="0")
        detections = []

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls.item())
                label = result.names[cls_id]
                conf = float(box.conf.item())
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                area = (x2 - x1) * (y2 - y1) #TODO: CONSIDER ALTERNATIVE FUNCTION
                
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                detections.append((label, conf, (x1, y1, x2, y2), area))
                
                
        return detections, frame