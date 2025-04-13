import cv2
import numpy as np
from shapely.geometry import Polygon

class Tag:
    def __init__(self, id, corners, VISIBLE):
        self.id = id
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.p4 = None
        self.confidence = 0
        self.centroid = np.zeros(2).astype(int)

        self.corners = corners
        # This update corner, confidence, centroid
        self.update_corners(corners=corners)
        self.VISIBLE = VISIBLE

    def update_corners(self, corners):
        self.corners = corners
        self.p1 = corners[0]
        self.p2 = corners[1]
        self.p3 = corners[2]
        self.p4 = corners[3]
        self.update_confidence()
        # self.confidence_cross_check = self.quad_area()
        self.update_centroid()
        

    def update_confidence(self):
        
        x = self.corners[:, 0]
        y = self.corners[:, 1]
        self.confidence = 0.5 * np.abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    def update_centroid(self):
        polygon = Polygon([self.p1, self.p2, self.p3, self.p4])
        self.centroid[0] = polygon.centroid.x
        self.centroid[1] = polygon.centroid.y

    def triangle_area(self, p1, p2, p3):
        return 0.5 * abs(
            p1[0]*(p2[1] - p3[1]) +
            p2[0]*(p3[1] - p1[1]) +
            p3[0]*(p1[1] - p2[1])
        )

    def quad_area(self):  # points should be a list of 4 (x, y) tuples in order
        return self.triangle_area(self.corners[0], self.corners[1], self.corners[2]) + self.triangle_area(self.corners[0], self.corners[2], self.corners[3])
