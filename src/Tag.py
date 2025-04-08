import cv2
import numpy as np

class Tag:
    def __init__(self, id, corners, VISIBLE):
        self.id = id
        self.x1 = None
        self.y1 = None
        self.x2 = None
        self.y2 = None
        self.corners = corners
        self.update_corners(corners=corners)
        self.VISIBLE = VISIBLE

    def update_corners(self, corners):
        self.corners = corners
        self.x1 = corners[0]
        self.y1 = corners[1]
        self.x2 = corners[2]
        self.y2 = corners[3]
