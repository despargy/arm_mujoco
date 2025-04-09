import cv2
import numpy as np

class Tag:
    def __init__(self, id, corners, VISIBLE):
        self.id = id
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.p4 = None
        self.corners = corners
        self.update_corners(corners=corners)
        self.VISIBLE = VISIBLE

    def update_corners(self, corners):
        self.corners = corners
        self.p1 = corners[0]
        self.p2 = corners[1]
        self.p3 = corners[2]
        self.p4 = corners[3]
