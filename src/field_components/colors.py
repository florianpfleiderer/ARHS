#!/usr/bin/env python

from enum import Enum
import cv2
from globals.globals import *
import numpy as np

# enum for colors with default value (RGB), min and max values (HSV)
class Color(Enum):
    RED = ((0, 0, 255), (0, 50, 50), (5, 255, 255))
    GREEN = ((0, 255, 0), (55, 50, 50), (65, 255, 255))
    BLUE = ((255, 0, 0), (115, 50, 50), (125, 255, 255))
    YELLOW = ((0, 255, 255), (25, 50, 50), (35, 255, 255))
    ORANGE = ((0, 165, 255), (10, 50, 50), (20, 255, 255))

    def __init__(self, default, min, max):
        self.default = default
        self.min = min
        self.max = max

    def default(self):
        return self.default
    
    def min_hsv(self):
        return self.min

    def max_hsv(self):
        return self.max

    def get_range(self):
        return self.min, self.max
    
    def in_range(self, color_rgb):
        hsv_compare = cv2.cvtColor(color_rgb, cv2.COLOR_BGR2HSV)
        return self.min_hsv <= hsv_compare <= self.max_hsv

    def __str__(self) -> str:
        return self.name.lower()
    
    def mask(self, image):
        #filter color
        hsv =  cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([[self.min_hsv]]), np.array([[self.max_hsv]]))
        #reduce noise
        kernel = np.ones((COLOR_MASK_SMOOTHING, COLOR_MASK_SMOOTHING),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask =  cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask
    
    @classmethod
    def from_string(cls, color_name):
        return Color._member_map_[color_name]