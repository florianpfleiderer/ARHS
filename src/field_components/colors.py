#!/usr/bin/env python

from enum import Enum
import cv2
from globals.globals import *

#color_min, color_max
# COLORS = {'green': ([65, 80, 50], [83, 255, 255]),
#           'blue': ([90, 70, 90], [110, 255, 255]),
#           'yellow': ([22, 70, 50], [30, 255, 255]),
#           'red':([171, 40, 50], [179, 255, 180])}

# enum for colors with default value (RGB), min and max values (HSV)
class Color(Enum):
    SIM_RED = ((0, 0, 255), (175, 50, 50), (5, 255, 255))
    SIM_GREEN = ((0, 255, 0), (55, 50, 50), (65, 255, 255) )
    SIM_BLUE = ((255, 0, 0), (115, 50, 50), (125, 255, 255))
    SIM_YELLOW = ((0, 255, 255), (25, 50, 50), (35, 255, 255))
    SIM_ORANGE = ((0, 165, 255), (0, 0, 0), (255, 255, 255))

    REAL_RED = ((0, 0, 255), (171, 46, 33), (10, 255, 255)) # ([171, 40, 50], [179, 255, 180])
    REAL_GREEN = ((0, 255, 0), (63, 60, 60), (82, 255, 255)) # ([65, 80, 50], [83, 255, 255])
    REAL_BLUE = ((255, 0, 0), (94, 55, 101), (103, 255, 255)) # ([90, 70, 90], [110, 255, 255])
    REAL_YELLOW = ((0, 255, 255), (23, 70, 60), (34, 255, 220)) # ([22, 70, 50], [30, 255, 255])
    REAL_ORANGE = ((0, 165, 255), (10, 50, 50), (20, 255, 255))

    RED = SIM_RED if SIMULATION_MODE else REAL_RED
    GREEN = SIM_GREEN if SIMULATION_MODE else REAL_GREEN
    BLUE = SIM_BLUE if SIMULATION_MODE else REAL_BLUE
    YELLOW = SIM_YELLOW if SIMULATION_MODE else REAL_YELLOW
    ORANGE = SIM_ORANGE if SIMULATION_MODE else REAL_ORANGE

    MAGENTA = ((255, 0, 255), (140, 50, 50), (160, 255, 255))

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
        h_lo = self.min_hsv[0]
        h_hi = self.max_hsv[0]

        if h_lo > h_hi:
            h_in_range = h_lo <= hsv_compare[0] <= 255 or 0 <= hsv_compare[0] <= h_hi
        else:
            h_in_range = h_lo <= hsv_compare[0] <= h_hi

        return h_in_range and self.min_hsv[1:] <= hsv_compare[1:] <= self.max_hsv[1:]

    def __str__(self) -> str:
        return self.name.upper()
    
    @classmethod
    def from_string(cls, color_name):
        return Color._member_map_[color_name]