#!/usr/bin/env python

from field_components.colors import Color
from globals.globals import *
from player.msg import *

class FieldObject:
    '''Class representing a field object.
    
    Attributes:
        color: color of the object
        type: type of the object
        spherical_distance: distance of the object in spherical coordinates
        half_size: half size of the object in spherical coordinates
        area_detect_range: range of the area of the object in pixels
        ratio_detect_range: range of the ratio of the object in pixels
        position: absolute position of the object in the field in x, y coordinates
        
    '''
    
    def __init__(self, color, type, spherical_distance, half_size):
        self.color = color
        self.type = type
        self.spherical_distance = spherical_distance
        self.half_size = half_size

        self.area_detect_range = (None, None)
        self.ratio_detect_range = (None, None)

        # absolute position in the field in x, y coordinates
        # this is later set by field class
        self.position = None

    def get_angles(self):
        theta_min = self.spherical_distance[1] - self.half_size[1]
        theta_max = self.spherical_distance[1] + self.half_size[1]
        phi_min = self.spherical_distance[2] - self.half_size[2]
        phi_max = self.spherical_distance[2] + self.half_size[2]
        return theta_min, theta_max, phi_min, phi_max

    def merge(self, *field_objects, return_type):
        theta_min, theta_max, phi_min, phi_max = self.get_angles()
        r = self.spherical_distance[0]

        for fo in field_objects:
            angles = fo.get_angles()
            theta_min = min([theta_min, angles[0]])
            theta_max = max([theta_max, angles[1]])
            phi_min = min([phi_min, angles[2]])
            phi_max = max([phi_max, angles[3]])
            r += fo.spherical_distance[0]

        r /= (len(field_objects) + 1)
        new_distance = (r, (theta_min + theta_max) / 2, (phi_min + phi_max) / 2)
        new_size = (0, (theta_max - theta_min) / 2, (phi_max - phi_min) / 2)
        return return_type(new_distance, new_size)


    def __str__(self) -> str:
        return f"{self.color.name} {self.type} {self.spherical_distance[0]:.2f}m {self.spherical_distance[2]:.1f}d"

class Robot(FieldObject):
    color = Color.RED
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, None)

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "robot", distance, half_size)

class YellowPuck(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "puck", distance, half_size)

class BluePuck(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "puck", distance, half_size)

class YellowGoal(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "goal", distance, half_size)

class BlueGoal(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "goal", distance, half_size)

class Pole(FieldObject):
    color = Color.GREEN
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, 0.4)

    def __init__(self, distance, half_size):
        super().__init__(Color.GREEN, "pole", distance, half_size)

class LaserPoint(FieldObject):
    color = Color.ORANGE

    def __init__(self, distance, half_size):
        super().__init__(Color.ORANGE, "laser point", distance, half_size)

class GenericObject(FieldObject):
    color = Color.MAGENTA

    def __init__(self, distance, half_size):
        super().__init__(Color.MAGENTA, "generic", distance, half_size)