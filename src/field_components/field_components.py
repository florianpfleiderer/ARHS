#!/usr/bin/env python

from field_components.colors import Color
from globals.globals import *
from player.msg import *
from math_utils.vector_utils import *
from typing import *

class FieldObject:
    '''Class representing a field object.
    
    Attributes:
        color: color of the object
        type: type of the object
        spherical_distance: distance of the object in spherical coordinates
            in relation to the robot
        half_size: half size of the object 
        area_detect_range: range of the area of the object in pixels
        ratio_detect_range: range of the ratio of the object in pixels
        position: absolute position of the object in the field in x, y coordinates
        
    '''
    def __init__(self, color, type, distance, half_size):
        self.color: Color = color
        self.type = type
        self.distance: TupleVector3 = distance
        self.half_size: TupleVector3 = half_size
        self.position: TupleVector3 = None
        self.area_detect_range = (None, None)
        self.ratio_detect_range = (None, None)

    def merge(self, *field_objects, return_type):
        min_corner = self.distance + self.half_size
        max_corner = self.distance - self.half_size
        for fo in field_objects:
            min_corner.tuple = tuple(map(min, list(min_corner.tuple), list((fo.distance + fo.half_size).tuple)))
            max_corner.tuple = tuple(map(max, list(max_corner.tuple), list((fo.distance - fo.half_size).tuple)))
        return return_type((max_corner + min_corner) / 2, (max_corner - min_corner) / 2)

    def get_field_component(self):
        player_dist = Vector3(*self.distance.tuple)
        half_size = Vector3(*self.half_size.tuple)
        return FieldComponent(self.color.name, self.type, player_dist, half_size)


    def __str__(self) -> str:
        value = self.distance.convert()
        return f"{self.color.name} {self.type} {value[0]:.2f}m {value[2]:.1f}d"

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

class RisingEdge(FieldObject):
    color = Color.GREEN

    def __init__(self, distance, half_size):
        super().__init__(Color.GREEN, "rising edge", distance, half_size)

class FallingEdge(FieldObject):
    color = Color.RED

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "falling edge", distance, half_size)