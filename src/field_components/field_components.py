#!/usr/bin/env python

from globals.globals import *
from player.msg import *
from math_utils.vector_utils import *
from typing import *
from field_components.colors import Color
import sys

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

    @classmethod
    def from_field_component(cls, field_component: FieldComponent):
        typeclass = getattr(sys.modules[__name__], field_component.type)
        return typeclass(TupleVector3.from_vector3(field_component.player_distance),
                         TupleVector3.from_vector3(field_component.half_size))

    def __str__(self) -> str:
        value = self.distance.convert()
        return f"{self.color.name} {self.type} {value[0]:.2f}m {value[2]:.1f}d"

class Robot(FieldObject):
    color = Color.RED
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, None)

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "Robot", distance, half_size)

class YellowPuck(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "YellowPuck", distance, half_size)

class BluePuck(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "BluePuck", distance, half_size)

class YellowGoal(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "YellowGoal", distance, half_size)

class BlueGoal(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "BlueGoal", distance, half_size)

class Pole(FieldObject):
    color = Color.GREEN
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, 0.4)

    def __init__(self, distance, half_size):
        super().__init__(Color.GREEN, "Pole", distance, half_size)

class LaserPoint(FieldObject):
    color = Color.ORANGE

    def __init__(self, distance, half_size):
        super().__init__(Color.ORANGE, "LaserPoint", distance, half_size)

class GenericObject(FieldObject):
    color = Color.MAGENTA

    def __init__(self, distance, half_size):
        super().__init__(Color.MAGENTA, "GenericObject", distance, half_size)

class RisingEdge(FieldObject):
    color = Color.GREEN

    def __init__(self, distance, half_size):
        super().__init__(Color.GREEN, "RisingEdge", distance, half_size)

class FallingEdge(FieldObject):
    color = Color.RED

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "FallingEdge", distance, half_size)


if __name__ == "__main__":
    fc = FieldComponent("yellow", "YellowPuck", Vector3(0, 0, 0), Vector3(0, 0, 0))
    fo = FieldObject.from_field_component(fc)

    print(fo)