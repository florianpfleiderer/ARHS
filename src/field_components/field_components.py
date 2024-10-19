#!/usr/bin/env python

import rospy
import sys
import cv2
import copy
import random
import time
import numpy as np
from globals.globals import *
from globals.tick import *
from player.msg import FieldComponent
from geometry_msgs.msg import Vector3
from math_utils.vector_utils import TupleVector3, TupleRotator3, Coordinate
from typing import List, Dict, NamedTuple, Any, Tuple
from field_components.colors import Color
import visualization.screen_utils as sc
import visualization.imgops as imgops
from data_utils.topic_handlers import FieldComponentsSubscriber, FieldDimensionsSubscriber
from itertools import combinations
from math_utils.pointcloud import PointCloud
import math_utils.pointcloud as pc

# Since the ratios of pole positions is known, any 3 adjacent poles along one line can be used to calculate the field dimensions.
# Assuming ordering of the poles from right to left (ordered by increasing angle), we can create a lookup table
# to determine which poles we have in order to calculate the field dimensions.
# The result has a 180d rotational symmetry, so detecting the closest goal or puck is required to determine the orientation.
# The origin (x, y) of the field is assumed to be in A.
# 
# Pole setup (in units):
# 
#                    5
# 
#     N   M    L     K     J    I   H
#
#
# 3                                     h
#
#
#     A   B    C     D     E    F   G
#     |0.5|0.75| 1.25| 1.25|0.75|0.5|
#                    w
#
# Ratio table:
# 0.5  : 0.75 = 2/3 => A, B, C => w = BC * 20/3 => x = A.x
# 0.75 : 1.25 = 3/5 => B, C, D => w = CD * 4    => x = B.x - w * 0.1
# 1.25 : 1.25 = 1   => C, D, E => w = CD * 4    => x = C.x - w * 0.25
# 1.25 : 0.75 = 5/3 => D, E, F => w = DE * 4    => x = D.x - w * 0.5
# 0.75 : 0.5  = 3/2 => E, F, G => w = EF * 20/3 => x = E.x - w * 0.75
#
# For width calculation, the longer distance is used
# h = w * 3/5
# y = 0
#

DIMENSION_FACTORS = [[2/3, 3/5, 1, 5/3, 3/2], [20/3, 4, 4, 4, 20/3], [0, 0.1, 0.25, 0.5, 0.75]]

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
    def __init__(self, color: Color, type, distance: TupleVector3, half_size: TupleVector3):
        self.color: Color = color
        self.type = type
        self.distance: TupleVector3 = distance
        self.half_size: TupleVector3 = half_size
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
        return FieldComponent(self.color.__str__(), self.type, player_dist, half_size)

    @classmethod
    def from_field_component(cls, field_component: FieldComponent):
        typeclass = getattr(sys.modules[__name__], field_component.type)
        return typeclass(TupleVector3.from_vector3(field_component.player_distance),
                         TupleVector3.from_vector3(field_component.half_size))
    
    def draw_icon(self, image, rect):
        pass

    def __str__(self) -> str:
        value = self.distance.convert()
        return f"{self.color.name} {self.type} {value[0]:.2f}m {value[2]:.1f}d"
    
    @classmethod
    def default(cls):
        return cls(TupleVector3(), cls.default_half_size)

class Robot(FieldObject):
    color = Color.RED
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, None)
    default_half_size = TupleVector3((0.3, 0.3, 0.3))

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "Robot", distance, Robot.default_half_size)

    def draw_icon(self, image, rect):
        # body + wheels
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.35, 0.75), sc.scale_rect(rect, 0.15, 0.2), 0, 0, 360, (10, 10, 10), -1)
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0.3, 0.35), sc.get_point_in_rect(rect, 1, 0.85), (0, 0, 255), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.8, 0.8), sc.scale_rect(rect, 0.15, 0.2), 0, 0, 360, (10, 10, 10), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.83, 0.8), sc.scale_rect(rect, 0.06, 0.11), 0, 0, 360, (0, 200, 200), -1)

        # head shadow
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0.3, 0.4), sc.get_point_in_rect(rect, 0.7, 0.55), (0, 0, 155), -1)

        # platform + yellow dots
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0.64, 0.34), sc.get_point_in_rect(rect, 1.08, 0.42), (10, 10, 10), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.57, 0.45), sc.scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.42, 0.45), sc.scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.3, 0.45), sc.scale_rect(rect, 0.04, 0.12), 0, -90, 90, (0, 100, 150), -1)

        # head
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0.15, 0.13), sc.get_point_in_rect(rect, 0.65, 0.5), (10, 10, 10), -1)

        # left eye
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.4, 0.22), sc.scale_rect(rect, 0.1, 0.18), 0, -60, 190, (255, 255, 255), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.375, 0.1), sc.scale_rect(rect, 0.08, 0.14), 0, -22, 158, (255, 255, 255), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.35, 0.23), sc.scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 255), -1)
        cv2.line(image, sc.get_point_in_rect(rect, 0.25, 0.2), sc.get_point_in_rect(rect, 0.5, 0), (10, 10, 10), 3)
        cv2.line(image, sc.get_point_in_rect(rect, 0.25, 0.18), sc.get_point_in_rect(rect, 0.5, -0.03), (10, 10, 10), 3)


        # right eye
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.125, 0.18), sc.scale_rect(rect, 0.1, 0.18), 0, 5, 250, (255, 255, 255), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.15, 0.1), sc.scale_rect(rect, 0.08, 0.14), 0, 35, 215, (255, 255, 255), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.075, 0.19), sc.scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 255), -1)
        cv2.line(image, sc.get_point_in_rect(rect, 0.25, 0.2), sc.get_point_in_rect(rect, 0.05, -0.05), (10, 10, 10), 3)
        cv2.line(image, sc.get_point_in_rect(rect, 0.25, 0.18), sc.get_point_in_rect(rect, 0.05, -0.08), (10, 10, 10), 3)

class Player(FieldObject):
    default_half_size = TupleVector3((0.3, 0.3, 0.3))

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "Player", distance, Player.default_half_size)

    def draw_icon(self, image, rect):
        # body + wheels
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.65, 0.75), sc.scale_rect(rect, 0.15, 0.2), 0, 0, 360, (10, 10, 10), -1)
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0, 0.35), sc.get_point_in_rect(rect, 0.7, 0.85), (0, 0, 255), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.2, 0.8), sc.scale_rect(rect, 0.15, 0.2), 0, 0, 360, (10, 10, 10), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.17, 0.8), sc.scale_rect(rect, 0.06, 0.11), 0, 0, 360, (0, 200, 200), -1)

        # head shadow
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0.3, 0.4), sc.get_point_in_rect(rect, 0.7, 0.55), (0, 0, 155), -1)

        # platform + yellow dots
        cv2.rectangle(image, sc.get_point_in_rect(rect, -0.08, 0.34), sc.get_point_in_rect(rect, 0.36, 0.42), (10, 10, 10), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.43, 0.45), sc.scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.58, 0.45), sc.scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.7, 0.45), sc.scale_rect(rect, 0.04, 0.12), 0, 90, 270, (0, 100, 150), -1)

        # head
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0.35, 0.13), sc.get_point_in_rect(rect, 0.85, 0.5), (0, 0, 255), -1)

        # right eye
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.6, 0.12), sc.scale_rect(rect, 0.1, 0.18), 0, 0, 360, (200, 200, 200), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.65, 0.13), sc.scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 0), -1)

        # left eye
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.875, 0.08), sc.scale_rect(rect, 0.1, 0.18), 0, 0, 360, (200, 200, 200), -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.925, 0.09), sc.scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 0), -1)

class YellowPuck(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?
    default_half_size = TupleVector3((0.05, 0.05, 0.2))

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "YellowPuck", distance, YellowPuck.default_half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.circle(image, sc.get_point_in_rect(rect, 0.5, 0.5), round(w/2), self.color.default(), -1)

class BluePuck(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?
    default_half_size = TupleVector3((0.05, 0.05, 0.2))

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "BluePuck", distance, BluePuck.default_half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.circle(image, sc.get_point_in_rect(rect, 0.5, 0.5), round(w/2), self.color.default(), -1)

class YellowGoal(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?
    default_half_size = TupleVector3((0.25, 0.5, 0))

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "YellowGoal", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0, 0), sc.get_point_in_rect(rect, 1, 1), self.color.default(), -1)

class BlueGoal(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?
    default_half_size = TupleVector3((0.25, 0.5, 0))

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "BlueGoal", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0, 0), sc.get_point_in_rect(rect, 1, 1), self.color.default(), -1)

class Pole(FieldObject):
    color = Color.GREEN
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, 0.4)
    default_half_size = TupleVector3((0.05, 0.05, 0.3))

    def __init__(self, distance, half_size):
        super().__init__(Color.GREEN, "Pole", distance, Pole.default_half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.circle(image, sc.get_point_in_rect(rect, 0.5, 0.5), round(w/2), self.color.default(), -1)

class LaserPoint(FieldObject):
    color = Color.ORANGE

    def __init__(self, distance, half_size):
        super().__init__(Color.ORANGE, "LaserPoint", distance, half_size)

class GenericObject(FieldObject):
    color = Color.MAGENTA

    def __init__(self, distance, half_size):
        super().__init__(Color.MAGENTA, "GenericObject", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.putText(image, "?", sc.get_point_in_rect(rect, 0, 1), CV2_DEFAULT_FONT, 1, self.color.default(), CV2_DEFAULT_THICKNESS, cv2.LINE_AA)

class RisingEdge(FieldObject):
    color = Color.GREEN

    def __init__(self, distance, half_size):
        super().__init__(Color.GREEN, "RisingEdge", distance, half_size)

class FallingEdge(FieldObject):
    color = Color.RED

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "FallingEdge", distance, half_size)

class Fan(FieldObject):
    def __init__(self, distance, half_size):
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        color = Color.BLUE
        color.default_bgr = (b, g, r)

        super().__init__(color, "Fan", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        shirt = self.color.default()
        skin = (168, 189, 255)
        black = (10, 10, 10)
        dark_red = (0, 0, 100)
        brown = (16, 21, 36)
        # shirt
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0.33, 0.5), sc.get_point_in_rect(rect, 0.66, 0.8), shirt, -1)
        # head
        cv2.circle(image, sc.get_point_in_rect(rect, 0.5, 0.3), round(w * 0.3), skin, -1)
        cv2.circle(image, sc.get_point_in_rect(rect, 0.4, 0.25), round(w * 0.05), black, -1)
        cv2.circle(image, sc.get_point_in_rect(rect, 0.6, 0.25), round(w * 0.05), black, -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.5, 0.37), sc.scale_rect(rect, 0.16, 0.125), 0, 0, 180, dark_red, -1)
        # hands
        cv2.circle(image, sc.get_point_in_rect(rect, 0.1, 0.35), round(w * 0.08), skin, -1)
        cv2.circle(image, sc.get_point_in_rect(rect, 0.9, 0.35), round(w * 0.08), skin, -1)
        # feet
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.35, 0.8), sc.scale_rect(rect, 0.1, 0.07), 0, 0, 360, brown, -1)
        cv2.ellipse(image, sc.get_point_in_rect(rect, 0.65, 0.8), sc.scale_rect(rect, 0.1, 0.07), 0, 0, 360, brown, -1)

class TypeSlot:
    def __init__(self, index: int, type: str, item: Any=None):
        self.index: int = index
        self.type: str = type
        self.item: Any = item

class TypeList:
    def __init__(self, *types: Tuple[str, int]):
        self.slots = []
        index = 0
        for t in types:
            for i in range(t[1]):
                self.slots.append(TypeSlot(index, t[0], None))
                index += 1

        self.current_index = 0
    
    def __getitem__(self, key):
        if type(key) is int:
            return self.slots[key].item
        
        elif type(key) is str:
            return [i.item for i in self.slots if i.type == key and i.item is not None]
        
    def __setitem__(self, key, values):
        if type(key) is int:
            if type(values) is not list:
                values = [values]
            
            for i in range(len(values)):
                self.slots[key + i].item = values[i]
        
        elif type(key) is str:
            index = 0
            for slot in self.slots:
                if slot.type == key and slot.item is None:
                    slot.item = values[index]
                    index += 1
        
        
    def add_item(self, type, item):
        for i in self.slots:
            if i.type == type and i.item is None:
                i.item = item
                return i.index
        return None

    def __len__(self):
        return len([slot for slot in self.slots if slot.item is not None])

    def items(self):
        self.current_index = 0
        while self.current_index < len(self.slots):
            if self.slots[self.current_index].item is not None:
                yield self.slots[self.current_index].item
            self.current_index += 1

    def __iter__(self):
        self.current_index = 0
        return self
    
    def __next__(self):
        while self.current_index < len(self.slots) and self.slots[self.current_index].item is None:
            self.current_index += 1

        self.current_index += 1
        if self.current_index <= len(self.slots):
            return self.slots[self.current_index - 1]
        raise StopIteration
        

class Field(FieldObject):
    '''The Field class is a special type of FieldObject that stores other FieldObjects with absolute positions.
    The meaning of its member variables is slightly different that that of the regular FieldObject class.

    Attributes:
        field_objects: Dictionary that maps type names to Lists of FieldObjects
            The origin of the field, i.e. the position of Pole A is (0, 0).
        distance: Vector3 that stores the absolute position of the field (origin - player position)
        half_size: Vector3 that stores the half size of the field (x = w/2, y = h/2)
        angle_offset: Rotator3 that stores the angle offset of the field (player rotation)
    '''
    def __init__(self):
        super().__init__(Color.GREEN, "Field", TupleVector3((0, 0, 0)), TupleVector3((0, 0, 0)))
        self.field_component_sub = FieldComponentsSubscriber()
        self.field_dimensions_sub = FieldDimensionsSubscriber()

        self.field_objects: TypeList = TypeList(("Player", 1),
                                                ("Robot", 1),
                                                ("YellowPuck", 3),
                                                ("BluePuck", 3),
                                                ("YellowGoal", 1),
                                                ("BlueGoal", 1),
                                                ("Pole", 14),
                                                ("GenericObject", 32))

        self.angle_offset: TupleRotator3 = TupleRotator3()
        self.initialized = False

    # def get_objects_by_class(self, class_name):
    #     return [o for o in self.field_objects if o.type == class_name]


    def calculate_dimensions(self, *poles):
        if len(poles) < 3:
            rospy.logwarn(f"Not enough poles to calculate dimensions, got {len(poles)}")
            return None, None
        
        for pole_comb in combinations(poles, 3):
            pos1: TupleVector3 = pole_comb[0].distance
            pos2: TupleVector3 = pole_comb[1].distance
            pos3: TupleVector3 = pole_comb[2].distance

            angle_threshold = 35
            ratio_threshold = 0.05

            angle = (pos2 - pos1).angle(pos3 - pos2)
            if angle > angle_threshold:
                rospy.logwarn(f"Poles not in straight line, got {angle:.2f}")
                continue
            
            dist12 = pos1.distance(pos2)
            dist23 = pos2.distance(pos3)
            detect_ratio = dist12 / dist23 if dist23 != 0 else 0

            dim_factor = sorted(zip(*DIMENSION_FACTORS), key=lambda e: abs(detect_ratio - e[0]))[0]

            if abs(detect_ratio - dim_factor[0]) > ratio_threshold:
                rospy.logwarn(f"No matching ratio, got {detect_ratio}")
                continue
            
            w = max(dist12, dist23) * dim_factor[1]
            return w, w * 3/5

        return None, None

    def set_field_dimensions(self, w, h):
        self.half_size = TupleVector3((w/2, h/2, 0))

    def update_field_dimensions(self):    
        if self.field_dimensions_sub.data is not None:
            w, h = self.field_dimensions_sub.data.w, self.field_dimensions_sub.data.h

        elif self.field_component_sub.data is not None:
            detected_poles = [FieldObject.from_field_component(fc) for fc in self.field_component_sub.data if fc.type == "Pole"]
            detected_poles.sort(key=lambda pole: pole.distance.convert(Coordinate.CYLINDRICAL)[1])
            
            w, h = self.calculate_dimensions(*detected_poles)
            
            if w is None:
                return False
        else:
            return False  
        
        self.set_field_dimensions(w, h)
        self.field_objects["Pole"] = self.generate_poles(w, h)
        rospy.loginfo(f"Field dimensions: {self.half_size[0]*2} x {self.half_size[1]*2}")
        return True

    def update_field_offset(self, detected_field_objects):
        t1 = time.perf_counter()
        base = PointCloud(np.vstack([self.get_relative_field_distance(fo.distance).tuple for fo in self.field_objects.items()]))
        compare = PointCloud(np.vstack([fo.distance.tuple * (1, 1, 0) for fo in detected_field_objects]))
        t2 = time.perf_counter()
        print(f'creating PointCloud: {t2-t1:.4f}')

        img = imgops.empty_image((500, 500))
        t1 = time.perf_counter()
        base.draw(img, (0, 255, 0), 50)
        compare.draw(img, (255, 0, 0), 50)
        t2 = time.perf_counter()
        print(f'drawing PointCloud: {t2-t1:.4f}')

        t1 = time.perf_counter()
        origin_offset, angle_offset = base.get_twist(compare)
        t2 = time.perf_counter()
        print(f'get_twist: {t2-t1:.4f}')
        pc.draw_vector(img, origin_offset.tuple, self.distance.tuple, color=(0, 0, 255), scale=10)
        cv2.imshow("update", img)
        cv2.waitKey(50)

        print("found offset: ", origin_offset, angle_offset)

        if origin_offset is None:
            return False
        
        self.distance += origin_offset
        self.angle_offset += angle_offset
        return True

    def get_abs_field_distance(self, relative_distance: TupleVector3):
        return relative_distance + self.angle_offset + self.distance
    
    def get_relative_field_distance(self, abs_distance: TupleVector3):
        return abs_distance - self.distance - self.angle_offset

    def get_abs_field_object(self, fo: FieldObject):
        fo_copy = copy.copy(fo)
        fo_copy.distance = self.get_abs_field_distance(fo_copy.distance)
        return fo_copy
    
    def get_player_relative_field_object(self, fo: FieldObject):
        fo_copy = copy.copy(fo)
        fo_copy.distance = self.get_relative_field_distance(fo_copy.distance)
        return fo_copy

    def update_field_objects(self, detected_objects: List[FieldObject]):
        update_counter = 0
        new_counter = 0

        detected_local = [self.get_abs_field_object(fo) for fo in detected_objects]
        for compare_fo in detected_local:
            for slot in self.field_objects:
                self_fo = slot.item
                if self_fo.distance.distance_xy(compare_fo.distance) < 0.15:
                    if type(self_fo) == GenericObject:
                        slot.item = compare_fo

                    elif type(self_fo) != Pole:
                        self_fo.distance = (compare_fo.distance + self_fo.distance) / 2

                    update_counter += 1
                    break
            else:
                self.field_objects.add_item(compare_fo.type, compare_fo)
                new_counter += 1

        print(f"updated {update_counter} field objects, added {new_counter} new field objects")
        
    def update(self):
        t1 = time.perf_counter()
        self.update_field_dimensions()
        t2 = time.perf_counter()
        print(f'update_field_dimensons: {t2-t1:.4f}s')

        if self.field_component_sub.data is None:
            rospy.logwarn("No FieldComponents detected!")
            return False

        if len(self.field_objects) == 0:
            rospy.logwarn("FieldObjects could not be determined!")
            return False

        detected_field_objects = [FieldObject.from_field_component(fc) for fc in self.field_component_sub.data]
        t1 = time.perf_counter()
        self.update_field_offset(detected_field_objects)
        t2 = time.perf_counter()
        print(f'update_field_offset: {t2-t1:.4f}s')

        player = Player(self.distance, (0.3, 0.3, 0.3))
        t1 = time.perf_counter()
        self.update_field_objects([player, *detected_field_objects])
        t2 = time.perf_counter()
        print(f'update_field_objects: {t2-t1:.4f}s')

        print(f"total: {len(self.field_objects)} field objects")  

        return True  

    def draw(self, screen, draw_text=False, draw_center=False, draw_icon=True, draw_rect=False, draw_cube=False):
        for fo in self.field_objects.items():
            fo_rel = self.get_player_relative_field_object(fo)
            screen.draw_object(fo_rel, draw_text, draw_center, draw_icon, draw_rect, draw_cube)
        cv2.putText(screen.image, f"{self.half_size[0]*2:.2f} x {self.half_size[1]*2:.2f}", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        
    def generate_poles(self, w, h):
        poles = []
        origin = TupleVector3((0, 0, 0))
        vec_w = TupleVector3((w, 0, 0))
        offset_h = TupleVector3((0, h, 0))
        
        distances = [origin + vec_w * x for x in [0.1, 0.25, 0.5, 0.75, 0.9, 1]]
        distances.insert(0, origin)

        distances.extend([dist + offset_h for dist in distances])

        poles = [Pole(dist, TupleVector3((0.05, 0.05, 0))) for dist in distances]

        return poles


def test_field():
    field = Field()
    field.half_size = TupleVector3((5, 3, 0))
    field.distance = TupleVector3((1, 1, 0))
    field.angle_offset = TupleRotator3((10, 0, 0))

    pole = Pole(TupleVector3((1, 2, 0)), Pole.default_half_size)
    field.field_objects.add_item("Pole", pole)

    img = imgops.empty_image((500, 500))
    drel = field.get_relative_field_distance(pole.distance)
    pc.draw_vector(img, drel.tuple, color=(255, 0, 0))
    pc.draw_vector(img, pole.distance.tuple, color=(0, 255, 0))
    print(drel)
    print(field.get_abs_field_distance(field.get_relative_field_distance(pole.distance)))


    cv2.imshow("offset test", img)
    cv2.waitKey(50)
    time.sleep(100)

if __name__ == "__main__":
    test_field()