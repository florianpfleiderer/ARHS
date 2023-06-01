#!/usr/bin/env python

import rospy
import sys
import cv2
import copy
import random
from globals.globals import *
from player.msg import FieldComponent
from geometry_msgs.msg import Vector3
from math_utils.vector_utils import TupleVector3, TupleRotator3, Coordinate
from math_utils.pointcloud import PointCloud
from typing import List
from field_components.colors import Color
import visualization.screen_utils as sc 
from visualization.screen_components import Screen
from data_utils.topic_handlers import FieldComponentsSubscriber
from itertools import combinations

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
    
    def draw_icon(self, image, rect):
        pass

    def __str__(self) -> str:
        value = self.distance.convert()
        return f"{self.color.name} {self.type} {value[0]:.2f}m {value[2]:.1f}d"

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

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "YellowGoal", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.rectangle(image, sc.get_point_in_rect(rect, 0, 0), sc.get_point_in_rect(rect, 1, 1), self.color.default(), -1)

class BlueGoal(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?

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

class Field(FieldObject):
    def __init__(self):
        super().__init__(Color.GREEN, "Field", TupleVector3((0, 0, 0)), TupleVector3((0, 0, 0)))
        self.field_component_sub = FieldComponentsSubscriber()
        self.field_objects: List[FieldObject] = []
        self.angle_offset: TupleRotator3 = TupleRotator3()
        self.initialized = False

    def get_objects_by_class(self, class_name):
        return [o for o in self.field_objects if o.type == class_name]

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
        self.initialized = True
        self.field_objects = self.generate_poles(*self.half_size.tuple[:2])

    def update_field_dimensions(self, detected_field_objects):        
        from math_utils.field_calculation_functions import get_vector_cloud_offset_2D_max    

        detected_poles = [fo for fo in detected_field_objects if fo.type == "Pole"]
        
        print(f"detected {len(detected_poles)} poles")
        detected_poles.sort(key=lambda pole: pole.distance.convert(Coordinate.CYLINDRICAL)[1])
        
        w, h = self.calculate_dimensions(*detected_poles)
        
        if w is None:
            return False
        
        if self.half_size == (0, 0, 0):
            self.half_size = TupleVector3((w/2, h/2, 0))
        else:
            self.half_size[0] = (self.half_size[0] + w/2) / 2
            self.half_size[1] = (self.half_size[1] + h/2) / 2

        rospy.loginfo(f"Field dimensions: {self.half_size[0]*2} x {self.half_size[1]*2}")
        return True

    def calculate_field_offset(self, detected_field_objects):
        from math_utils.field_calculation_functions import get_vector_cloud_offset_2D_max
        base = [fo.distance + self.distance + self.angle_offset for fo in self.field_objects]
        compare = [fo.distance * (1, 1, 0) for fo in detected_field_objects]

        origin_offset, angle_offset = get_vector_cloud_offset_2D_max(base, compare, 0.2, 3)

        if origin_offset is None:
            return False
        
        self.distance += origin_offset
        self.angle_offset += angle_offset
        return True

    def get_abs_field_object(self, fo: FieldObject):
        local_fo = copy.copy(fo)
        local_fo.distance = local_fo.distance - self.angle_offset - self.distance
        return local_fo
    
    def get_player_relative_field_object(self, fo: FieldObject):
        fo_copy = copy.copy(fo)
        fo_copy.distance = fo_copy.distance + self.distance + self.angle_offset
        return fo_copy

    def update_field_objects(self, detected_objects: List[FieldObject]):
        update_counter = 0
        new_counter = 0

        detected_local = list(map(lambda fo: self.get_abs_field_object(fo), detected_objects))
        for fo in detected_local:
            for i in range(len(self.field_objects)):
                self_obj = self.field_objects[i]
                if fo.distance.distance_xy(self_obj.distance) < 0.15:
                    if type(self_obj) == GenericObject:
                        self.field_objects.pop(i)
                        self.field_objects.insert(i, fo)

                    elif type(self_obj) != Pole:
                        self_obj.distance = (fo.distance + self_obj.distance) / 2

                    update_counter += 1
                    break
            else:
                self.field_objects.append(fo)
                new_counter += 1

        print(f"updated {update_counter} field objects, added {new_counter} new field objects")
        
    def update(self):
        if self.field_component_sub.data is None:
            return False
        
        detected_field_objects = [FieldObject.from_field_component(fc) for fc in self.field_component_sub.data]

        if not self.initialized:
            if self.update_field_dimensions(detected_field_objects):
                self.initialized = True
                self.field_objects = self.generate_poles(*self.half_size.tuple[:2])
        else:
            self.calculate_field_offset(detected_field_objects)

        if self.initialized:
            player = Player(-self.distance, (0.3, 0.3, 0.3))
            self.update_field_objects([player, *detected_field_objects])

        print(f"total: {len(self.field_objects)} field objects")  

        return True  

    def draw(self, screen: Screen, draw_text=False, draw_center=False, draw_icon=True, draw_rect=False, draw_cube=False):
        for fo in self.field_objects:
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

