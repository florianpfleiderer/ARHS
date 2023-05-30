#!/usr/bin/env python

from globals.globals import *
from player.msg import *
from math_utils.vector_utils import *
from typing import *
from field_components.colors import Color
import sys
import cv2
from visualization.screen_utils import *
import random
from data_utils.topic_handlers import *

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

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "Robot", distance, half_size)

class Player(FieldObject):
    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "Player", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        # body + wheels
        cv2.ellipse(image, get_point_in_rect(rect, 0.65, 0.75), scale_rect(rect, 0.15, 0.2), 0, 0, 360, (10, 10, 10), -1)
        cv2.rectangle(image, get_point_in_rect(rect, 0, 0.35), get_point_in_rect(rect, 0.7, 0.85), (0, 0, 255), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.2, 0.8), scale_rect(rect, 0.15, 0.2), 0, 0, 360, (10, 10, 10), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.17, 0.8), scale_rect(rect, 0.06, 0.11), 0, 0, 360, (0, 200, 200), -1)

        # head shadow
        cv2.rectangle(image, get_point_in_rect(rect, 0.3, 0.4), get_point_in_rect(rect, 0.7, 0.55), (0, 0, 155), -1)

        # platform + yellow dots
        cv2.rectangle(image, get_point_in_rect(rect, -0.08, 0.34), get_point_in_rect(rect, 0.36, 0.42), (10, 10, 10), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.43, 0.45), scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.57, 0.45), scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.7, 0.45), scale_rect(rect, 0.04, 0.12), 0, 90, 270, (0, 100, 150), -1)

        # head
        cv2.rectangle(image, get_point_in_rect(rect, 0.35, 0.13), get_point_in_rect(rect, 0.85, 0.5), (0, 0, 255), -1)

        # right eye
        cv2.ellipse(image, get_point_in_rect(rect, 0.6, 0.12), scale_rect(rect, 0.1, 0.18), 0, 0, 360, (200, 200, 200), -1)
        # cv2.ellipse(image, get_point_in_rect(rect, 0.6, 0.12), scale_rect(rect, 0.1, 0.2), 0, 0, 360, (0, 0, 0), 1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.65, 0.13), scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 0), -1)

        # left eye
        cv2.ellipse(image, get_point_in_rect(rect, 0.875, 0.08), scale_rect(rect, 0.1, 0.18), 0, 0, 360, (200, 200, 200), -1)
        # cv2.ellipse(image, get_point_in_rect(rect, 0.875, 0.08), scale_rect(rect, 0.1, 0.2), 0, 0, 360, (0, 0, 0), 1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.925, 0.09), scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 0), -1)


class YellowPuck(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "YellowPuck", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.circle(image, get_point_in_rect(rect, 0.5, 0.5), round(w/2), self.color.default(), -1)

class BluePuck(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "BluePuck", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.circle(image, get_point_in_rect(rect, 0.5, 0.5), round(w/2), self.color.default(), -1)

class YellowGoal(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "YellowGoal", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.rectangle(image, get_point_in_rect(rect, 0, 0), get_point_in_rect(rect, 1, 1), self.color.default(), -1)

class BlueGoal(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, 10) # (1.5, None)?

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "BlueGoal", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.rectangle(image, get_point_in_rect(rect, 0, 0), get_point_in_rect(rect, 1, 1), self.color.default(), -1)

class Pole(FieldObject):
    color = Color.GREEN
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, 0.4)

    def __init__(self, distance, half_size):
        super().__init__(Color.GREEN, "Pole", distance, half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.circle(image, get_point_in_rect(rect, 0.5, 0.5), round(w/2), self.color.default(), -1)

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
        cv2.putText(image, "?", get_point_in_rect(rect, 0, 1), CV2_DEFAULT_FONT, 1, self.color.default(), CV2_DEFAULT_THICKNESS, cv2.LINE_AA)

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
        cv2.rectangle(image, get_point_in_rect(rect, 0.33, 0.5), get_point_in_rect(rect, 0.66, 0.8), shirt, -1)
        # head
        cv2.circle(image, get_point_in_rect(rect, 0.5, 0.3), round(w * 0.3), skin, -1)
        cv2.circle(image, get_point_in_rect(rect, 0.4, 0.25), round(w * 0.05), black, -1)
        cv2.circle(image, get_point_in_rect(rect, 0.6, 0.25), round(w * 0.05), black, -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.5, 0.37), scale_rect(rect, 0.16, 0.125), 0, 0, 180, dark_red, -1)
        # hands
        cv2.circle(image, get_point_in_rect(rect, 0.1, 0.35), round(w * 0.08), skin, -1)
        cv2.circle(image, get_point_in_rect(rect, 0.9, 0.35), round(w * 0.08), skin, -1)
        # feet
        cv2.ellipse(image, get_point_in_rect(rect, 0.35, 0.8), scale_rect(rect, 0.1, 0.07), 0, 0, 360, brown, -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.65, 0.8), scale_rect(rect, 0.1, 0.07), 0, 0, 360, brown, -1)

class Field(FieldObject):
    def __init__(self):
        super().__init__(Color.GREEN, "Field", TupleVector3((0, 0, 0)), TupleRotator3((0, 0, 0)))
        self.field_component_sub = FieldComponentsSubscriber()
        self.field_objects: List[FieldObject] = []
        self.origin: Pole = None

    def get_objects_by_class(self, class_name):
        return [o for o in self.field_objects if o.type == class_name]

    def calculate_dimensions(self, *poles):
        if len(poles) < 3:
            rospy.logwarn(f"Not enough poles to calculate dimensions, got {len(poles)}")
            return False
        
        # sort poles by angle phi
        sorted_poles = sorted(poles, key=lambda pole: pole.distance.convert(Coordinate.CYLINDRICAL)[1])

        pos1: TupleVector3 = sorted_poles[0].distance
        pos2: TupleVector3 = sorted_poles[1].distance
        pos3: TupleVector3 = sorted_poles[2].distance

        angle_threshold = 15
        ratio_threshold = 0.1

        angle = (pos2 - pos1).angle(pos3 - pos2)
        if angle > angle_threshold:
            rospy.logwarn(f"Poles are not in a straight line, got {angle}")
            return False
        
        dist12 = pos1.distance(pos2)
        dist23 = pos2.distance(pos3)
        detect_ratio = dist12 / dist23

        dim_factor = sorted([(dim, detect_ratio) for dim in zip(*DIMENSION_FACTORS)], key=lambda e: abs(e[0][0] - e[1]))[0][0]

        if abs(detect_ratio - dim_factor[0]) > ratio_threshold:
            rospy.logwarn(f"No matching ratio, got {detect_ratio}")
            return False
        
        w = max(dist12, dist23) * dim_factor[1]

        unit = ((pos3 + pos2 - 2 * pos1) / 2).unit_vector() # average unit vector of the two distances from pos1
        A_dist = pos1 - unit * dim_factor[2] * w
        G_dist = pos1 + unit * (1 - dim_factor[2]) * w
        
        return (w, w * 3/5), A_dist, G_dist

    def update(self):
        if self.field_component_sub.data is None:
            return
        
        detected_field_objects = [FieldObject.from_field_component(fc) for fc in self.field_component_sub.data]
        detected_field_objects = sorted(detected_field_objects, key=lambda fo: fo.distance.convert(Coordinate.CYLINDRICAL)[1])
        detected_poles = [fo for fo in detected_field_objects if fo.type == "Pole"]

        if len(detected_poles) < 3:
            return
        
        for i in range(len(detected_poles) - 2):
            result = self.calculate_dimensions(*detected_field_objects[i:i+3])
            if result != False:
                break
        
        if result == False:
            return
        
        initial_set = self.field_objects == []

        if initial_set:
            rospy.loginfo(f"Field dimensions: {result[0][0]} x {result[0][1]}, origin: {result[1]}")
            self.half_size = TupleVector3((-result[0][0]/2, -result[0][1]/2, 0))
            self.distance = -result[1] + self.half_size

            self.field_objects.extend(self.generate_poles(result[1], result[2]))
            return

        # for fo in detected_field_objects:
        #     if self.origin.distance.approx(fo.distance, 0.05):
        #         self.origin = (fo.distance + self.origin) / 2
        #         rospy.loginfo(f"updated origin: {self.origin}")
        #         break

        
    def generate_poles(self, A_pole_dist: TupleVector3, G_pole_dist: TupleVector3):
        poles = []
        A_dist = A_pole_dist * (1, 1, 0)
        G_dist = G_pole_dist * (1, 1, 0)

        AG_dist = G_dist - A_dist

        distances = [A_dist + AG_dist * x for x in [0.1, 0.25, 0.5, 0.75, 0.9]]
        distances.append(A_dist)
        distances.append(G_dist)

        AG_dist_tup = AG_dist.tuple
        AN_dist = TupleVector3((-AG_dist_tup[1], AG_dist_tup[0], 0)) * 3/5

        distances.extend([dist + AN_dist for dist in distances])

        poles = [Pole(dist, TupleVector3((0.05, 0.05, 0))) for dist in distances]

        return poles



    # def draw_icon(self, image, rect):
    #     pts = np.array([[0, 0], [0, 1], [1, 1], [1, 0]], np.int32)
    #     pts = pts.reshape((-1, 1, 2))
    #     cv2.polylines(image, [pts], True, self.color.default(), 2)

