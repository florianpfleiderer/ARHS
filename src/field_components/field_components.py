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
    default_half_size = TupleVector3((0.3, 0.3, 0.3))

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "Robot", distance, Robot.default_half_size)

    def draw_icon(self, image, rect):
        # body + wheels
        cv2.ellipse(image, get_point_in_rect(rect, 0.35, 0.75), scale_rect(rect, 0.15, 0.2), 0, 0, 360, (10, 10, 10), -1)
        cv2.rectangle(image, get_point_in_rect(rect, 0.3, 0.35), get_point_in_rect(rect, 1, 0.85), (0, 0, 255), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.8, 0.8), scale_rect(rect, 0.15, 0.2), 0, 0, 360, (10, 10, 10), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.83, 0.8), scale_rect(rect, 0.06, 0.11), 0, 0, 360, (0, 200, 200), -1)

        # head shadow
        cv2.rectangle(image, get_point_in_rect(rect, 0.3, 0.4), get_point_in_rect(rect, 0.7, 0.55), (0, 0, 155), -1)

        # platform + yellow dots
        cv2.rectangle(image, get_point_in_rect(rect, 0.64, 0.34), get_point_in_rect(rect, 1.08, 0.42), (10, 10, 10), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.57, 0.45), scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.42, 0.45), scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.3, 0.45), scale_rect(rect, 0.04, 0.12), 0, -90, 90, (0, 100, 150), -1)

        # head
        cv2.rectangle(image, get_point_in_rect(rect, 0.15, 0.13), get_point_in_rect(rect, 0.65, 0.5), (10, 10, 10), -1)

        # left eye
        cv2.ellipse(image, get_point_in_rect(rect, 0.4, 0.22), scale_rect(rect, 0.1, 0.18), 0, -60, 190, (255, 255, 255), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.375, 0.1), scale_rect(rect, 0.08, 0.14), 0, -22, 158, (255, 255, 255), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.35, 0.23), scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 255), -1)
        cv2.line(image, get_point_in_rect(rect, 0.25, 0.2), get_point_in_rect(rect, 0.5, 0), (10, 10, 10), 3)
        cv2.line(image, get_point_in_rect(rect, 0.25, 0.18), get_point_in_rect(rect, 0.5, -0.03), (10, 10, 10), 3)


        # right eye
        cv2.ellipse(image, get_point_in_rect(rect, 0.125, 0.18), scale_rect(rect, 0.1, 0.18), 0, 5, 250, (255, 255, 255), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.15, 0.1), scale_rect(rect, 0.08, 0.14), 0, 35, 215, (255, 255, 255), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.075, 0.19), scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 255), -1)
        cv2.line(image, get_point_in_rect(rect, 0.25, 0.2), get_point_in_rect(rect, 0.05, -0.05), (10, 10, 10), 3)
        cv2.line(image, get_point_in_rect(rect, 0.25, 0.18), get_point_in_rect(rect, 0.05, -0.08), (10, 10, 10), 3)

class Player(FieldObject):
    default_half_size = TupleVector3((0.3, 0.3, 0.3))

    def __init__(self, distance, half_size):
        super().__init__(Color.RED, "Player", distance, Player.default_half_size)

    def draw_icon(self, image, rect):
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
        cv2.ellipse(image, get_point_in_rect(rect, 0.58, 0.45), scale_rect(rect, 0.06, 0.12), 0, 0, 360, (0, 100, 150), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.7, 0.45), scale_rect(rect, 0.04, 0.12), 0, 90, 270, (0, 100, 150), -1)

        # head
        cv2.rectangle(image, get_point_in_rect(rect, 0.35, 0.13), get_point_in_rect(rect, 0.85, 0.5), (0, 0, 255), -1)

        # right eye
        cv2.ellipse(image, get_point_in_rect(rect, 0.6, 0.12), scale_rect(rect, 0.1, 0.18), 0, 0, 360, (200, 200, 200), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.65, 0.13), scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 0), -1)

        # left eye
        cv2.ellipse(image, get_point_in_rect(rect, 0.875, 0.08), scale_rect(rect, 0.1, 0.18), 0, 0, 360, (200, 200, 200), -1)
        cv2.ellipse(image, get_point_in_rect(rect, 0.925, 0.09), scale_rect(rect, 0.05, 0.09), 0, 0, 360, (0, 0, 0), -1)

class YellowPuck(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?
    default_half_size = TupleVector3((0.05, 0.05, 0.2))

    def __init__(self, distance, half_size):
        super().__init__(Color.YELLOW, "YellowPuck", distance, YellowPuck.default_half_size)

    def draw_icon(self, image, rect):
        x, y, w, h = rect
        cv2.circle(image, get_point_in_rect(rect, 0.5, 0.5), round(w/2), self.color.default(), -1)

class BluePuck(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?
    default_half_size = TupleVector3((0.05, 0.05, 0.2))

    def __init__(self, distance, half_size):
        super().__init__(Color.BLUE, "BluePuck", distance, BluePuck.default_half_size)

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
    default_half_size = TupleVector3((0.05, 0.05, 0.3))

    def __init__(self, distance, half_size):
        super().__init__(Color.GREEN, "Pole", distance, Pole.default_half_size)

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
        self.angle_offset: TupleRotator3 = None

    def get_objects_by_class(self, class_name):
        return [o for o in self.field_objects if o.type == class_name]

    def calculate_dimensions(self, *poles):
        if len(poles) != 3:
            rospy.logwarn(f"Not enough poles to calculate dimensions, got {len(poles)}")
            return False

        pos1: TupleVector3 = poles[0].distance
        pos2: TupleVector3 = poles[1].distance
        pos3: TupleVector3 = poles[2].distance

        angle_threshold = 35
        ratio_threshold = 0.05

        angle = (pos2 - pos1).angle(pos3 - pos2)
        if angle > angle_threshold:
            rospy.logwarn(f"Poles not in straight line, got {angle:.2f}")
            return None, None
        
        dist12 = pos1.distance(pos2)
        dist23 = pos2.distance(pos3)
        detect_ratio = dist12 / dist23 if dist23 != 0 else 0

        dim_factor = sorted(zip(*DIMENSION_FACTORS), key=lambda e: abs(detect_ratio - e[0]))[0]

        if abs(detect_ratio - dim_factor[0]) > ratio_threshold:
            rospy.logwarn(f"No matching ratio, got {detect_ratio}")
            return None, None
        
        w = max(dist12, dist23) * dim_factor[1]
        return w, w * 3/5

    def update(self):
        from math_utils.field_calculation_functions import get_vector_cloud_offset_2D_max

        if self.field_component_sub.data is None:
            return
        
        detected_field_objects = [FieldObject.from_field_component(fc) for fc in self.field_component_sub.data]
        detected_poles = [fo for fo in detected_field_objects if fo.type == "Pole"]

        if len(detected_poles) < 3:
            return
        
        print(f"detected {len(detected_poles)} poles")
        detected_poles.sort(key=lambda pole: pole.distance.convert(Coordinate.CYLINDRICAL)[1])
        
        for i in range(len(detected_poles) - 2):
            w, h = self.calculate_dimensions(*detected_poles[i:i+3])
            if w is not None:
                break
        
        if w is None:
            if self.half_size.tuple == (0, 0, 0):
                return
            else:
                w = self.half_size.tuple[0] * 2
                h = self.half_size.tuple[1] * 2
        else:
            self.half_size = TupleVector3((w/2, h/2, 0))

        rospy.loginfo(f"Field dimensions: {w} x {h}")
        gen_poles = self.generate_poles(w, h)

        base = [pole.distance for pole in gen_poles]
        compare = [fo.distance * (1, 1, 0) for fo in detected_poles]
        origin_offset, angle_offset = get_vector_cloud_offset_2D_max(base, compare, 0.2, 3)

        if origin_offset is not None:
            self.distance = origin_offset
            self.angle_offset = angle_offset

            self.field_objects = gen_poles
            self.field_objects.append(Player(-origin_offset, (0.3, 0.3, 0.3)))
        else:
            rospy.logwarn("No offset found")

    def draw(self, screen: Screen, draw_text=False, draw_center=False, draw_icon=True, draw_rect=False):
        for fo in self.field_objects:
            fo_copy = copy.copy(fo)
            fo_copy.distance = fo_copy.distance + self.distance + self.angle_offset
            screen.draw_object(fo_copy, draw_text, draw_center, draw_icon, draw_rect)
        cv2.putText(screen.image, f"{self.half_size.tuple[0]*2:.2f} x {self.half_size.tuple[1]*2:.2f}", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        
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


    # def draw_icon(self, image, rect):
    #     pts = np.array([[0, 0], [0, 1], [1, 1], [1, 0]], np.int32)
    #     pts = pts.reshape((-1, 1, 2))
    #     cv2.polylines(image, [pts], True, self.color.default(), 2)

