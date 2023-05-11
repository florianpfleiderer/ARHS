#!/usr/bin/env python

from player.msg import ScreenPosition, PolarVector2
from globals.globals import *
from globals.tick import *
from math_utils.math_function_utils import *
from math_utils.vector_utils import *
from data_utils.data_validation import *
from enum import Enum
import numpy as np
import field_components.field_components as fc
from field_components.colors import Color
from visualization.screen_utils import *

class Screen:
    def __init__(self, name, dimensions, FOV, projection, origin_offset, angle_offset):
        self.name = name
        self.origin_offset = origin_offset
        self.dimensions = dimensions
        self.FOV = FOV
        self.projection = projection
        self.angle_offset = angle_offset
        self.image = empty_image(dimensions)

    def get_local_distance(self, distance):
        return distance - self.origin_offset
    
    def get_global_distance(self, local_distance):
        return local_distance + self.origin_offset

    def get_rect(self, obj):
        if type(obj) is tuple:
            alpha_min, alpha_max, theta_min, theta_max = obj
        elif issubclass(type(obj), fc.FieldObject):
            alpha_min, alpha_max, theta_min, theta_max = self.get_angles(obj)

        x = int(screen_angle_to_pos(- alpha_max, self.dimensions[0], self.FOV[0], self.projection))
        y = int(screen_angle_to_pos(theta_min - 90, self.dimensions[1], self.FOV[1], self.projection))
        w = int(screen_angle_to_pos(- alpha_min, self.dimensions[0], self.FOV[0], self.projection) - x)
        h = int(screen_angle_to_pos(theta_max - 90, self.dimensions[1], self.FOV[1], self.projection) - y)
        return x, y, w, h
    
    def get_angles(self, obj):
        if type(obj) is tuple:
            x, y, w, h = obj
            alpha_min = - screen_pos_to_angle(x + w, self.dimensions[0], self.FOV[0], self.projection)
            alpha_max = - screen_pos_to_angle(x, self.dimensions[0], self.FOV[0], self.projection)
            theta_min = screen_pos_to_angle(y, self.dimensions[1], self.FOV[1], self.projection) + 90
            theta_max = screen_pos_to_angle(y + h, self.dimensions[1], self.FOV[1], self.projection) + 90
        elif issubclass(type(obj), fc.FieldObject):
            alpha_min, alpha_max, theta_min, theta_max = obj.get_angles()
        return alpha_min, alpha_max, theta_min, theta_max
    
    def get_center(self, obj):
        if type(obj) is tuple:
            x, y, w, h = obj
        elif issubclass(type(obj), fc.FieldObject):
            x, y, w, h = self.get_rect(obj)
        return int(x + w / 2), int(y + h / 2)

    def calculate_screen_area(self, obj):
        if type(obj) is tuple:
            x, y, w, h = obj
        elif issubclass(type(obj), fc.FieldObject):
            x, y, w, h = self.get_rect_field_object(obj)
        return w * h
    
    def calculate_screen_ratio(self, obj):
        if type(obj) is tuple:
            x, y, w, h = obj
        elif issubclass(type(obj), fc.FieldObject):
            x, y, w, h = self.get_rect(obj)
        return w / h
    
    def create_field_object(self, rect, r, obj_type):
        phi_min, phi_max, theta_min, theta_max = self.get_angles(rect)
        local_distance = TupleVector3((r, (theta_min + theta_max) / 2, (phi_min + phi_max) / 2), Coordinate.SPHERICAL)
        local_corner = TupleVector3((r, theta_max, phi_max), Coordinate.SPHERICAL)

        global_distance = self.get_global_distance(local_distance)
        global_corner = self.get_global_distance(local_corner)
        global_size = global_corner - global_distance

        return obj_type(global_distance, global_size)

    def show_image(self):
        if self.image is not None:
            cv2.imshow(self.name, self.image)

    def draw_object(self, obj, draw_text=True):
        if issubclass(type(obj), fc.FieldObject):
            rect = self.get_rect(obj)
            x, y, w, h = rect
            cv2.rectangle(self.image, rect, obj.color.default, 1)
            cv2.circle(self.image, self.get_center(rect), 2, Color.ORANGE.default, -1)
            if draw_text:
                cv2.putText(self.image, str(obj), (x, int(y - 50 * CV2_DEFAULT_FONT_SCALE)),
                            CV2_DEFAULT_FONT, CV2_DEFAULT_FONT_SCALE,
                            Color.YELLOW.default, CV2_DEFAULT_THICKNESS, cv2.LINE_AA)    
                cv2.putText(self.image, f"{x}, {y}, {w} x {h}", (x, int(y - 10 * CV2_DEFAULT_FONT_SCALE)),
                            CV2_DEFAULT_FONT, CV2_DEFAULT_FONT_SCALE,
                            Color.YELLOW.default, CV2_DEFAULT_THICKNESS, cv2.LINE_AA)
                
    @classmethod
    def KinectScreen(cls, name):
        return cls(name, KINECT_DIMENSIONS, KINECT_FOV, ProjectionType.PLANAR, TupleVector3(KINECT_OFFSET), TupleVector3(KINECT_ANGLE, Coordinate.SPHERICAL))
    
    @classmethod
    def LaserScreen(cls, name):
        return cls(name, LASER_DIMENSIONS, LASER_FOV, ProjectionType.SPHERICAL, TupleVector3(LASER_OFFSET), TupleVector3(coordinates=Coordinate.SPHERICAL))
    
    @classmethod
    def OriginScreen(cls, name):
        return cls(name, (0, 0), (360, 360), ProjectionType.SPHERICAL, TupleVector3(), TupleVector3(coordinates=Coordinate.SPHERICAL))

class ImageViewer:
    def __init__(self, name):
        self.name = name
        self.v = Validator()
        self.window = cv2.namedWindow(self.name)

    def show(self, image):
        if not self.v.guard_none(image):
            return
        
        cv2.imshow(self.name, image)

class TestParameter:
    def __init__(self, name, default):
        self._default = default
        self._test_value = default
        self.name = name

    def get_value(self, is_testing):
        return self._test_value if is_testing else self._default
    
    def set_value(self, value):
        self._test_value = value

    def get_default(self):
        return self._default
    
    def show(self):
        print(f"{self.name}: {str(self._test_value)}")

class TrackbarParameter(TestParameter):
    def __init__(self, default, trackbar_name, window_name, value_transformation=lambda x: x, inverse_transformation=lambda x: x):
        super().__init__(trackbar_name, value_transformation(default))
        self.trackbar_name = trackbar_name
        self.window = window_name
        self.value_transformation = value_transformation
        self.inverse_transformation = inverse_transformation

        cv2.namedWindow(self.window)
        cv2.createTrackbar(self.trackbar_name, self.window, self._default, 100, self.update_value)
        
    def set_value(self, value):
        cv2.setTrackbarPos(self.trackbar_name, self.window, self.value_transformation(value))

    def update_value(self, value):
        super().set_value(self.inverse_transformation(value))

class TestImage(TestParameter):
    def __init__(self, name, image):
        super().__init__(name, image)
    
    def show(self):
        ImageViewer(self.name).show(self.get_value(is_testing=True))


def kinect_screen_test():
    sc = Screen.KinectScreen("Kinect")

    print(sc.get_rect(sc.get_angles((100, 100, 100, 200))))
    assert sc.get_rect(sc.get_angles((100, 100, 100, 200))) == (100, 100, 100, 200)

    print(cartesian_to_spherical(spherical_to_cartesian((1, 90, 0))))
    assert cartesian_to_spherical(spherical_to_cartesian((1, 90, 0))) == (1, 90, 0)

    print("--")
    fo = sc.create_field_object((100, 100, 100, 100), 1, fc.Robot)
    print(fo)
    print(fo.distance, fo.half_size)

    print(fo.get_angles())
    print(sc.get_rect(fo))
    print(sc.get_angles(fo))
    print(sc.get_center(fo))

    sc.draw_object(fo)

    lsc = Screen.LaserScreen("Laser")
    lsc.draw_object(fo)
    lsc.show_image()

    sc.show_image()
    while True:
        cv2.waitKey(10)


def laser_screen_test():
    sc = Screen.LaserScreen("Laser")
    fo = sc.create_field_object(sc.get_rect((0, 50, 20, 90)), 1, fc.LaserPoint)

    sck = Screen.KinectScreen("Kinect")
    sck.draw_object(fo)
    sc.draw_object(fo)

    sck.show_image()
    sc.show_image()
    while True:
        cv2.waitKey(10)

# test screen object creation and correct visualization
if __name__ == "__main__":
    kinect_screen_test()
    laser_screen_test()