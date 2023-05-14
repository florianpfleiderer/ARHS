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
from typing import Tuple

class Screen:
    def __init__(self, name, dimensions, FOV, projection, origin_offset, angle_offset):
        self.name: str = name
        self.origin_offset: TupleVector3 = origin_offset
        self.dimensions: Tuple = dimensions
        self.FOV: Tuple = FOV
        self.projection: ProjectionType = projection
        self.angle_offset: TupleRotator3 = angle_offset
        self.image = empty_image(dimensions)

    def get_local_distance(self, distance) -> TupleVector3:
        return distance - self.origin_offset - self.angle_offset 
    
    def get_global_distance(self, local_distance) -> TupleVector3:
        return local_distance + self.angle_offset + self.origin_offset

    def get_rect(self, obj):
        if type(obj) is tuple:
            x_ang, y_ang, w_ang, h_ang = obj
        elif issubclass(type(obj), fc.FieldObject):
            x_ang, y_ang, w_ang, h_ang = self.get_angles(obj)

        x = round(screen_angle_to_pos(x_ang, self.dimensions[0], self.FOV[0], self.projection))
        y = round(screen_angle_to_pos(y_ang, self.dimensions[1], self.FOV[1], self.projection))
        w = round(screen_angle_to_pos(x_ang + w_ang, self.dimensions[0], self.FOV[0], self.projection)) - x
        h = round(screen_angle_to_pos(y_ang + h_ang, self.dimensions[1], self.FOV[1], self.projection)) - y
        return x, y, abs(w), abs(h)
    
    def get_angles(self, obj):
        if type(obj) is tuple:
            x, y, w, h = obj
            x_ang = screen_pos_to_angle(x, self.dimensions[0], self.FOV[0], self.projection)
            y_ang = screen_pos_to_angle(y, self.dimensions[1], self.FOV[1], self.projection)
            w_ang = screen_pos_to_angle(x + w, self.dimensions[0], self.FOV[0], self.projection) - x_ang
            h_ang = screen_pos_to_angle(y + h, self.dimensions[1], self.FOV[1], self.projection) - y_ang

        elif issubclass(type(obj), fc.FieldObject):
            corner_min = self.get_local_distance(obj.distance - obj.half_size).convert(Coordinate.SPHERICAL)
            corner_max = self.get_local_distance(obj.distance + obj.half_size).convert(Coordinate.SPHERICAL)
            
            x_ang = - corner_max[2]# alpha max
            y_ang = corner_min[1] - 90 # theta min
            w_ang = - corner_min[2] - x_ang # alpha min
            h_ang = corner_max[1] - 90 - y_ang # theta max
        return x_ang, y_ang, w_ang, h_ang
    
    def get_center(self, obj):
        if type(obj) is tuple:
            x, y, w, h = obj
        elif issubclass(type(obj), fc.FieldObject):
            x, y, w, h = self.get_rect(obj)
        return round(x + w / 2), round(y + h / 2)

    def calculate_screen_area(self, obj):
        if type(obj) is tuple:
            x, y, w, h = obj
        elif issubclass(type(obj), fc.FieldObject):
            x, y, w, h = self.get_rect(obj)
        return w * h
    
    def calculate_screen_ratio(self, obj):
        if type(obj) is tuple:
            x, y, w, h = obj
        elif issubclass(type(obj), fc.FieldObject):
            x, y, w, h = self.get_rect(obj)
        return w / h
    
    def create_field_object(self, rect, r, obj_type):
        x_ang, y_ang, w_ang, h_ang = self.get_angles(rect)
        local_distance = TupleVector3((r, y_ang + h_ang / 2 + 90, - (x_ang + w_ang / 2)), Coordinate.SPHERICAL)
        local_corner = TupleVector3((r, y_ang + h_ang + 90, - x_ang), Coordinate.SPHERICAL)

        global_distance = self.get_global_distance(local_distance)
        global_corner = self.get_global_distance(local_corner)
        global_size = global_corner - global_distance
        global_size.coordinates = Coordinate.CARTESIAN

        return obj_type(global_distance, global_size)

    def show_image(self):
        if self.image is not None:
            cv2.imshow(self.name, self.image)

    def is_in_sight(self, rect):
        x, y, w, h = rect
        if not check_range(x, 0, self.dimensions[0]):
            return False
        if not check_range(y , 0, self.dimensions[1]):
            return False

        return True

    def draw_object(self, obj, draw_text=True, draw_center=True):
        if issubclass(type(obj), fc.FieldObject):
            rect = self.get_rect(obj)

            # if not self.is_in_sight(rect):
            #     return
            
            x, y, w, h = rect
            if w < 0 or h < 0:
                rospy.logerr(f"Negative width or height: {w} x {h}, field object: {obj}")
                return
            
            if w > self.dimensions[0] or h > self.dimensions[1]:
                rospy.logerr(f"Object is too big: {w} x {h}, field object: {obj}")
                return
            
            cv2.rectangle(self.image, (x, y), (x+w, y+h), obj.color.default, 1)

            if draw_center:
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
        return cls(name, KINECT_DIMENSIONS, KINECT_FOV, ProjectionType.PLANAR, TupleVector3(KINECT_OFFSET), TupleRotator3(KINECT_ANGLE))
    
    @classmethod
    def LaserScreen(cls, name):
        return cls(name, LASER_DIMENSIONS, LASER_FOV, ProjectionType.SPHERICAL, TupleVector3(LASER_OFFSET), TupleRotator3())
    
    @classmethod
    def OriginScreen(cls, name):
        return cls(name, (0, 0), (360, 360), ProjectionType.SPHERICAL, TupleVector3(), TupleRotator3())
    
    @classmethod
    def BirdEyeScreen(cls, name):
        return cls(name, (640, 480), KINECT_FOV, ProjectionType.SPHERICAL, TupleVector3((0, 0, 10)), TupleRotator3((0, 90, 0)))

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

    rect = (100, 100, 100, 200)
    print(sc.get_angles(rect))
    print(sc.get_rect(sc.get_angles(rect)))
    assert sc.get_rect(sc.get_angles(rect)) == rect

    angle = (1, 90, 0)
    print(cartesian_to_spherical(spherical_to_cartesian(angle)))
    assert cartesian_to_spherical(spherical_to_cartesian(angle)) == angle

    print("--")
    fo = sc.create_field_object((100, 100, 100, 100), 1, fc.Robot)

    print("Field object:", fo)
    print("fo distance:", fo.distance)
    print("fo half size:", fo.half_size)

    print("sc rect:", sc.get_rect(fo))
    print("sc angles:", sc.get_angles(fo))
    print("sc center:", sc.get_center(fo))

    sc.draw_object(fo)

    lsc = Screen.LaserScreen("Laser")
    lsc.draw_object(fo)
    lsc.show_image()

    sc.show_image()
    while True:
        cv2.waitKey(10)


def laser_screen_test():
    sc = Screen.LaserScreen("Laser")
    fo = sc.create_field_object(sc.get_rect((-10, -10, 40, 90)), 1, fc.LaserPoint)

    sck = Screen.KinectScreen("Kinect")
    sck.draw_object(fo)
    sc.draw_object(fo)

    sck.show_image()
    sc.show_image()
    while True:
        cv2.waitKey(10)

def birdeye_screen_test():
    sc = Screen.BirdEyeScreen("BirdEye")
    center_x = 320
    center_y = 240

    dist = TupleVector3((1, 90, 0), Coordinate.SPHERICAL)
    fo = fc.GenericObject(dist, TupleVector3((1, 1, -1), Coordinate.SPHERICAL))

    for i in range(-180, 180, 10):
        fo = fc.GenericObject(TupleVector3((1, 90, i), Coordinate.SPHERICAL), TupleVector3((1, 1, -1), Coordinate.SPHERICAL))
        print(fo.distance.convert())
        sc.draw_object(fo, False)
        sc.show_image()
        cv2.waitKey(10)

    print(sc.get_local_distance(dist))
    print(sc.get_global_distance(TupleVector3((1, 100, 0), Coordinate.SPHERICAL)))

    test(sc.get_local_distance(sc.get_global_distance(dist)), dist)
    test(sc.get_local_distance(TupleVector3((0, 0, 0), Coordinate.SPHERICAL)).value(), (10, 90, 0))

    test((TupleVector3((0, 0, 0)) - TupleVector3((0, 0, 10))).convert(Coordinate.SPHERICAL), (10, 90, 0))

    # sc.draw_object(fo)
    sc.show_image()
    while True:
        cv2.waitKey(10)

# test screen object creation and correct visualization
if __name__ == "__main__":
    # kinect_screen_test()
    # laser_screen_test()
    birdeye_screen_test()