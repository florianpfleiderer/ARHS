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
    def __init__(self, name, origin_offset, dimensions, FOV, projection):
        self.name = name
        self.origin_offset = origin_offset
        self.dimensions = dimensions
        self.FOV = FOV
        self.projection = projection
        self.image = empty_image(dimensions)
        cv2.namedWindow(self.name, cv2.WINDOW_AUTOSIZE)
        self.show_image()

    def get_local_distance(self, distance):
        return cartesian_to_spherical(subtract_vectors(spherical_to_cartesian(distance), self.origin_offset))
    
    def get_global_distance(self, local_distance):
        return cartesian_to_spherical(sum_vectors(spherical_to_cartesian(local_distance), self.origin_offset))

    def get_rect_field_object(self, field_obj):
        local_dist = self.get_local_distance(field_obj.spherical_distance)
        global_corner = sum_vectors(field_obj.spherical_distance, field_obj.half_size)
        local_corner = self.get_local_distance(global_corner)
        local_half_size = subtract_vectors(local_corner, local_dist)
        return self.get_rect(local_dist[2] - local_half_size[2], local_dist[2] + local_half_size[2], local_dist[1] - local_half_size[1], local_dist[1] + local_half_size[1])

    def get_rect(self, *angles):
        phi_min, phi_max, theta_min, theta_max = angles
        x = int(angle_to_pos(phi_max, self.dimensions[1], self.FOV[0], self.projection))
        y = int(angle_to_pos(theta_min - 90, self.dimensions[0], self.FOV[1], self.projection))
        w = int(angle_to_pos(phi_min, self.dimensions[1], self.FOV[0], self.projection) - x)
        h = int(angle_to_pos(theta_max - 90, self.dimensions[0], self.FOV[1], self.projection) - y)
        return x, y, w, h
    
    def get_angles(self, *rect):
        x, y, w, h = rect
        phi_min = pos_to_angle(x + w, self.dimensions[1], self.FOV[0], self.projection)
        phi_max = pos_to_angle(x, self.dimensions[1], self.FOV[0], self.projection)
        theta_min = pos_to_angle(y, self.dimensions[0], self.FOV[1], self.projection) + 90
        theta_max = pos_to_angle(y + h, self.dimensions[0], self.FOV[1], self.projection) + 90
        return phi_min, phi_max, theta_min, theta_max
    
    def get_center(self, rect):
        x, y, w, h = rect
        return int(x + w / 2), int(y + h / 2)
    
    def get_center_field_object(self, field_obj):
        return self.get_center(self.get_rect_field_object(field_obj))

    def draw_object(self, obj, draw_text=True):
        if issubclass(type(obj), fc.FieldObject):
            x, y, w, h = self.get_rect_field_object(obj)
            cv2.rectangle(self.image, (x, y), (x+w, y+h), obj.color.default, 1)
            cv2.circle(self.image, self.get_center((x, y, w, h)), 2, Color.ORANGE.default, -1)
            if draw_text:
                cv2.putText(self.image, str(obj), (x, int(y - 50 * CV2_DEFAULT_FONT_SCALE)),
                            CV2_DEFAULT_FONT, CV2_DEFAULT_FONT_SCALE,
                            Color.YELLOW.default, CV2_DEFAULT_THICKNESS, cv2.LINE_AA)    
                cv2.putText(self.image, f"{x}, {y}, {w} x {h}", (x, int(y - 10 * CV2_DEFAULT_FONT_SCALE)),
                            CV2_DEFAULT_FONT, CV2_DEFAULT_FONT_SCALE,
                            Color.YELLOW.default, CV2_DEFAULT_THICKNESS, cv2.LINE_AA)
                
    def calculate_screen_area_field_object(self, field_obj):
        return self.calculate_screen_area(self.get_rect_field_object(field_obj))
    
    def calculate_screen_ratio_field_object(self, field_obj):
        return self.calculate_screen_ratio(self.get_rect_field_object(field_obj))

    def calculate_screen_area(self, rect):
        x, y, w, h = rect
        return w * h
    
    def calculate_screen_ratio(self, rect):
        x, y, w, h = rect
        return w / h
    
    def create_field_object(self, rect, r, obj_type):
        phi_min, phi_max, theta_min, theta_max = self.get_angles(*rect)
        local_distance = (r, (theta_min + theta_max) / 2, (phi_min + phi_max) / 2)
        local_corner = (r, theta_max, phi_max)

        global_distance = self.get_global_distance(local_distance)
        global_corner = self.get_global_distance(local_corner)
        global_size = subtract_vectors(global_corner, global_distance)

        return obj_type(global_distance, global_size)

    def show_image(self):
        cv2.imshow(self.name, self.image)

    @classmethod
    def KinectScreen(cls, name):
        return cls(name, KINECT_OFFSET, KINECT_DIMENSIONS, KINECT_FOV, ProjectionType.PLANAR)
    
    @classmethod
    def LaserScreen(cls, name):
        return cls(name, LASER_OFFSET, LASER_DIMENSIONS, LASER_FOV, ProjectionType.SPHERICAL)
    
    @classmethod
    def OriginScreen(cls, name):
        return cls(name, (0, 0, 0), (0, 0), (360, 360), ProjectionType.SPHERICAL)


class ScreenObject:
    def __init__(self, theta_min, theta_max, alpha_min, alpha_max):
        self.theta_min = theta_min
        self.theta_max = theta_max
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max

        self.properties = ScreenPosition(theta_min=theta_min, theta_max=theta_max, alpha_min=alpha_min, alpha_max=alpha_max)

    def get_screen_area(self, screen):
        return screen.calculate_screen_area(self)

    def get_ratio(self, screen):
        return screen.calculate_screen_ratio(self)
    
    def get_center_angles(self):
        return (self.theta_max + self.theta_min) / 2, (self.alpha_max + self.alpha_min) / 2
    
    @classmethod
    def from_screen_position(cls, scp):
        return cls(scp.theta_min, scp.theta_max, scp.alpha_min, scp.alpha_max)
    
    def merge(self, *screen_objects):
        theta_min = self.theta_min
        alpha_min = self.alpha_min
        theta_max = self.theta_max
        alpha_max = self.alpha_max
        for so in screen_objects:
            theta_min = max(theta_min, so.theta_min)
            alpha_min = max(alpha_min, so.alpha_min)
            theta_max = min(theta_max, so.theta_max)
            alpha_max = min(alpha_max, so.alpha_max)

        return ScreenObject(theta_min, theta_max, alpha_min, alpha_max)

    def __str__(self):
        return f"ScreenObject: {round(self.theta_min, 2)}t, {round(self.theta_max, 2)}t, {round(self.alpha_min, 2)}a, {round(self.alpha_max, 2)}a"

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

    fo = sc.create_field_object((100, 100, 100, 100), 1, fc.Robot)

    print(sc.get_angles(100, 100, 100, 200))
    print(sc.get_rect(*sc.get_angles(100, 100, 100, 200)))

    print(cartesian_to_spherical(spherical_to_cartesian((1, 90, 0))))

    print("--")
    print(sum_vectors(spherical_to_cartesian((1, 90, 0)), sc.origin_offset))
    print(cartesian_to_spherical(sum_vectors(spherical_to_cartesian((1, 90, 0)), sc.origin_offset)))
    print(sc.get_global_distance((1, 90, 0)))
    print(sc.get_local_distance(sc.get_global_distance((1, 90, 0))))
    print(sc.get_rect_field_object(fo))
    print(fo.spherical_distance, fo.half_size)
    print(fo)

    sc.draw_object(fo)

    lsc = Screen.LaserScreen("Laser")
    lsc.draw_object(fo)
    lsc.show_image()

    sc.show_image()
    while True:
        cv2.waitKey(10)


def laser_screen_test():
    sc = Screen.LaserScreen("Laser")
    fo = sc.create_field_object(sc.get_rect(0, 50, 20, 90), 1, fc.LaserPoint)

    sck = Screen.KinectScreen("Kinect")
    sck.draw_object(fo)
    sc.draw_object(fo)

    sck.show_image()
    sc.show_image()
    while True:
        cv2.waitKey(10)

# test screen object creation and correct visualization
if __name__ == "__main__":
    laser_screen_test()