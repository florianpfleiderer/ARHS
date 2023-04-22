#!/usr/bin/env python

from player.msg import ScreenPosition, PolarVector2
from globals.globals import *
from globals.tick import *
from math_utils.math_function_utils import *
from field_components.colors import *
from data_utils.data_validation import *
from enum import Enum
import numpy as np
from field_components.field_components import *
from field_components.colors import *

class ProjectionType(Enum):
    PLANAR = 0
    SPHERICAL = 1

class ScreenObject:
    def __init__(self, theta_min, theta_max, alpha_min, alpha_max, dimensions=(480, 640), FOV=KINECT_FOV, projection_type=ProjectionType.PLANAR):
        self.theta_min = theta_min
        self.theta_max = theta_max
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max

        self.dimensions = dimensions
        self.FOV = FOV
        self.projection_type = projection_type

        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0

        self.calculate_xywh()

        self.properties = ScreenPosition(theta_min=theta_min, theta_max=theta_max, alpha_min=alpha_min, alpha_max=alpha_max)

    def calculate_xywh(self):
        self.x = int(ScreenObject.angle_to_pos(self.theta_min, self.dimensions[1], self.FOV[0], self.projection_type))
        self.y = int(ScreenObject.angle_to_pos(self.alpha_min, self.dimensions[0], self.FOV[1], self.projection_type))
        self.w = int(ScreenObject.angle_to_pos(self.theta_max, self.dimensions[1], self.FOV[0], self.projection_type) - self.x)
        self.h = int(ScreenObject.angle_to_pos(self.alpha_max, self.dimensions[0], self.FOV[1], self.projection_type) - self.y)

    def set_FOV(self, FOV):
        self.FOV = FOV
        self.calculate_xywh()

    def set_dimensions(self, dimensions):
        self.dimensions = dimensions
        self.calculate_xywh()

    def set_projection_type(self, projection_type):
        self.projection_type = projection_type
        self.calculate_xywh()

    def get_center_point(self):
        cx = self.x + self.w / 2
        cy = self.y + self.h / 2
        return int(cx), int(cy)

    def get_corner_points(self):
        return (self.x, self.y), (self.x + self.w, self.y + self.h)
    
    def get_area(self):
        return self.w * self.h

    def get_ratio(self):
        return self.w / self.h
    
    def get_field_distance(self, depth_img):
        cx, cy = self.get_center_point()
        return depth_img[cy, cx]
    
    def get_field_angle(self):
        return (self.theta_max + self.theta_min) / 2
    
    def get_field_vector(self, depth_img):
        return PolarVector2(self.get_field_distance(depth_img), self.get_field_angle())

    def draw_bounds(self, window):
        corners = self.get_corner_points()
        cv2.rectangle(window, corners[0], corners[1], Color.YELLOW.default, CV2_DEFAULT_THICKNESS)

        cv2.putText(window, str(self), corners[0], CV2_DEFAULT_FONT, CV2_DEFAULT_FONT_SCALE, Color.YELLOW.default, CV2_DEFAULT_THICKNESS, cv2.LINE_AA)

    def draw_center(self, window):
        cv2.circle(window, self.get_center_point(), 2, Color.ORANGE.default, -CV2_DEFAULT_THICKNESS)

    def draw(self, window):
        self.draw_bounds(window)
        self.draw_center(window)

    def __str__(self) -> str:
        return f"({self.x}, {self.y}) {self.w}x{self.h}"
    
    @classmethod
    def angle_to_pos(cls, angle, image_dimension, FOV, projection_type):
        if projection_type == ProjectionType.PLANAR:
            return (1 - tand(angle) / tand(FOV / 2)) * image_dimension / 2
        elif projection_type == ProjectionType.SPHERICAL:
            return (1 / 2 - angle / FOV) * image_dimension
        
    @classmethod
    def pos_to_angle(cls, pos, image_dimension, FOV, projection_type):
        if projection_type == ProjectionType.PLANAR:
            return atand((1 - 2 * pos / image_dimension) * tand(FOV / 2))
        elif projection_type == ProjectionType.SPHERICAL:
            return (1 / 2 - pos / image_dimension) * FOV
    
    @classmethod
    def from_screen_position(cls, scp):
        return cls(scp.theta_min, scp.theta_max, scp.alpha_min, scp.alpha_max)
    
    @classmethod
    def from_rectangle_tuple(cls, rect, dimensions=(480, 640), FOV=KINECT_FOV, projection_type=ProjectionType.PLANAR):
        theta_min = cls.pos_to_angle(rect[0], dimensions[1], FOV[0], projection_type)
        theta_max = cls.pos_to_angle(rect[0] + rect[2], dimensions[1], FOV[0], projection_type)
        alpha_min = cls.pos_to_angle(rect[1], dimensions[0], FOV[1], projection_type)
        alpha_max = cls.pos_to_angle(rect[1] + rect[3], dimensions[0], FOV[1], projection_type)
        return cls(theta_min, theta_max, alpha_min, alpha_max, dimensions, FOV, projection_type)
    
    def merge(self, *screen_objects):
        theta_min, alpha_min = min([(so.theta_min, so.alpha_min) for so in [self, *screen_objects]])
        theta_max, alpha_max = max([(so.theta_max, so.alpha_max) for so in [self, *screen_objects]])
        return ScreenObject(theta_min, theta_max, alpha_min, alpha_max)

class ImageViewer:
    def __init__(self, name):
        self.name = name
        self.v = Validator()
        self.window = cv2.namedWindow(self.name)

    def show(self, image):
        if not self.v.guard_none(image):
            return
        
        cv2.imshow(self.name, image)

    def draw_objects(self, objects):
        if not self.v.guard_none(self.window):
            return
        
        for obj in objects:
            obj.draw(self.window)

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
    def __init__(self, default, trackbar_name, window_name):
        super().__init__(trackbar_name, default)
        self.trackbar_name = trackbar_name
        self.window = window_name

        cv2.namedWindow(self.window)
        cv2.createTrackbar(self.trackbar_name, self.window, default, 255, self.update_value)
        
    def set_value(self, value):
        cv2.setTrackbarPos(self.trackbar_name, self.window, value)

    def update_value(self, value):
        super().set_value(value)

class TestImage(TestParameter):
    def __init__(self, name, image):
        super().__init__(name, image)
    
    def show(self):
        ImageViewer(self.name).show(self.get_value(is_testing=True))

# test screen object creation and correct visualization
if __name__ == "__main__":
    test_image = TestImage("test", np.zeros((480, 640, 3), np.uint8))
    sos = [] 
    
    # sos.append(ScreenObject(KINECT_FOV[0], -KINECT_FOV[0], KINECT_FOV[1], -KINECT_FOV[1]))
    # sos.append(ScreenObject(KINECT_FOV[0] / 2, -KINECT_FOV[0] / 2, KINECT_FOV[1] / 2, -KINECT_FOV[1] / 2))
    # sos.append(ScreenObject(0, 0, 0, 0))

    # sos.append(ScreenObject.from_rectangle_tuple((100, 100, 440, 280)))
    # print(sos[0].x, sos[0].y, sos[0].w, sos[0].h, sos[0].theta_min, sos[0].theta_max, sos[0].alpha_min, sos[0].alpha_max)

    # sos.append(ScreenObject(22.5, -22.5, 14.8, -14.8))

    # for so in sos:
    #     so.draw(test_image.get_value(is_testing=True))


    so = ScreenObject.from_rectangle_tuple((100, 100, 440, 280))
    fo = FieldObject("RED", "test", PolarVector2(2, 0), so.properties)

    so.draw(test_image.get_value(is_testing=True))
    fo.draw(test_image.get_value(is_testing=True))

    test_image.show()
    while True:
        cv2.waitKey(10)