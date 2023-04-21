#!/usr/bin/env python

from player.msg import ScreenPosition, PolarVector2
from globals.globals import *
from math_utils.math_function_utils import *
from field_components.colors import *
from data_utils.data_validation import *

class ScreenObject:
    def __init__(self, x, y, w, h):
        self.x = int(x)
        self.y = int(y)
        self.w = int(w)
        self.h = int(h)

        self.properties = ScreenPosition(x=self.x, y=self.y, w=self.w, h=self.h)

    def get_center(self):
        cx = int(self.x + self.w/2)
        cy = int(self.y + self.h/2)
        return cx, cy

    def get_corner_points(self):
        return ((self.x, self.y), (self.x + self.w, self.y + self.h))
    
    def get_area(self):
        return self.w * self.h

    def get_ratio(self):
        return self.w / self.h
    
    def get_field_distance(self, depth_img):
        cx, cy = self.get_center()
        return depth_img[cy, cx]
    
    def get_field_angle(self, depth_img):
        cx = self.get_center()[0]
        w = depth_img.shape[1]
        return atand((1 - 2 * cx / w) * KINECT_TAN_X)
    
    def get_field_vector(self, depth_img):
        return PolarVector2(self.get_field_distance(depth_img), self.get_field_angle(depth_img))

    def draw_bounds(self, window):
        corners = self.get_corner_points()
        cv2.rectangle(window, corners[0], corners[1], Color.YELLOW.default, CV2_DEFAULT_THICKNESS)

    def draw_center(self, window):
        cv2.circle(window, self.get_center(), 2, Color.ORANGE.default, -CV2_DEFAULT_THICKNESS)

    def draw(self, window):
        self.draw_bounds(window)
        self.draw_center(window)

    def __str__(self) -> str:
        return f"({self.x}, {self.y}) {self.w}x{self.h}"
    
    def get_angles(self):
        ax_min = self.pos_to_angle(self.x)
        ax_max = self.pos_to_angle(self.x + self.w)
        ay_min = self.pos_to_angle(self.y)
        ay_max = self.pos_to_angle(self.y + self.h)
        return ax_min, ax_max, ay_min, ay_max

    @classmethod
    def angle_to_pos(cls, angle, image_dimension, FOV):
        return (1 - tand(angle) / tand(FOV / 2)) * image_dimension / 2
    
    @classmethod
    def pos_to_angle(cls, pos, image_dimension, FOV):
        return atand((1 - 2 * pos / image_dimension) * tand(FOV / 2))
    
    @classmethod
    def from_angles(cls, ax_min, ay_min, ax_max, ay_max, image_w, image_h, FOV_x, FOV_y):
        x_min = cls.angle_to_pos(ax_min, image_w, FOV_x)
        x_max = cls.angle_to_pos(ax_max, image_w, FOV_x)
        y_min = cls.angle_to_pos(ay_min, image_h, FOV_y)
        y_max = cls.angle_to_pos(ay_max, image_h, FOV_y)

        w = x_max - x_min
        h = y_max - y_min
        return cls(x_min, y_min, w, h)
    
    @classmethod
    def from_screen_position(cls, scp):
        return cls(scp.x, scp.y, scp.w, scp.h)
    
    @classmethod
    def from_tuple(cls, properties):
        return cls(properties[0], properties[1], properties[2], properties[3])
    
    def merge(self, *screen_objects):
        x_min, y_min = min([(so.x, so.y) for so in [self, *screen_objects]])
        x_max, y_max = max([(so.x + so.w, so.y + so.h) for so in [self, *screen_objects]])
        return ScreenObject(x_min, y_min, x_max - x_min, y_max - y_min)

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