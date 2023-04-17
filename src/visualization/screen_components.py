#!/usr/bin/env python

from player.msg import ScreenPosition
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

class ImageViewer:
    def __init__(self, name, image):
        self.name = name
        self.image = image
        self.v = Validator()

    def is_valid(self):
        return self.v.guard_none(self.image)

    def show(self):
        if not self.is_valid():
            return
        
        cv2.imshow(self.name, self.image)

    def draw_objects(self, objects):
        if not self.is_valid():
            return
        
        for obj in objects:
            obj.draw(self.image)
