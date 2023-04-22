#!/usr/bin/env python

from visualization.screen_components import *
from field_components.colors import *
from globals.globals import *

class FieldObject:
    def __init__(self, color_name, type, distance, screen_pos):
        self.color_name = color_name
        self.type = type
        self.distance = distance
        self.screen_pos = screen_pos

        self.screen_obj = ScreenObject.from_screen_position(self.screen_pos)

        self.area_detect_range = (None, None)
        self.ratio_detect_range = (None, None)

        self.color = Color.from_string(self.color_name)

    @classmethod
    def from_screen_object(cls, screen_object, depth_img):
        return cls("RED", "unknown", screen_object.get_field_vector(depth_img), screen_object.properties)

    @classmethod
    def from_field_component(cls, field_component):
        return cls(field_component.color_name, field_component.type, field_component.player_distance, field_component.screen_position)

    def check_parameters(self, color, ratio, area):
        if not self.color.in_range(color):
            return False
        
        if not check_range(self.area_detect_range, ratio):
            return False
        
        if not check_range(self.area_detect_range, area):
            return False
        
        return True

    def draw_text(self, window):
        cv2.putText(window, str(self), (self.screen_obj.x, self.screen_obj.y-10),
                    CV2_DEFAULT_FONT, CV2_DEFAULT_FONT_SCALE,
                    Color.YELLOW.default, CV2_DEFAULT_THICKNESS, cv2.LINE_AA)

    def draw(self, window, FOV=KINECT_FOV, projection_type=ProjectionType.PLANAR):
        self.screen_obj.set_FOV(FOV)
        self.screen_obj.set_dimensions((window.shape))
        self.screen_obj.set_projection_type(projection_type)
        self.screen_obj.draw(window)
        self.draw_text(window)

    def __str__(self) -> str:
        return f"{self.color_name} {self.type} {self.distance.r:.2f}m {self.distance.theta:.1f}d"

class Robot(FieldObject):
    color = Color.RED
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, None)

    def __init__(self, player_distance, screen_pos):
        super().__init__("RED", "robot", player_distance, screen_pos)

class YellowPuck(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?

    def __init__(self, player_distance, screen_pos):
        super().__init__("YELLOW", "puck", player_distance, screen_pos)

class BluePuck(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7) # (0.2, 0.4)?

    def __init__(self, player_distance, screen_pos):
        super().__init__("BLUE", "puck", player_distance, screen_pos)

class YellowGoal(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (3, 7) # (1.5, None)?

    def __init__(self, player_distance, screen_pos):
        super().__init__("YELLOW", "goal", player_distance, screen_pos)

class BlueGoal(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (3, 7) # (1.5, None)?

    def __init__(self, player_distance, screen_pos):
        super().__init__("BLUE", "goal", player_distance, screen_pos)

class Pole(FieldObject):
    color = Color.GREEN
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, 0.4)

    def __init__(self, player_distance, screen_pos):
        super().__init__("GREEN", "pole", player_distance, screen_pos)
