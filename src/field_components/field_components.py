#!/usr/bin/env python

from visualization.screen_components import *
from field_components.colors import *
from globals.globals import *

class FieldObject:
    def __init__(self, *properties):
        self.color_name, self.type, self.distance, self.screen_pos = properties

        self.screen_obj = ScreenObject.from_screen_position(self.screen_pos)

        self.area_detect_range = (None, None)
        self.ratio_detect_range = (None, None)

        self.color = Color.from_string(self.color_name)

    @classmethod
    def from_field_component(cls, properties):
        return cls(properties.color_name, properties.type, properties.player_distance, properties.screen_position)

    def check_parameters(self, color, ratio, area):
        if not self.color.in_range(color):
            return False
        
        if not check_range(self.area_detect_range, ratio):
            return False
        
        if not check_range(self.area_detect_range, area):
            return False
        
        return True

    def draw_text(self, window):
        cv2.putText(window, str(self), (self.screen_pos.x, self.screen_pos.y-10),
                    CV2_DEFAULT_FONT, CV2_DEFAULT_FONT_SCALE,
                    Color.YELLOW.default, CV2_DEFAULT_THICKNESS, cv2.LINE_AA)

    def draw(self, window):
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
    ratio_detect_range = (0.2, 0.7)

    def __init__(self, player_distance, screen_pos):
        super().__init__("YELLOW", "puck", player_distance, screen_pos)

class BluePuck(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (0.2, 0.7)

    def __init__(self, player_distance, screen_pos):
        super().__init__("BLUE", "puck", player_distance, screen_pos)

class YellowGoal(FieldObject):
    color = Color.YELLOW
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, None)

    def __init__(self, player_distance, screen_pos):
        super().__init__("YELLOW", "goal", player_distance, screen_pos)

class BlueGoal(FieldObject):
    color = Color.BLUE
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (1.5, None)

    def __init__(self, player_distance, screen_pos):
        super().__init__("BLUE", "goal", player_distance, screen_pos)

class Pole(FieldObject):
    color = Color.GREEN
    area_detect_range = (AREA_MIN, None)
    ratio_detect_range = (None, 0.4)

    ratio_detect_range = (None, None)

    def __init__(self, player_distance, screen_pos):
        super().__init__("GREEN", "pole", player_distance, screen_pos)
