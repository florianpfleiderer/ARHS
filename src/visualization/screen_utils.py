#!/usr/bin/env python

from enum import Enum
from globals import globals
import math_utils.math_function_utils as mf

class ProjectionType(Enum):
    PLANAR = 0
    SPHERICAL = 1

def screen_angle_to_pos(angle, image_dimension, FOV, projection_type):
    if projection_type == ProjectionType.PLANAR:
        return round((1 + mf.tand(angle) / mf.tand(FOV / 2)) * image_dimension / 2)
    elif projection_type == ProjectionType.SPHERICAL:
        return round((1 / 2 + angle / FOV) * image_dimension)
    
def screen_pos_to_angle(pos, image_dimension, FOV, projection_type):
    if projection_type == ProjectionType.PLANAR:
        return mf.atand((2 * pos / image_dimension - 1) * mf.tand(FOV / 2))
    elif projection_type == ProjectionType.SPHERICAL:
        return (pos / image_dimension - 1 / 2) * FOV

def get_point_in_rect(rect, w_perc, h_perc):
    x, y, w, h = rect
    return (round(x + w * w_perc), round(y + h * h_perc))

def scale_rect(rect, w_perc, h_perc):
    x, y, w, h = rect
    return (round(w * w_perc), round(h * h_perc))

if __name__ == "__main__":
    dim = globals.KINECT_DIMENSIONS[1] # 640 / 480
    FOV = globals.KINECT_FOV[1] # 62 / 48.6
    pt = ProjectionType.PLANAR

    # for x in range(50):
    #     print(x, angle_to_pos(pos_to_angle(x, 0, 0, dim, FOV, pt), 0, 0, dim, FOV, pt))

    print(screen_pos_to_angle(0, dim, FOV, pt))
    print(screen_pos_to_angle(100, dim, FOV, pt))
    print(screen_pos_to_angle(150, dim, FOV, pt))
    print(screen_pos_to_angle(200, dim, FOV, pt))
    print(screen_pos_to_angle(320, dim, FOV, pt))
    print(screen_pos_to_angle(400, dim, FOV, pt))
    # print(screen_pos_to_angle(640, dim, FOV, pt))

    print(screen_angle_to_pos(22.45, dim, FOV, pt))
    assert screen_angle_to_pos(screen_pos_to_angle(100, dim, FOV, pt), dim, FOV, pt) == 100
    
    pt = ProjectionType.SPHERICAL
    FOV = 360
    dim = 360

    print(screen_pos_to_angle(0, dim, FOV, pt))
    print(screen_pos_to_angle(100, dim, FOV, pt))
    print(screen_pos_to_angle(360, dim, FOV, pt))
    assert screen_angle_to_pos(screen_pos_to_angle(100, dim, FOV, pt), dim, FOV, pt) == 100