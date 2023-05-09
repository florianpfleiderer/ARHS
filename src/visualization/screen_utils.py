#!/usr/bin/env python

from globals import globals
from math_utils.math_function_utils import *
from enum import Enum
import numpy as np

def empty_image(dimensions):
    return np.zeros([dimensions[1], dimensions[0], 3], dtype=np.uint8)

class ProjectionType(Enum):
    PLANAR = 0
    SPHERICAL = 1

def screen_angle_to_pos(angle, image_dimension, FOV, projection_type):
    if projection_type == ProjectionType.PLANAR:
        return round((1 + tand(angle) / tand(FOV / 2)) * image_dimension / 2)
    elif projection_type == ProjectionType.SPHERICAL:
        return round((1 / 2 + angle / FOV) * image_dimension)
    
def screen_pos_to_angle(pos, image_dimension, FOV, projection_type):
    if projection_type == ProjectionType.PLANAR:
        return atand((2 * pos / image_dimension - 1) * tand(FOV / 2))
    elif projection_type == ProjectionType.SPHERICAL:
        return (pos / image_dimension - 1 / 2) * FOV

if __name__ == "__main__":
    dim = globals.KINECT_DIMENSIONS[0] #640
    FOV = globals.KINECT_FOV[0] #62
    pt = ProjectionType.PLANAR

    # for x in range(50):
    #     print(x, angle_to_pos(pos_to_angle(x, 0, 0, dim, FOV, pt), 0, 0, dim, FOV, pt))

    print(screen_pos_to_angle(0, dim, FOV, pt))
    print(screen_pos_to_angle(100, dim, FOV, pt))
    print(screen_pos_to_angle(150, dim, FOV, pt))
    print(screen_pos_to_angle(200, dim, FOV, pt))
    print(screen_pos_to_angle(320, dim, FOV, pt))
    print(screen_pos_to_angle(400, dim, FOV, pt))
    print(screen_pos_to_angle(640, dim, FOV, pt))

    print(screen_angle_to_pos(22.45, dim, FOV, pt))
    assert screen_angle_to_pos(screen_pos_to_angle(100, dim, FOV, pt), dim, FOV, pt) == 100
    
    pt = ProjectionType.SPHERICAL
    FOV = 360
    dim = 360

    print(screen_pos_to_angle(0, dim, FOV, pt))
    print(screen_pos_to_angle(100, dim, FOV, pt))
    print(screen_pos_to_angle(360, dim, FOV, pt))
    assert screen_angle_to_pos(screen_pos_to_angle(100, dim, FOV, pt), dim, FOV, pt) == 100