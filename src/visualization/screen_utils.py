#!/usr/bin/env python

from math_utils.math_function_utils import *
from enum import Enum
import numpy as np

def empty_image(dimensions):
    return np.zeros([dimensions[0], dimensions[1], 3], dtype=np.uint8)

class ProjectionType(Enum):
    PLANAR = 0
    SPHERICAL = 1

def angle_to_pos(angle, image_dimension, FOV, projection_type):
    if projection_type == ProjectionType.PLANAR:
        return round((1 + tand(angle) / tand(FOV / 2)) * image_dimension / 2)
    elif projection_type == ProjectionType.SPHERICAL:
        return round((1 / 2 + angle / FOV) * image_dimension)
    
def pos_to_angle(pos, image_dimension, FOV, projection_type):
    if projection_type == ProjectionType.PLANAR:
        return atand((2 * pos / image_dimension - 1) * tand(FOV / 2))
    elif projection_type == ProjectionType.SPHERICAL:
        return (pos / image_dimension - 1 / 2) * FOV

if __name__ == "__main__":
    dim = 640
    FOV = 62
    pt = ProjectionType.PLANAR

    # for x in range(50):
    #     print(x, angle_to_pos(pos_to_angle(x, 0, 0, dim, FOV, pt), 0, 0, dim, FOV, pt))

    print(pos_to_angle(420, 0, 0, dim, FOV, pt))
    dim = 480
    FOV = 48.6