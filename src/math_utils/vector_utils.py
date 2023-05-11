#!/usr/bin/env python

import math
from geometry_msgs.msg import Vector3
from player.msg import PolarVector2
import rospy
from math_utils.math_function_utils import *

def tup2_from_polarvector2(vector: PolarVector2):
    return (vector.r, vector.theta)

def tup3_from_polarvector2(vector: PolarVector2):
    return vector.r, vector.theta, vector.phi

def polarvector2_from_tup2(tup):
    return PolarVector2(tup[0], tup[1])

def add_polars(vector1, vector2):
    return cartesian_to_polar(polar_to_cartesian(vector1) + polar_to_cartesian(vector2))

def add_cartesian_to_polar(cartesian, polar):
    return cartesian_to_polar(cartesian + polar_to_cartesian(polar))

def polar_to_cartesian(polar_vector):
    r, theta = polar_vector
    return (r * cosd(theta), r * sind(theta))

def cartesian_to_polar(cartesian_vector):
    x, y = cartesian_vector
    return (math.sqrt(x ** 2 + y ** 2), math.atan2(y, x))

def polar_to_string(polar_vector):
    return f"r={polar_vector.r} theta={polar_vector.theta}"

def spherical_to_cartesian(spherical_vector):
    r, theta, phi = spherical_vector
    return (r * sind(theta) * cosd(phi), r * sind(theta) * sind(phi), r * cosd(theta))

def cartesian_to_spherical(cartesian_vector):
    x, y, z = cartesian_vector
    r = math.sqrt(x ** 2 + y ** 2 + z ** 2)
    theta = acosd(z / r)
    phi = atan2d(y, x)
    return (r, theta, phi)

def sum_vectors(*vectors):
    return (sum([vector[0] for vector in vectors]), sum([vector[1] for vector in vectors]), sum([vector[2] for vector in vectors]))

def subtract_vectors(vector1, vector2):
    return (vector1[0] - vector2[0], vector1[1] - vector2[1], vector1[2] - vector2[2])

if __name__ == "__main__":
    print(spherical_to_cartesian((1, 90, 90)))
    print(cartesian_to_spherical((0, 1, 0)))