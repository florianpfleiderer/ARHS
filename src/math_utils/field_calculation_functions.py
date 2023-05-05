#!/usr/bin/env python

from math import sqrt
from math_utils.math_function_utils import cosd, sind, asind
from field_components.field_components import Pole

def cosine_theorem(pole_a : Pole, pole_b : Pole):
    '''Calculates the distance between two poles using the cosine theorem.
    
    Args:
        pole_a: first pole
        pole_b: second pole
    
    Returns:
        distance between the two poles
    '''
    a = pole_a.spherical_distance[0]
    b = pole_b.spherical_distance[0]
    gamma = abs(pole_a.spherical_distance[1] - pole_b.spherical_distance[1])

    return sqrt(a**2 + b**2 - 2 * a * b * cosd(gamma))

def get_position(pole_a: Pole, pole_b: Pole):
    '''This function calculates the angle from pole to Robot.'''
    
    dist = cosine_theorem(pole_a, pole_b)

    rho = asind(pole_a.spherical_distance[0]/dist *sind(abs(pole_a.spherical_distance[1] - pole_b.spherical_distance[1])))
    pos_x = pole_b.position[0] - pole_b.spherical_distance[0]*cosd(rho)
    pos_y = pole_b.position[1] - pole_b.spherical_distance[0]*sind(rho)

    return pos_x, pos_y

