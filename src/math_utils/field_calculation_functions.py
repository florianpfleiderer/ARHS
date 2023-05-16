#!/usr/bin/env python

import rospy

from math import sqrt
from math_utils.math_function_utils import cosd, sind, asind
from field_components.field_components import Pole
from typing import Tuple

def cosine_theorem(pole_a : Pole, pole_b : Pole):
    '''Calculates the distance between two poles using the cosine theorem.
    
    Args:
        pole_a: first pole
        pole_b: second pole
    
    Returns:
        distance between the two poles
    '''
    a = pole_a.distance.tuple[0]
    b = pole_b.distance.tuple[0]
    gamma = abs(pole_a.distance.tuple[2] - pole_b.distance.tuple[2])

    return sqrt(a**2 + b**2 - 2 * a * b * cosd(gamma))

def get_position(pole_a: Pole, pole_b: Pole, pole_c: Pole) -> Tuple:
    '''This function calculates the angle from pole to Robot.
    Poles given to this function should be ordered from left to right.
    The function check_poles() does this.
    '''
    if pole_c:
        if pole_a is None or pole_b is None:
            rospy.logwarn('Pole is None')
            return None
        if pole_a.position is None or pole_b.position is None:
            rospy.logwarn('Pole position is None')
            return None
        
        # case 1: pole a and b
        dist = cosine_theorem(pole_a, pole_b)
        gamma = abs(pole_a.distance[2] - pole_b.distance[2]) 

        rho = asind(pole_a.distance[0]/dist *sind(gamma))
        if pole_a.position[1] == 0:
            pos_x = pole_b.position[0] + pole_b.distance[0]*cosd(rho)
            pos_y = pole_b.position[1] + pole_b.distance[0]*sind(rho)
        elif pole_a.position[1] == 3:
            pos_x = pole_b.position[0] - pole_b.distance[0]*cosd(rho)
            pos_y = pole_b.position[1] - pole_b.distance[0]*sind(rho)

        # case 2: pole a and c
        dist = cosine_theorem(pole_a, pole_c)
        gamma = abs(pole_a.distance[2] - pole_c.distance[2]) 

        rho = asind(pole_a.distance[0]/dist *sind(gamma))
        if pole_a.position[1] == 0:
            pos_x1 = pole_c.position[0] + pole_c.distance[0]*cosd(rho)
            pos_y1 = pole_c.position[1] + pole_c.distance[0]*sind(rho)
        elif pole_a.position[1] == 3:
            pos_x1 = pole_c.position[0] - pole_c.distance[0]*cosd(rho)
            pos_y1 = pole_c.position[1] - pole_c.distance[0]*sind(rho)

        return (pos_x + pos_x1 / 2, pos_y + pos_y1 / 2)
    else:
        if pole_a is None or pole_b is None:
            rospy.logwarn('Pole is None')
            return None
        if pole_a.position is None or pole_b.position is None:
            rospy.logwarn('Pole position is None')
            return None
        
        # case 1: pole a and b
        dist = cosine_theorem(pole_a, pole_b)
        gamma = abs(pole_a.distance[2] - pole_b.distance[2]) 

        rho = asind(pole_a.distance[0]/dist *sind(gamma))
        if pole_a.position[1] == 0:
            pos_x = pole_b.position[0] + pole_b.distance[0]*cosd(rho)
            pos_y = pole_b.position[1] + pole_b.distance[0]*sind(rho)
        elif pole_a.position[1] == 3:
            pos_x = pole_b.position[0] - pole_b.distance[0]*cosd(rho)
            pos_y = pole_b.position[1] - pole_b.distance[0]*sind(rho)

        return (pos_x, pos_y)


