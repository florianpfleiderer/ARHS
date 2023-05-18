#!/usr/bin/env python

import rospy

from math import sqrt
from math_utils.math_function_utils import cosd, sind, asind
from field_components.field_components import Pole
from typing import Tuple, List
from math_utils.vector_utils import *
import random
import cv2
from visualization.imgops import *

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


def find_max_distance(vectors: List[TupleVector3]):
    '''Finds the maximum distance between two vectors in a list of vectors.'''
    v1 = max(vectors)
    v2 = max(vectors, key=lambda v: v1-v)

    return v1, v2

def find_distance(dist, vectors: List[TupleVector3]):
    '''Finds the given distance between two vectors in the given list'''
    for i in range(len(vectors)):
        for j in range(i + 1, len(vectors) - i):
            if abs(vectors[i].distance(vectors[j]) - dist) < 0.1:
                return vectors[i], vectors[j]

def get_vector_cloud_offset_2D_max(base: List[TupleVector3], compare: List[TupleVector3]):
    '''Calculates the offset distance and rotation of two clouds of vectors based on the maximum
    distance found in the compare list. The compare list must be a subset of the objects from the base list.'''
    v1_max, v2_max = find_max_distance(compare)
    base_v1, base_v2 = find_distance(v1_max.distance(v2_max), base)

    return get_vector_offset_2D([base_v1, base_v2], [v1_max, v2_max])

def get_vector_cloud_offset_2D(base: List[TupleVector3], compare: List[TupleVector3]):
    '''Calculates the average offset distance and rotation of two clouds of vectors.
    the base and compare lists are assumed to contain the same objects in the corresponding order.
    
    Args:
        base: base cloud of vectors
        compare: cloud of vectors to compare
    
    Returns:
        offset of the two clouds
    '''
    offsets = []
    offset_rotations = []

    for i in range(len(compare)-1):
        offset, offset_rotation = get_vector_offset_2D(base[i:i+2], compare[i:i+2])
        offsets.append(offset)
        offset_rotations.append(offset_rotation)

    offset = sum(offsets) / len(offsets)
    offset_rotation = sum(offset_rotations) / len(offset_rotations)

    return offset, offset_rotation

def get_vector_offset_2D(base: List[TupleVector3], compare: List[TupleVector3]):
    '''Calculates the offset distance and angle of two sets of two vectors.'''

    offset_angle = (base[1] - base[0]).angle(compare[1] - compare[0])
    offset_rotation = TupleRotator3((offset_angle, 0, 0))

    offset = compare[0] - offset_rotation - base[0]

    return offset, offset_rotation

def get_random_vector_cloud(count=10):
    return [TupleVector3(((random.random() - 0.5) * 20, (random.random() - 0.5) * 20, 0)) for i in range(count)]

def draw_vector_cloud(image, cloud: List[TupleVector3], color: Tuple[float, float, float] = (0, 0, 255)):
    dim = image.shape
    center = (int(dim[0]/2), int(dim[1]/2))
    for v in cloud:
        cv2.arrowedLine(image, center, (center[0] + int(v.tuple[0] * 20), center[1] + int(v.tuple[1] * 20)), color, 2)

def draw_vector(image, vector: TupleVector3, point: TupleVector3, color: Tuple[float, float, float] = (0, 0, 255)):
    dim = image.shape
    center = (int(dim[0]/2), int(dim[1]/2))
    cv2.arrowedLine(image, (center[0] + int(point.tuple[0] * 20), center[1] + int(point.tuple[1] * 20)),
                           (center[0] + int((point + vector).tuple[0] * 20), center[1] + int((point + vector).tuple[1] * 20)), color, 2)

if __name__ == "__main__":
    base = get_random_vector_cloud(10)

    winname = "test"
    random_intensity = TrackbarParameter(1, "random_intensity", winname, value_factor=0.1)

    while True:
        img = empty_image((500, 500))
        random_rotator = TupleRotator3((random.random() * 10 * random_intensity.get_value(True), 0, 0))
        random_offset = TupleVector3((random.random() - 0.5, random.random() - 0.5, 0)) * 2 * random_intensity.get_value(True)
        compare = [v + random_offset + random_rotator for v in base]

        compare.pop(2)
        compare.pop(5)
        compare.pop(6)
        random.shuffle(compare)

        offset, offset_rotation = get_vector_cloud_offset_2D_max(base, compare)

        print(offset, random_offset)
        print(offset_rotation, random_rotator)

        result = [v - offset_rotation - offset for v in compare]

        v1, v2 = find_max_distance(base)
        draw_vector(img, v2 - v1, v1, (255, 255, 255))

        draw_vector_cloud(img, base, (0, 255, 0))
        draw_vector_cloud(img, compare, (255, 0, 0))
        draw_vector_cloud(img, result, (0, 0, 255))
        draw_vector_cloud(img, [offset], (0, 255, 255))

        cv2.putText(img, "base", (0, 490), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
        cv2.putText(img, "compare", (0, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))
        cv2.putText(img, "result", (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
        cv2.imshow(winname, img)

        cv2.waitKey(10)
        time.sleep(3)