#!/usr/bin/env python
import math
import numpy as np

def tand(x):
    return math.tan(x * math.pi / 180)

def atand(x):
    return math.atan(x) * 180 / math.pi

def atan2d(y, x):
    return math.atan2(y, x) * 180 / math.pi

def sind(x):
    return math.sin(x * math.pi / 180)

def asind(x):
    if abs(x) > 1:
        if x < 1.001:
            x = 1
        elif x > -1.001:
            x = -1
    return math.asin(x) * 180 / math.pi

def cosd(x):
    return math.cos(x * math.pi / 180)

def acosd(x):
    if abs(x) > 1:
        if round(x, 5) == 1:
            x = 1
        elif round(x, 5) == -1:
            x = -1
        else:
            raise ValueError(f"Value {x} is not in domain")
    return math.acos(x) * 180 / math.pi

def check_range(value, min, max):
    low = value if min is None else min
    high = value if max is None else max
    return low <= value <= high

def safe_div(numerator, denominator):
    '''Performs safe division, returns numerator if denominator is 0.'''
    return numerator if denominator == 0 else numerator / denominator

def safe_percent(numerator, denominator):
    '''Performs safe division, returns 1 if any of the parameters are 0.'''
    return 1 if denominator == 0 or numerator == 0 else numerator / denominator

def relative_difference(value1, value2):
    '''Returns the absolute percentage difference of value1 to value2'''
    return abs(safe_percent(value1, value2) - 1)

def np_angle(npvector1: np.ndarray, npvector2: np.ndarray):
    l = np.linalg.norm(npvector1) * np.linalg.norm(npvector2)
    if l == 0:
        assert False
    dot = np.dot(npvector1, npvector2)
    return acosd(np.dot(npvector1, npvector2) / l) if l != 0 else 0

def np_area(npvector1: np.ndarray, npvector2: np.ndarray):
    return np.linalg.norm(np.cross(npvector1, npvector2))

def np_length(npvector: np.ndarray):
    return np.linalg.norm(npvector)

def np_angle_xy(npvector1: np.ndarray, npvector2: np.ndarray):
    return atan2d(npvector2[1], npvector2[0]) - atan2d(npvector1[1], npvector1[0])
