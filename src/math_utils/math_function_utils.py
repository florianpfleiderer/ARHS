#!/usr/bin/env python
import math

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
        if x < 1.001:
            x = 1
        elif x > -1.001:
            x = -1
    return math.acos(x) * 180 / math.pi

def check_range(value, min, max):
    low = value if min is None else min
    high = value if max is None else max
    return low <= value <= high