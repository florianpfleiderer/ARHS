#!/usr/bin/env python
import math

def tand(x):
    return math.tan(x * math.pi / 180)

def atand(x):
    return math.atan(x) * 180 / math.pi

def sind(x):
    return math.sin(x * math.pi / 180)

def check_range(value, min, max):
    low = value if min is None else min
    high = value if max is None else max
    return low <= value <= high