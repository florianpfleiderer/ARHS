#!/usr/bin/env python

from math import *
from globals.globals import *

def laser_theta(i, laser_scan):
    return 180 * (1 + i * laser_scan.angle_increment / pi)

def laser_index(theta, laser_scan):
    return int((theta / 180 - 1) * pi / laser_scan.angle_increment)

def range_denoise(ranges, kernel):
    new_ranges = []
    size = len(ranges)
    for i, r in enumerate(ranges):
        sum_value = 0

        for k in range(i - int(kernel/2), i + int(kernel/2)):
            if not 0 <= k < size or abs(ranges[k] - ranges[i]) > LASER_EDGE_THRESHOLD:
                sum_value += ranges[i]

            else:
                sum_value += ranges[k]


        mean = sum_value / kernel
        new_ranges.append(mean)

    return new_ranges