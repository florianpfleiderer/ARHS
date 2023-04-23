#!/usr/bin/env python

from math import *
from globals.globals import *
from visualization.screen_components import *

def laser_theta(i, laser_scan):
    return 180 * (1 + i * laser_scan.angle_increment / pi)

def laser_alpha(laser_scan, theta, offset, height):
    d = laser_scan.ranges[laser_index(theta, laser_scan)]
    return offset - atand(height / d)

def laser_index(theta, laser_scan):
    return int((theta / 180 - 1) * pi / laser_scan.angle_increment)

def range_denoise(ranges, kernel):
    new_ranges = []
    size = len(ranges)
    for i, r in enumerate(ranges):
        sum_value = 0

        for k in range(i - int(kernel/2), i + int(kernel/2)):
            if not 0 <= k < size or abs(ranges[k] - r) > LASER_EDGE_THRESHOLD:
                sum_value += r

            else:
                sum_value += ranges[k]

        mean = sum_value / kernel
        new_ranges.append(mean)

    return new_ranges

def show_laser(laser_scan, image):
    laser_points = []
    for i, r in enumerate(laser_scan.ranges):
        theta = laser_theta(i, laser_scan)
        alpha = laser_alpha(laser_scan, theta, KINECT_ANGLE, KINECT_HEIGHT)
        laser_points.append(ScreenObject(theta, theta, alpha, alpha))

    for laser_point in laser_points:
        laser_point.draw(image)