#!/usr/bin/env python

from math import *
from globals.globals import *
from visualization.screen_components import *
from field_components.field_components import *

def laser_phi(i, laser_scan):
    return 180 * (i * laser_scan.angle_increment / pi + 1)

def laser_theta(laser_scan, phi, height, distance, angle_offset=0):
    # d = laser_scan.ranges[laser_index(phi, laser_scan)]
    return atand(- height / distance) + 90 - angle_offset

def laser_index(phi, laser_scan):
    return round((phi / 180 - 1) * pi / laser_scan.angle_increment)

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

def limit_ranges(ranges, *bounds):
    return [min(max(r, bounds[0]), bounds[1]) for r in ranges]

def draw_laser_points(laser_scan, laser_screen, rgb_screen):
    for i, r in enumerate(laser_scan.ranges):
        if check_range(r, *LASER_RANGE):
            phi = laser_phi(i, laser_scan)
            theta = 90
            fo = laser_screen.create_field_object(laser_screen.get_rect((phi, phi, theta, theta)), r, LaserPoint)
            laser_screen.draw_object(fo, False)
            if check_range(phi, rgb_screen.FOV[0] / 2, - rgb_screen.FOV[0] / 2):
                rgb_screen.draw_object(fo, False)


def detect_edges(laser_scan, laser_ranges):       
    edges = []
    max_scan_angle = SCAN_MAX_ANGLE
    min_index = laser_index(max_scan_angle, laser_scan)
    max_index = laser_index(-max_scan_angle, laser_scan)

    for index in range(min_index, max_index, -1):
        range_diff = laser_ranges[index] - laser_ranges[index + 1]
        if range_diff > LASER_EDGE_THRESHOLD:
            edges.append((index, True, range_diff))
        
        elif range_diff < -LASER_EDGE_THRESHOLD:
            edges.append((index, False, range_diff))

    return edges

def detect_contours(laser_scan, laser_ranges):
    contours = []
    prev_range = LASER_RANGE[1]
    for i, r in enumerate(laser_ranges):
        if abs(r - prev_range) > LASER_EDGE_THRESHOLD:
            angle = laser_phi(i, laser_scan) - laser_scan.angle_increment * 90 / pi
            if len(contours) > 0 and contours[len(contours) - 1][1] is None:
                contours[len(contours) - 1][1] = angle

            if r <= LASER_RANGE[1] - 0.1 and r < prev_range:
                contours.append([angle, None])
        prev_range = r

    if contours[len(contours) - 1][1] is None:
        contours[len(contours) - 1][1] = SCAN_MAX_ANGLE

    return contours

def object_from_contour(contour, laser_scan, screen, object_type=GenericObject):
    max_phi, min_phi = contour
    center_phi = (max_phi + min_phi) / 2
    center_index = laser_index(center_phi, laser_scan)

    d = laser_scan.ranges[center_index]

    laser_height = LASER_OFFSET[2]
    
    center_theta = 90
    min_theta = center_theta - atand(0.3 / d)
    max_theta = center_theta + atand(laser_height / d)

    if check_range(d, *LASER_RANGE):
        rect = screen.get_rect((min_phi, max_phi, min_theta, max_theta)) 
        return screen.create_field_object(rect, d, object_type)

def test_detection():
    import time
    from topic_handlers import LaserSubscriber
    rospy.init_node("test")
    ls_sub = LaserSubscriber("laser", "robot1/front_laser/scan")
    time.sleep(1)
    laser_scan = ls_sub.get_scan()

    print(laser_index(100, laser_scan))
    print(laser_phi(80, laser_scan))
    assert laser_index(laser_phi(100, laser_scan), laser_scan) == 100

    print(detect_contours(laser_scan, limit_ranges(laser_scan.ranges, *LASER_RANGE)))

    rospy.spin()

if __name__ == "__main__":
    test_detection()