#!/usr/bin/env python

from math import *
from globals.globals import *
from visualization.screen_components import *
from field_components.field_components import *
from sensor_msgs.msg import LaserScan

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

def draw_laser_points(laser_scan: LaserScan, laser_screen: Screen, rgb_screen: Screen):
    for i, r in enumerate(laser_scan.ranges):
        if check_range(r, *LASER_RANGE):
            x_ang = - laser_phi(i, laser_scan)
            y_ang = 0
            fo = laser_screen.create_field_object(laser_screen.get_rect((x_ang, y_ang, 0, 0)), r, LaserPoint)
            laser_screen.draw_object(fo, False)
            if check_range(x_ang, - rgb_screen.FOV[0] / 2, rgb_screen.FOV[0] / 2):
                rgb_screen.draw_object(fo, False)


def detect_edges(laser_scan, laser_ranges):
    # edge = (index, is_rising, range_diff)       
    edges = []
    max_scan_angle = SCAN_MAX_ANGLE
    min_index = laser_index(max_scan_angle, laser_scan)
    max_index = laser_index(-max_scan_angle, laser_scan)

    for index in range(min_index, max_index):
        range_diff = laser_ranges[index] - laser_ranges[index + 1]
        if range_diff > LASER_EDGE_THRESHOLD:
            edges.append((index, True, range_diff))
        
        elif range_diff < -LASER_EDGE_THRESHOLD:
            edges.append((index, False, range_diff))

    return edges

def draw_edges(laser_scan, edges, laser_screen: Screen):
    for edge in edges:
        x_ang = - laser_phi(edge[0], laser_scan)
        y_ang = -laser_screen.FOV[1] / 2
        w_ang = 0
        h_ang = laser_screen.FOV[1] - 5
        fo = laser_screen.create_field_object(laser_screen.get_rect((x_ang, y_ang, w_ang, h_ang)), laser_scan.range_max * 100, RisingEdge if edge[1] else FallingEdge)
        laser_screen.draw_object(fo, False, False)

def detect_contours(laser_scan, laser_ranges, edges):
    # contour = (max_phi, min_phi)
    #
    # R = rising edge, F = falling edge
    # detection algorithm:
    # example:
    # RRRFFFFRRRFFFFFRRRRFRFFRRF
    # 1. find all RF pairs
    # RR(RF)FFFRR(RF)FFFFRRR(RF)(RF)FRRF
    # 2. expand pairs into bigger islands [nRmF]
    # R[RRFF]FFR[RRFF]FFFRR[RRF][RFF]RRF
    # 3. continue expanding until biggest islands are found
    # [RRRFFFF][RRRFFFFF][RRRRF][RFF][RRF]
    #
    # result: 17 objects
    # 
    # By ensuring that the sequence starts with R and ends with F, all edges will be accounted for.

    contours = []



    return contours

def object_from_contour(contour, laser_scan: LaserScan, screen: Screen, object_type=GenericObject):
    max_phi, min_phi = contour
    center_phi = (max_phi + min_phi) / 2
    center_index = laser_index(center_phi, laser_scan)

    d = laser_scan.ranges[center_index]

    x_ang = - max_phi
    w_ang = max_phi - min_phi

    laser_height = LASER_OFFSET[2]
    y_ang_center = 90
    y_ang_min = y_ang_center - atand(0.3 / d)
    y_ang_max = y_ang_center + atand(laser_height / d)

    if check_range(d, *LASER_RANGE):
        rect = screen.get_rect((x_ang, y_ang_min, w_ang, y_ang_max - y_ang_min)) 
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