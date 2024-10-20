#!/usr/bin/env python

import rospy
import cv2
import math
from globals.globals import *
from sensor_msgs.msg import LaserScan
import visualization.imgops as imgops
from typing import NamedTuple, List, Tuple
import math_utils.math_function_utils as mf
from visualization.screen_components import Screen
from field_components.field_components import LaserPoint, GenericObject, RisingEdge, FallingEdge

class LaserEdge(NamedTuple):
    index: int
    angle: float
    is_rising: bool
    is_falling: bool
    range_diff: float

class LaserContour(NamedTuple):
    start_angle: float
    end_angle: float

class LaserScanHandler:
    def __init__(self, laser_scan: LaserScan):
        self.laser_scan: LaserScan = laser_scan
        self.ranges = []
        
        self.update(self.laser_scan)

    def get_range_bounds(self) -> Tuple[float]:
        return self.laser_scan.range_min, self.laser_scan.range_max
    
    def get_ranges(self):
        return self.ranges
    
    def get_usable_scan_range(self) -> Tuple[int]:
        return self.laser_index(SCAN_MAX_ANGLE), self.laser_index(SCAN_MIN_ANGLE)

    def laser_angle(self, laser_index):
        return 180 * (laser_index * self.laser_scan.angle_increment / pi + 1)
    
    def laser_index(self, laser_angle):
        return round((laser_angle / 180 - 1) * pi / self.laser_scan.angle_increment)

    def draw_laser_points(self, laser_screen: Screen, *screens: Screen):
        for i, r in enumerate(self.laser_scan.ranges):
            if mf.check_range(r, *LASER_RANGE):
                x_ang = - self.laser_angle(i)
                y_ang = 0
                fo = laser_screen.create_field_object(laser_screen.get_rect((x_ang, y_ang, 0, 0)), r, LaserPoint)
                laser_screen.draw_object(fo, False)

                for screen in screens:
                    if mf.check_range(x_ang, - screen.FOV[0] / 2, screen.FOV[0] / 2):
                        screen.draw_object(fo, False, True, False, False, False)

    def update(self, laser_scan: LaserScan):
        if laser_scan is None:
            rospy.logerr("LaserScanHandler update failed. LaserScan is None!")
            return
        
        self.laser_scan = laser_scan
        self.ranges = limit_ranges(self.laser_scan.ranges, *self.get_range_bounds())

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

def detect_edges(laser_scan: LaserScanHandler):
    edges: List[LaserEdge] = []

    min_index, max_index = laser_scan.get_usable_scan_range()
    laser_ranges = laser_scan.get_ranges()

    for index in range(min_index, max_index):
        range_diff = laser_ranges[index] - laser_ranges[index + 1]
        if range_diff > LASER_EDGE_THRESHOLD:
            edges.append(LaserEdge(index, laser_scan.laser_angle(index), True, False, range_diff))
        
        elif range_diff < -LASER_EDGE_THRESHOLD:
            edges.append(LaserEdge(index, laser_scan.laser_angle(index), False, True, range_diff))

    return edges

def draw_edges(laser_scan: LaserScanHandler, edges: List[LaserEdge], laser_screen: Screen):
    for edge in edges:
        x_ang = - laser_scan.laser_angle(edge.index)
        y_ang = -laser_screen.FOV[1] / 2
        w_ang = 0
        h_ang = laser_screen.FOV[1] - 5
        fo = laser_screen.create_field_object(laser_screen.get_rect((x_ang, y_ang, w_ang, h_ang)), laser_scan.get_range_bounds()[1] * 100, RisingEdge if edge.is_rising else FallingEdge)
        laser_screen.draw_object(fo, False, False, False, True, False)

def detect_contours(laser_scan: LaserScanHandler, edges: List[LaserEdge]):
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

    if len(edges) == 0:
        return []

    contours: List[LaserContour] = []

    if not edges[0].is_rising:
        edges.insert(0, LaserEdge(0, SCAN_MAX_ANGLE, True, False, -edges[0].range_diff))

    if edges[-1].is_rising:
        edges.append(LaserEdge(laser_scan.get_usable_scan_range()[0], SCAN_MIN_ANGLE, False, True, -edges[-1].range_diff))

    for i, edge in enumerate(edges[:-1]):
        if edge.is_rising and edges[i+1].is_falling:
            contours.extend(expand_contour(i, i+1, edges))

    return contours

def expand_contour(start_index, end_index, edges: List[LaserEdge]):
    contour = LaserContour(edges[start_index].angle, edges[end_index].angle)

    lbound_condition = 0 <= start_index - 1 < len(edges) and edges[start_index - 1].is_rising
    ubound_contidion = 0 <= end_index + 1 < len(edges) and edges[end_index + 1].is_falling

    if lbound_condition and ubound_contidion:
        return [contour, *expand_contour(start_index - 1, end_index + 1, edges)]
    
    elif lbound_condition:
        return [contour, *expand_contour(start_index - 1, end_index, edges)]
    
    elif ubound_contidion:
        return [contour, *expand_contour(start_index, end_index + 1, edges)]
    
    return [contour]


def object_from_contour(contour: LaserContour, laser_scan: LaserScanHandler, screen: Screen, object_type=GenericObject):
    start_index = laser_scan.laser_index(contour.start_angle)
    end_index = laser_scan.laser_index(contour.end_angle)

    if end_index == start_index:
        d = laser_scan.get_ranges()[start_index+1]
    else:
        d = sum([laser_scan.get_ranges()[i+1] for i in range(start_index, end_index)]) / (end_index - start_index)

    x_ang = - contour.start_angle - laser_scan.laser_scan.angle_increment * 180 / (pi * 2)
    w_ang = abs(contour.start_angle - contour.end_angle)

    laser_height = LASER_OFFSET[2]
    y_ang_center = 0
    y_ang_min = y_ang_center - mf.atand(0.3 / d)
    y_ang_max = y_ang_center + mf.atand(laser_height / d)

    if mf.check_range(d, *LASER_RANGE):
        rect = screen.get_rect((x_ang, y_ang_min, w_ang, y_ang_max - y_ang_min)) 
        return screen.create_field_object(rect, d, object_type)

def test_detection():
    import time
    from topic_handlers import LaserSubscriber
    rospy.init_node("test")
    laser_screen = Screen.LaserScreen("laser")

    ls_sub = LaserSubscriber()
    time.sleep(2)

    laser_scan = ls_sub.copy_data()
    ls_handler = LaserScanHandler(laser_scan)

    print(ls_handler.laser_index(100))
    print(ls_handler.laser_angle(80))
    assert ls_handler.laser_index(ls_handler.laser_angle(100)) == 100

    edges = detect_edges(ls_handler)
    contours = detect_contours(ls_handler, edges)
    objects = [object_from_contour(contour) for contour in contours]

    laser_screen.image = imgops.laser_scan_to_image(laser_scan, laser_screen.dimensions)
    for o in objects:
        laser_screen.draw_object(o)

    laser_screen.show_image()

    cv2.waitKey(10)
    rospy.spin()

if __name__ == "__main__":
    test_detection()