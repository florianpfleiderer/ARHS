#!/usr/bin/env python

from math import *
import cv2
import numpy as np
from globals.globals import *
from sensor_msgs.msg import LaserScan

def denoise(image, kernel_size, do_open=True):
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)

    if do_open:
        mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return mask

def mask_color(image, color):
    hsv =  cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # hsv = cv2.fastNlMeansDenoisingColored(hsv, None, 10, 10, 7, 21)

    min_hsv = color.min_hsv()
    max_hsv = color.max_hsv()

    if min_hsv[0] > max_hsv[0]:
        mask = cv2.inRange(hsv, np.array([*min_hsv]), np.array([255, max_hsv[1], max_hsv[2]]))
        mask += cv2.inRange(hsv, np.array([0, min_hsv[1], min_hsv[2]]), np.array([*max_hsv]))
    else:
        mask = cv2.inRange(hsv, np.array([*min_hsv]), np.array([*max_hsv]))

    return mask

def invert_mask(mask):
    return cv2.bitwise_not(mask)

def apply_mask(mask, image):
    return cv2.bitwise_and(image, image, mask=mask)

def edges(image, lower_threshold, upper_threshold):
    return cv2.Canny(image, lower_threshold, upper_threshold)

def get_contours(image):
    return cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

def get_inner_contours(contours, hierarchies):
    inner_contours = []

    for contour, hierarchy in zip(contours, hierarchies[0]):
        if hierarchy[2] < 0: # no child contour
            inner_contours.append(contour)
            
    return inner_contours

def convert_gray2bgr(image):
    image_norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
    return cv2.cvtColor(image_norm, cv2.COLOR_GRAY2BGR)

# convert a list of scan ranges into a grayscale image
def laser_scan_to_image(laser_scan: LaserScan, dimensions):
    max_range = laser_scan.range_max
    min_range = laser_scan.range_min
    image = np.zeros((100, len(laser_scan.ranges), 3), np.uint8)
    for i, r in enumerate(laser_scan.ranges):
        if min_range < r < max_range:
            range_value = int(255 * (1 - (r-min_range)/(max_range-min_range)))
            image[:, i] = (range_value, range_value, range_value)
            
        else:
            image[:, i] = (0, 0, 50)

        increment = 180 * laser_scan.angle_increment / pi
        angle = 180 + i * increment
        limit = -increment
        if abs(angle - KINECT_FOV[0]/2) <= limit \
            or abs(angle + KINECT_FOV[0]/2) <= limit \
            or abs(angle - SCAN_MIN_ANGLE) <= limit \
            or abs(angle - SCAN_MAX_ANGLE) <= limit:

            image[40:60, i] = (255, 0, 255)

    return resize(image, dimensions)

def resize(image, new_size):
    return cv2.resize(image, new_size)

def scale(image, factor):
    return cv2.resize(image, (0, 0), fx=factor, fy=factor, interpolation=cv2.INTER_LINEAR if factor > 1 else cv2.INTER_AREA)