#!/usr/bin/env python

import cv2
import numpy as np

def denoise(image, factor):
    kernel = np.ones((factor, factor),np.uint8)
    mask = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
    mask =  cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return mask

def mask_color(image, color):
    hsv =  cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([*color.min_hsv()]), np.array([*color.max_hsv()]))
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