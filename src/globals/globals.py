#!/usr/bin/env python
from math_utils.math_function_utils import *
import cv2

SIMULATION_MODE = True

KINECT_FOV = (62, 48.6)
KINECT_MAX_RANGE = 5.0
KINECT_HEIGHT = 0.45
KINECT_ANGLE = 0

AREA_MIN = 1000
AREA_YELLOW = 800
CANNY_THRESHOLD_UPPER = 40
CANNY_THRESHOLD_LOWER = 40
CANNY_SMOOTHING = 3
COLOR_MASK_SMOOTHING = 5

CV2_DEFAULT_FONT = cv2.FONT_HERSHEY_SIMPLEX
CV2_DEFAULT_FONT_SCALE = 0.25
CV2_DEFAULT_THICKNESS = 1

LASER_EDGE_THRESHOLD = 0.1
LASER_HEIGHT = 0.4
SCAN_MIN_ANGLE = -135
SCAN_MAX_ANGLE = 135

MAX_ANGULAR_SPEED = math.pi / 2
MAX_LINEAR_SPEED = 0.5
TARGET_SIZE = 0.04
REPELLING_FORCE_SIZE = 0.1
REPELLING_FORCE_THRESHOLD = 0.5
REPELLING_FORCE_MULTIPLIER = -0.1

TICK_RATE = 10

LOCAL_PLAYER = "robot1/"
# LOCAL_PLAYER = ""