#!/usr/bin/env python
from math_utils.math_function_utils import *
import cv2

SIMULATION_MODE = True
LAUNCH_MODE = False

KINECT_FOV = (62.0, 48.6)
KINECT_RANGE = (0.5, 5.0) #if SIMULATION_MODE else (500.0, 5000.0)
KINECT_OFFSET = (0.13, 0.0, 0.45) if SIMULATION_MODE else (0.10, 0.0, 0.54)
KINECT_DIMENSIONS = (640, 480)
KINECT_ANGLE = (0, -3, 0) if SIMULATION_MODE else (0, -6, 0)

AREA_MIN = 400 #1000
AREA_YELLOW = 800
CANNY_THRESHOLD_UPPER = 40
CANNY_THRESHOLD_LOWER = 40
CANNY_SMOOTHING = 5
COLOR_MASK_SMOOTHING = 5

CV2_DEFAULT_FONT = cv2.FONT_HERSHEY_SIMPLEX
CV2_DEFAULT_FONT_SCALE = 0.25
CV2_DEFAULT_THICKNESS = 1

LASER_OFFSET = (0.1, 0.0, 0.28) if SIMULATION_MODE else (0.09, 0.0, 0.3)
LASER_EDGE_THRESHOLD = 0.2
SCAN_MIN_ANGLE = -135
SCAN_MAX_ANGLE = 135
LASER_RANGE = (0, 6.0)
LASER_FOV = (360, 90)
LASER_DIMENSIONS = (1000, 300)
LASER_MAX_OBJECT_SIZE = 0.8
LASER_INDEX_MARGIN = 0.014 if SIMULATION_MODE else 0.01

MAX_ANGULAR_SPEED = math.pi / 3
MAX_LINEAR_SPEED = 0.3
TARGET_SIZE = 0.2
REPELLING_FORCE_SIZE = 0.1
REPELLING_FORCE_THRESHOLD = 0.5

TICK_RATE = 10 / 10

NAMESPACE = "" if LAUNCH_MODE else "robot1/"
LASER_PATH = "front_laser/scan"
DEPTH_PATH = "kinect/depth/image_raw" if SIMULATION_MODE else "kinect/depth_registered/image_raw"
IMAGE_PATH = "kinect/rgb/image_raw"

ATTRACTION_FACTOR = 0.1
REPULSION_FACTOR = 0.005
TARGET_REACHED_R_THRESHOLD = 0.1
TARGET_REACHED_THETA_THRESHOLD = 0.1


# Laser scan:
# header: 
#   seq: 1436
#   stamp: 
#     secs: 1412
#     nsecs:  14000000
#   frame_id: "robot1/front_laser"
# angle_min: 3.141590118408203              left
# angle_max: -3.1241400241851807            right
# angle_increment: -0.017453286796808243    clockwise (seen from above)
# time_increment: 0.0
# scan_time: 0.0
# range_min: 0.03999999910593033
# range_max: 6.0
