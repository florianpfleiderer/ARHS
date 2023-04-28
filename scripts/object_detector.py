#!/usr/bin/env python
import rospy

import cv2
import numpy as np
import sys
import time

from player.msg import FieldComponent, PolarVector2

from field_components.field_components import *
from visualization.screen_components import *
import visualization.imgops as imgops
from globals.globals import *
from math_utils.math_function_utils import *
from data_utils.topic_handlers import ImageSubscriber, LaserSubscriber
from list_utils.filtering import *
from globals.tick import *

CLASSES = {'pole': Pole,
           'yellowpuck': YellowPuck,
           'bluepuck': BluePuck,
           'yellowgoal': YellowGoal,
           'bluegoal': BlueGoal,
           'robot': Robot}

class Detector:
    def __init__(self, detection_function):
        self.test_parameters = {}
        self.detection_function = detection_function
        self.avg_detection_time = 0
        self.detection_counter = 0
        self.counter_cap = 1000

        self.interval_start = 0
        self.printer_interval = 2

        self.detected_objects = 0

    def detect(self, *args):
        self.detection_counter += 1

        detect_start_time = time.time()
        detection_result = self.detection_function(*args)
        detect_duration = time.time() - detect_start_time

        counter = min(self.counter_cap, self.detection_counter)
        self.avg_detection_time = (self.avg_detection_time * (counter - 1) + detect_duration) / counter

        if time.time() - self.interval_start >= self.printer_interval:
            print(f"detection call {self.detection_counter}, detected {self.detected_objects} average time (last {self.counter_cap}) {self.avg_detection_time}")
            self.interval_start = time.time()

        return detection_result

    def show_test_parameters(self):
        for param in list(self.test_parameters.values()):
            param.show()

    def add_test_parameters(self, *params):
        for param in params:
            try:
                name = param.name

            except AttributeError:
                name = str(type(param))

            self.test_parameters[name] = param

class KinectDetector(Detector):
    def __init__(self, testmode): 
        super().__init__(self.detect_field_objects)    

        self.testmode = testmode
        
        self.rgb_sub = ImageSubscriber("rgb image", LOCAL_PLAYER + "kinect/rgb/image_raw", "bgr8")
        self.depth_sub = ImageSubscriber("depth image", LOCAL_PLAYER + "kinect/depth/image_raw", "32FC1")

        self.thresh_upper = TrackbarParameter(CANNY_THRESHOLD_UPPER, "upper", "rgb image")
        self.thresh_lower = TrackbarParameter(CANNY_THRESHOLD_LOWER, "lower", "rgb image")
        # self.add_test_parameters(self.thresh_upper, self.thresh_lower)
    
    def is_valid_data(self):
        return self.rgb_sub.is_valid() and self.depth_sub.is_valid()

    def detect_field_objects(self, base_class):
        rgb_image = self.rgb_sub.get_image()
        depth_image = self.depth_sub.get_image()

        color_mask = imgops.mask_color(rgb_image, base_class.color)
        depth_masked_image = imgops.convert_gray2bgr(imgops.apply_mask(color_mask, depth_image))
        depth_masked_image = imgops.denoise(depth_masked_image, COLOR_MASK_SMOOTHING)

        edges = imgops.edges(depth_masked_image, self.thresh_lower.get_value(self.testmode), self.thresh_upper.get_value(self.testmode))
        edges = imgops.denoise(edges, CANNY_SMOOTHING, False)
        edges = cv2.blur(edges, (3, 3)) # needed?

        self.add_test_parameters(TestImage("rgb image", rgb_image),
                                 TestImage("depth_masked_image", depth_masked_image),
                                 TestImage("edges", edges))
            
        contours = imgops.get_contours(edges)
        if contours == ((), None):
            return []
        
        contours = imgops.get_inner_contours(*contours)

        cv2.drawContours(rgb_image, contours, -1, Color.YELLOW.default, 1)

        def filter_cb(contour):
            x, y, w, h = cv2.boundingRect(contour)
            result = check_range(w*h, *base_class.area_detect_range)
            result &= check_range(w/h, *base_class.ratio_detect_range)
            return result

        contours = filter_list(contours, filter_cb)

        screen_objects = [ScreenObject(*cv2.boundingRect(contour)) for contour in contours]

        if base_class in [YellowGoal, BlueGoal, Robot] and len(screen_objects) > 1:
            screen_objects = [screen_objects[0].merge(*screen_objects[1:])]

        distances = [screen_obj.get_field_vector(depth_image) for screen_obj in screen_objects]

        field_objects = [base_class(distance, screen_obj) for distance, screen_obj in zip(distances, screen_objects)]

        self.detected_objects = len(field_objects)
        return field_objects
    

class LaserScanDetector:
    def __init__(self):
        self.laser_sub = LaserSubscriber("laser scan", LOCAL_PLAYER + "front_laser/scan")
        self.laser_scan = None

        self.rgb_sub = ImageSubscriber("rgb image", LOCAL_PLAYER + "kinect/rgb/image_raw", "bgr8")
        self.rgb_image = None

    def is_valid_data(self):
        return self.laser_sub.is_valid() and self.rgb_sub.is_valid()
       
    def detect_objects(self):
        new_object = True
        min_theta = 0
        max_theta = 0

        previous_range = 0
        previous_theta = -180

        self.rgb_image = self.rgb_sub.get_image()
        self.laser_scan = self.laser_sub.get_scan()

        detected_objects = []

        for index, range in enumerate(self.laser_scan.ranges):
            current_theta = index * self.laser_scan.angle_increment

            if not LASER_MIN_ANGLE <= current_theta <= LASER_MAX_ANGLE:
                continue

            # if not self.laser_scan.range_min <= range <= self.laser_scan.range_max:
            #     continue

            if abs(previous_range - range) > LASER_EDGE_THRESHOLD:
                if new_object:
                    new_object = False
                    min_index = index
                    min_theta = current_theta

                else:
                    max_index = previous_index
                    max_theta = previous_theta

                    center_theta = (max_theta - min_theta) / 2
                    center_index = int((max_index - min_index) / 2)

                    imh = self.rgb_image.shape[0]
                    imw = self.rgb_image.shape[1]

                    d = self.laser_scan.ranges[min_index]
                    min_alpha = atand((KINECT_HEIGHT - LASER_HEIGHT - 0.2) / d)

                    d = self.laser_scan.ranges[max_index]
                    max_alpha = atand((KINECT_HEIGHT - LASER_HEIGHT - 0.2) / d)

                    d = self.laser_scan.ranges[center_index]

                    new_distance = PolarVector2(d, center_theta)
                    screen_obj = ScreenObject.from_angles(min_theta, min_alpha + KINECT_ANGLE, max_theta, max_alpha + KINECT_ANGLE, imw, imh, KINECT_FOV_X, KINECT_FOV_Y)
                    detected_objects.append(FieldObject(FieldComponent("?", "?", new_distance, screen_obj.properties)))

                    new_object = True

            previous_index = index
            previous_theta = current_theta
            previous_range = range

        return detected_objects


if __name__ == '__main__':
    rospy.init_node("object_detector_node")
    rospy.loginfo("Initialised ObjectDetector")

    args = rospy.myargv(argv=sys.argv)
    testmode = args[1] if len(args) > 1 else True
    detect_classes = [CLASSES[key.lower()] for key in args[2:]] if len(args) > 2 else list(CLASSES.values())

    kinect_det = KinectDetector(testmode)
    laser_det = LaserScanDetector()

    def run_kinect_detection():
        if kinect_det.is_valid_data():      
            found_objects = []

            for cls in detect_classes:
                found_objects.extend(kinect_det.detect(cls))
    
            for obj in found_objects:
                obj.draw(kinect_det.test_parameters["rgb image"].get_value(testmode))

            if testmode:    
                kinect_det.show_test_parameters()
        else:
            rospy.loginfo("Waiting for images to process...")

    def run_laser_detection():       
        if laser_det.is_valid_data():
            found_objects = laser_det.detect_objects()
            print(laser_det.rgb_image.shape)

            laser_det.rgb_sub.show_image()
            kinect_det.rgb_sub.draw_objects(found_objects)   
        else:
            rospy.loginfo("Waiting for laser scan to process...")

    
    rospy.loginfo("Starting loop")
    ticker = CallbackTicker(TICK_RATE,
                            run_kinect_detection,
                            # run_laser_detection
                            )
    
    imgticker = CVTicker(TICK_RATE)

    while not rospy.is_shutdown():
        ticker.tick()
        imgticker.tick()
