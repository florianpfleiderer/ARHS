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

CLASSES = {'pole': Pole,
           'yellowpuck': YellowPuck,
           'bluepuck': BluePuck,
           'yellowgoal': YellowGoal,
           'bluegoal': BlueGoal,
           'robot': Robot}

class KinectDetector:
    def __init__(self):     
        self.test_parameters = []
        
        self.rgb_sub = ImageSubscriber("rgb image", "robot1/kinect/rgb/image_raw", "bgr8")
        self.depth_sub = ImageSubscriber("depth image", "robot1/kinect/depth/image_raw", "32FC1")
    
    def is_valid_data(self):
        return self.rgb_sub.is_valid() and self.depth_sub.is_valid()

    def detect_field_objects(self, base_class, testmode=False):
        rgb_image = self.rgb_sub.get_image()
        depth_image = self.depth_sub.get_image()

        rgb_image = imgops.denoise(rgb_image, COLOR_MASK_SMOOTHING)
        color_mask = imgops.mask_color(rgb_image, base_class.color)
        depth_masked = imgops.apply_mask(color_mask, depth_image)
        depth_masked_image = imgops.convert_gray2bgr(depth_masked)

        rgb_image = imgops.denoise(depth_masked_image, CANNY_SMOOTHING)

        thresh_upper = TrackbarParameter(CANNY_THRESHOLD_UPPER, "upper", "rgb image")
        thresh_lower = TrackbarParameter(CANNY_THRESHOLD_LOWER, "lower", "rgb image")
        edges = imgops.edges(depth_masked_image, thresh_lower.get_value(testmode), thresh_upper.get_value(testmode))

        contours = imgops.get_contours(edges)
        if contours == ((), None):
            return []
        
        contours = imgops.get_inner_contours(*contours)

        cv2.drawContours(rgb_image, contours, -1, Color.YELLOW.default, 1)

        if testmode:
            self.test_parameters.extend([TestImage("rgb_image", rgb_image),
                                        TestImage("depth_masked_image", depth_masked_image),
                                        TestImage("edges", edges),
                                        thresh_upper, thresh_lower])

        def filter_cb(contour):
            x, y, w, h = cv2.boundingRect(contour)
            result = check_range(w*h, *base_class.area_detect_range)
            result &= check_range(w/h, *base_class.ratio_detect_range)
            return result

        contours = filter_list(contours, filter_cb)

        screen_objects = [ScreenObject(*cv2.boundingRect(contour)) for contour in contours]

        if type(base_class) in [YellowGoal, BlueGoal, Robot] and len(screen_objects) > 1:
            screen_objects = [screen_objects[0].merge(screen_objects[1:])]

        distances = [screen_obj.get_field_vector(depth_image) for screen_obj in screen_objects]

        field_objects = [base_class(distance, screen_obj) for distance, screen_obj in zip(distances, screen_objects)]

        return field_objects

    def show_test_parameters(self):
        for param in self.test_parameters:
            param.show()

class LaserScanDetector:
    def __init__(self):
        self.laser_sub = LaserSubscriber("laser scan", "robot1/front_laser/scan")
        self.laser_scan = None

        self.rgb_sub = ImageSubscriber("rgb image", "robot1/kinect/rgb/image_raw", "bgr8")
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
    
    # def visualize(self, field_objects):
    #     cv2.imshow("Laser Scan", self.rgb_image)

    #     for field_object in field_objects:
    #         field_object.draw(self.rgb_image)


if __name__ == '__main__':
    rospy.init_node("object_detector_node")
    rospy.loginfo("Initialised ObjectDetector")

    kinect_det = KinectDetector()
    laser_det = LaserScanDetector()

    loop_rate = rospy.Rate(TICK_RATE)

    testmode = True
    detect_classes = CLASSES.values()  

    args = rospy.myargv(argv=sys.argv)
    if len(args) > 1:
        testmode = args[1]
    if len(args) > 2:
        testobjects = [CLASSES[key.lower()] for key in args[2:]]

    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        if kinect_det.is_valid_data():          
            kinect_det.rgb_sub.show_image() 
            found_objects = []
            for cls in detect_classes:
                found_objects.extend(kinect_det.detect_field_objects(cls, testmode))

            kinect_det.rgb_sub.draw_objects(found_objects)     
            kinect_det.show_test_parameters()
        else:
            rospy.loginfo("Waiting for images to process...")

        # if laser_det.is_valid_data():
        #     found_objects = laser_det.detect_objects()
        #     print(laser_det.rgb_image.shape)

        #     laser_det.rgb_sub.show_image()

        #     laser_vis.image = laser_det.rgb_image
        #     laser_vis.show_image()
        #     laser_vis.draw_objects(found_objects)
        # else:
        #     rospy.loginfo("Waiting for images to process...")
        
        loop_rate.sleep()
        cv2.waitKey(TICK_TIME)
